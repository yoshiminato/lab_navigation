import math
import shutil
import subprocess
import time
from threading import Thread
from pathlib import Path

from cv_bridge import CvBridge
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image, CameraInfo

from .dataset import Dataset, write_json
from .camera_contract import CameraContract
from .image_writer import ImageWriter


class Recorder(Node):
    def __init__(self):
        super().__init__('sfm_recorder')
        defaults = dict(output_dir='', image_topic='/camera/image_raw',
                        camera_info_topic='/camera/camera_info', save_fps=2.0,
                        warmup_sec=2.0, duration_sec=0.0, no_frame_timeout_sec=15.0,
                        writer_queue_size=4, device='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.options = {key: self.get_parameter(key).value for key in defaults}
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.options['use_sim_time'] = self.use_sim_time
        if not self.options['output_dir']:
            raise ValueError('output_dir is required')
        for key in ['warmup_sec', 'duration_sec', 'no_frame_timeout_sec']:
            if not math.isfinite(self.options[key]) or self.options[key] < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        if self.options['writer_queue_size'] < 1:
            raise ValueError('writer_queue_size must be a positive integer')
        self.dataset = Dataset(self.options['output_dir'], self.options['save_fps'],
                               dict(source='live_ros_image', parameters=self.options,
                                    timestamp_clock='simulation' if self.use_sim_time else 'system',
                                    received_ns_semantics='Host UNIX receipt time, even in simulation'))
        self.bridge = CvBridge()
        self.contract = CameraContract()
        self.controls_thread = None
        self.writer = ImageWriter(self.dataset, self.convert_image, self.options['writer_queue_size'])
        self.started = time.monotonic()
        self.source_started_ns = None
        self.last_clock_ns = None
        self.last_received = self.started
        self.error = None
        self.done = False
        self.info_saved = False
        self.first_image = True
        self.create_subscription(Image, self.options['image_topic'], self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.options['camera_info_topic'], self.on_info,
                                 qos_profile_sensor_data)
        # Poll even while /clock is paused so shutdown and explicit wall-time
        # frame timeouts still work. Capture duration uses the source clock.
        self.create_timer(0.2, self.on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_log_sec = -1
        self.get_logger().info(f"Saving to {self.dataset.path}; warmup {self.options['warmup_sec']} s")

    def on_info(self, message):
        if self.done:
            return
        try:
            info = message_to_ordereddict(message)
            self.contract.observe_info(info)
            if not self.info_saved:
                write_json(self.dataset.path / 'camera_info.json', info)
                self.dataset.update(camera_calibrated=bool(message.k[0] != 0))
                self.info_saved = True
        except Exception as error:
            self.fail(error)

    def convert_image(self, message):
        return self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')

    def save_controls_at_start(self):
        # A best-effort V4L2 query must not fill the PNG queue while it waits.
        self.dataset.update(camera_controls_at_start=self.controls_snapshot('camera_controls_at_start.txt'))

    def fail(self, error):
        self.error = str(error)
        self.get_logger().error(self.error)
        self.done = True

    def on_image(self, message):
        if self.done:
            return
        self.last_received = time.monotonic()
        try:
            elapsed = self.capture_elapsed()
            if elapsed is None or elapsed < self.options['warmup_sec']:
                return
            self.contract.observe_image(message.width, message.height, message.encoding,
                                        message.header.frame_id)
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            # Gazebo may publish an initial frame at zero before time starts.
            # Preserve real stamps; never substitute wall time for simulation.
            if self.use_sim_time and stamp == 0:
                return
            received = time.time_ns()
            if not self.dataset.eligible(stamp):
                return
            self.writer.submit(message, stamp, received)
            if self.first_image:
                self.dataset.update(frame_id=message.header.frame_id, source_encoding=message.encoding,
                                    header_to_receive_offset_ns=None if self.use_sim_time else received - stamp)
                self.first_image = False
                if self.options['device']:
                    self.controls_thread = Thread(target=self.save_controls_at_start,
                                                  name='sfm_camera_controls', daemon=True)
                    self.controls_thread.start()
        except Exception as error:
            self.fail(error)

    def capture_elapsed(self):
        if not self.use_sim_time:
            return time.monotonic() - self.started
        now = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now < self.last_clock_ns:
            raise ValueError('Simulation /clock went backwards; start a new capture session')
        if now <= 0:
            return None
        if self.source_started_ns is None:
            self.source_started_ns = now
        self.last_clock_ns = now
        return (now - self.source_started_ns) / 1e9

    def on_timer(self):
        try:
            elapsed = self.capture_elapsed()
        except ValueError as error:
            self.fail(error)
            return
        if self.writer.error:
            self.fail(f'PNG writer failed: {self.writer.error}')
        if (self.options['no_frame_timeout_sec'] > 0 and
                time.monotonic() - self.last_received > self.options['no_frame_timeout_sec']):
            self.fail('No camera frames received within timeout')
        if (elapsed is not None and self.options['duration_sec'] > 0 and
                elapsed >= self.options['duration_sec'] + self.options['warmup_sec']):
            self.done = True
        self.dataset.update(elapsed_sec=elapsed if elapsed is not None else 0.0)
        log_sec = int(time.monotonic() - self.started)
        if log_sec % 5 == 0 and log_sec != self.last_log_sec:
            self.last_log_sec = log_sec
            self.get_logger().info(str(self.dataset.stats))

    def controls_snapshot(self, filename):
        if not self.options['device']:
            return 'Not applicable: subscribing to an existing ROS image topic'
        if not shutil.which('v4l2-ctl'):
            return 'v4l2-ctl not installed'
        try:
            result = subprocess.run(['v4l2-ctl', '-d', self.options['device'], '--all'],
                                    capture_output=True, text=True, timeout=5)
            (self.dataset.path / filename).write_text(result.stdout + result.stderr)
            return f'exit_code={result.returncode}'
        except (OSError, subprocess.TimeoutExpired) as error:
            return str(error)

    def finish(self):
        # Drain accepted frames in order before closing the manifest/session.
        try:
            self.writer.close()
        except Exception as error:
            self.error = str(error)
        if self.controls_thread is not None:
            self.controls_thread.join()
        self.dataset.metadata['writer'] = dict(queue_size=self.options['writer_queue_size'],
            peak_queue=self.writer.peak_queue, max_convert_and_write_sec=self.writer.max_write_sec)
        self.dataset.metadata['camera_info_received'] = self.info_saved
        # A best-effort metadata query must not prevent flushing the dataset.
        self.dataset.metadata['camera_controls_at_end'] = self.controls_snapshot('camera_controls_at_end.txt')
        failure = self.dataset.path / 'capture_failure.json'
        if failure.exists():
            self.error = failure.read_text()
        if self.dataset.stats['saved'] == 0 and not self.error:
            self.error = 'No images saved'
        self.dataset.close('failed' if self.error else 'complete', self.error)


def main(args=None):
    rclpy.init(args=args)
    node = None
    failed = False
    try:
        node = Recorder()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        failed = True
        if node:
            node.error = str(error)
        raise
    finally:
        if node:
            try:
                node.finish()
                failed = bool(node.error) or failed
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)
