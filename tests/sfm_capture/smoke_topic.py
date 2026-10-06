"""Validate the lab launch against external ROS images, with optional simulated time.

Use ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY to isolate this synthetic capture.
The simulation runs faster than wall time, pauses for 3 wall seconds, and can
reset its clock to check that captures are failed rather than silently mixed.
"""
import argparse
import csv
import json
from pathlib import Path
import signal
import subprocess
import time
import uuid

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
import rosbag2_py
from sensor_msgs.msg import Image, CameraInfo, Imu
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from sfm_capture.check_dataset import check
from sfm_capture.bag_support import inspect_bag


def run(simulation=False, reset_clock=False):
    workspace = Path(__file__).resolve().parents[2]
    output = workspace / 'validation' / ('external_capture_' + uuid.uuid4().hex[:8])
    output.parent.mkdir(exist_ok=True)
    log_path = output.with_suffix('.log')
    log = log_path.open('w')
    rclpy.init()
    node = rclpy.create_node('synthetic_external_camera')
    image_pub = node.create_publisher(Image, '/sfm_smoke/image', qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, '/sfm_smoke/info', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, '/mpu6050/imu', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    static_pub = node.create_publisher(TFMessage, '/tf_static',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'camera_optical_frame'
    transform.transform.rotation.w = 1.
    static_pub.publish(TFMessage(transforms=[transform]))
    process = subprocess.Popen([
        'ros2', 'launch', 'lab_navigation', 'sfm_capture.launch.py',
        'source:=' + ('simulation' if simulation else 'topic'),
        'output_dir:=' + str(output), 'image_topic:=/sfm_smoke/image',
        'camera_info_topic:=/sfm_smoke/info', 'device:=/dev/sfm_must_not_be_opened',
        'camera_config:=/nonexistent/sfm_must_not_be_read.yaml',
        'warmup_sec:=' + ('0.5' if simulation else '0.0'),
        'duration_sec:=4.0', 'bag_images:=true',
    ], stdout=log, stderr=subprocess.STDOUT)
    pixels = np.arange(32 * 16 * 3, dtype=np.uint8).reshape(16, 32, 3)
    simulated_ns = 0
    pause_until = None
    paused = False
    paused_alive_checked = False
    reset_sent = False
    publish_at = 0.
    deadline = time.monotonic() + 25
    try:
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now < publish_at:
                continue
            publish_at = now + (0.05 if simulation else 0.1)
            if simulation:
                if image_pub.get_subscription_count() == 0 or clock_pub.get_subscription_count() < 2:
                    continue
                if simulated_ns >= 2_000_000_000 and not paused:
                    paused = True
                    pause_until = now + 3.0
                if pause_until is not None and now < pause_until:
                    clock = Clock()
                    clock.clock.sec = simulated_ns // 1_000_000_000
                    clock.clock.nanosec = simulated_ns % 1_000_000_000
                    clock_pub.publish(clock)
                    if now > pause_until - 0.5:
                        paused_alive_checked = True
                    continue
                if reset_clock and paused and not reset_sent:
                    simulated_ns = 0
                    reset_sent = True
                clock = Clock()
                clock.clock.sec = simulated_ns // 1_000_000_000
                clock.clock.nanosec = simulated_ns % 1_000_000_000
                clock_pub.publish(clock)
                stamp = clock.clock
                simulated_ns += 100_000_000
            else:
                stamp = node.get_clock().now().to_msg()
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = 'camera_optical_frame'
            image.width, image.height, image.step, image.encoding = 32, 16, 96, 'rgb8'
            image.data = pixels.tobytes()
            info = CameraInfo()
            info.header = image.header
            info.width, info.height = 32, 16
            info.distortion_model = 'plumb_bob'
            info.d = [0.] * 5
            info.k = [20., 0., 16., 0., 20., 8., 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [20., 0., 16., 0., 0., 20., 8., 0., 0., 0., 1., 0.]
            image_pub.publish(image)
            info_pub.publish(info)
            imu = Imu()
            imu.header = image.header
            imu.orientation_covariance[0] = -1.
            imu_pub.publish(imu)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.pose.pose.orientation.w = 1.
            odom_pub.publish(odom)
        assert process.poll() is not None, f'Capture timed out; inspect {log_path}'
        session = json.loads((output / 'session.json').read_text())
        if reset_clock:
            assert reset_sent and paused_alive_checked
            assert session['status'] == 'failed'
            assert 'backwards' in session['error'], session
            print(json.dumps(dict(output=str(output), expected_clock_reset_failure=session['error'])))
            return
        assert process.returncode == 0, log_path
        result = check(output)
        assert result['images'] >= 6 and result['span_sec'] >= 2.5, result
        assert (result['width'], result['height']) == (32, 16)
        assert session['camera_info_received'] and session['camera_calibrated']
        assert not list(output.glob('camera_controls*'))
        assert not (output / 'camera_parameters.yaml').exists()
        setup = json.loads((output / 'capture_setup.json').read_text())
        assert setup['device_resolved'] is None
        report = inspect_bag(output / 'bag')
        for topic in ('/sfm_smoke/image', '/sfm_smoke/info', '/tf_static', '/mpu6050/imu', '/odom'):
            assert report['topics'][topic]['message_count'] > 0, report
        assert '/camera/image_raw' not in report['topics']
        assert json.loads((output / 'bag_report.json').read_text())['status'] == 'finalized'
        if simulation:
            assert paused_alive_checked, 'Capture ended during the simulation pause'
            assert session['timestamp_clock'] == 'simulation'
            assert '/clock' in report['topics']
            rows = list(csv.DictReader((output / 'timestamps.csv').open()))
            assert all(0 < int(row['stamp_ns']) < 20_000_000_000 for row in rows)
            assert session['header_to_receive_offset_ns'] is None
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(output / 'bag'), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            while reader.has_next():
                topic, data, recorded_ns = reader.read_next()
                assert 0 < recorded_ns < 20_000_000_000, (topic, recorded_ns)
        print(json.dumps(dict(output=str(output), images=result['images'],
                              span_sec=result['span_sec'], clock=session['timestamp_clock'])))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulation', action='store_true')
    parser.add_argument('--reset-clock', action='store_true')
    args = parser.parse_args()
    if args.reset_clock and not args.simulation:
        parser.error('--reset-clock requires --simulation')
    run(args.simulation, args.reset_clock)
