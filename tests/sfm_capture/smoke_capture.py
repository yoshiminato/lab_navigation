"""Exercise the real launch/rosbag shutdown with a synthetic camera, IMU and TF.

The camera action is replaced only in this test. No USB device or robot commands.
Run in an isolated ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import uuid

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import rosbag2_py

from launch import LaunchDescription, LaunchService, LaunchDescriptionSource
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.events import Shutdown
from sfm_capture.bag_support import inspect_bag
from sfm_capture.check_dataset import check


def publish_camera():
    rclpy.init()
    node = rclpy.create_node('synthetic_capture_camera')
    image_pub = node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    pixels = np.arange(32 * 16 * 3, dtype=np.uint8).reshape(16, 32, 3)

    def publish():
        image = Image()
        image.header.stamp = node.get_clock().now().to_msg()
        image.header.frame_id = 'camera_optical_frame'
        image.width, image.height, image.step, image.encoding = 32, 16, 96, 'rgb8'
        image.data = pixels.tobytes()
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = 32, 16
        image_pub.publish(image)
        info_pub.publish(info)
        imu = Imu()
        imu.header.stamp = image.header.stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation_covariance[0] = -1.
        imu.angular_velocity.z = 0.2
        imu.linear_acceleration.z = 9.81
        imu_pub.publish(imu)
        odom = Odometry()
        odom.header.stamp = image.header.stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)

    node.create_timer(0.1, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_capture(include_images, manual_stop=False):
    workspace = Path(__file__).resolve().parents[2]
    output = workspace / 'validation' / ('synthetic_capture_' + uuid.uuid4().hex[:8])
    spec = importlib.util.spec_from_file_location('capture_launch',
        workspace / 'src/sfm_capture/launch/capture.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_node = module.Node

    def test_node(**options):
        if options.get('package') == 'usb_cam':
            return ExecuteProcess(cmd=[sys.executable, str(Path(__file__).resolve()), '--camera'],
                                  output='screen')
        return original_node(**options)

    module.Node = test_node
    # A retained static transform is published before rosbag starts; this tests
    # the actual late-joining transient_local subscription rather than a mock.
    rclpy.init()
    static_node = rclpy.create_node('synthetic_static_transform')
    static_pub = static_node.create_publisher(TFMessage, '/tf_static',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'camera_optical_frame'
    transform.transform.rotation.w = 1.
    static_pub.publish(TFMessage(transforms=[transform]))
    service = LaunchService()
    # IncludeLaunchDescription assigns the real launch arguments into context.
    class SyntheticSource(LaunchDescriptionSource):
        def get_launch_description(self, context):
            return module.generate_launch_description()

    actions = [IncludeLaunchDescription(
        SyntheticSource(), launch_arguments={
            'output_dir': str(output), 'device': '/dev/null', 'warmup_sec': '0.0',
            'duration_sec': '0.0' if manual_stop else '5.0',
            'bag_images': str(include_images).lower(),
        }.items())]
    if manual_stop:
        actions.append(TimerAction(period=5.0, actions=[
            EmitEvent(event=Shutdown(reason='test stop with recorder still running'))]))
    service.include_launch_description(LaunchDescription(actions))
    try:
        assert service.run() == 0
        result = check(output)
        assert result['images'] >= 5, result
        report = inspect_bag(output / 'bag')
        topics = report['topics']
        assert topics['/imu/data']['message_count'] >= 10
        assert topics['/odom']['message_count'] >= 10
        assert topics['/tf_static']['message_count'] >= 1
        assert ('/camera/image_raw' in topics) == include_images
        launch_report = json.loads((output / 'bag_report.json').read_text())
        assert launch_report['status'] == 'finalized'
        assert launch_report['returncode'] == 0
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(output / 'bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf_static']))
        topic, data, recorded_ns = reader.read_next()
        message = deserialize_message(data, TFMessage)
        assert message.transforms[0].child_frame_id == 'camera_optical_frame'
        print(json.dumps(dict(output=str(output), **result), indent=2))
    finally:
        static_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', action='store_true')
    parser.add_argument('--bag-images', action='store_true')
    parser.add_argument('--manual-stop', action='store_true')
    args = parser.parse_args()
    if args.camera:
        publish_camera()
    else:
        run_capture(args.bag_images, args.manual_stop)
