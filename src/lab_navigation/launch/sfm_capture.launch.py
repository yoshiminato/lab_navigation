"""Collect SfM images using the camera and sensor topics in lab_navigation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'extra_bag_topics', default_value='/odom /mpu6050/imu',
            description='Additional lab_navigation topics to save alongside the camera and TF'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('sfm_capture'), 'launch', 'capture.launch.py'])),
            launch_arguments={
                'extra_bag_topics': LaunchConfiguration('extra_bag_topics'),
            }.items()),
    ])
