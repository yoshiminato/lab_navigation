"""Capture Isaac Sim images and a separate camera ground-truth TF stream."""
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_dir = str(Path.cwd() / 'captures' /
                      datetime.now().strftime('isaac_%Y%m%d_%H%M%S_%f'))
    return LaunchDescription([
        DeclareLaunchArgument('gt_tf_topic', default_value='/sfm_gt_tf',
                              description='Dedicated Isaac camera ground-truth TF topic'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('output_dir', default_value=default_dir,
                              description='New capture session directory'),
        DeclareLaunchArgument('save_fps', default_value='2.0'),
        DeclareLaunchArgument('warmup_sec', default_value='2.0'),
        DeclareLaunchArgument('duration_sec', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('sfm_capture'), 'launch', 'capture.launch.py'])),
            launch_arguments={
                'source': 'simulation',
                'use_sim_time': 'true',
                'record_bag': 'true',
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'extra_bag_topics': LaunchConfiguration('gt_tf_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'save_fps': LaunchConfiguration('save_fps'),
                'warmup_sec': LaunchConfiguration('warmup_sec'),
                'duration_sec': LaunchConfiguration('duration_sec'),
            }.items()),
    ])
