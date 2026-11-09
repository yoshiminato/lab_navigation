"""Run LiDAR-driven navigation while recording HLoc poses for evaluation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_file_path',
        default_value=(
            '/home/user/lab_navigation_ws/src/lab_navigation/'
            'map/hv_building_lab/hv_building_lab.yaml'
        ),
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file_path',
        default_value=(
            '/home/user/lab_navigation_ws/src/lab_navigation/'
            'params/nav2_params.yaml'
        ),
    )
    localization_params_arg = DeclareLaunchArgument(
        'localization_params_file_path',
        default_value=(
            '/home/user/lab_navigation_ws/src/lab_navigation/'
            'params/localization.yaml'
        ),
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0'
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate', default_value='921600'
    )
    start_agent_arg = DeclareLaunchArgument(
        'start_micro_ros_agent', default_value='true'
    )
    start_rviz_arg = DeclareLaunchArgument('start_rviz', default_value='true')

    hloc_data_path_arg = DeclareLaunchArgument(
        'hloc_data_path',
        default_value=(
            '/home/user/lab_navigation_ws/captures/'
            'hv_building_lab/20261010'
        ),
    )
    local_method_arg = DeclareLaunchArgument(
        'local_method', default_value='superpoint',
    )
    bag_output_dir_arg = DeclareLaunchArgument(
        'bag_output_dir',
        default_value=(
            '/home/user/lab_navigation_ws/captures/'
            'hv_building_lab/evaluation/run_01'
        ),
    )
    record_bag_arg = DeclareLaunchArgument('record_bag', default_value='true')

    lidar_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lab_navigation'),
                'launch',
                'navigation_real_lidar.launch.py',
            ])
        ),
        launch_arguments={
            'map_yaml_file_path': LaunchConfiguration('map_yaml_file_path'),
            'nav2_params_file_path': LaunchConfiguration('nav2_params_file_path'),
            'localization_params_file_path':
                LaunchConfiguration('localization_params_file_path'),
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'start_micro_ros_agent': LaunchConfiguration('start_micro_ros_agent'),
            'start_rviz': LaunchConfiguration('start_rviz'),
        }.items(),
    )

    camera = Node(
        package='image_tools',
        executable='cam2image',
        name='evaluation_camera',
        output='screen',
        parameters=[{
            'width': 1920,
            'height': 1080,
            'reliability': 'reliable',
            'history': 'keep_last',
            'depth': 1,
        }],
        remappings=[('image', '/camera/image_raw')],
    )

    hloc_localizer = Node(
        package='hloc_localizer',
        executable='hloc_localizer',
        name='hloc_localizer_evaluation',
        output='screen',
        parameters=[{
            'data_path': LaunchConfiguration('hloc_data_path'),
            'image_topic': '/camera/image_raw',
            'local_method': LaunchConfiguration('local_method'),
            'num_matched': 10,
            'min_inliers': 12,
            'ransac_thresh': 12.0,
            'descriptor_cache_size': 40,
            'map_frame': 'map',
        }],
    )

    # HLoc is a pose-only observer in this experiment. The HLoc TF publisher
    # is deliberately omitted so LiDAR Localization remains the sole source
    # of the map-to-odom correction used by Nav2.
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--storage', 'sqlite3',
            '--output', LaunchConfiguration('bag_output_dir'),
            '/pcl_pose',
            '/hloc/camera_pose',
            '/hloc/localization_status',
            '/tf',
            '/tf_static',
            '/odom',
            '/cmd_vel',
            '/cmd_vel_slow',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag')),
    )

    return LaunchDescription([
        map_yaml_arg,
        nav2_params_arg,
        localization_params_arg,
        serial_port_arg,
        serial_baudrate_arg,
        start_agent_arg,
        start_rviz_arg,
        hloc_data_path_arg,
        local_method_arg,
        bag_output_dir_arg,
        record_bag_arg,
        lidar_navigation,
        camera,
        hloc_localizer,
        bag_record,
    ])
