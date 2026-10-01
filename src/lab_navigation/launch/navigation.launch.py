from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paths retained from the currently working configuration.
    declare_pcd_file_path_cmd = DeclareLaunchArgument(
        'pcd_file_path',
        default_value='/home/user/lab_navigation_ws/src/lab_navigation/pcd/hv_building_lab.pcd',
    )
    declare_map_yaml_file_path_cmd = DeclareLaunchArgument(
        'map_yaml_file_path',
        default_value='/home/user/lab_navigation_ws/src/lab_navigation/map/hv_building_lab/hv_building_lab.yaml',
    )
    declare_nav2_params_file_path_cmd = DeclareLaunchArgument(
        'nav2_params_file_path',
        default_value='/home/user/lab_navigation_ws/src/lab_navigation/params/nav2_params.yaml',
    )
    declare_localization_params_file_path_cmd = DeclareLaunchArgument(
        'localization_params_file_path',
        default_value='/home/user/lab_navigation_ws/src/lab_navigation/params/localization.yaml',
    )

    # micro-ROS Agent and ros2_control are launched by diffbot.launch.py.
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    serial_baudrate_arg = DeclareLaunchArgument('serial_baudrate', default_value='921600')
    start_agent_arg = DeclareLaunchArgument('start_micro_ros_agent', default_value='true')
    
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lab_navigation'), 'launch', 'diffbot_custom.launch.py'
            ])
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'start_micro_ros_agent': LaunchConfiguration('start_micro_ros_agent'),
        }.items(),
    )

    # カスタムvelodynelancherの起動
    # velodyne_laserscan_nodeの起動を除外したもの
    velodyne = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=velodyne',
            '--',
            'bash',
            '-c',
            'ros2 launch lab_navigation velodyne_points.launch.py; exec bash'
        ],
        output='screen'
    )

    # Source only the integrated workspace; do not source the old localization_ws.
    # The parent shell must have already sourced ROS Humble + micro_ros_ws.
    lidar_localization = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=lidar_localization', '--',
            'bash', '-c', [
                'ros2 launch lidar_localization_ros2_custom lidar_localization.launch.py ',
                'localization_param_dir:=',
                LaunchConfiguration('localization_params_file_path'),
                '; exec bash',
            ],
        ],
        output='screen',
    )

    obstacle_detection = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=obstacle_detection', '--',
            'bash', '-c',
            'ros2 launch obstacle_cloud_to_scan obstacle_cloud_to_scan.launch.py; exec bash',
        ],
        output='screen',
    )

    throttle = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'topic_tools', 'throttle', 'messages',
            '/cmd_vel', '60.0', '/cmd_vel_slow',
        ],
        output='screen',
    )

    # Resolves the copied, currently customized nav2_bringup from lab_navigation_ws.
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py',
            ])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_yaml_file_path'),
        }.items(),
    )

    # Only specify params_file once.
    nav2 = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=nav2', '--',
            'bash', '-c', [
                'ros2 launch nav2_bringup navigation_launch.py ',
                'use_sim_time:=false params_file:=',
                LaunchConfiguration('nav2_params_file_path'),
                ' log_level:=info; exec bash',
            ],
        ],
        output='screen',
    )

    rviz = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=rviz', '--',
            'bash', '-c',
            'ros2 launch nav2_bringup rviz_launch.py; exec bash',
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_pcd_file_path_cmd,
        declare_map_yaml_file_path_cmd,
        declare_nav2_params_file_path_cmd,
        declare_localization_params_file_path_cmd,
        serial_port_arg,
        serial_baudrate_arg,
        start_agent_arg,
        ros2_control,
        velodyne,
        lidar_localization,
        obstacle_detection,
        throttle,
        map_server,
        nav2,
        rviz,
    ])
