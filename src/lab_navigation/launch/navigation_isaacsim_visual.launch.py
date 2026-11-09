"""Isaac Sim + HLOC Visual Localization + Nav2.

Follow the original LiDAR launch layout: independent GNOME Terminal tabs
for localization, obstacle detection, Nav2 and RViz.

The per-map alignment parameter file is:
  <lab_navigation share>/map/<map_name>/world_to_map.yaml

Only hloc_tf_publisher publishes map -> odom; no AMCL or LiDAR localization.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # Launch configurations
    # ============================================================
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_xacro = LaunchConfiguration('robot_xacro')
    nav2_params = LaunchConfiguration('nav2_params')
    map_config = LaunchConfiguration('map')
    alignment_yaml = LaunchConfiguration('alignment_yaml')
    rviz_config = LaunchConfiguration('rviz_config')

    start_hloc = LaunchConfiguration('start_hloc')
    start_navigation = LaunchConfiguration('start_navigation')
    start_rviz = LaunchConfiguration('start_rviz')

    image_topic = LaunchConfiguration('image_topic')
    data_path = LaunchConfiguration('data_path')
    local_method = LaunchConfiguration('local_method')
    num_matched = LaunchConfiguration('num_matched')
    camera_frame = LaunchConfiguration('camera_frame')
    map_frame = LaunchConfiguration('map_frame')

    pkg = FindPackageShare('lab_navigation')

    # ============================================================
    # Default paths
    # ============================================================
    default_robot_xacro = PathJoinSubstitution([
        pkg, 'urdf', 'diffbot.urdf.xacro',
    ])
    default_nav2_params = PathJoinSubstitution([
        pkg, 'params', 'nav2_params.yaml',
    ])
    default_map_config = PathJoinSubstitution([
        pkg, 'map', map_name, [map_name, '.yaml'],
    ])
    default_alignment_yaml = PathJoinSubstitution([
        pkg, 'map', map_name, 'world_to_map.yaml',
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz',
    ])

    # ============================================================
    # Launch arguments
    # ============================================================
    arguments = [
        DeclareLaunchArgument(
            'map_name', default_value='warehouse_multiple_shelves',
            description='Select occupancy map and its world_to_map.yaml.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_xacro', default_value=default_robot_xacro),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_params),
        DeclareLaunchArgument('map', default_value=default_map_config),
        DeclareLaunchArgument(
            'alignment_yaml', default_value=default_alignment_yaml,
            description='Per-map world-to-map correction for hloc_tf_publisher.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('start_hloc', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'data_path',
            default_value='/home/user/lab_navigation_ws/captures/isaac_001'),
        DeclareLaunchArgument('local_method', default_value='sift'),
        DeclareLaunchArgument('num_matched', default_value='20'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument('map_frame', default_value='map'),
    ]

    # ============================================================
    # Robot control (unchanged: launches its own controllers)
    # ============================================================
    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg, 'launch', 'diffbot_control.launch.py',
        ])),
        launch_arguments={
            'gui': 'false',
            'use_sim_time': use_sim_time,
            'start_micro_ros_agent': 'false',
            'robot_xacro': robot_xacro,
        }.items(),
    )

    # ============================================================
    # Map Server: not AMCL
    # ============================================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'yaml_filename': map_config,
        }],
    )
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ============================================================
    # HLOC Localizer: separate GNOME terminal tab
    # ============================================================
    hloc_localizer = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=hloc_localizer',
            '--',
            'bash', '-c',
            [
                'ros2 run hloc_localizer hloc_localizer --ros-args ',
                '-r __node:=hloc_localizer',
                ' -p use_sim_time:=', use_sim_time,
                ' -p data_path:=', data_path,
                ' -p image_topic:=', image_topic,
                ' -p local_method:=', local_method,
                ' -p num_matched:=', num_matched,
                ' -p map_frame:=', map_frame,
                '; exec bash',
            ],
        ],
        output='screen',
        condition=IfCondition(start_hloc),
    )

    # ============================================================
    # HLOC TF Publisher: separate GNOME terminal tab
    # Loads the YAML selected by map_name, then overrides common args
    # ============================================================
    hloc_tf_publisher = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=hloc_tf_publisher',
            '--',
            'bash', '-c',
            [
                'ros2 run hloc_localizer hloc_tf_publisher --ros-args ',
                '--params-file ', alignment_yaml,
                ' -p use_sim_time:=', use_sim_time,
                ' -p pose_topic:=/hloc/camera_pose',
                ' -p map_frame:=', map_frame,
                ' -p odom_frame:=odom',
                ' -p base_frame:=base_link',
                ' -p camera_frame:=', camera_frame,
                '; exec bash',
            ],
        ],
        output='screen',
        condition=IfCondition(start_hloc),
    )

    # ============================================================
    # Obstacle Detection: separate GNOME terminal tab
    # ============================================================
    obstacle_cloud_to_scan = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=obstacle_detection',
            '--',
            'bash', '-c',
            [
                'ros2 launch obstacle_cloud_to_scan ',
                'obstacle_cloud_to_scan.launch.py ',
                'use_sim_time:=', use_sim_time,
                '; exec bash',
            ],
        ],
        output='screen',
    )

    # ============================================================
    # cmd_vel throttle
    # ============================================================
    throttle = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'topic_tools', 'throttle', 'messages',
            '/cmd_vel', '60.0', '/cmd_vel_slow',
        ],
        output='screen',
        condition=IfCondition(start_navigation),
    )

    # ============================================================
    # Nav2: separate GNOME terminal tab (no localization_launch.py)
    # ============================================================
    nav2 = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=nav2',
            '--',
            'bash', '-c',
            [
                'ros2 launch nav2_bringup navigation_launch.py ',
                'use_sim_time:=', use_sim_time,
                ' params_file:=', nav2_params,
                ' log_level:=info',
                '; exec bash',
            ],
        ],
        output='screen',
        condition=IfCondition(start_navigation),
    )

    # ============================================================
    # RViz: separate GNOME terminal tab
    # ============================================================
    rviz = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=rviz',
            '--',
            'bash', '-c',
            [
                'rviz2 -d ', rviz_config,
                ' --ros-args -p use_sim_time:=', use_sim_time,
                '; exec bash',
            ],
        ],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(arguments + [
        robot_control,
        map_server,
        map_lifecycle_manager,
        hloc_localizer,
        hloc_tf_publisher,
        obstacle_cloud_to_scan,
        throttle,
        nav2,
        rviz,
    ])
