"""Isaac Sim + HLOC + Nav2: GLIM map alignment per map_name.

Place each map's world_to_map.yaml in:
  lab_navigation/map/<map_name>/world_to_map.yaml

This ROS parameter file defines rigid T_map_world.  Do not silently assume
identity for an uncalibrated map: a missing file should fail startup.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def terminal_tab(title, command, condition=None):
    """Run a subsystem in its own GNOME Terminal tab."""
    return ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            f'--title={title}',
            '--',
            'bash',
            '-c',
            [*command, '; exec bash'],
        ],
        output='screen',
        condition=condition,
    )


def generate_launch_description():
    pkg = FindPackageShare('lab_navigation')
    nav_pkg = FindPackageShare('nav2_bringup')

    map_name = LaunchConfiguration('map_name')
    sim_time = LaunchConfiguration('use_sim_time')
    robot_xacro = LaunchConfiguration('robot_xacro')
    nav2_params = LaunchConfiguration('nav2_params')
    map_yaml = LaunchConfiguration('map')
    rviz_config = LaunchConfiguration('rviz_config')
    start_rviz = LaunchConfiguration('start_rviz')
    start_navigation = LaunchConfiguration('start_navigation')
    start_hloc = LaunchConfiguration('start_hloc')
    image_topic = LaunchConfiguration('image_topic')
    data_path = LaunchConfiguration('data_path')
    local_method = LaunchConfiguration('local_method')
    num_matched = LaunchConfiguration('num_matched')
    camera_frame = LaunchConfiguration('camera_frame')
    map_frame = LaunchConfiguration('map_frame')

    # The default follows map_name. An explicit override remains possible.
    alignment_yaml = LaunchConfiguration('alignment_yaml')

    arguments = [
        DeclareLaunchArgument('map_name', default_value='warehouse_multiple_shelves'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_xacro', default_value=PathJoinSubstitution([
            pkg, 'urdf', 'diffbot.urdf.xacro'])),
        DeclareLaunchArgument('nav2_params', default_value=PathJoinSubstitution([
            pkg, 'params', 'nav2_params.yaml'])),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            pkg, 'map', map_name, [map_name, '.yaml']])),
        DeclareLaunchArgument('alignment_yaml', default_value=PathJoinSubstitution([
            pkg, 'map', map_name, 'world_to_map.yaml']),
            description='Per-map rigid T_map_world parameters for hloc_tf_publisher.'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([
            nav_pkg, 'rviz', 'nav2_default_view.rviz'])),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_hloc', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('data_path',
                              default_value='/home/user/lab_navigation_ws/captures/isaac_gt_test_002/colmap_gt'),
        DeclareLaunchArgument('local_method', default_value='sift'),
        DeclareLaunchArgument('num_matched', default_value='20'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument('map_frame', default_value='map'),
    ]

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg, 'launch', 'diffbot_control.launch.py'])),
        launch_arguments={
            'gui': 'false',
            'use_sim_time': sim_time,
            'start_micro_ros_agent': 'false',
            'robot_xacro': robot_xacro,
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{
            'use_sim_time': ParameterValue(sim_time, value_type=bool),
            'yaml_filename': map_yaml,
        }],
    )
    map_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen', parameters=[{
            'use_sim_time': ParameterValue(sim_time, value_type=bool),
            'autostart': True, 'node_names': ['map_server'],
        }],
    )

    hloc = terminal_tab(
        'hloc_localizer',
        [
            'ros2 run hloc_localizer hloc_localizer --ros-args '
            '-r __node:=hloc_localizer '
            '-p use_sim_time:=', sim_time,
            ' -p data_path:=', data_path,
            ' -p image_topic:=', image_topic,
            ' -p local_method:=', local_method,
            ' -p num_matched:=', num_matched,
            ' -p map_frame:=', map_frame,
        ],
        condition=IfCondition(start_hloc),
    )
    hloc_tf = terminal_tab(
        'hloc_tf_publisher',
        [
            'ros2 run hloc_localizer hloc_tf_publisher --ros-args '
            '--params-file ', alignment_yaml,
            ' -p use_sim_time:=', sim_time,
            ' -p pose_topic:=/hloc/camera_pose',
            ' -p map_frame:=', map_frame,
            ' -p odom_frame:=odom',
            ' -p base_frame:=base_link',
            ' -p camera_frame:=', camera_frame,
        ],
        condition=IfCondition(start_hloc),
    )

    obstacles = terminal_tab(
        'obstacle_detection',
        [
            'ros2 launch obstacle_cloud_to_scan '
            'obstacle_cloud_to_scan.launch.py use_sim_time:=', sim_time,
        ],
    )

    throttle = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'throttle', 'messages',
             '/cmd_vel', '60.0', '/cmd_vel_slow'], output='screen',
        condition=IfCondition(start_navigation),
    )

    nav2 = terminal_tab(
        'nav2',
        [
            'ros2 launch nav2_bringup navigation_launch.py '
            'use_sim_time:=', sim_time,
            ' params_file:=', nav2_params,
            ' log_level:=info',
        ],
        condition=IfCondition(start_navigation),
    )

    rviz = terminal_tab(
        'rviz',
        [
            'rviz2 -d ', rviz_config,
            ' --ros-args -p use_sim_time:=', sim_time,
        ],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(arguments + [
        robot, map_server, map_manager, hloc_tf, hloc,
        obstacles, throttle, nav2, rviz,
    ])
