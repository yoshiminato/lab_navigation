from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Launch configurations
    # ============================================================

    map_name = LaunchConfiguration('map_name')

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    robot_xacro = LaunchConfiguration(
        'robot_xacro'
    )

    localization_params = LaunchConfiguration(
        'localization_params'
    )

    nav2_params = LaunchConfiguration(
        'nav2_params'
    )

    map_config = LaunchConfiguration(
        'map'
    )

    rviz_config = LaunchConfiguration(
        'rviz_config'
    )


    # ============================================================
    # Default paths
    # ============================================================

    default_robot_xacro = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'urdf',
        'diffbot.urdf.xacro'
    ])

    default_localization_params = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'params',
        'localization',
        [map_name, '.yaml']
    ])

    default_nav2_params = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'params',
        'nav2_params.yaml'
    ])

    default_map_config = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'map',
        map_name,
        [map_name, '.yaml']
    ])

    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    ])


    # ============================================================
    # Launch arguments
    # ============================================================

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='warehouse_multiple_shelves',
        description='Map name used to select the occupancy map and localization YAML.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock published by Isaac Sim.'
    )

    declare_robot_xacro = DeclareLaunchArgument(
        'robot_xacro',
        default_value=default_robot_xacro,
        description='Robot Xacro file.'
    )

    declare_localization_params = DeclareLaunchArgument(
        'localization_params',
        default_value=default_localization_params,
        description=(
            'Parameter YAML for '
            'lidar_localization_ros2_custom.'
        )
    )

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=default_nav2_params,
        description='Nav2 parameter YAML.'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_config,
        description='Occupancy grid map YAML.'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz configuration file.'
    )


    # ============================================================
    # ros2_control / robot state publisher
    #
    # Isaac Sim's Action Graph is the MCU-compatible endpoint:
    #
    #   diffbot_base_controller
    #       -> /mcu/wheel_commands -> Isaac Sim
    #   Isaac Sim
    #       -> /mcu/wheel_states -> diffbot_base_controller
    #
    # diffbot_base_controller publishes both /odom and odom -> base_link.
    # robot_state_publisher publishes the URDF transforms, including:
    #
    #   base_link -> velodyne
    #   base_link -> camera_link
    #
    # ============================================================

    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lab_navigation'),
                'launch',
                'diffbot_control.launch.py',
            ])
        ),
        launch_arguments={
            'gui': 'false',
            'use_sim_time': use_sim_time,
            'start_micro_ros_agent': 'false',
            'robot_xacro': robot_xacro,
        }.items(),
    )


    # ============================================================
    # Map Server
    #
    # IMPORTANT:
    #
    # AMCL is NOT started.
    #
    # map -> odom
    #
    # is provided by:
    #
    # lidar_localization_ros2_custom
    #
    # ============================================================

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',

        name='map_server',

        output='screen',

        parameters=[
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_config,
            }
        ]
    )


    # ============================================================
    # Map Server Lifecycle Manager
    # ============================================================

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',

        name='lifecycle_manager_map',

        output='screen',

        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,

                'node_names': [
                    'map_server'
                ],
            }
        ]
    )


    # ============================================================
    # LiDAR Localization
    #
    # Separate terminal
    #
    # Isaac Sim directly publishes:
    #
    #   /velodyne_points
    #
    # Therefore:
    #
    #   - Velodyne driver is NOT required
    #   - ros_gz_bridge is NOT required
    #
    # ============================================================

    lidar_localization = ExecuteProcess(

        cmd=[
            'gnome-terminal',

            '--tab',
            '--title=lidar_localization',

            '--',

            'bash',
            '-c',

            [
                'ros2 launch '
                'lidar_localization_ros2_custom '
                'lidar_localization.launch.py ',

                'localization_param_dir:=',
                localization_params,

                ' use_sim_time:=',
                use_sim_time,

                '; exec bash'
            ]
        ],

        output='screen'
    )


    # ============================================================
    # Obstacle Detection
    #
    # Separate terminal
    #
    # Input:
    #
    #   /velodyne_points
    #
    #       ↓
    #
    # obstacle_cloud_to_scan
    #
    #       ↓
    #
    # LaserScan for Nav2 obstacle layer
    #
    # ============================================================

    obstacle_cloud_to_scan = ExecuteProcess(

        cmd=[
            'gnome-terminal',

            '--tab',
            '--title=obstacle_detection',

            '--',

            'bash',
            '-c',

            [
                'ros2 launch '
                'obstacle_cloud_to_scan '
                'obstacle_cloud_to_scan.launch.py ',

                'use_sim_time:=',
                use_sim_time,

                '; exec bash'
            ]
        ],

        output='screen'
    )


    # ============================================================
    # cmd_vel throttle
    #
    # Nav2: /cmd_vel -> /cmd_vel_slow
    # /cmd_vel_slow is remapped to diffbot_base_controller's
    # cmd_vel_unstamped input by diffbot_control.launch.py.
    #
    # ============================================================

    throttle = ExecuteProcess(

        cmd=[
            'ros2',
            'run',
            'topic_tools',
            'throttle',
            'messages',
            '/cmd_vel',
            '60.0',
            '/cmd_vel_slow',
            '--ros-args',
            '-p',
            ['use_sim_time:=', use_sim_time],
        ],

        output='screen'
    )


    # ============================================================
    # Nav2
    #
    # Separate terminal
    #
    # localization_launch.py is NOT used because
    # lidar_localization_ros2_custom provides localization.
    # ============================================================

    nav2 = ExecuteProcess(

        cmd=[
            'gnome-terminal',

            '--tab',
            '--title=nav2',

            '--',

            'bash',
            '-c',

            [
                'ros2 launch '
                'nav2_bringup '
                'navigation_launch.py ',

                'use_sim_time:=',
                use_sim_time,

                ' params_file:=',
                nav2_params,

                ' log_level:=info',

                '; exec bash'
            ]
        ],

        output='screen'
    )


    # ============================================================
    # RViz
    #
    # Separate terminal
    # ============================================================

    rviz = ExecuteProcess(

        cmd=[
            'gnome-terminal',

            '--tab',
            '--title=rviz',

            '--',

            'bash',
            '-c',

            [
                'rviz2 -d ',
                rviz_config,

                ' --ros-args '
                '-p use_sim_time:=',
                use_sim_time,

                '; exec bash'
            ]
        ],

        output='screen'
    )


    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription([

        # --------------------------------------------------------
        # Launch arguments
        # --------------------------------------------------------

        declare_map_name,

        declare_use_sim_time,

        declare_robot_xacro,

        declare_localization_params,

        declare_nav2_params,

        declare_map,

        declare_rviz_config,


        # --------------------------------------------------------
        # Core nodes
        # --------------------------------------------------------

        robot_control,

        map_server,

        map_lifecycle_manager,


        # --------------------------------------------------------
        # Independent subsystems
        #
        # Each subsystem gets its own terminal tab
        # for easier debugging and cleaner logs.
        # --------------------------------------------------------

        lidar_localization,

        obstacle_cloud_to_scan,

        throttle,

        nav2,

        rviz,
    ])
