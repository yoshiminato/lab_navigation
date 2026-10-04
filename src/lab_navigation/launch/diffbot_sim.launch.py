from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    ExecuteProcess,
    DeclareLaunchArgument
)

from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    FindExecutable
)

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ============================================================
    # Paths
    # ============================================================

    robot_xacro = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'urdf',
        'diffbot_sim.urdf.xacro'
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'params',
        'sim_diff_drive_controller.yaml'
    ])

    localization_params = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'params',
        'localization_simulation.yaml'
    ])

    nav2_params = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'params',
        'nav2_params.yaml'
    ])

    map_config = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'map',
        'hv_building_lab_simulation/hv_building_lab_simulation.yaml'
    ])


    # ============================================================
    # Robot description
    # ============================================================

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        robot_xacro
    ])

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }


    # ============================================================
    # Gazebo
    # ============================================================

    model_path = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'models'
    ])

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=model_path
    )

    world_file = PathJoinSubstitution([
        FindPackageShare('lab_navigation'),
        'world',
        'campus_world.sdf'
    ])

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])

        ),

        launch_arguments={
            'gz_args': [
                '-r -v 4 ',
                world_file
            ]
        }.items()

    )


    # ============================================================
    # robot_state_publisher
    # ============================================================

    robot_state_publisher = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        output='screen',

        parameters=[
            robot_description,
            {
                'use_sim_time': True
            }
        ]

    )


    # ============================================================
    # throttle
    #
    # /cmd_vel
    #    ↓
    # /cmd_vel_slow
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
        ],

        output='screen',

    )


    # ============================================================
    # Spawn robot
    # ============================================================

    spawn_robot = Node(

        package='ros_gz_sim',

        executable='create',

        output='screen',

        arguments=[
            '-topic',
            'robot_description',

            '-name',
            'diffbot',

            '-allow_renaming',
            'true',

            '-x',
            '-2.0',

            '-y',
            '0.0',

            '-z',
            '0.50',

            '-Y',
            '1.5708',
        ]

    )


    # ============================================================
    # joint_state_broadcaster
    # ============================================================

    joint_state_broadcaster = Node(

        package='controller_manager',

        executable='spawner',

        output='screen',

        arguments=[
            'joint_state_broadcaster'
        ]

    )


    # ============================================================
    # diff_drive_controller
    # ============================================================

    diff_drive_controller = Node(

        package='controller_manager',

        executable='spawner',

        output='screen',

        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers
        ]

    )


    # ============================================================
    # Spawn完了
    #    ↓
    # joint_state_broadcaster
    #    ↓
    # diff_drive_controller
    # ============================================================

    start_joint_state_broadcaster = RegisterEventHandler(

        OnProcessExit(

            target_action=spawn_robot,

            on_exit=[
                joint_state_broadcaster
            ]

        )

    )


    start_diff_drive_controller = RegisterEventHandler(

        OnProcessExit(

            target_action=joint_state_broadcaster,

            on_exit=[
                diff_drive_controller
            ]

        )

    )


    # ============================================================
    # Map Server
    # ============================================================

    map_server = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py',
            ])

        ),

        launch_arguments={
            'map': map_config,
            'use_sim_time': 'true',
        }.items(),

    )


    # ============================================================
    # Gazebo clock -> ROS 2
    # ============================================================

    clock_bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],

        output='screen'

    )


    # ============================================================
    # Velodyne PointCloud2
    #
    # Gazebo:
    #   /velodyne/points
    #
    # ROS2:
    #   /velodyne_points
    # ============================================================

    velodyne_bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        arguments=[
            '/velodyne/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],

        remappings=[
            (
                '/velodyne/points',
                '/velodyne_points'
            )
        ],

        parameters=[
            {
                'override_frame_id': 'velodyne'
            }
        ],

        output='screen'

    )


    # ============================================================
    # IMU
    #
    # Gazebo:
    #   /imu
    #
    # ROS2:
    #   /mpu6050/imu
    # ============================================================

    imu_bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],

        remappings=[
            (
                '/imu',
                '/mpu6050/imu'
            )
        ],

        output='screen'

    )


    # ============================================================
    # RGB CAMERA
    #
    # Gazebo:
    #   /camera/image_raw
    #   /camera/camera_info
    #
    # ROS2:
    #   /camera/image_raw
    #   /camera/camera_info
    # ============================================================

    camera_bridge = Node(

        package='ros_gz_bridge',

        executable='parameter_bridge',

        arguments=[

            '/camera/image_raw'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            '/camera/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',

        ],

        parameters=[
            {
                'override_frame_id':
                'camera_optical_frame'
            }
        ],

        output='screen'

    )


    # ============================================================
    # Lidar localization
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

                ' use_sim_time:=true',

                '; exec bash',
            ],

        ],

        output='screen',

    )


    # ============================================================
    # Obstacle detection
    # ============================================================

    obstacle_cloud_to_scan = ExecuteProcess(

        cmd=[

            'gnome-terminal',
            '--tab',
            '--title=obstacle_detection',
            '--',

            'bash',
            '-c',

            'ros2 launch '
            'obstacle_cloud_to_scan '
            'obstacle_cloud_to_scan.launch.py; '
            'exec bash',

        ],

        output='screen',

    )


    # ============================================================
    # Topic relay
    #
    # /diff_drive_base_controller/odom
    #                ↓
    # /odom
    #
    # /cmd_vel_slow
    #                ↓
    # /diff_drive_base_controller/cmd_vel_unstamped
    # ============================================================

    odom_relay = Node(

        package='topic_tools',

        executable='relay',

        name='odom_relay',

        arguments=[
            '/diff_drive_base_controller/odom',
            '/odom'
        ],

        output='screen'

    )


    cmd_vel_relay = Node(

        package='topic_tools',

        executable='relay',

        name='cmd_vel_relay',

        arguments=[
            '/cmd_vel_slow',
            '/diff_drive_base_controller/cmd_vel_unstamped'
        ],

        output='screen'

    )


    # ============================================================
    # Nav2
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

                'use_sim_time:=true '
                'params_file:=',

                nav2_params,

                ' log_level:=info; '
                'exec bash',
            ],

        ],

        output='screen',

    )


    # ============================================================
    # RViz
    # ============================================================

    rviz = ExecuteProcess(

        cmd=[

            'gnome-terminal',
            '--tab',
            '--title=rviz',
            '--',

            'bash',
            '-c',

            'ros2 launch '
            'nav2_bringup '
            'rviz_launch.py; '
            'exec bash',

        ],

        output='screen',

    )


    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription([

        set_gazebo_resource_path,

        gazebo,

        robot_state_publisher,

        map_server,

        obstacle_cloud_to_scan,

        lidar_localization,

        start_joint_state_broadcaster,

        start_diff_drive_controller,

        clock_bridge,

        velodyne_bridge,

        imu_bridge,

        camera_bridge,

        odom_relay,

        cmd_vel_relay,

        nav2,

        rviz,

        throttle,

        spawn_robot,

    ])