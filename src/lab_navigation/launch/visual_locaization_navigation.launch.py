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
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Launch arguments
    # ============================================================

    declare_pcd_file_path_cmd = DeclareLaunchArgument(
        'pcd_file_path',
        default_value=(
            '/home/user/lab_navigation_ws/'
            'src/lab_navigation/pcd/'
            'hv_building_lab.pcd'
        ),
    )

    declare_map_yaml_file_path_cmd = DeclareLaunchArgument(
        'map_yaml_file_path',
        default_value=(
            '/home/user/lab_navigation_ws/'
            'src/lab_navigation/map/'
            'hv_building_lab/'
            'hv_building_lab.yaml'
        ),
    )

    declare_nav2_params_file_path_cmd = DeclareLaunchArgument(
        'nav2_params_file_path',
        default_value=(
            '/home/user/lab_navigation_ws/'
            'src/lab_navigation/params/'
            'nav2_params.yaml'
        ),
    )

    declare_localization_params_file_path_cmd = DeclareLaunchArgument(
        'localization_params_file_path',
        default_value=(
            '/home/user/lab_navigation_ws/'
            'src/lab_navigation/params/'
            'localization.yaml'
        ),
    )

    # ============================================================
    # micro-ROS / ros2_control
    # ============================================================

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='921600',
    )

    start_agent_arg = DeclareLaunchArgument(
        'start_micro_ros_agent',
        default_value='true',
    )

    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lab_navigation'),
                'launch',
                'diffbot_custom.launch.py',
            ])
        ),
        launch_arguments={
            'serial_port':
                LaunchConfiguration('serial_port'),

            'serial_baudrate':
                LaunchConfiguration('serial_baudrate'),

            'start_micro_ros_agent':
                LaunchConfiguration('start_micro_ros_agent'),
        }.items(),
    )

    # ============================================================
    # Velodyne
    # ============================================================

    # カスタムVelodyne launcher
    # velodyne_laserscan_nodeの起動を除外したもの
    velodyne = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=velodyne',
            '--',
            'bash',
            '-c',
            (
                'ros2 launch lab_navigation '
                'velodyne_points.launch.py; '
                'exec bash'
            ),
        ],
        output='screen',
    )

    # ============================================================
    # LiDAR Localization
    # ============================================================

    # Visual Localizationへ切り替えるため無効化
    #
    # lidar_localization = ExecuteProcess(
    #     cmd=[
    #         'gnome-terminal',
    #         '--tab',
    #         '--title=lidar_localization',
    #         '--',
    #         'bash',
    #         '-c',
    #         [
    #             'ros2 launch '
    #             'lidar_localization_ros2_custom '
    #             'lidar_localization.launch.py ',
    #             'localization_param_dir:=',
    #             LaunchConfiguration(
    #                 'localization_params_file_path'
    #             ),
    #             '; exec bash',
    #         ],
    #     ],
    #     output='screen',
    # )

    # ============================================================
    # Camera
    # ============================================================
    
    # USBカメラ画像を /camera/image_raw に配信
    #
    # SfM側のカメラ解像度:
    #   1920 x 1080
    #
    # HLoc側のQoS:
    #   Reliability = RELIABLE
    #   Durability  = VOLATILE
    #   History     = KEEP_LAST
    #   Depth       = 1
    camera = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=camera',
            '--',
            'bash',
            '-c',
            (
                'ros2 run image_tools cam2image '
                '--ros-args '
                '-r image:=/camera/image_raw '
                '-p width:=1920 '
                '-p height:=1080 '
                '-p reliability:=reliable '
                '-p history:=keep_last '
                '-p depth:=1; '
                'exec bash'
            ),
        ],
        output='screen',
    )

    # ============================================================
    # HLoc Visual Localization
    # ============================================================

    hloc_localizer = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=hloc_localizer',
            '--',
            'bash',
            '-c',
            (
                'ros2 run hloc_localizer hloc_localizer '
                '--ros-args '
                '-p data_path:='
                '/home/user/lab_navigation_ws/'
                'captures/hv_building_lab/20261010 '
                '-p image_topic:=/camera/image_raw '
                '-p local_method:=superpoint_lightglue '
                '-p num_matched:=10 '
                '-p min_inliers:=12 '
                '-p ransac_thresh:=12.0 '
                '-p descriptor_cache_size:=40 '
                '-p map_frame:=map; '
                'exec bash'
            ),
        ],
        output='screen',
    )

    # ============================================================
    # HLoc TF Publisher
    # ============================================================

    # /hloc/camera_pose と
    # odom -> base_link, base_link -> camera
    # を組み合わせて map -> odom を配信
    hloc_tf_publisher = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=hloc_tf_publisher',
            '--',
            'bash',
            '-c',
            (
                'ros2 run hloc_localizer '
                'hloc_tf_publisher '
                '--ros-args '
                '-p pose_topic:=/hloc/camera_pose '
                '-p map_frame:=map '
                '-p odom_frame:=odom '
                '-p base_frame:=base_link '
                '-p camera_frame:=camera_optical_frame; '
                'exec bash'
            ),
        ],
        output='screen',
    )

    # ============================================================
    # Obstacle detection
    # ============================================================

    obstacle_detection = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--tab',
            '--title=obstacle_detection',
            '--',
            'bash',
            '-c',
            (
                'ros2 launch obstacle_cloud_to_scan '
                'obstacle_cloud_to_scan.launch.py; '
                'exec bash'
            ),
        ],
        output='screen',
    )

    # ============================================================
    # cmd_vel throttle
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
    # Map server / Nav2 localization launch
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
            'map':
                LaunchConfiguration(
                    'map_yaml_file_path'
                ),
        }.items(),
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
                'ros2 launch nav2_bringup '
                'navigation_launch.py '
                'use_sim_time:=false '
                'params_file:=',
                LaunchConfiguration(
                    'nav2_params_file_path'
                ),
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
            (
                'ros2 launch nav2_bringup '
                'rviz_launch.py; '
                'exec bash'
            ),
        ],
        output='screen',
    )

    # ============================================================
    # Launch
    # ============================================================

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

        # lidar_localization,

        camera,
        hloc_localizer,
        hloc_tf_publisher,

        obstacle_detection,
        throttle,

        map_server,
        nav2,
        rviz,
    ])