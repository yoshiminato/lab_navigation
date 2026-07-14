from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    # PCDファイルのパスを指定するためのコマンドを定義
    declare_pcd_file_path_cmd = DeclareLaunchArgument(
        'pcd_file_path',
        default_value='/home/user/pcd/hallway.pcd'
    )

    # 地図のyamlファイルのパスを指定するためのコマンドを定義
    declare_map_yaml_file_path_cmd = DeclareLaunchArgument(
        'map_yaml_file_path',
        default_value='/home/user/map/hallway/hallway.yaml'
    )

    # Nav2パラメータファイルのパスを指定するためのコマンドを定義
    declare_nav2_params_file_path_cmd = DeclareLaunchArgument(
        'nav2_params_file_path',
        default_value='/home/user/lab_navigation_ws/src/lab_navigation/params/nav2_params.yaml'
    )

    # localizationパラメータファイルのパスを指定するためのコマンドを定義
    declare_localization_params_file_path_cmd = DeclareLaunchArgument(
        'localization_params_file_path',
        default_value='/home/user/lab_navigation_ws/src/lab_navigation/params/localization.yaml'
    )

    declare_camera_device_cmd = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0'
    )

    # # micro-ROSエージェントを起動するためのコマンドを定義
    # micro_ros = ExecuteProcess(
    #     cmd=[
    #         'gnome-terminal', '--tab', '--title=micro_ros', '--',
    #         'bash', '-c',
    #         'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6; exec bash'
    #     ],
    #     output='screen'
    # )

    # robot_state_publisherとjoint_state_publisherを起動するためのコマンドを定義
    ros2_control = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=ros2_control_ws', '--',
            'bash', '-c',
            'ros2 launch ros2_control_diff_drive diffbot.launch.py '
            'start_micro_ros_agent:=true micro_ros_device:=/dev/ttyUSB0 '
            'micro_ros_baudrate:=921600; exec bash'
        ],
        output='screen'
    )

    # Velodyneドライバを起動するためのコマンドを定義
    velodyne = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=velodyne', '--',
            'bash', '-c',
            'ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py; exec bash'
        ],
        output='screen'
    )

    # lidar_localizationを起動するためのコマンドを定義
    localization_params_file_path = LaunchConfiguration('localization_params_file_path')
    lidar_localization = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=lidar_localization', '--',
            'bash', '-c',
            [
                'ros2 launch lidar_localization_ros2 lidar_localization.launch.py localization_param_dir:=',
                localization_params_file_path,
                '; exec bash'
            ]
        ],
        output='screen'
    )

    # 障害物検出ノードを起動するためのコマンドを定義
    obstacle_detection = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=obstacle_detection', '--',
            'bash', '-c',
            'ros2 launch obstacle_cloud_to_scan obstacle_cloud_to_scan.launch.py; exec bash'
        ],
        output='screen'
    )

    # USBカメラを起動するためのコマンドを定義
    camera_device = LaunchConfiguration('camera_device')
    usb_camera = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=usb_camera', '--',
            'bash', '-c',
            [
                'ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=',
                camera_device,
                '; exec bash'
            ]
        ],
        output='screen'
    )

    # topic_tools throttle を起動
    throttle = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'topic_tools', 'throttle', 'messages',
            '/cmd_vel', '60.0', '/cmd_vel_slow'
        ],
        output='screen'
    )

    # map_serverを起動するためのコマンドを定義
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_yaml_file_path = LaunchConfiguration('map_yaml_file_path')
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file_path
        }.items()
    )

    # Nav2を起動するためのコマンドを定義
    nav2_params_file_path = LaunchConfiguration('nav2_params_file_path')
    params_file_path = LaunchConfiguration('nav2_params_file_path')
    nav2 = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=nav2', '--',
            'bash', '-c',
            [
                'ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false log_level:=info params_file:=',
                nav2_params_file_path,
                '; exec bash'
            ]
        ],
        output='screen'
    )

    # Rvizを起動するためのコマンドを定義
    rviz = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=rviz', '--',
            'bash', '-c',
            'ros2 launch nav2_bringup rviz_launch.py; exec bash'
        ],
        output='screen'
    )

    camera_velodyne_rviz  = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=rviz_camera_velodyne', '--',
            'bash', '-c',
            'ros2 launch lab_navigation sensors_visualize.launch.py; exec bash'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_pcd_file_path_cmd,
        declare_map_yaml_file_path_cmd,
        declare_nav2_params_file_path_cmd,
        declare_localization_params_file_path_cmd,
        declare_camera_device_cmd,
        # micro_ros,
        ros2_control,
        velodyne,
        lidar_localization,
        obstacle_detection,
        usb_camera,
        throttle,
        map_server,
        nav2,
        rviz,
        camera_velodyne_rviz
    ])
