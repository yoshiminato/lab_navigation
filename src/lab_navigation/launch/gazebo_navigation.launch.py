from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    # Nav2パラメータファイルのパス（Gazebo用に変更）
    declare_nav2_params_file_path_cmd = DeclareLaunchArgument(
        'nav2_params_file_path',
        default_value='/home/user/lab_navigation_ws/src/lab_navigation/params/nav2_gazebo_params.yaml'
    )

    # localizationパラメータファイルのパス（Gazebo用に変更）
    declare_localization_params_file_path_cmd = DeclareLaunchArgument(
        'localization_params_file_path',
        default_value='/home/user/lab_navigation_ws/src/lab_navigation/params/localization_gazebo.yaml'
    )

    # Gazeboを起動するためのコマンドを定義
    gazebo = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=gazebo', '--',
            'bash', '-c',
            'ros2 launch gazebo_ros gazebo.launch.py; exec bash'
        ],
        output='screen'
    )

    # URDFをGazeboへSpawnさせる（robot_state_publisherが出している /robot_description を受け取る想定）
    spawn_entity = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=spawn_entity', '--',
            'bash', '-c',
            'ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity diffbot; exec bash'
        ],
        output='screen'
    )

    # robot_state_publisherとjoint_state_publisherを起動
    # ※ Gazebo連携が必要な場合、URDF(xacro)内に<gazebo>タグとgazebo_ros2_controlプラグインの記述が必要です。
    ros2_control = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=ros2_control_ws', '--',
            'bash', '-c',
            'ros2 launch ros2_control_diff_drive diffbot.launch.py use_sim_time:=true; exec bash'
        ],
        output='screen'
    )

    # ※Velodyneドライバ(実機用)はGazeboシミュレーションのため除外

    # lidar_localizationを起動
    localization_params_file_path = LaunchConfiguration('localization_params_file_path')
    lidar_localization = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=lidar_localization', '--',
            'bash', '-c',
            [
                'ros2 launch lidar_localization_ros2 lidar_localization.launch.py localization_param_dir:=',
                localization_params_file_path,
                ' use_sim_time:=true',
                '; exec bash'
            ]
        ],
        output='screen'
    )

    # 障害物検出ノードを起動
    obstacle_detection = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=obstacle_detection', '--',
            'bash', '-c',
            'ros2 launch obstacle_cloud_to_scan obstacle_cloud_to_scan.launch.py use_sim_time:=true; exec bash'
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

    # map_serverを起動（use_sim_time:=trueに設定）
    map_yaml_file_path = LaunchConfiguration('map_yaml_file_path')
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/user/navigation_ws/install/nav2_bringup/share/nav2_bringup/launch/localization_launch.py'
        ),
        launch_arguments={
            'map': map_yaml_file_path,
            'use_sim_time': 'true'
        }.items()
    )

    # Nav2を起動（use_sim_time:=trueに変更）
    nav2_params_file_path = LaunchConfiguration('nav2_params_file_path')
    nav2 = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=nav2', '--',
            'bash', '-c',
            [
                'ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=',
                nav2_params_file_path,
                ' log_level:=info',
                '; exec bash'
            ]
        ],
        output='screen'
    )

    # Rvizを起動
    rviz = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=rviz', '--',
            'bash', '-c',
            'ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true; exec bash'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_pcd_file_path_cmd,
        declare_map_yaml_file_path_cmd,
        declare_nav2_params_file_path_cmd,
        declare_localization_params_file_path_cmd,
        gazebo,
        ros2_control,
        spawn_entity,
        lidar_localization,
        obstacle_detection,
        throttle,
        map_server,
        nav2,
        rviz,
    ])
