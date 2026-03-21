from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # joyノードを起動
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # teleop_twist_joyをパラメータファイル付きで起動
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=['/home/user/lab_navigation/src/lab_navigation/config/ps3.config.yaml']

    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
