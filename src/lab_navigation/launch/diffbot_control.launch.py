# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ============================================================
    # Launch arguments
    # ============================================================

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time from /clock.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_micro_ros_agent",
            default_value="true",
            description="Start micro-ROS agent.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port used by micro-ROS agent.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_baudrate",
            default_value="921600",
            description="Serial baudrate used by micro-ROS agent.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xacro",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("lab_navigation"),
                    "urdf",
                    "diffbot_real.urdf.xacro",
                ]
            ),
            description="Robot Xacro used by ros2_control and robot_state_publisher.",
        )
    )

    # ============================================================
    # Initialize launch configurations
    # ============================================================

    gui = LaunchConfiguration("gui")

    use_mock_hardware = LaunchConfiguration(
        "use_mock_hardware"
    )

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    start_micro_ros_agent = LaunchConfiguration(
        "start_micro_ros_agent"
    )

    serial_port = LaunchConfiguration(
        "serial_port"
    )

    serial_baudrate = LaunchConfiguration(
        "serial_baudrate"
    )

    robot_xacro = LaunchConfiguration(
        "robot_xacro"
    )

    # ============================================================
    # micro-ROS Agent
    #
    # 実機:
    #   start_micro_ros_agent:=true
    #
    # Isaac Sim:
    #   start_micro_ros_agent:=false
    #
    # mock hardware:
    #   Agent不要
    # ============================================================

    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=[
            "serial",
            "--dev",
            serial_port,
            "-b",
            serial_baudrate,
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    start_micro_ros_agent,
                    "'.lower() == 'true' and '",
                    use_mock_hardware,
                    "'.lower() != 'true'",
                ]
            )
        ),
    )

    # ============================================================
    # Robot Description
    # ============================================================

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_xacro,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # ============================================================
    # ros2_control controller config
    # ============================================================

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(
                "ros2_control_diff_drive"
            ),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    # ============================================================
    # RViz config
    # ============================================================

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "ros2_control_diff_drive"
            ),
            "rviz",
            "diffbot.rviz",
        ]
    )

    # ============================================================
    # ros2_control_node
    #
    # Isaac Simでは:
    #   use_sim_time:=true
    #
    # 実機では:
    #   use_sim_time:=false
    # ============================================================

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
            {
                "use_sim_time": use_sim_time
            },
        ],
        output="both",
        remappings=[
            (
                "~/robot_description",
                "/robot_description",
            ),
            (
                "/diffbot_base_controller/cmd_vel_unstamped",
                "/cmd_vel_slow",
            ),
            (
                "/diffbot_base_controller/odom",
                "/odom",
            ),
        ],
    )

    # ============================================================
    # robot_state_publisher
    # ============================================================

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    # ============================================================
    # RViz2
    # ============================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            rviz_config_file,
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time
            },
        ],
        condition=IfCondition(gui),
    )

    # ============================================================
    # joint_state_broadcaster
    # ============================================================

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--service-call-timeout",
            "30",
        ],
    )

    # ============================================================
    # diff_drive_controller
    # ============================================================

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--service-call-timeout",
            "30",
        ],
    )

    # ============================================================
    # joint_state_broadcaster 起動後にRVizを起動
    # ============================================================

    delay_rviz_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    rviz_node
                ],
            )
        )
    )

    # ============================================================
    # joint_state_broadcaster 起動後に
    # diff_drive_controllerを起動
    # ============================================================

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    robot_controller_spawner
                ],
            )
        )
    )

    # ============================================================
    # Nodes
    # ============================================================

    nodes = [
        micro_ros_agent,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(
        declared_arguments + nodes
    )
