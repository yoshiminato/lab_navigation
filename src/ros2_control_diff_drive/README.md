# DiffBot: ros2_control + micro-ROS

This package controls the differential-drive base with the existing
`topic_based_ros2_control/TopicBasedSystem` hardware plugin. The former custom
`DiffBotSystemHardware` and its binary `LibSerial` protocol are no longer part
of the package.

## Data flow and TF ownership

```text
/cmd_vel_slow
  -> diffbot_base_controller (differential-drive kinematics)
  -> left/right velocity command interfaces
  -> TopicBasedSystem
  -> /diffbot/mcu_joint_commands (sensor_msgs/JointState)
  -> micro-ROS Agent -> ESP32 motor PID

ESP32 encoders
  -> /diffbot/mcu_joint_states (sensor_msgs/JointState)
  -> TopicBasedSystem state interfaces
  -> diffbot_base_controller -> /odom and odom -> base_link TF
  -> joint_state_broadcaster -> /joint_states
  -> robot_state_publisher -> wheel/link TF
```

The removed C++ hardware interface did not own TF publication. Wheel command
conversion and odometry are handled by `diff_drive_controller`; joint TF is
handled by `joint_state_broadcaster` together with `robot_state_publisher`.
Localization or Nav2 remains responsible for `map -> odom`.

## Dependencies

For ROS 2 Humble, install the released topic-based plugin:

```bash
sudo apt install ros-humble-topic-based-ros2-control
```

The micro-ROS Agent must also be built/installed and sourced. On this machine it
is available in `/home/user/micro_ros_ws`.

Use a Humble-compatible `micro_ros_arduino` library for the ESP32. A library
built for a different ROS distribution is not guaranteed to interoperate with
the Humble Agent. The sketch targets Arduino-ESP32 core 2.0.9.

## Build and run

```bash
cd /home/user/lab_navigation_ws
source /opt/ros/humble/setup.bash
source /home/user/micro_ros_ws/install/setup.bash
colcon build --packages-select ros2_control_diff_drive
source install/setup.bash

ros2 launch ros2_control_diff_drive diffbot.launch.py \
  start_micro_ros_agent:=true \
  micro_ros_device:=/dev/ttyUSB0 \
  micro_ros_baudrate:=921600
```

The Agent is disabled by default so the same launch file can be used for mock
tests. Start it manually, if preferred, with:

```bash
ros2 run micro_ros_agent micro_ros_agent \
  serial --dev /dev/ttyUSB0 -b 921600
```

Mock hardware can be launched without an ESP32 or Agent:

```bash
ros2 launch ros2_control_diff_drive diffbot.launch.py \
  use_mock_hardware:=true
```

## ESP32 firmware

Open `src/lab_navigation/arduino/diffbot_micro_ros` as an Arduino sketch. It:

- subscribes to `/diffbot/mcu_joint_commands`;
- publishes `/diffbot/mcu_joint_states` at 50 Hz;
- publishes `/imu/data_raw` at 50 Hz;
- publishes `/battery_state` at 2 Hz;
- runs wheel PID at 100 Hz and stops both motors after a 300 ms command timeout.

Joint positions use radians and velocities use radians per second. The joint
names must remain `left_wheel_joint` and `right_wheel_joint` on both sides.
Before enabling the motors, verify the direction pins and confirm whether the
encoder's `1060` pulses/revolution value must be multiplied by four.
