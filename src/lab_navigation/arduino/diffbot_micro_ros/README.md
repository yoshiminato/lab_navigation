# DiffBot micro-ROS firmware

This sketch replaces the old `0xAA 0x55` binary serial protocol with micro-ROS.
Open the `diffbot_micro_ros` directory as an Arduino sketch and write it to the ESP32.

## ROS topics

- Subscribe `/diffbot/mcu_joint_commands` (`sensor_msgs/msg/JointState`)
- Publish `/diffbot/mcu_joint_states` (`sensor_msgs/msg/JointState`)
- Publish `/imu/data_raw` (`sensor_msgs/msg/Imu`)
- Publish `/battery_state` (`sensor_msgs/msg/BatteryState`)

The command/state joint names are exactly `left_wheel_joint` and
`right_wheel_joint`; positions are radians and velocities are radians/second.
`/joint_states` and `/odom` are intentionally not published by the ESP32 because
the ROS 2 controllers publish them.

## Required versions and startup

Use a Humble-compatible `micro_ros_arduino` library with the Humble Agent. The
currently installed Arduino library should be checked because a library built
for another ROS distribution is not guaranteed to interoperate.

This sketch currently targets Arduino-ESP32 core 2.0.9. Core 3.x changed the
LEDC API, so keep 2.0.9 unless the PWM setup code is updated as well.

The sketch uses a custom 921600-baud transport. Start the Agent with the same
rate, either via `diffbot.launch.py start_micro_ros_agent:=true` or manually:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```

Do not use `Serial.print()` in this sketch; `Serial` carries XRCE-DDS frames.

## Hardware checks

- The current tracked firmware pins are used: left direction GPIO 18, right
  direction GPIO 5. The older pasted conversation shows these two pins in the
  opposite order, so verify the actual wiring before enabling the motors.
- `TICKS_PER_REV=1060` is treated as a one-channel pulse count and multiplied
  by four for quadrature decoding. Change `QUADRATURE_COUNTS_PER_REV` if 1060 is
  already the four-edge count.
- MPU-6050 is expected at I2C address `0x68`, SDA 21, SCL 22. The current
  firmware assumes the sensor is mounted with `+Y` forward and `+X` right, and
  converts that into ROS `base_link` convention: `+X` forward, `+Y` left,
  `+Z` up.
- PID gains and `MAX_PWM=50` intentionally retain the current robot settings.

For safety, motor PID is scheduled at 100 Hz without being triggered by message
arrival and stops the motors when commands are absent for 300 ms or the Agent
disconnects. It still shares the Arduino `loop()` with ROS and I2C work, so it
is not a hard real-time task; I2C blocking is limited to 10 ms.
