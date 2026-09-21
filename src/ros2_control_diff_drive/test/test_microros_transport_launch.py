"""Exercise the real hardware plugin with an in-process MCU topic simulator."""
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


@pytest.mark.rostest
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros2_control_diff_drive'),
            'launch', 'diffbot.launch.py')),
        launch_arguments={
            'gui': 'false', 'start_micro_ros_agent': 'false',
            'use_mock_hardware': 'false',
        }.items(),
    )
    return LaunchDescription([bringup, ReadyToTest()])


class TestTransport(unittest.TestCase):
    def test_feedback_commands_odometry_and_timeout(self):
        rclpy.init()
        node = rclpy.create_node('mcu_simulator')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        states = node.create_publisher(JointState, '/mcu/wheel_states', qos)
        cmd_vel = node.create_publisher(Twist, '/cmd_vel_slow', 1)
        commands, joints, odometry = [], [], []
        node.create_subscription(Float64MultiArray, '/mcu/wheel_commands',
                                 lambda msg: commands.append((time.monotonic(), list(msg.data))), qos)
        node.create_subscription(JointState, '/joint_states', joints.append, 10)
        node.create_subscription(Odometry, '/odom', odometry.append, 10)
        position = [2.0, 3.0]
        velocity = [0.0, 0.0]
        sending_states = True
        malformed = False
        moving = False
        last_tick = time.monotonic()

        def tick():
            nonlocal last_tick
            now = time.monotonic()
            dt = now - last_tick
            last_tick = now
            if commands:
                velocity[:] = commands[-1][1]
            for i in range(2):
                position[i] += velocity[i] * dt
            if sending_states:
                msg = JointState()
                # Deliberately reverse order: the plugin must match by joint name.
                msg.name = ['right_wheel_joint', 'left_wheel_joint']
                msg.position = list(reversed(position))
                msg.velocity = list(reversed(velocity))
                if malformed:
                    msg.position = [float('nan'), position[0]]
                states.publish(msg)
            twist = Twist()
            if moving:
                twist.linear.x = 0.135
                twist.angular.z = 0.1
            cmd_vel.publish(twist)

        timer = node.create_timer(0.05, tick)

        def wait_until(predicate, timeout):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return True
            return False

        try:
            self.assertTrue(wait_until(lambda: bool(odometry and joints and commands), 15),
                            'Real hardware plugin did not activate with simulated MCU feedback')
            self.assertAlmostEqual(odometry[-1].pose.pose.position.x, 0.0, delta=0.001)
            self.assertAlmostEqual(odometry[-1].pose.pose.orientation.z, 0.0, delta=0.001)
            moving = True
            self.assertTrue(wait_until(lambda: commands[-1][1][1] > commands[-1][1][0] > 0.2, 5),
                            'Differential wheel commands did not reach the MCU topic')
            start_x = odometry[-1].pose.pose.position.x
            self.assertTrue(wait_until(lambda: odometry[-1].pose.pose.position.x > start_x + 0.03, 5),
                            'Wheel feedback did not advance controller odometry')
            joint_msg = joints[-1]
            left = joint_msg.name.index('left_wheel_joint')
            right = joint_msg.name.index('right_wheel_joint')
            self.assertGreater(joint_msg.velocity[right], joint_msg.velocity[left])
            self.assertGreater(joint_msg.position[right], joint_msg.position[left])
            # Invalid messages must NOT refresh the state watchdog.
            malformed = True
            fault_start = time.monotonic()
            self.assertTrue(wait_until(
                lambda: any(t > fault_start + 0.4 and values == [0.0, 0.0]
                            for t, values in commands), 3),
                'Invalid feedback did not cause an explicit zero command')
            sending_states = False
            # Reconnection alone must not resume motion after the latched hardware fault.
            malformed = False
            sending_states = True
            after_fault = len(commands)
            wait_until(lambda: False, 0.8)
            self.assertTrue(all(values == [0.0, 0.0] for _, values in commands[after_fault:]))
        finally:
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
