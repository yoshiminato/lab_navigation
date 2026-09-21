// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef ROS2_CONTROL_DIFF_DRIVE__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DIFF_DRIVE__DIFFBOT_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ros2_control_diff_drive
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)
  ~DiffBotSystemHardware() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  using Clock = std::chrono::steady_clock;
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_command();
  void stop_motion();
  void stop_executor();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  std::condition_variable state_cv_;
  std::array<double, 2> positions_{};
  std::array<double, 2> velocities_{};
  std::array<double, 2> position_offsets_{};
  std::array<double, 2> commands_{};
  Clock::time_point last_state_{};
  Clock::time_point last_command_{};
  bool received_state_{false};
  bool active_{false};
  bool fault_{false};
  double state_timeout_{0.5};
  double command_timeout_{0.5};
  double startup_timeout_{20.0};
  std::string command_topic_{"/mcu/wheel_commands"};
  std::string state_topic_{"/mcu/wheel_states"};
  std::vector<double> hw_commands_, hw_positions_, hw_velocities_;
};
}  // namespace ros2_control_diff_drive
#endif
