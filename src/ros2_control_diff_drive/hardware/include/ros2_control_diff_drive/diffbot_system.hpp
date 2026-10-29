<<<<<<< HEAD
=======
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
>>>>>>> feature/hloc-nav2-integration
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
<<<<<<< HEAD
#include "std_msgs/msg/float32.hpp"
#include <libserial/SerialPort.h>

namespace ros2_control_diff_drive
{

static constexpr uint8_t HEADER1 = 0xAA;
static constexpr uint8_t HEADER2 = 0x55;

enum class ReceiveState {
  WAIT_FOR_HEADER1,
  WAIT_FOR_HEADER2,
  RECEIVE_DATA
};

#pragma pack(push, 1) // パケット構造を1バイト単位でパック

struct CommandPacket {
  uint8_t header1;
  uint8_t header2;
  float left_velocity_cmd = 0.0f;
  float right_velocity_cmd = 0.0f;
  uint8_t checksum = 0; // 簡単なエラーチェック用
} __attribute__((packed));

struct StatusPacket {
  uint8_t header1;
  uint8_t header2;
  float left_position;
  float left_velocity;
  float right_position;
  float right_velocity;
  float battery_voltage;
  uint8_t checksum = 0; // 簡単なエラーチェック用
} __attribute__((packed));

#pragma pack(pop)

=======

namespace ros2_control_diff_drive
{
>>>>>>> feature/hloc-nav2-integration
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
<<<<<<< HEAD

  // チェックサム計算ヘルパー
  uint8_t calculate_checksum(const uint8_t* data, size_t len);

  bool receive_packet(StatusPacket* packet);

  // サブスクライバーとパブリッシャー
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 受信状態変数
  ReceiveState receive_state_ = ReceiveState::WAIT_FOR_HEADER1;
  std::vector<uint8_t> rx_buffer_;
  StatusPacket rx_data_;

  // Humbleでの内部データ保持用配列（rad, rad/s）
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  LibSerial::SerialPort serial_port_;
  std::string device_name_ = "/dev/ttyUSB0"; 
=======
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
>>>>>>> feature/hloc-nav2-integration
};
}  // namespace ros2_control_diff_drive
#endif
