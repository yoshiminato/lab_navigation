#ifndef ROS2_CONTROL_DIFF_DRIVE__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DIFF_DRIVE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
  uint8_t checksum = 0; // 簡単なエラーチェック用
} __attribute__((packed));

#pragma pack(pop)

class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // コールバック関数
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // チェックサム計算ヘルパー
  uint8_t calculate_checksum(const uint8_t* data, size_t len);

  bool receive_packet(StatusPacket* packet);

  // サブスクライバーとパブリッシャー
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
};

}  // namespace ros2_control_diff_drive

#endif  // ROS2_CONTROL_DIFF_DRIVE__DIFFBOT_SYSTEM_HPP_