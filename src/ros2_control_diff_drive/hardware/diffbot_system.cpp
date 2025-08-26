
// diffbot_system.cpp
// ROS2コントロール用の差動二輪ロボット（DiffBot）のハードウェアインターフェース実装
#include "ros2_control_diff_drive/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>  // std::memcpyを使用するために追加
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ros2_control_diff_drive
{

// ハードウェア初期化処理
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 親クラスの初期化を呼び出し、失敗したらエラーを返す
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 各ジョイントの状態・コマンド用ベクトルを初期化
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // ジョイントごとにインターフェースの数や型をチェック
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // コマンドインターフェースが1つであることを確認
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // コマンドインターフェースが速度であることを確認
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // ステートインターフェースが2つであることを確認
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 1つ目が位置、2つ目が速度であることを確認
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  rx_buffer_.resize(sizeof(StatusPacket));
  rx_data_ = {};

  // バッテリー残量用のノードとパブリッシャーを初期化
  node_ = rclcpp::Node::make_shared("diffbot_hw_node");
  battery_pub_ = node_->create_publisher<std_msgs::msg::Float32>("battery_level", 10);

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ステートインターフェース（位置・速度）をエクスポート
std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}


// コマンドインターフェース（速度コマンド）をエクスポート
std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}


// ハードウェアの設定（シリアルポートの初期化など）
hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    // シリアルポートを開き、通信設定を行う
    serial_port_.Open(device_name_);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Serial port %s could not be opened.", device_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 位置・速度・コマンド値を初期化
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ハードウェアのアクティベート処理
hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // コマンド値を現在の速度値で初期化
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_commands_[i] = hw_velocities_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ハードウェアのディアクティベート処理
hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ハードウェアから現在の状態（エンコーダ値など）を読み取る
hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try{
    if (receive_packet(&rx_data_)) {
      if (hw_positions_.size() >= 2) {
        hw_positions_[0] = rx_data_.left_position;
        hw_velocities_[0] = rx_data_.left_velocity;
        hw_positions_[1] = rx_data_.right_position;
        hw_velocities_[1] = rx_data_.right_velocity;
      }
      
      // バッテリー残量をパブリッシュ
      if (battery_pub_) {
        std_msgs::msg::Float32 battery_msg;
        battery_msg.data = rx_data_.battery_voltage;
        battery_pub_->publish(battery_msg);
      }
    }
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}


// コマンド値（左右車輪の速度）をハードウェアに送信
hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  CommandPacket tx_data;
  tx_data.header1 = HEADER1;
  tx_data.header2 = HEADER2;

  // コマンド値を構造体に格納
  if (hw_commands_.size() >= 2) {
    tx_data.left_velocity_cmd = hw_commands_[0];
    tx_data.right_velocity_cmd = hw_commands_[1];
  }

  size_t  len = sizeof(CommandPacket) - 1;
  uint8_t  *p = reinterpret_cast<uint8_t*>(&tx_data);
  uint8_t  cs = calculate_checksum(p, len);

  tx_data.checksum = cs;

  // シリアル通信で送信
  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&tx_data);
  std::vector<uint8_t> tx_buffer(ptr, ptr + sizeof(tx_data));
  
  try {
    serial_port_.Write(tx_buffer);
    serial_port_.DrainWriteBuffer();
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool DiffBotSystemHardware::receive_packet(StatusPacket* packet) {
  
  static size_t rx_index = 0;

  while(serial_port_.IsDataAvailable()) {
    uint8_t byte;
    
    serial_port_.ReadByte(byte);
  
    switch (receive_state_)
    {
      case ReceiveState::WAIT_FOR_HEADER1:
        if (byte == HEADER1) receive_state_ = ReceiveState::WAIT_FOR_HEADER2;
        break;
      
      case ReceiveState::WAIT_FOR_HEADER2:
        if (byte == HEADER2) {
          receive_state_ = ReceiveState::RECEIVE_DATA;
          rx_index = 0;
          rx_buffer_[rx_index++] = HEADER1;
          rx_buffer_[rx_index++] = HEADER2;
        } else {
          receive_state_ = ReceiveState::WAIT_FOR_HEADER1;
        }
        break;

      case ReceiveState::RECEIVE_DATA:
        rx_buffer_[rx_index++] = byte;
        if(rx_index < sizeof(StatusPacket)) break;
        
        size_t   len = sizeof(StatusPacket) - 1;
        uint8_t  cs  = calculate_checksum(rx_buffer_.data(), len);

        if (byte != cs) {
          receive_state_ = ReceiveState::WAIT_FOR_HEADER1;
          break;
        }

        std::memcpy(packet, rx_buffer_.data(), sizeof(StatusPacket));
        receive_state_ = ReceiveState::WAIT_FOR_HEADER1;
        return true;

        break;
      }
  }
  return false;
}

uint8_t DiffBotSystemHardware::calculate_checksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) {
    cs ^= data[i];
  }
  return cs;
}


// ジョイント状態のコールバック（未使用）
void DiffBotSystemHardware::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr /*msg*/)
{
  // 現状未実装
}


}  // namespace ros2_control_diff_drive


// プラグインとしてエクスポート
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_diff_drive::DiffBotSystemHardware, hardware_interface::SystemInterface)
