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
#include "ros2_control_diff_drive/diffbot_system.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ros2_control_diff_drive
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

DiffBotSystemHardware::~DiffBotSystemHardware() {stop_executor();}

CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Exactly two wheel joints required");
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
      joint.state_interfaces.size() != 2 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      return CallbackReturn::ERROR;
    }
  }
  try {
    auto number = [this](const std::string & key, double & value) {
        const auto it = info_.hardware_parameters.find(key);
        if (it != info_.hardware_parameters.end()) {value = std::stod(it->second);}
        if (!std::isfinite(value) || value <= 0.0) {throw std::invalid_argument(key);}
      };
    number("state_timeout_sec", state_timeout_);
    number("command_timeout_sec", command_timeout_);
    number("startup_timeout_sec", startup_timeout_);
    for (auto item : {std::make_pair("command_topic", &command_topic_),
        std::make_pair("state_topic", &state_topic_)})
    {
      const auto it = info_.hardware_parameters.find(item.first);
      if (it != info_.hardware_parameters.end()) {*item.second = it->second;}
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Invalid parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  hw_positions_.assign(2, 0.0);
  hw_velocities_.assign(2, 0.0);
  hw_commands_.assign(2, 0.0);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (size_t i = 0; i < 2; ++i) {
    result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (size_t i = 0; i < 2; ++i) {
    result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return result;
}

CallbackReturn DiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  stop_executor();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_state_ = active_ = fault_ = false;
    commands_.fill(0.0);
  }
  try {
    // Ignore controller_manager's node-name remappings for this private communication node.
    node_ = std::make_shared<rclcpp::Node>(
      "diffbot_hw_node", rclcpp::NodeOptions().use_global_arguments(false));
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    velocity_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, qos);
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, qos, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_callback(msg);
      });
    command_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50), [this]() {publish_command();});
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    running_ = true;
    executor_thread_ = std::thread([this]() {
        try {
          while (running_ && rclcpp::ok()) {
            executor_->spin_some(std::chrono::milliseconds(5));
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node_->get_logger(), "Communication executor failed: %s", e.what());
          std::lock_guard<std::mutex> lock(mutex_);
          fault_ = true;
          state_cv_.notify_all();
        }
      });
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "%s", e.what());
    stop_executor();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const bool ready = state_cv_.wait_for(lock, std::chrono::duration<double>(startup_timeout_),
    [this]() {
      return fault_ || (received_state_ &&
        std::chrono::duration<double>(Clock::now() - last_state_).count() < state_timeout_);
    });
  if (!ready || fault_) {
    RCLCPP_ERROR(node_->get_logger(), "No fresh MCU wheel state; check Agent, firmware and ROS_DOMAIN_ID");
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < 2; ++i) {
    position_offsets_[i] = positions_[i];
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = velocities_[i];
    hw_commands_[i] = 0.0;
  }
  commands_.fill(0.0);
  last_command_ = Clock::now();
  active_ = true;
  return CallbackReturn::SUCCESS;
}

void DiffBotSystemHardware::stop_motion()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    commands_.fill(0.0);
  }
  // The MCU watchdog is the fallback if this last zero command cannot be delivered.
  if (velocity_pub_ && rclcpp::ok()) {
    try {publish_command();} catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Unable to publish final stop: %s", e.what());
    }
  }
}

void DiffBotSystemHardware::stop_executor()
{
  stop_motion();
  running_ = false;
  if (executor_) {executor_->cancel();}
  if (executor_thread_.joinable()) {executor_thread_.join();}
  command_timer_.reset();
  joint_state_sub_.reset();
  velocity_pub_.reset();
  executor_.reset();
  node_.reset();
}

CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{stop_motion(); return CallbackReturn::SUCCESS;}
CallbackReturn DiffBotSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{stop_executor(); return CallbackReturn::SUCCESS;}
CallbackReturn DiffBotSystemHardware::on_shutdown(const rclcpp_lifecycle::State &)
{stop_executor(); return CallbackReturn::SUCCESS;}
CallbackReturn DiffBotSystemHardware::on_error(const rclcpp_lifecycle::State &)
{stop_executor(); return CallbackReturn::SUCCESS;}

void DiffBotSystemHardware::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::array<double, 2> positions, velocities;
  if (msg->name.size() != 2 || msg->position.size() != 2 || msg->velocity.size() != 2) {return;}
  for (size_t i = 0; i < 2; ++i) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), info_.joints[i].name);
    if (it == msg->name.end()) {return;}
    const size_t index = std::distance(msg->name.begin(), it);
    positions[i] = msg->position[index];
    velocities[i] = msg->velocity[index];
    if (!std::isfinite(positions[i]) || !std::isfinite(velocities[i])) {return;}
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Clock::now();
  if (active_ && received_state_) {
    const double dt = std::chrono::duration<double>(now - last_state_).count();
    // A reconnect or encoder reset must not silently jump the controller's odometry.
    if (dt > state_timeout_ ||
      std::abs(positions[0] - positions_[0]) > 20.0 * dt + 0.1 ||
      std::abs(positions[1] - positions_[1]) > 20.0 * dt + 0.1)
    {
      fault_ = true;
    }
  }
  positions_ = positions;
  velocities_ = velocities;
  last_state_ = now;
  received_state_ = true;
  state_cv_.notify_all();
}

void DiffBotSystemHardware::publish_command()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(2, 0.0);
  // Serialize timer and lifecycle publication so a stop cannot be followed by an old command.
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Clock::now();
  if (active_ && (
      !received_state_ || std::chrono::duration<double>(now - last_state_).count() > state_timeout_ ||
      std::chrono::duration<double>(now - last_command_).count() > command_timeout_))
  {
    if (!fault_) {RCLCPP_ERROR(node_->get_logger(), "MCU state/control update timed out; stopping");}
    fault_ = true;
  }
  if (active_ && !fault_) {msg.data.assign(commands_.begin(), commands_.end());}
  velocity_pub_->publish(msg);
}

return_type DiffBotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_ && (!received_state_ ||
      std::chrono::duration<double>(Clock::now() - last_state_).count() > state_timeout_))
  {fault_ = true;}
  if (fault_) {return return_type::ERROR;}
  if (received_state_) {
    for (size_t i = 0; i < 2; ++i) {
      hw_positions_[i] = positions_[i] - position_offsets_[i];
      hw_velocities_[i] = velocities_[i];
    }
  }
  return return_type::OK;
}

return_type DiffBotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const double command : hw_commands_) {
    if (!std::isfinite(command)) {fault_ = true;}
  }
  if (fault_) {commands_.fill(0.0); return return_type::ERROR;}
  if (active_) {
    std::copy(hw_commands_.begin(), hw_commands_.end(), commands_.begin());
    last_command_ = Clock::now();
  }
  return return_type::OK;
}
}  // namespace ros2_control_diff_drive
PLUGINLIB_EXPORT_CLASS(
  ros2_control_diff_drive::DiffBotSystemHardware, hardware_interface::SystemInterface)
