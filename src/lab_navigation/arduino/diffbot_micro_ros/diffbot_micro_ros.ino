#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <micro_ros_arduino.h>

#include <micro_ros_utilities/type_utilities.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <rmw_microros/timing.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>

// ESP32 pin assignment from the current tracked motor firmware.
constexpr uint8_t BATTERY_PIN = 34;
constexpr uint8_t LEFT_ENCODER_A_PIN = 13;
constexpr uint8_t LEFT_ENCODER_B_PIN = 14;
constexpr uint8_t RIGHT_ENCODER_A_PIN = 27;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 26;
constexpr uint8_t LEFT_PWM_PIN = 16;
constexpr uint8_t LEFT_DIRECTION_PIN = 5;
constexpr uint8_t RIGHT_PWM_PIN = 17;
constexpr uint8_t RIGHT_DIRECTION_PIN = 18;

constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr int MIN_PWM = 5;
constexpr int MAX_PWM = 50;

// TICKS_PER_REV is the single-channel pulse count. Both phases are sampled on
// CHANGE, so the actual quadrature count is four times this value.
constexpr float TICKS_PER_REV = 1060.0f;
constexpr float QUADRATURE_COUNTS_PER_REV = TICKS_PER_REV * 4.0f;
constexpr float TWO_PI_F = 6.28318530717958647692f;

constexpr float KP = 25.0f;
constexpr float KI = 1.0f;
constexpr float KD = 0.0f;
constexpr float COMMAND_ZERO_EPSILON = 0.001f;
constexpr float MAX_WHEEL_COMMAND_RAD_S = 8.0f;

constexpr uint32_t CONTROL_PERIOD_US = 10000;       // 100 Hz
constexpr uint32_t JOINT_STATE_PERIOD_MS = 20;      // 50 Hz
constexpr uint32_t IMU_PERIOD_MS = 20;              // 50 Hz
constexpr uint32_t BATTERY_PERIOD_MS = 500;         // 2 Hz
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;
constexpr uint32_t AGENT_SEARCH_PERIOD_MS = 500;
constexpr uint32_t AGENT_CHECK_PERIOD_MS = 1000;

constexpr uint32_t MICRO_ROS_BAUDRATE = 921600;
constexpr size_t SERIAL_BUFFER_SIZE = 4096;

constexpr char LEFT_JOINT_NAME[] = "left_wheel_joint";
constexpr char RIGHT_JOINT_NAME[] = "right_wheel_joint";
constexpr char JOINT_COMMAND_TOPIC[] = "/diffbot/mcu_joint_commands";
constexpr char JOINT_STATE_TOPIC[] = "/diffbot/mcu_joint_states";
constexpr char IMU_TOPIC[] = "/imu/data_raw";
constexpr char BATTERY_TOPIC[] = "/battery_state";
constexpr char IMU_FRAME_ID[] = "base_link";
constexpr char BATTERY_FRAME_ID[] = "base_link";

constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr float GRAVITY = 9.80665f;
constexpr float DEG_TO_RAD_F = 0.01745329251994329577f;
constexpr float ACCEL_BIAS_X = -0.0616f;
constexpr float ACCEL_BIAS_Y = -0.0814f;
constexpr float ACCEL_BIAS_Z = 0.1918f;
constexpr float GYRO_BIAS_X = -1.30f;
constexpr float GYRO_BIAS_Y = -1.90f;
constexpr float GYRO_BIAS_Z = -0.75f;

constexpr float R1_OHM = 99000.0f;
constexpr float R2_OHM = 10000.0f;
constexpr float VOLTAGE_DIVIDER_RATIO = (R1_OHM + R2_OHM) / R2_OHM;

volatile int32_t left_encoder_count = 0;
volatile int32_t right_encoder_count = 0;
volatile uint8_t left_last_ab = 0;
volatile uint8_t right_last_ab = 0;

float target_left_velocity = 0.0f;
float target_right_velocity = 0.0f;
float measured_left_position = 0.0f;
float measured_right_position = 0.0f;
float measured_left_velocity = 0.0f;
float measured_right_velocity = 0.0f;
bool command_received = false;
bool imu_available = false;
uint32_t last_command_ms = 0;
uint32_t last_control_us = 0;
uint32_t last_joint_state_ms = 0;
uint32_t last_imu_ms = 0;
uint32_t last_battery_ms = 0;
uint32_t last_agent_action_ms = 0;

struct PidState
{
  float integral = 0.0f;
  float previous_error = 0.0f;
};

PidState left_pid;
PidState right_pid;

rcl_allocator_t allocator;
rclc_support_t support = {};
rcl_node_t node = rcl_get_zero_initialized_node();
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
rcl_publisher_t joint_state_publisher = rcl_get_zero_initialized_publisher();
rcl_publisher_t imu_publisher = rcl_get_zero_initialized_publisher();
rcl_publisher_t battery_publisher = rcl_get_zero_initialized_publisher();
rcl_subscription_t joint_command_subscription = rcl_get_zero_initialized_subscription();

sensor_msgs__msg__JointState joint_state_message = {};
sensor_msgs__msg__JointState joint_command_message = {};
sensor_msgs__msg__Imu imu_message = {};
sensor_msgs__msg__BatteryState battery_message = {};

micro_ros_utilities_memory_conf_t joint_message_memory_conf = {};
micro_ros_utilities_memory_conf_t sensor_message_memory_conf = {};

bool support_initialized = false;
bool node_initialized = false;
bool executor_initialized = false;
bool command_subscription_initialized = false;
bool joint_publisher_initialized = false;
bool imu_publisher_initialized = false;
bool battery_publisher_initialized = false;

enum class AgentState
{
  WAITING,
  AVAILABLE,
  CONNECTED,
  DISCONNECTED
};

AgentState agent_state = AgentState::WAITING;

extern "C" bool high_speed_transport_open(uxrCustomTransport * /*transport*/)
{
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial.begin(MICRO_ROS_BAUDRATE);
  return true;
}

extern "C" bool high_speed_transport_close(uxrCustomTransport * /*transport*/)
{
  Serial.flush();
  Serial.end();
  return true;
}

extern "C" size_t high_speed_transport_write(
  uxrCustomTransport * /*transport*/, const uint8_t * buffer, size_t length,
  uint8_t * error_code)
{
  if (error_code != nullptr) {
    *error_code = 0;
  }
  return Serial.write(buffer, length);
}

extern "C" size_t high_speed_transport_read(
  uxrCustomTransport * /*transport*/, uint8_t * buffer, size_t length, int timeout,
  uint8_t * error_code)
{
  if (error_code != nullptr) {
    *error_code = 0;
  }
  Serial.setTimeout(timeout > 0 ? static_cast<unsigned long>(timeout) : 1UL);
  return Serial.readBytes(reinterpret_cast<char *>(buffer), length);
}

void IRAM_ATTR left_encoder_isr()
{
  const uint8_t current_ab =
    (digitalRead(LEFT_ENCODER_A_PIN) << 1) | digitalRead(LEFT_ENCODER_B_PIN);
  int8_t delta = 0;
  switch ((left_last_ab << 2) | current_ab) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: delta = 1; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: delta = -1; break;
    default: break;
  }
  left_encoder_count += delta;
  left_last_ab = current_ab;
}

void IRAM_ATTR right_encoder_isr()
{
  const uint8_t current_ab =
    (digitalRead(RIGHT_ENCODER_A_PIN) << 1) | digitalRead(RIGHT_ENCODER_B_PIN);
  int8_t delta = 0;
  switch ((right_last_ab << 2) | current_ab) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: delta = 1; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: delta = -1; break;
    default: break;
  }
  right_encoder_count += delta;
  right_last_ab = current_ab;
}

void set_motors(int left_pwm, int right_pwm)
{
  const int left_magnitude = abs(left_pwm) > MAX_PWM ? MAX_PWM : abs(left_pwm);
  const int right_magnitude = abs(right_pwm) > MAX_PWM ? MAX_PWM : abs(right_pwm);

  if (left_magnitude < MIN_PWM) {
    ledcWrite(LEFT_PWM_CHANNEL, 0);
  } else {
    digitalWrite(LEFT_DIRECTION_PIN, left_pwm > 0 ? HIGH : LOW);
    ledcWrite(LEFT_PWM_CHANNEL, left_magnitude);
  }

  if (right_magnitude < MIN_PWM) {
    ledcWrite(RIGHT_PWM_CHANNEL, 0);
  } else {
    digitalWrite(RIGHT_DIRECTION_PIN, right_pwm > 0 ? HIGH : LOW);
    ledcWrite(RIGHT_PWM_CHANNEL, right_magnitude);
  }
}

void reset_pid(PidState & pid)
{
  pid.integral = 0.0f;
  pid.previous_error = 0.0f;
}

void stop_motors_and_clear_command()
{
  target_left_velocity = 0.0f;
  target_right_velocity = 0.0f;
  command_received = false;
  reset_pid(left_pid);
  reset_pid(right_pid);
  set_motors(0, 0);
}

int calculate_pid_pwm(float target, float measured, float dt, PidState & pid)
{
  if (fabsf(target) < COMMAND_ZERO_EPSILON) {
    reset_pid(pid);
    return 0;
  }

  const float error = target - measured;
  pid.integral += error * dt;
  if (KI > 0.0f) {
    const float integral_limit = static_cast<float>(MAX_PWM) / KI;
    pid.integral = fmaxf(-integral_limit, fminf(integral_limit, pid.integral));
  }
  const float derivative = dt > 0.0f ? (error - pid.previous_error) / dt : 0.0f;
  pid.previous_error = error;

  const float output = KP * error + KI * pid.integral + KD * derivative;
  return static_cast<int>(lroundf(fmaxf(-MAX_PWM, fminf(MAX_PWM, output))));
}

void update_motor_control()
{
  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - last_control_us;
  if (elapsed_us < CONTROL_PERIOD_US) {
    return;
  }
  last_control_us = now_us;
  const float dt = elapsed_us * 1.0e-6f;

  noInterrupts();
  const int32_t left_count = left_encoder_count;
  const int32_t right_count = right_encoder_count;
  interrupts();

  static int32_t previous_left_count = left_count;
  static int32_t previous_right_count = right_count;
  const int32_t left_delta = left_count - previous_left_count;
  const int32_t right_delta = right_count - previous_right_count;
  previous_left_count = left_count;
  previous_right_count = right_count;

  measured_left_position = TWO_PI_F * left_count / QUADRATURE_COUNTS_PER_REV;
  measured_right_position = TWO_PI_F * right_count / QUADRATURE_COUNTS_PER_REV;
  measured_left_velocity = TWO_PI_F * left_delta / QUADRATURE_COUNTS_PER_REV / dt;
  measured_right_velocity = TWO_PI_F * right_delta / QUADRATURE_COUNTS_PER_REV / dt;

  const bool command_timed_out =
    !command_received || (millis() - last_command_ms > COMMAND_TIMEOUT_MS);
  if (command_timed_out || agent_state != AgentState::CONNECTED) {
    stop_motors_and_clear_command();
    return;
  }

  const int left_pwm =
    calculate_pid_pwm(target_left_velocity, measured_left_velocity, dt, left_pid);
  const int right_pwm =
    calculate_pid_pwm(target_right_velocity, measured_right_velocity, dt, right_pid);
  set_motors(left_pwm, right_pwm);
}

void joint_command_callback(const void * message_input)
{
  const auto * message =
    static_cast<const sensor_msgs__msg__JointState *>(message_input);
  if (message == nullptr || message->name.size != message->velocity.size) {
    return;
  }

  bool found_left = false;
  bool found_right = false;
  float new_left = 0.0f;
  float new_right = 0.0f;

  for (size_t index = 0; index < message->name.size; ++index) {
    const char * name = message->name.data[index].data;
    const float velocity = static_cast<float>(message->velocity.data[index]);
    if (name == nullptr || !isfinite(velocity)) {
      return;
    }
    if (strcmp(name, LEFT_JOINT_NAME) == 0) {
      new_left = velocity;
      found_left = true;
    } else if (strcmp(name, RIGHT_JOINT_NAME) == 0) {
      new_right = velocity;
      found_right = true;
    }
  }

  if (!found_left || !found_right) {
    return;
  }

  target_left_velocity =
    fmaxf(-MAX_WHEEL_COMMAND_RAD_S, fminf(MAX_WHEEL_COMMAND_RAD_S, new_left));
  target_right_velocity =
    fmaxf(-MAX_WHEEL_COMMAND_RAD_S, fminf(MAX_WHEEL_COMMAND_RAD_S, new_right));
  last_command_ms = millis();
  command_received = true;
}

bool initialize_message_memory()
{
  joint_message_memory_conf.max_string_capacity = 24;
  joint_message_memory_conf.max_ros2_type_sequence_capacity = 2;
  joint_message_memory_conf.max_basic_type_sequence_capacity = 2;

  sensor_message_memory_conf.max_string_capacity = 24;
  sensor_message_memory_conf.max_ros2_type_sequence_capacity = 1;
  sensor_message_memory_conf.max_basic_type_sequence_capacity = 1;

  if (!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &joint_state_message, joint_message_memory_conf) ||
    !micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &joint_command_message, joint_message_memory_conf) ||
    !micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      &imu_message, sensor_message_memory_conf) ||
    !micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      &battery_message, sensor_message_memory_conf))
  {
    return false;
  }

  joint_state_message.name.size = 2;
  joint_state_message.position.size = 2;
  joint_state_message.velocity.size = 2;
  joint_state_message.effort.size = 0;
  if (!rosidl_runtime_c__String__assign(
      &joint_state_message.name.data[0], LEFT_JOINT_NAME) ||
    !rosidl_runtime_c__String__assign(
      &joint_state_message.name.data[1], RIGHT_JOINT_NAME) ||
    !rosidl_runtime_c__String__assign(&imu_message.header.frame_id, IMU_FRAME_ID) ||
    !rosidl_runtime_c__String__assign(&battery_message.header.frame_id, BATTERY_FRAME_ID))
  {
    return false;
  }

  imu_message.orientation.w = 1.0;
  imu_message.orientation_covariance[0] = -1.0;

  battery_message.temperature = NAN;
  battery_message.current = NAN;
  battery_message.charge = NAN;
  battery_message.capacity = NAN;
  battery_message.design_capacity = NAN;
  battery_message.percentage = NAN;
  battery_message.power_supply_status =
    sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
  battery_message.power_supply_health =
    sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_message.power_supply_technology =
    sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  battery_message.present = true;
  battery_message.cell_voltage.size = 0;
  battery_message.cell_temperature.size = 0;
  return true;
}

void reset_entity_handles()
{
  support = rclc_support_t{};
  node = rcl_get_zero_initialized_node();
  executor = rclc_executor_get_zero_initialized_executor();
  joint_state_publisher = rcl_get_zero_initialized_publisher();
  imu_publisher = rcl_get_zero_initialized_publisher();
  battery_publisher = rcl_get_zero_initialized_publisher();
  joint_command_subscription = rcl_get_zero_initialized_subscription();

  support_initialized = false;
  node_initialized = false;
  executor_initialized = false;
  command_subscription_initialized = false;
  joint_publisher_initialized = false;
  imu_publisher_initialized = false;
  battery_publisher_initialized = false;
}

bool create_micro_ros_entities()
{
  reset_entity_handles();
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
    return false;
  }
  support_initialized = true;

  if (rclc_node_init_default(&node, "esp32_diffbot", "", &support) != RCL_RET_OK) {
    return false;
  }
  node_initialized = true;

  if (rclc_publisher_init_best_effort(
      &joint_state_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      JOINT_STATE_TOPIC) != RCL_RET_OK)
  {
    return false;
  }
  joint_publisher_initialized = true;

  if (rclc_publisher_init_best_effort(
      &imu_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), IMU_TOPIC) != RCL_RET_OK)
  {
    return false;
  }
  imu_publisher_initialized = true;

  if (rclc_publisher_init_default(
      &battery_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      BATTERY_TOPIC) != RCL_RET_OK)
  {
    return false;
  }
  battery_publisher_initialized = true;

  if (rclc_subscription_init_default(
      &joint_command_subscription, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      JOINT_COMMAND_TOPIC) != RCL_RET_OK)
  {
    return false;
  }
  command_subscription_initialized = true;

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    return false;
  }
  executor_initialized = true;

  if (rclc_executor_add_subscription(
      &executor, &joint_command_subscription, &joint_command_message,
      &joint_command_callback, ON_NEW_DATA) != RCL_RET_OK)
  {
    return false;
  }

  (void)rmw_uros_sync_session(1000);
  const uint32_t now_ms = millis();
  last_joint_state_ms = now_ms;
  last_imu_ms = now_ms;
  last_battery_ms = now_ms;
  stop_motors_and_clear_command();
  return true;
}

void destroy_micro_ros_entities()
{
  stop_motors_and_clear_command();

  if (support_initialized) {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    if (rmw_context != nullptr) {
      (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }
  }
  if (executor_initialized) {
    (void)rclc_executor_fini(&executor);
  }
  if (command_subscription_initialized && node_initialized) {
    (void)rcl_subscription_fini(&joint_command_subscription, &node);
  }
  if (battery_publisher_initialized && node_initialized) {
    (void)rcl_publisher_fini(&battery_publisher, &node);
  }
  if (imu_publisher_initialized && node_initialized) {
    (void)rcl_publisher_fini(&imu_publisher, &node);
  }
  if (joint_publisher_initialized && node_initialized) {
    (void)rcl_publisher_fini(&joint_state_publisher, &node);
  }
  if (node_initialized) {
    (void)rcl_node_fini(&node);
  }
  if (support_initialized) {
    (void)rclc_support_fini(&support);
  }
  reset_entity_handles();
}

builtin_interfaces__msg__Time current_ros_time()
{
  builtin_interfaces__msg__Time stamp = {};
  if (rmw_uros_epoch_synchronized()) {
    const int64_t now_ns = rmw_uros_epoch_nanos();
    stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
  }
  return stamp;
}

void publish_joint_state()
{
  joint_state_message.header.stamp = current_ros_time();
  joint_state_message.position.data[0] = measured_left_position;
  joint_state_message.position.data[1] = measured_right_position;
  joint_state_message.velocity.data[0] = measured_left_velocity;
  joint_state_message.velocity.data[1] = measured_right_velocity;
  (void)rcl_publish(&joint_state_publisher, &joint_state_message, nullptr);
}

bool read_mpu6050()
{
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(
      static_cast<uint8_t>(MPU6050_ADDRESS), static_cast<size_t>(14), true) != 14)
  {
    return false;
  }

  const auto read_word = []() -> int16_t {
      return static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    };
  const int16_t accel_x = read_word();
  const int16_t accel_y = read_word();
  const int16_t accel_z = read_word();
  (void)read_word();  // temperature
  const int16_t gyro_x = read_word();
  const int16_t gyro_y = read_word();
  const int16_t gyro_z = read_word();

  const float sensor_accel_x = (accel_x / 16384.0f - ACCEL_BIAS_X) * GRAVITY;
  const float sensor_accel_y = (accel_y / 16384.0f - ACCEL_BIAS_Y) * GRAVITY;
  const float sensor_accel_z = (accel_z / 16384.0f - ACCEL_BIAS_Z) * GRAVITY;
  const float sensor_gyro_x = (gyro_x / 131.0f - GYRO_BIAS_X) * DEG_TO_RAD_F;
  const float sensor_gyro_y = (gyro_y / 131.0f - GYRO_BIAS_Y) * DEG_TO_RAD_F;
  const float sensor_gyro_z = (gyro_z / 131.0f - GYRO_BIAS_Z) * DEG_TO_RAD_F;

  // The MPU is mounted with +Y forward and +X right. Convert that sensor frame
  // into ROS base_link convention: +X forward, +Y left, +Z up.
  imu_message.linear_acceleration.x = sensor_accel_y;
  imu_message.linear_acceleration.y = -sensor_accel_x;
  imu_message.linear_acceleration.z = sensor_accel_z;
  imu_message.angular_velocity.x = sensor_gyro_y;
  imu_message.angular_velocity.y = -sensor_gyro_x;
  imu_message.angular_velocity.z = sensor_gyro_z;
  return true;
}

void publish_imu()
{
  if (!imu_available || !read_mpu6050()) {
    return;
  }
  imu_message.header.stamp = current_ros_time();
  (void)rcl_publish(&imu_publisher, &imu_message, nullptr);
}

// 実バッテリー値のへの変換近似間数
constexpr float BATTERY_CALIBRATION_CUBIC_COEFFICIENT =
    31.15033913f;

constexpr float BATTERY_CALIBRATION_QUADRATIC_COEFFICIENT =
    -124.8253752f;

constexpr float BATTERY_CALIBRATION_LINEAR_COEFFICIENT =
    186.3876122f;

constexpr float BATTERY_CALIBRATION_CONSTANT_TERM =
    -83.70186445f;

float calibrate_battery_voltage(float measured_value)
{
  return
    BATTERY_CALIBRATION_CUBIC_COEFFICIENT
      * measured_value * measured_value * measured_value
    + BATTERY_CALIBRATION_QUADRATIC_COEFFICIENT
      * measured_value * measured_value
    + BATTERY_CALIBRATION_LINEAR_COEFFICIENT
      * measured_value
    + BATTERY_CALIBRATION_CONSTANT_TERM;
}

void publish_battery()
{
  const uint32_t pin_millivolts = analogReadMilliVolts(BATTERY_PIN);
  battery_message.header.stamp = current_ros_time();
  // battery_message.voltage = pin_millivolts * 0.001f * VOLTAGE_DIVIDER_RATIO;
  const float measured_battery_voltage = pin_millivolts * 0.001f;
  // battery_message.voltage = calibrate_battery_voltage(measured_battery_voltage);
  battery_message.voltage = measured_battery_voltage;
  (void)rcl_publish(&battery_publisher, &battery_message, nullptr);
}

void initialize_hardware()
{
  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  left_last_ab =
    (digitalRead(LEFT_ENCODER_A_PIN) << 1) | digitalRead(LEFT_ENCODER_B_PIN);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), left_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B_PIN), left_encoder_isr, CHANGE);

  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
  right_last_ab =
    (digitalRead(RIGHT_ENCODER_A_PIN) << 1) | digitalRead(RIGHT_ENCODER_B_PIN);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), right_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B_PIN), right_encoder_isr, CHANGE);

  pinMode(LEFT_DIRECTION_PIN, OUTPUT);
  pinMode(RIGHT_DIRECTION_PIN, OUTPUT);
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CHANNEL);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL);
  set_motors(0, 0);

  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  Wire.begin(21, 22);
  Wire.setTimeOut(10);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  imu_available = Wire.endTransmission(true) == 0;
}

void setup()
{
  initialize_hardware();
  if (!initialize_message_memory()) {
    set_motors(0, 0);
    while (true) {
      delay(1000);
    }
  }

  (void)rmw_uros_set_custom_transport(
    true, nullptr, high_speed_transport_open, high_speed_transport_close,
    high_speed_transport_write, high_speed_transport_read);

  last_control_us = micros();
  last_agent_action_ms = millis();
  agent_state = AgentState::WAITING;
}

void loop()
{
  update_motor_control();
  const uint32_t now_ms = millis();

  switch (agent_state) {
    case AgentState::WAITING:
      if (now_ms - last_agent_action_ms >= AGENT_SEARCH_PERIOD_MS) {
        last_agent_action_ms = now_ms;
        agent_state =
          rmw_uros_ping_agent(100, 1) == RMW_RET_OK ?
          AgentState::AVAILABLE : AgentState::WAITING;
      }
      break;

    case AgentState::AVAILABLE:
      stop_motors_and_clear_command();
      if (create_micro_ros_entities()) {
        agent_state = AgentState::CONNECTED;
        last_agent_action_ms = millis();
      } else {
        destroy_micro_ros_entities();
        agent_state = AgentState::WAITING;
        last_agent_action_ms = millis();
      }
      break;

    case AgentState::CONNECTED:
      (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

      if (now_ms - last_joint_state_ms >= JOINT_STATE_PERIOD_MS) {
        last_joint_state_ms = now_ms;
        publish_joint_state();
      }
      if (now_ms - last_imu_ms >= IMU_PERIOD_MS) {
        last_imu_ms = now_ms;
        publish_imu();
      }
      if (now_ms - last_battery_ms >= BATTERY_PERIOD_MS) {
        last_battery_ms = now_ms;
        publish_battery();
      }
      if (now_ms - last_agent_action_ms >= AGENT_CHECK_PERIOD_MS) {
        last_agent_action_ms = now_ms;
        if (rmw_uros_ping_agent(20, 1) != RMW_RET_OK) {
          agent_state = AgentState::DISCONNECTED;
        }
      }
      break;

    case AgentState::DISCONNECTED:
      destroy_micro_ros_entities();
      agent_state = AgentState::WAITING;
      last_agent_action_ms = millis();
      break;
  }

  update_motor_control();
}
