// ESP32 / Arduino-ESP32 2.x / micro_ros_arduino (Humble).
// USB Serial is exclusively owned by micro-ROS; do not print debug text to it.
#include <Arduino.h>
#include <math.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float32.h>

#define BATTERY 34
#define LEFT_A 13
#define LEFT_B 14
#define RIGHT_A 27
#define RIGHT_B 26
#define LEFT_PWM 16
#define LEFT_DIR 5
#define RIGHT_PWM 17
#define RIGHT_DIR 18
#define LEFT_PWM_CH 0
#define RIGHT_PWM_CH 1
#define MAX_PWM 70
#define MIN_PWM 5
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define COUNTS_PER_REV (1060.0 * 4.0)
#define CONTROL_PERIOD_MS 50
#define COMMAND_TIMEOUT_MS 500
// Match the PC's ROS_DOMAIN_ID. The Agent does not translate domains.
#ifndef MICRO_ROS_DOMAIN_ID
#define MICRO_ROS_DOMAIN_ID 0
#endif

const float VOLTAGE_DIVIDER_RATIO = (99000.0f + 10000.0f) / 10000.0f;
const float Kp = 25.0f, Ki = 1.0f, Kd = 0.0f;
portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
volatile int64_t left_count = 0, right_count = 0;
volatile int left_last_AB = 0, right_last_AB = 0;
DRAM_ATTR const int8_t QUADRATURE_DELTA[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

// All fields shared between the communication and control tasks are protected.
double target_velocity[2] = {0.0, 0.0};
double wheel_position[2] = {0.0, 0.0};
double wheel_velocity[2] = {0.0, 0.0};
uint32_t last_command_ms = 0;
bool command_valid = false;
bool connected = false;

void IRAM_ATTR leftEncoder()
{
  const int ab = (digitalRead(LEFT_A) << 1) | digitalRead(LEFT_B);
  portENTER_CRITICAL_ISR(&encoder_mux);
  left_count += QUADRATURE_DELTA[(left_last_AB << 2) | ab];
  left_last_AB = ab;
  portEXIT_CRITICAL_ISR(&encoder_mux);
}

void IRAM_ATTR rightEncoder()
{
  const int ab = (digitalRead(RIGHT_A) << 1) | digitalRead(RIGHT_B);
  portENTER_CRITICAL_ISR(&encoder_mux);
  right_count += QUADRATURE_DELTA[(right_last_AB << 2) | ab];
  right_last_AB = ab;
  portEXIT_CRITICAL_ISR(&encoder_mux);
}

void setMotor(int left, int right)
{
  const int pwm[2] = {left, right};
  const int channels[2] = {LEFT_PWM_CH, RIGHT_PWM_CH};
  const int pins[2] = {LEFT_DIR, RIGHT_DIR};
  for (size_t i = 0; i < 2; ++i) {
    if (abs(pwm[i]) < MIN_PWM) {
      ledcWrite(channels[i], 0);
    } else {
      digitalWrite(pins[i], pwm[i] > 0);
      ledcWrite(channels[i], constrain(abs(pwm[i]), MIN_PWM, MAX_PWM));
    }
  }
}

void invalidateCommand()
{
  portENTER_CRITICAL(&state_mux);
  command_valid = false;
  target_velocity[0] = target_velocity[1] = 0.0;
  portEXIT_CRITICAL(&state_mux);
}

// Runs independently of Agent discovery, entity creation and serial publication.
// Only this task writes PWM after setup, including the communication watchdog.
void controlTask(void *)
{
  int64_t previous[2] = {0, 0};
  float integral[2] = {0.0f, 0.0f};
  float previous_error[2] = {0.0f, 0.0f};
  uint32_t previous_us = micros();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    int64_t counts[2];
    portENTER_CRITICAL(&encoder_mux);
    const uint32_t now_us = micros();
    counts[0] = left_count;
    counts[1] = right_count;
    portEXIT_CRITICAL(&encoder_mux);
    const double dt = static_cast<uint32_t>(now_us - previous_us) * 1e-6;
    if (dt <= 0.0) {continue;}
    previous_us = now_us;
    double position[2], velocity[2], targets[2];
    for (size_t i = 0; i < 2; ++i) {
      position[i] = 2.0 * PI * counts[i] / COUNTS_PER_REV;
      velocity[i] = 2.0 * PI * (counts[i] - previous[i]) / COUNTS_PER_REV / dt;
      previous[i] = counts[i];
    }
    portENTER_CRITICAL(&state_mux);
    if (!connected || !command_valid ||
      static_cast<uint32_t>(millis() - last_command_ms) >= COMMAND_TIMEOUT_MS)
    {
      command_valid = false;
      target_velocity[0] = target_velocity[1] = 0.0;
    }
    for (size_t i = 0; i < 2; ++i) {
      targets[i] = target_velocity[i];
      wheel_position[i] = position[i];
      wheel_velocity[i] = velocity[i];
    }
    portEXIT_CRITICAL(&state_mux);
    int pwm[2] = {0, 0};
    for (size_t i = 0; i < 2; ++i) {
      if (fabs(targets[i]) < 0.0001) {
        integral[i] = previous_error[i] = 0.0f;
      } else {
        const float error = targets[i] - velocity[i];
        integral[i] += error * dt;
        if (Ki > 0.0f) {integral[i] = constrain(integral[i], -MAX_PWM / Ki, MAX_PWM / Ki);}
        const float derivative = (error - previous_error[i]) / dt;
        previous_error[i] = error;
        // Clamp before conversion to an integer, including unexpectedly large inputs.
        const float output = Kp * error + Ki * integral[i] + Kd * derivative;
        pwm[i] = static_cast<int>(constrain(output, -float(MAX_PWM), float(MAX_PWM)));
      }
    }
    setMotor(pwm[0], pwm[1]);
  }
}

rcl_allocator_t allocator;
rclc_support_t support{};
rcl_node_t node{};
rcl_subscription_t command_sub{};
rcl_publisher_t state_pub{}, battery_pub{};
rclc_executor_t executor{};
bool support_ready = false, node_ready = false, sub_ready = false;
bool state_pub_ready = false, battery_pub_ready = false, executor_ready = false;

// Preallocated storage: no message allocation inside the running control loop.
std_msgs__msg__Float64MultiArray command_msg{};
double command_data[2];
sensor_msgs__msg__JointState state_msg{};
rosidl_runtime_c__String joint_names[2]{};
char left_name[] = "left_wheel_joint";
char right_name[] = "right_wheel_joint";
char empty_frame[] = "";
double state_positions[2], state_velocities[2];
std_msgs__msg__Float32 battery_msg{};

void commandCallback(const void * input)
{
  const auto * msg = static_cast<const std_msgs__msg__Float64MultiArray *>(input);
  if (msg->data.size != 2 || !isfinite(msg->data.data[0]) || !isfinite(msg->data.data[1]) ||
    fabs(msg->data.data[0]) > 20.0 || fabs(msg->data.data[1]) > 20.0)
  {
    invalidateCommand();
    return;
  }
  portENTER_CRITICAL(&state_mux);
  target_velocity[0] = msg->data.data[0];
  target_velocity[1] = msg->data.data[1];
  last_command_ms = millis();
  command_valid = true;
  portEXIT_CRITICAL(&state_mux);
}

void initMessages()
{
  command_msg.data.data = command_data;
  command_msg.data.size = 0;
  command_msg.data.capacity = 2;
  joint_names[0].data = left_name;
  joint_names[0].size = sizeof(left_name) - 1;
  joint_names[0].capacity = sizeof(left_name);
  joint_names[1].data = right_name;
  joint_names[1].size = sizeof(right_name) - 1;
  joint_names[1].capacity = sizeof(right_name);
  state_msg.name.data = joint_names;
  state_msg.name.size = state_msg.name.capacity = 2;
  state_msg.position.data = state_positions;
  state_msg.position.size = state_msg.position.capacity = 2;
  state_msg.velocity.data = state_velocities;
  state_msg.velocity.size = state_msg.velocity.capacity = 2;
  state_msg.header.frame_id.data = empty_frame;
  state_msg.header.frame_id.size = 0;
  state_msg.header.frame_id.capacity = 1;
}

bool createEntities()
{
  allocator = rcl_get_default_allocator();
  support = {};
  node = rcl_get_zero_initialized_node();
  command_sub = rcl_get_zero_initialized_subscription();
  state_pub = rcl_get_zero_initialized_publisher();
  battery_pub = rcl_get_zero_initialized_publisher();
  executor = rclc_executor_get_zero_initialized_executor();
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&options, allocator) != RCL_RET_OK) {return false;}
  rcl_ret_t result = rcl_init_options_set_domain_id(&options, MICRO_ROS_DOMAIN_ID);
  if (result == RCL_RET_OK) {
    result = rclc_support_init_with_options(&support, 0, nullptr, &options, &allocator);
  }
  (void) rcl_init_options_fini(&options);
  if (result != RCL_RET_OK) {return false;}
  support_ready = true;
  if (rclc_node_init_default(&node, "diffbot_mcu", "", &support) != RCL_RET_OK) {return false;}
  node_ready = true;
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
  qos.depth = 1;
  if (rclc_subscription_init(&command_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/mcu/wheel_commands", &qos) != RCL_RET_OK) {return false;}
  sub_ready = true;
  if (rclc_publisher_init(&state_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/mcu/wheel_states", &qos) != RCL_RET_OK) {return false;}
  state_pub_ready = true;
  // Retain the existing reliable /battery_level subscription contract.
  if (rclc_publisher_init_default(&battery_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/battery_level") != RCL_RET_OK) {return false;}
  battery_pub_ready = true;
  (void) rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&battery_pub), 10);
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {return false;}
  executor_ready = true;
  if (rclc_executor_add_subscription(&executor, &command_sub, &command_msg,
    &commandCallback, ON_NEW_DATA) != RCL_RET_OK) {return false;}
  // A zero header timestamp is used if epoch synchronization is unavailable.
  (void) rmw_uros_sync_session(100);
  return true;
}

void destroyEntities()
{
  portENTER_CRITICAL(&state_mux);
  connected = false;
  command_valid = false;
  target_velocity[0] = target_velocity[1] = 0.0;
  portEXIT_CRITICAL(&state_mux);
  if (support_ready) {
    (void) rmw_uros_set_context_entity_destroy_session_timeout(
      rcl_context_get_rmw_context(&support.context), 0);
  }
  if (executor_ready) {(void) rclc_executor_fini(&executor); executor_ready = false;}
  if (sub_ready) {(void) rcl_subscription_fini(&command_sub, &node); sub_ready = false;}
  if (state_pub_ready) {(void) rcl_publisher_fini(&state_pub, &node); state_pub_ready = false;}
  if (battery_pub_ready) {(void) rcl_publisher_fini(&battery_pub, &node); battery_pub_ready = false;}
  if (node_ready) {(void) rcl_node_fini(&node); node_ready = false;}
  if (support_ready) {(void) rclc_support_fini(&support); support_ready = false;}
}

void setup()
{
  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);
  left_last_AB = (digitalRead(LEFT_A) << 1) | digitalRead(LEFT_B);
  right_last_AB = (digitalRead(RIGHT_A) << 1) | digitalRead(RIGHT_B);
  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B), rightEncoder, CHANGE);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  ledcSetup(LEFT_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM, LEFT_PWM_CH);
  ledcAttachPin(RIGHT_PWM, RIGHT_PWM_CH);
  setMotor(0, 0);
  initMessages();
  set_microros_transports();  // default Arduino Serial transport: 115200 baud
  if (xTaskCreate(controlTask, "motor_control", 4096, nullptr, 2, nullptr) != pdPASS) {
    for (;;) {setMotor(0, 0); delay(1000);}
  }
}

void loop()
{
  static bool entities_ready = false;
  static uint32_t last_ping = 0, last_state = 0, last_battery = 0;
  const uint32_t now = millis();
  if (!entities_ready) {
    if (static_cast<uint32_t>(now - last_ping) >= 500) {
      last_ping = now;
      if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) {
        entities_ready = createEntities();
        if (!entities_ready) {
          destroyEntities();
        } else {
          portENTER_CRITICAL(&state_mux);
          connected = true;
          command_valid = false;
          portEXIT_CRITICAL(&state_mux);
          last_ping = millis();
        }
      }
    }
    delay(1);
    return;
  }
  if (static_cast<uint32_t>(now - last_ping) >= 500) {
    last_ping = now;
    if (rmw_uros_ping_agent(20, 1) != RMW_RET_OK) {
      destroyEntities();
      entities_ready = false;
      return;
    }
  }
  const rcl_ret_t spin_result = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT) {
    destroyEntities();
    entities_ready = false;
    return;
  }
  if (static_cast<uint32_t>(now - last_state) >= CONTROL_PERIOD_MS) {
    last_state = now;
    portENTER_CRITICAL(&state_mux);
    for (size_t i = 0; i < 2; ++i) {
      state_positions[i] = wheel_position[i];
      state_velocities[i] = wheel_velocity[i];
    }
    portEXIT_CRITICAL(&state_mux);
    const int64_t ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : 0;
    state_msg.header.stamp.sec = ns / 1000000000LL;
    state_msg.header.stamp.nanosec = ns % 1000000000LL;
    if (rcl_publish(&state_pub, &state_msg, nullptr) != RCL_RET_OK) {
      destroyEntities();
      entities_ready = false;
      return;
    }
  }
  if (static_cast<uint32_t>(now - last_battery) >= 1000) {
    last_battery = now;
    battery_msg.data = analogReadMilliVolts(BATTERY) / 1000.0f * VOLTAGE_DIVIDER_RATIO;
    (void) rcl_publish(&battery_pub, &battery_msg, nullptr);
  }
  delay(1);
}
