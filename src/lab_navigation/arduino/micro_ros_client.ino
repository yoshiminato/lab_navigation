#include <Arduino.h>

// ---------------- ハードウェア ----------------
#define BATTERY 34
#define LEFT_A 13
#define LEFT_B 14
#define RIGHT_A 27
#define RIGHT_B 26
#define LEFT_PWM 16
#define LEFT_DIR 18
#define RIGHT_PWM 17
#define RIGHT_DIR 5
#define MAX_PWM 40
#define MIN_PWM 15
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define LEFT_PWM_CH 0
#define RIGHT_PWM_CH 1

#define R1 99000
#define R2 10000
const float VOLTAGE_DIVIDER_RATIO = (R1 + R2) / R2;

// ---------------- ロボットパラメータ ----------------
#define WHEEL_RADIUS 0.135
#define WHEEL_BASE   0.50
#define TICKS_PER_REV 1060.0

double PWM_SCALE_LEFT  = 9e-2;
double PWM_SCALE_RIGHT = 9e-2;



#define MAX_VEL 0.26

// ---------------- 通信パケット関連 ----------------
#define HEADER1 0xAA
#define HEADER2 0x55
#define WAIT_FOR_HEADER1 0
#define WAIT_FOR_HEADER2 1
#define RECEIVE_DATA 2
struct CommandPacket {
  uint8_t header1;
  uint8_t header2;
  float left_velocity_cmd;
  float right_velocity_cmd;
  uint8_t checksum; // 簡単なエラーチェック用
} __attribute__((packed));

struct StatusPacket {
  uint8_t header1;
  uint8_t header2;
  float left_position;
  float left_velocity;
  float right_position;
  float right_velocity;
  float battery_voltage;
  uint8_t checksum; // 簡単なエラーチェック用
} __attribute__((packed));

CommandPacket rx_data;
StatusPacket tx_data;

// 簡単なチェックサム計算 (XORベース)
uint8_t calculateChecksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) {
    cs ^= data[i];
  }
  return cs;
}

bool receivePacket(CommandPacket* packet) {

  static uint8_t buffer[sizeof(CommandPacket)];
  static size_t  state = WAIT_FOR_HEADER1;
  static size_t  index = 0;

  while (Serial.available()) {
    uint8_t byte = Serial.read();

    switch(state) {
      case WAIT_FOR_HEADER1: // ヘッダー1待ち
        if (byte == HEADER1) state = WAIT_FOR_HEADER2;
        break;
      case WAIT_FOR_HEADER2: // ヘッダー2待ち
        if (byte == HEADER2) {
          state = RECEIVE_DATA;
          index = 0;
          buffer[index++] = HEADER1;
          buffer[index++] = HEADER2;
        } 
        else state = WAIT_FOR_HEADER1;
        break;
      case RECEIVE_DATA: // データ受信中
        
        buffer[index++] = byte;
        if (index < sizeof(CommandPacket)) break; // まだ完全に受信されていない
        
        size_t   len = sizeof(CommandPacket) - 1;   // チェックサムを除いたデータ部分のバイト数
        uint8_t  cs  = calculateChecksum(buffer, len); // チェックサム計算

        if (byte != cs) {
          state = WAIT_FOR_HEADER1; // 次のパケット受信に備えて状態をリセット
          break; // チェックサムエラー
        }

        memcpy(packet, buffer, sizeof(CommandPacket)); // 受信したデータを構造体にコピー
        state = WAIT_FOR_HEADER1; // 次のパケット受信に備えて状態をリセット
        return true; // 正常に受信完了

        break;
    }
  }
  return false; // パケットがまだ完全に受信されていない
}


// ---------------- グローバル ----------------
volatile long left_count = 0;
volatile long right_count = 0;
volatile int left_last_AB = 0;
volatile int right_last_AB = 0;

long prev_left = 0;
long prev_right = 0;
unsigned long last_time = 0;

double current_left_pos = 0;
double current_right_pos = 0;

// ---------------- エンコーダ ----------------
void IRAM_ATTR leftEncoder(){
  int A = digitalRead(LEFT_A);
  int B = digitalRead(LEFT_B);
  int AB = (A<<1)|B;
  int delta = 0;
  switch((left_last_AB<<2)|AB){
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: delta=1; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: delta=-1; break;
  }
  left_count += delta;
  left_last_AB = AB;
}

void IRAM_ATTR rightEncoder(){
  int A = digitalRead(RIGHT_A);
  int B = digitalRead(RIGHT_B);
  int AB = (A<<1)|B;
  int delta = 0;
  switch((right_last_AB<<2)|AB){
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: delta=1; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: delta=-1; break;
  }
  right_count += delta;
  right_last_AB = AB;
}

int calcPWM(float vel){
  if (vel == 0.0f) return 0; // 停止指令の場合は確実に0を返す
  const int scale = 10;
  long scaled_vel = (long)(vel*scale);
  long min_in     = 0;
  long max_in     = (long)(MAX_VEL/WHEEL_RADIUS)*scale;
  return (int)map(scaled_vel, min_in, max_in, MIN_PWM, MAX_PWM);
}

// ---------------- モータ ----------------
void setMotor(float l_vel, float r_vel){
  // if(abs(l_pwm)<MIN_PWM) ledcWrite(LEFT_PWM_CH,0);
  // else{
  //   digitalWrite(LEFT_DIR,l_pwm>0);
  //   ledcWrite(LEFT_PWM_CH,abs(l_pwm));
  // }
  // if(abs(r_pwm)<MIN_PWM) ledcWrite(RIGHT_PWM_CH,0);
  // else{
  //   digitalWrite(RIGHT_DIR,r_pwm>0);
  //   ledcWrite(RIGHT_PWM_CH,abs(r_pwm));
  // }
  digitalWrite(LEFT_DIR ,l_vel>0);
  ledcWrite(LEFT_PWM_CH, calcPWM(abs(l_vel)));
  digitalWrite(RIGHT_DIR,r_vel>0);
  ledcWrite(RIGHT_PWM_CH,calcPWM(abs(r_vel)));
}

// ---------------- setup ----------------
void setup(){
  Serial.begin(115200);

  // エンコーダ・モータピンの初期化
  pinMode(LEFT_A,INPUT_PULLUP);
  pinMode(LEFT_B,INPUT_PULLUP);
  left_last_AB = (digitalRead(LEFT_A)<<1)|digitalRead(LEFT_B);
  attachInterrupt(digitalPinToInterrupt(LEFT_A),leftEncoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B),leftEncoder,CHANGE);

  pinMode(RIGHT_A,INPUT_PULLUP);
  pinMode(RIGHT_B,INPUT_PULLUP);
  right_last_AB = (digitalRead(RIGHT_A)<<1)|digitalRead(RIGHT_B);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A),rightEncoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B),rightEncoder,CHANGE);

  pinMode(LEFT_DIR,OUTPUT);
  pinMode(RIGHT_DIR,OUTPUT);
  ledcSetup(LEFT_PWM_CH,PWM_FREQ,PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CH,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM,LEFT_PWM_CH);
  ledcAttachPin(RIGHT_PWM,RIGHT_PWM_CH);

  last_time = millis();
}

// ---------------- loop ----------------
void loop(){
  // --- 1. PCからの指令を受信 ---
  if (receivePacket(&rx_data)) {
    // int pwm_left  = (int)(rx_data.left_velocity_cmd*PWM_SCALE_LEFT*MAX_PWM);
    // int pwm_right = (int)(rx_data.right_velocity_cmd*PWM_SCALE_RIGHT*MAX_PWM);
    // pwm_left  = constrain(pwm_left,-MAX_PWM,MAX_PWM);
    // pwm_right = constrain(pwm_right,-MAX_PWM,MAX_PWM);
    // setMotor(pwm_left, pwm_right);
    setMotor(rx_data.left_velocity_cmd, rx_data.right_velocity_cmd);

  }

  // --- 2. 状態の計算とPCへの送信 ---
  unsigned long current_time = millis();
  double dt = (current_time - last_time) / 1000.0;
  
  if (dt >= 0.05) { // 約20Hz (50ms) で送信

    noInterrupts();
    long l = left_count;
    long r = right_count;
    interrupts();

    double l_diff = (l - prev_left) / 4.0;
    double r_diff = (r - prev_right) / 4.0;
    prev_left = l;
    prev_right = r;

    // pos: 全回転角度 (rad), vel: 角速度 (rad/s)
    current_left_pos  = 2 * PI * l / TICKS_PER_REV / 4.0;
    current_right_pos = 2 * PI * r / TICKS_PER_REV / 4.0;
    
    double left_vel  = (2 * PI * l_diff / TICKS_PER_REV) / dt;
    double right_vel = (2 * PI * r_diff / TICKS_PER_REV) / dt;

    uint32_t pin_millivolts = analogReadMilliVolts(BATTERY);
    float pin_voltage = pin_millivolts / 1000.0;
    float battery_voltage = pin_voltage * VOLTAGE_DIVIDER_RATIO; // 実際のバッテリー電圧

    tx_data.header1 = HEADER1;
    tx_data.header2 = HEADER2;
    tx_data.left_position  = current_left_pos;
    tx_data.left_velocity  = left_vel;
    tx_data.right_position = current_right_pos;
    tx_data.right_velocity = right_vel;
    tx_data.battery_voltage = battery_voltage;

    size_t   len = sizeof(StatusPacket) - 1; 
    uint8_t *ptr = (uint8_t*)&tx_data;       
    uint8_t   cs = calculateChecksum(ptr, len);
    tx_data.checksum = cs;

    // 構造体のメモリをそのまま送信
    Serial.write((uint8_t*)&tx_data, sizeof(StatusPacket));
    
    last_time = current_time;
  }
}