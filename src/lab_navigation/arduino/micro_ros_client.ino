#include <Arduino.h>

// ---------------- ハードウェア ----------------
#define LEFT_A 13
#define LEFT_B 14
#define RIGHT_A 27
#define RIGHT_B 26
#define LEFT_PWM 16
#define LEFT_DIR 5
#define RIGHT_PWM 17
#define RIGHT_DIR 18
#define MAX_PWM 255
#define MIN_PWM 0
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define LEFT_PWM_CH 0
#define RIGHT_PWM_CH 1

// ---------------- ロボットパラメータ ----------------
#define WHEEL_RADIUS 0.135
#define WHEEL_BASE   0.50
#define TICKS_PER_REV 1060.0

double PWM_SCALE = 9.0;
double MAX_LINEAR_VEL = 0.5;

// ---------------- 通信用構造体 ----------------
struct SendPacket {
  float left_velocity_cmd;
  float right_velocity_cmd;
} __attribute__((packed));

struct ReceivePacket {
  float left_position;
  float left_velocity;
  float right_position;
  float right_velocity;
} __attribute__((packed));

SendPacket rx_data;
ReceivePacket tx_data;

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

// ---------------- モータ ----------------
void setMotor(int l_pwm,int r_pwm){
  if(abs(l_pwm)<MIN_PWM) ledcWrite(LEFT_PWM_CH,0);
  else{
    digitalWrite(LEFT_DIR,l_pwm>0);
    ledcWrite(LEFT_PWM_CH,abs(l_pwm));
  }
  if(abs(r_pwm)<MIN_PWM) ledcWrite(RIGHT_PWM_CH,0);
  else{
    digitalWrite(RIGHT_DIR,r_pwm>0);
    ledcWrite(RIGHT_PWM_CH,abs(r_pwm));
  }
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
  if (Serial.available() >= sizeof(SendPacket)) {
    // 構造体のサイズ分だけ一気にバイナリ読み込み
    Serial.readBytes((char*)&rx_data, sizeof(SendPacket));

    // 目標速度(rad/s)からPWM値を計算 (簡易的なスカラー倍による開ループ制御の場合)
    // 実際の実装は以前のコードのPWM_SCALEなどを活用
    double target_left = rx_data.left_velocity_cmd * WHEEL_RADIUS;
    double target_right = rx_data.right_velocity_cmd * WHEEL_RADIUS;
    
    int pwm_left  = (int)(target_left/PWM_SCALE*MAX_PWM);
    int pwm_right = (int)(target_right/PWM_SCALE*MAX_PWM);
    pwm_left  = constrain(pwm_left,-MAX_PWM,MAX_PWM);
    pwm_right = constrain(pwm_right,-MAX_PWM,MAX_PWM);

    setMotor(pwm_left, pwm_right);
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

    tx_data.left_position  = current_left_pos;
    tx_data.left_velocity  = left_vel;
    tx_data.right_position = current_right_pos;
    tx_data.right_velocity = right_vel;

    // 構造体のメモリをそのまま送信
    Serial.write((uint8_t*)&tx_data, sizeof(ReceivePacket));
    
    last_time = current_time;
  }
}