#include <Arduino.h>

// ---------------- ハードウェア ----------------
#define BATTERY 34

#define LEFT_A 13
#define LEFT_B 14
#define RIGHT_A 27
#define RIGHT_B 26

#define LEFT_PWM 16
#define LEFT_DIR 5
#define RIGHT_PWM 17
#define RIGHT_DIR 18

#define MAX_PWM 50
#define MIN_PWM 5

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define LEFT_PWM_CH 0
#define RIGHT_PWM_CH 1

// ---------------- バッテリー電圧計測 ----------------
#define R1 99000
#define R2 10000

const float VOLTAGE_DIVIDER_RATIO =
    (float)(R1 + R2) / (float)R2;


// ---------------- ロボットパラメータ ----------------
#define WHEEL_RADIUS 0.135
#define WHEEL_BASE   0.50

/*
  エンコーダ仕様：

  TICKS_PER_REV = 1060 が
  「1相あたりのパルス数」で、

  A相/B相の両方をCHANGEで読み取る
  4逓倍方式の場合、

  1回転あたりの実カウント数は

      1060 × 4 = 4240

  になります。
*/
#define TICKS_PER_REV 1060.0
#define COUNTS_PER_REV (TICKS_PER_REV * 4.0)


// ---------------- 制御周期 ----------------
#define CONTROL_PERIOD_MS 50


// ---------------- 通信パケット関連 ----------------
#define HEADER1 0xAA
#define HEADER2 0x55

#define WAIT_FOR_HEADER1 0
#define WAIT_FOR_HEADER2 1
#define RECEIVE_DATA     2


struct CommandPacket {

  uint8_t header1;
  uint8_t header2;

  float left_velocity_cmd;
  float right_velocity_cmd;

  uint8_t checksum;

} __attribute__((packed));


struct StatusPacket {

  uint8_t header1;
  uint8_t header2;

  float left_position;
  float left_velocity;

  float right_position;
  float right_velocity;

  float battery_voltage;

  uint8_t checksum;

} __attribute__((packed));


CommandPacket rx_data;
StatusPacket tx_data;


// ---------------- チェックサム ----------------
uint8_t calculateChecksum(
  const uint8_t* data,
  size_t len
) {

  uint8_t cs = 0;

  for (size_t i = 0; i < len; i++) {

    cs ^= data[i];
  }

  return cs;
}


// ---------------- パケット受信 ----------------
bool receivePacket(CommandPacket* packet) {

  static uint8_t buffer[sizeof(CommandPacket)];

  static size_t state = WAIT_FOR_HEADER1;
  static size_t index = 0;


  while (Serial.available()) {

    uint8_t byte = Serial.read();


    switch (state) {

      // ---------------- Header1 ----------------
      case WAIT_FOR_HEADER1:

        if (byte == HEADER1) {

          state = WAIT_FOR_HEADER2;
        }

        break;


      // ---------------- Header2 ----------------
      case WAIT_FOR_HEADER2:

        if (byte == HEADER2) {

          state = RECEIVE_DATA;

          index = 0;

          buffer[index++] = HEADER1;
          buffer[index++] = HEADER2;

        } else {

          // AA AA 55 のようなケースにも少し強くする
          if (byte == HEADER1) {

            state = WAIT_FOR_HEADER2;

          } else {

            state = WAIT_FOR_HEADER1;
          }
        }

        break;


      // ---------------- データ受信 ----------------
      case RECEIVE_DATA:

        if (index >= sizeof(CommandPacket)) {

          // 念のため
          state = WAIT_FOR_HEADER1;
          index = 0;

          break;
        }


        buffer[index++] = byte;


        // まだ全部届いていない
        if (index < sizeof(CommandPacket)) {

          break;
        }


        // チェックサム計算
        {
          size_t len =
              sizeof(CommandPacket) - 1;

          uint8_t cs =
              calculateChecksum(
                buffer,
                len
              );


          uint8_t received_checksum =
              buffer[sizeof(CommandPacket) - 1];


          if (received_checksum != cs) {

            state = WAIT_FOR_HEADER1;
            index = 0;

            break;
          }
        }


        // 正常受信
        memcpy(
          packet,
          buffer,
          sizeof(CommandPacket)
        );


        state = WAIT_FOR_HEADER1;
        index = 0;

        return true;
    }
  }


  return false;
}


// ======================================================
// エンコーダ
// ======================================================

volatile long left_count = 0;
volatile long right_count = 0;

volatile int left_last_AB = 0;
volatile int right_last_AB = 0;


// ---------------- 左エンコーダ ----------------
void IRAM_ATTR leftEncoder() {

  int A = digitalRead(LEFT_A);
  int B = digitalRead(LEFT_B);

  int AB = (A << 1) | B;

  int delta = 0;


  switch ((left_last_AB << 2) | AB) {

    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:

      delta = 1;
      break;


    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:

      delta = -1;
      break;
  }


  left_count += delta;

  left_last_AB = AB;
}


// ---------------- 右エンコーダ ----------------
void IRAM_ATTR rightEncoder() {

  int A = digitalRead(RIGHT_A);
  int B = digitalRead(RIGHT_B);

  int AB = (A << 1) | B;

  int delta = 0;


  switch ((right_last_AB << 2) | AB) {

    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:

      delta = 1;
      break;


    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:

      delta = -1;
      break;
  }


  right_count += delta;

  right_last_AB = AB;
}


// ======================================================
// エンコーダ計算用
// ======================================================

long prev_left_count = 0;
long prev_right_count = 0;

unsigned long last_control_time = 0;

double current_left_pos = 0.0;
double current_right_pos = 0.0;

double current_left_vel = 0.0;
double current_right_vel = 0.0;


// ======================================================
// PID
// ======================================================

float Kp = 25.0;
float Ki = 1.0;
float Kd = 0.0;


float left_err_sum = 0.0;
float right_err_sum = 0.0;

float left_prev_err = 0.0;
float right_prev_err = 0.0;


// ======================================================
// モータ
// ======================================================

void setMotor(
  int l_pwm,
  int r_pwm
) {

  // ---------------- 左 ----------------

  if (abs(l_pwm) < MIN_PWM) {

    ledcWrite(
      LEFT_PWM_CH,
      0
    );

  } else {

    digitalWrite(
      LEFT_DIR,
      l_pwm > 0
    );


    ledcWrite(
      LEFT_PWM_CH,
      constrain(
        abs(l_pwm),
        MIN_PWM,
        MAX_PWM
      )
    );
  }


  // ---------------- 右 ----------------

  if (abs(r_pwm) < MIN_PWM) {

    ledcWrite(
      RIGHT_PWM_CH,
      0
    );

  } else {

    digitalWrite(
      RIGHT_DIR,
      r_pwm > 0
    );


    ledcWrite(
      RIGHT_PWM_CH,
      constrain(
        abs(r_pwm),
        MIN_PWM,
        MAX_PWM
      )
    );
  }
}


// ======================================================
// setup
// ======================================================

void setup() {

  Serial.begin(115200);


  // ---------------- 左エンコーダ ----------------

  pinMode(
    LEFT_A,
    INPUT
  );

  pinMode(
    LEFT_B,
    INPUT
  );


  left_last_AB =
      (digitalRead(LEFT_A) << 1)
      |
      digitalRead(LEFT_B);


  attachInterrupt(
    digitalPinToInterrupt(LEFT_A),
    leftEncoder,
    CHANGE
  );


  attachInterrupt(
    digitalPinToInterrupt(LEFT_B),
    leftEncoder,
    CHANGE
  );


  // ---------------- 右エンコーダ ----------------

  pinMode(
    RIGHT_A,
    INPUT_PULLUP
  );

  pinMode(
    RIGHT_B,
    INPUT_PULLUP
  );


  right_last_AB =
      (digitalRead(RIGHT_A) << 1)
      |
      digitalRead(RIGHT_B);


  attachInterrupt(
    digitalPinToInterrupt(RIGHT_A),
    rightEncoder,
    CHANGE
  );


  attachInterrupt(
    digitalPinToInterrupt(RIGHT_B),
    rightEncoder,
    CHANGE
  );


  // ---------------- モータ ----------------

  pinMode(
    LEFT_DIR,
    OUTPUT
  );

  pinMode(
    RIGHT_DIR,
    OUTPUT
  );


  ledcSetup(
    LEFT_PWM_CH,
    PWM_FREQ,
    PWM_RESOLUTION
  );


  ledcSetup(
    RIGHT_PWM_CH,
    PWM_FREQ,
    PWM_RESOLUTION
  );


  ledcAttachPin(
    LEFT_PWM,
    LEFT_PWM_CH
  );


  ledcAttachPin(
    RIGHT_PWM,
    RIGHT_PWM_CH
  );


  // 初期状態は停止
  setMotor(
    0,
    0
  );


  // 初期指令
  rx_data.left_velocity_cmd = 0.0f;
  rx_data.right_velocity_cmd = 0.0f;


  // 初期カウントを取得
  noInterrupts();

  prev_left_count = left_count;
  prev_right_count = right_count;

  interrupts();


  last_control_time = millis();
}


// ======================================================
// loop
// ======================================================

void loop() {

  /*
    =====================================================
    1. PCからの指令を常時受信
    =====================================================

    ここが今回の重要な修正点。

    「新しいパケットが来なかったらreturn」
    という処理は行わない。

    パケットが来たときだけrx_dataが更新され、
    それ以外は前回の指令値を保持する。
  */

  receivePacket(&rx_data);


  /*
    =====================================================
    2. 制御周期
    =====================================================
  */

  unsigned long current_time = millis();


  unsigned long elapsed_ms =
      current_time - last_control_time;


  if (elapsed_ms < CONTROL_PERIOD_MS) {

    return;
  }


  double dt =
      elapsed_ms / 1000.0;


  /*
    =====================================================
    3. エンコーダ値取得
    =====================================================
  */

  long left_now;
  long right_now;


  noInterrupts();

  left_now = left_count;
  right_now = right_count;

  interrupts();


  /*
    =====================================================
    4. カウント差分
    =====================================================

    この差分とdtは必ず同じ時間区間になる。
  */

  long left_count_diff =
      left_now - prev_left_count;


  long right_count_diff =
      right_now - prev_right_count;


  /*
    前回値更新

    last_control_timeもこの周期の最後で更新するため、
    差分とdtの時間範囲が一致する。
  */

  prev_left_count = left_now;
  prev_right_count = right_now;


  /*
    =====================================================
    5. 位置計算
    =====================================================

    単位：rad
  */

  current_left_pos =
      2.0
      * PI
      * (double)left_now
      / COUNTS_PER_REV;


  current_right_pos =
      2.0
      * PI
      * (double)right_now
      / COUNTS_PER_REV;


  /*
    =====================================================
    6. 速度計算
    =====================================================

    単位：rad/s

      Δθ
    ------- 
      Δt
  */

  current_left_vel =
      (
        2.0
        * PI
        * (double)left_count_diff
        / COUNTS_PER_REV
      )
      / dt;


  current_right_vel =
      (
        2.0
        * PI
        * (double)right_count_diff
        / COUNTS_PER_REV
      )
      / dt;


  /*
    =====================================================
    7. PID制御
    =====================================================
  */

  int l_pwm = 0;
  int r_pwm = 0;


  // ---------------- 左PID ----------------

  if (fabs(rx_data.left_velocity_cmd) < 0.0001f) {

    left_err_sum = 0.0f;
    left_prev_err = 0.0f;

    l_pwm = 0;

  } else {

    float err =
        rx_data.left_velocity_cmd
        -
        current_left_vel;


    left_err_sum +=
        err * dt;


    // アンチワインドアップ
    if (Ki > 0.0f) {

      left_err_sum =
          constrain(
            left_err_sum,
            -MAX_PWM / Ki,
            MAX_PWM / Ki
          );
    }


    float d_err =
        (err - left_prev_err)
        /
        dt;


    left_prev_err = err;


    l_pwm =
        (int)(
          Kp * err
          +
          Ki * left_err_sum
          +
          Kd * d_err
        );
  }


  // ---------------- 右PID ----------------

  if (fabs(rx_data.right_velocity_cmd) < 0.0001f) {

    right_err_sum = 0.0f;
    right_prev_err = 0.0f;

    r_pwm = 0;

  } else {

    float err =
        rx_data.right_velocity_cmd
        -
        current_right_vel;


    right_err_sum +=
        err * dt;


    if (Ki > 0.0f) {

      right_err_sum =
          constrain(
            right_err_sum,
            -MAX_PWM / Ki,
            MAX_PWM / Ki
          );
    }


    float d_err =
        (err - right_prev_err)
        /
        dt;


    right_prev_err = err;


    r_pwm =
        (int)(
          Kp * err
          +
          Ki * right_err_sum
          +
          Kd * d_err
        );
  }


  /*
    =====================================================
    8. モータ出力
    =====================================================
  */

  setMotor(
    l_pwm,
    r_pwm
  );


  /*
    =====================================================
    9. バッテリー電圧
    =====================================================
  */

  uint32_t pin_millivolts =
      analogReadMilliVolts(BATTERY);


  float pin_voltage =
      pin_millivolts
      /
      1000.0f;


  float battery_voltage =
      pin_voltage
      *
      VOLTAGE_DIVIDER_RATIO;


  /*
    =====================================================
    10. ステータスパケット作成
    =====================================================
  */

  tx_data.header1 = HEADER1;
  tx_data.header2 = HEADER2;


  tx_data.left_position =
      (float)current_left_pos;


  tx_data.left_velocity =
      (float)current_left_vel;


  tx_data.right_position =
      (float)current_right_pos;


  tx_data.right_velocity =
      (float)current_right_vel;


  tx_data.battery_voltage =
      battery_voltage;


  /*
    =====================================================
    11. チェックサム
    =====================================================
  */

  size_t len =
      sizeof(StatusPacket) - 1;


  uint8_t* ptr =
      (uint8_t*)&tx_data;


  tx_data.checksum =
      calculateChecksum(
        ptr,
        len
      );


  /*
    =====================================================
    12. PCへ送信
    =====================================================
  */

  Serial.write(
    (uint8_t*)&tx_data,
    sizeof(StatusPacket)
  );


  /*
    =====================================================
    13. 時刻更新
    =====================================================

    prev_left_count / prev_right_countと
    この時刻が同じ周期で更新されるのが重要。
  */

  last_control_time =
      current_time;
}