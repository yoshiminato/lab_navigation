# micro-ROSへの移行（ROS 2 Humble / ESP32）

作業ブランチ: `feature/microros-migration`

PCの`DiffBotSystemHardware`とマイコン間の独自シリアルパケットを、micro-ROSの
トピック通信へ置き換えた。USB配線は従来どおり利用する。シリアルポートを開くのは
micro-ROS Agentのみ。エンコーダ計算・PID・PWMはマイコン、odomと
`odom -> base_link`のTFは`diff_drive_controller`が担当する。

## 起動

マイコンに**新しいファームウェアを書き込んでから**起動する。旧独自パケットの
ファームウェアでは接続できない。Arduinoのシリアルモニタは閉じる。

このPCにあるAgentワークスペースを使う場合:

```bash
source /opt/ros/humble/setup.bash
source ~/micro_ros_ws/install/setup.bash
cd ~/lab_navigation_ws
colcon build --symlink-install --packages-select ros2_control_diff_drive lab_navigation
source install/setup.bash
ros2 launch ros2_control_diff_drive diffbot.launch.py gui:=false serial_port:=/dev/ttyUSB0
```

Agentはlaunchが自動起動する。実際に追加したコマンド相当は:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```

既にAgentを別ターミナルで起動している場合は、二重起動しないよう次を指定する:

```bash
ros2 launch ros2_control_diff_drive diffbot.launch.py gui:=false start_micro_ros_agent:=false
```

navigation側も同じlaunchをincludeするためAgentが起動する。
**navigationとdiffbotを同時に別々に起動しないこと。**

```bash
ros2 launch lab_navigation navigation.launch.py serial_port:=/dev/ttyUSB0
```

モックではAgentを起動せず、実機にも接続しない:

```bash
ros2 launch ros2_control_diff_drive diffbot.launch.py gui:=false use_mock_hardware:=true
```

Agentの存在確認は`ros2 pkg prefix micro_ros_agent`。
別PCではHumble向けAgentを導入し、そのワークスペースをsourceする。
`/dev/serial/by-id/...`も`serial_port`に指定できる。
`serial_baudrate`はマイコン側の`MICRO_ROS_SERIAL_BAUD`と一致させる（標準921600）。
IMU統合前の115200 baud版ファームウェアとは速度が異なるため、PCだけを更新せず
マイコンも書き換える。ライブラリの既定transportは115200固定なので、スケッチ内で
921600 baudのopen/read/write/close callbackを登録している。
`navigation.launch.py`からも`serial_baudrate:=921600`を指定できる。

## マイコン

対象コード: `src/lab_navigation/arduino/micro_ros_client.ino`

コンパイル確認済みの組み合わせ:

- ボード指定: `esp32:esp32:esp32`（ESP32 Dev Module）
- Arduino-ESP32: **2.0.9**（従来の`ledcSetup`/`ledcAttachPin` API）
- micro_ros_arduino: **v2.0.8-humble**
- Arduino CLI: 1.3.0

公式ライブラリ:
https://github.com/micro-ROS/micro_ros_arduino/releases/tag/v2.0.8-humble

Arduino IDEでは同名フォルダ`micro_ros_client/`に`.ino`を置き、Humble版ライブラリと
ESP32 2.xのボードパッケージでビルド・書き込みする。ESP32 3.xへのAPI移行は含まない。
既存の`~/Arduino/libraries/micro_ros_arduino`はKilted版だったため、Humble版の検証は
一時ディレクトリのライブラリを指定して実施し、既存ライブラリは変更していない。

CLIでのビルド例（`/path/to/micro_ros_arduino`は展開済みHumble版ライブラリ）:

```bash
mkdir -p /tmp/microros-build/micro_ros_client
cp src/lab_navigation/arduino/micro_ros_client.ino /tmp/microros-build/micro_ros_client/
arduino-cli compile --fqbn esp32:esp32:esp32 \
  --library /path/to/micro_ros_arduino /tmp/microros-build/micro_ros_client
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 \
  /tmp/microros-build/micro_ros_client
```

このPCでsnap経由の`arduino-cli`が既存のボードパッケージを見つけない場合は、
上記の`arduino-cli`部分を次で置き換える（検証に使用したCLI実体）:

```bash
/snap/arduino-cli/current/usr/bin/arduino-cli --config-file ~/.arduinoIDE/arduino-cli.yaml
```

マイコン側`MICRO_ROS_DOMAIN_ID`は既定値0。PCの`ROS_DOMAIN_ID`が0以外なら
同じ値に変更して再ビルドする。モータ・エンコーダ・電圧計測ピンとPIDゲインは
従来値を引き継いだ。起動時はPWMゼロ。

## 通信仕様

| トピック | 型 | 内容 | QoS / 周期 |
|---|---|---|---|
| `/mcu/wheel_commands` | `std_msgs/Float64MultiArray` | `data=[左,右]` rad/s、layoutは空 | best effort / volatile / depth 1、20 Hz |
| `/mcu/wheel_states` | `sensor_msgs/JointState` | name、position rad、velocity rad/s、左右2輪 | best effort / volatile / depth 1、20 Hz |
| `/mpu6050/imu` | `sensor_msgs/Imu` | 加速度 m/s²・角速度 rad/s、frame_id=`imu_link` | best effort / volatile / depth 1、目標50 Hz |
| `/battery_level` | `std_msgs/Float32` | 電圧 V（パーセントではない） | reliable / volatile、1 Hz |

輪名は`left_wheel_joint`、`right_wheel_joint`。PCは受信配列の並びに依存せず名前で対応付ける。
JointStateのheader時刻はAgentとの時刻同期が成功した場合の発行時刻、失敗時は0。
PCの鮮度判定はsteady clockの受信時刻で行う。
マイコンのメッセージ配列・文字列バッファは静的確保する。

`/joint_states`は引き続きjoint_state_broadcasterが発行する。マイコンは`/odom`やTFを
発行しない。既存の`/cmd_vel_slow`入力と`/odom`へのremapを維持。
`odom_baselink_tf_publisher`による同じTFの二重発行は避ける。

controller_managerの更新は従来の10 Hzのまま。odomの`publish_rate: 50.0`は上限設定で、
新しい状態の更新が50 Hzになるわけではない。周期のチューニングは実機測定後に行う。

## 起動待ちと停止

- hardware activate時に最大20秒、最初の有効な車輪状態を待つ。
- マイコンのPIDとPWMは通信処理から独立したFreeRTOSタスクで50 msごとに更新。
- マイコンの指令受信が500 ms途絶えたら、次の制御周期でPWMゼロ・PID積分値リセット。
- PCは状態またはros2_controlのwrite更新が500 ms途絶えたらゼロ指令へ切り替える。
- 不正な配列、輪名の不一致、NaN/Infの状態は鮮度を更新しない。
- マイコンはNaN/Inf、要素数が2以外、絶対値20 rad/s超の指令を受け付けず停止する。
- 大きなエンコーダ位置の不連続もPCでエラーにする（上限20 rad/sと0.1 radの余裕）。
- MCUはAgent再接続を試行するが、PC側の通信エラーはラッチし自動走行再開しない。
  接続を確認してbringupを再起動する。起動時の車輪位置を原点とし、初期の累積角度で
  odomが飛ばないようにしている。
- MCUリセット時は累積カウントもリセットされる。小さなリセットを位置差だけで完全に
  判別することはできないため、書き込み・リセット時にはbringupも再起動する。

タイムアウトとトピック名は`description/ros2_control/diffbot.ros2_control.xacro`の
hardwareパラメータで指定する。輪名やトピックを変える場合はファームウェア側も揃える。

## 検証

```bash
colcon test --packages-select ros2_control_diff_drive
colcon test-result --verbose
ros2 topic echo /mcu/wheel_states --qos-reliability best_effort
ros2 topic echo /mpu6050/imu --qos-reliability best_effort
ros2 topic hz /mpu6050/imu
ros2 topic echo /battery_level
ros2 topic echo /odom
```

`test_microros_transport_launch.py`は実際のhardware pluginを読み込み、模擬MCUのROS 2
トピックで指令、逆順の輪名、初期位置、odom、不正データによる停止を確認する。
USB/XRCE通信、実モータ、電圧計測、物理的な通信断の試験は実機での確認が必要。
初回は車輪を浮かせて方向・左右対応と停止動作を確認する。

## MPU-6050 IMU

提供されたコードを同じ`diffbot_mcu`ノードに統合した。

- I2C: SDA=GPIO21、SCL=GPIO22、アドレス`0x68`。
- 起動時にWHO_AM_Iを確認し、スリープを解除する。
- 加速度レンジ±2 g、ジャイロレンジ±250 deg/sを明示設定する。
- 加速度バイアス[g]: `[-0.0616, -0.0814, 0.1918]`。
- ジャイロバイアス[deg/s]: `[-1.30, -1.90, -0.75]`。
- 14バイトを一括取得し、温度2バイトを飛ばしてジャイロを読む。
- 加速度は16384 LSB/gからバイアスを引いて9.80665倍、角速度は131 LSB/(deg/s)
  からバイアスを引いてπ/180倍する。軸の向きと重力成分は提供コードどおり。
- 姿勢推定をしていないため`orientation_covariance[0] = -1`とする。
  加速度・角速度の共分散は不明を示すゼロ行列とし、架空の分散を設定しない。
- Agentとの時刻同期後、読み取り完了時のエポック時刻をstampに入れる。
  初回同期失敗時は5秒ごとに再試行し、同期成功までIMUをpublishしない。
- I2Cタイムアウトは5 ms。読み取り不足やNACK時はそのサンプルを破棄し、1秒ごとに
  再初期化を試す。IMU未接続でも車輪通信を継続する。
- Agent再接続時はIMU publisherも再作成する。PIDは別タスクで継続する。

IMUメッセージは約320バイトあり、50 Hzで約16 kB/sとなる。115200 baud・8N1の
理論最大11.52 kB/sをIMU単独で超えるので、車輪通信も含め921600 baudへ変更した。
50 Hzは目標周期であり、Agent確認・時刻同期・USBやI2C遅延による揺らぎは実機で測定する。
受信側はSensorDataQoSなどbest effortに対応したQoSを使う。

`imu_link`はセンサー座標系。取り付け位置・姿勢が未指定のため、`base_link -> imu_link`
のTFは追加していない。IMUを自己位置推定で使う際には実際の取り付けに合わせて設定する。
現在の`localization.yaml`の`use_imu: false`は維持しており、今回の変更でIMUをodomに
融合する処理は追加していない。

IMUの値・周期、USBの921600 baud通信、センサー切断・再接続は実機未検証。

参考:
- [sensor_msgs/Imu仕様](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Imu.html)
- [MPU-6050レジスタ資料](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf)
