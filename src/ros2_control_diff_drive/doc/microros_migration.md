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
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
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
`serial_baudrate`はマイコン側のtransportと一致させる（標準115200）。

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
ros2 topic echo /battery_level
ros2 topic echo /odom
```

`test_microros_transport_launch.py`は実際のhardware pluginを読み込み、模擬MCUのROS 2
トピックで指令、逆順の輪名、初期位置、odom、不正データによる停止を確認する。
USB/XRCE通信、実モータ、電圧計測、物理的な通信断の試験は実機での確認が必要。
初回は車輪を浮かせて方向・左右対応と停止動作を確認する。

## IMU統合の状態

依頼文にIMUのコード本文が含まれていなかったため、IMUはまだ統合していない。
使用ライブラリ、接続ピン、センサーの型番、発行データを確認したうえで同じ
micro-ROS nodeにpublisherと読み取り周期を追加する。架空のIMU値は発行しない。
