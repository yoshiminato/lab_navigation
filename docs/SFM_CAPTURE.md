# lab_navigation での SfM 用画像収集

`~/visual_localization_ws/src/sfm_capture` の保存処理をこのワークスペースへ移植しました。
実機とシミュレーションで、PNG連番・時刻CSV・CameraInfo・整合性検査・bag保存を共有します。
元のワークスペースはそのまま使用できます。

## 入力を選ぶ

| source | 入力 | カメラドライバー | 時刻 |
| --- | --- | --- | --- |
| usb（既定） | USBカメラ | usb_cam を起動 | 実時刻 |
| topic | 配信済みの ROS 2 Image / CameraInfo | 起動しない | 実時刻 |
| simulation | Gazebo 等の ROS 2 Image / CameraInfo | 起動しない | /clock |

保存対象の既定値は `/camera/image_raw` と `/camera/camera_info` です。
`lab_navigation` の起動口は、補助bagの追加トピックを `/odom /mpu6050/imu` に設定します。
`sfm_capture capture.launch.py` を直接起動する場合は元の補助トピックの既定値を保持しています。
既存の `save_fps`、`warmup_sec`、`duration_sec`、`calibration`、`record_bag`、
`bag_images`、`writer_queue_size`、`bag_max_size` も利用できます。

## ビルド

```bash
cd ~/lab_navigation_ws
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
colcon build --packages-select usb_cam sfm_capture lab_navigation --allow-overriding usb_cam
source install/local_setup.bash
ros2 pkg prefix usb_cam
```

usb_cam の参照先が `~/lab_navigation_ws/install/usb_cam` になることを確認します。
移植したドライバーは公式0.8.1（コミット `cb2ae6bc0a312de629f84fef933fb3e07dac1ef8`）です。
移植元の説明にある時刻変換修正
`epoch_time.tv_usec / 1000.0 → epoch_time.tv_usec` を
`src/external/usb_cam/include/usb_cam/utils.hpp` に保持しています。
元のUSBソースディレクトリは空だったため、公式の同版から復元しました。
システムの `/opt/ros/humble` は変更していません。

## シミュレーションで保存

端末1で既存のシミュレーションを起動します。

```bash
source ~/lab_navigation_ws/install/local_setup.bash
ros2 launch lab_navigation diffbot_sim.launch.py
```

端末2で収集を起動します。Gazeboのカメラ、CameraInfo、clockは既存のbridgeを使います。

```bash
cd ~/lab_navigation_ws
source install/local_setup.bash
ros2 launch lab_navigation sfm_capture.launch.py \
  source:=simulation \
  output_dir:=$PWD/captures/sim_001 \
  save_fps:=2.0 duration_sec:=60.0
```

`output_dir` は存在しない新しいディレクトリを指定します。
省略すると、カレントディレクトリの `captures/日時/` が自動で使われます。
`duration_sec:=0.0`（既定）ならCtrl+Cまで保存します。
このlaunchを終了しても、別端末のGazeboや画像publisherは動き続けます。

- 画像はImage.header.stampを保持し、その時刻で最大2fpsへ間引きます。
- warmupと収集時間はシミュレーションの秒数です。Gazeboを一時停止すると経過時間も止まります。
- 0時刻の初期画像は保存せず、正の時刻が始まってから保存します。
- clockまたは画像時刻の逆行は失敗として記録します。ワールドのリセット後は新しい出力先で再起動します。
- bagにもシミュレーション時刻を使い、`/clock` を含めます。
- `no_frame_timeout_sec:=auto` はシミュレーションでタイムアウトを無効にします。
  必要なら `no_frame_timeout_sec:=30.0` のように実時間の無受信タイムアウトを明示できます。
  未配信トピックを指定したままの場合も既定では待ち続けるため、終了後は画像枚数を検査します。

保存が始まらない場合は、同じROS_DOMAIN_IDの端末で確認します。

```bash
ros2 topic info /camera/image_raw -v
ros2 topic echo /clock --once
```

## USBカメラで保存

```bash
ros2 launch lab_navigation sfm_capture.launch.py \
  source:=usb device:=/dev/video0 \
  output_dir:=$PWD/captures/usb_001
```

USBの解像度・露光・フォーカス設定は `src/sfm_capture/config/camera.yaml` を使います。
別設定は `camera_config:=/絶対パス/camera.yaml`、実機校正は
`calibration:=/絶対パス/calibration.yaml` で指定します。
USBドライバーはカメラの自動露光やフォーカス設定を書き込むため、撮影前に設定値を確認してください。
実機の無受信タイムアウトは既定15秒、warmupと収集時間は実時間です。

実機ですでにカメラノードが動いている場合は、そのpublisherを利用します。

```bash
ros2 launch lab_navigation sfm_capture.launch.py \
  source:=topic \
  image_topic:=/front_camera/image_raw \
  camera_info_topic:=/front_camera/camera_info
```

topicとsimulationではUSBデバイスやUSB設定ファイルを開かず、
publisherから受信したCameraInfoを保存・検査します。解像度やframe_idも入力に従います。
未配信のCameraInfoを新たに生成する処理はありません。
一般のtopic入力でもシミュレーション時刻を使いたい場合は `use_sim_time:=true` を指定できます。

## 保存物とSfM

```text
captures/sim_001/
├── images/00000001.png ...
├── timestamps.csv
├── session.json
├── capture_setup.json
├── camera_info.json       # 受信できた場合
├── bag/                   # record_bag:=true（既定）
└── bag_report.json
```

USB入力にはカメラ設定・コントロール値・指定した校正YAMLのスナップショットも残します。
PNG保存は元解像度を維持し、リサイズ・切り抜き・歪み補正を行いません。
寸法・encoding・frame_id・CameraInfoの途中変更、書き込みエラー、保存キューのあふれを検出します。

bagの既定はCameraInfo・TF・補助センサです。全画像もbagへ保存する場合は
`bag_images:=true`、PNGとメタデータだけなら `record_bag:=false` を指定します。
補助トピックは `extra_bag_topics:="/odom /mpu6050/imu"` で置き換えられます。
未配信の補助トピックは `missing_or_empty_topics` に出ますが、画像収集自体は継続します。

```bash
ros2 run sfm_capture check_dataset captures/sim_001
ros2 run sfm_capture export_bag captures/sim_001/bag captures/sim_export_001 --save-fps 1.0
```

export_bagはbag_images:=trueで記録した非圧縮Imageが必要です。
カスタムトピックの場合は `--image-topic` と `--camera-info-topic` も指定します。
ライブ収集のCSVのstamp_nsは入力時刻、received_nsはホストのUNIX受信時刻です。
bag書き出しのreceived_nsはbagの記録時刻です。
シミュレーションではこの2つの時計は異なり、差をセンサ遅延として解釈しません。
COLMAPに渡すのは `captures/sim_001/images/` です。
詳しいSfM手順は [COLMAP_SFM.md](COLMAP_SFM.md) を参照してください。
保存済みTFから画像ごとの位置・姿勢を抽出し、COLMAPへ登録する方法は
[POSE_PRIORS.md](POSE_PRIORS.md) を参照してください。

## 検証

2026-10-05の検証結果:

| 確認 | 結果 |
| --- | --- |
| usb_cam / sfm_capture / lab_navigation のビルド | 成功 |
| 既存33件とカスタムトピックの回帰テスト | 34件成功 |
| USB起動経路（カメラだけ合成publisherへ置換） | PNG9枚、bag確定、静的TF・IMU・odom保存 |
| 外部ROSトピック入力 | PNG8枚、カスタムImage/CameraInfo、bag確定 |
| シミュレーション時刻・加速・3秒一時停止 | PNG9枚、画像とbagがシミュレーション時刻、bag確定 |
| シミュレーション時計リセット | 逆行を検出してセッションをfailedとして終了 |
| シミュレーションbagから1fpsで再書き出し | 成功、整合性検査成功 |

USB実機は未接続です。Gazebo本体は起動せず、同じROS 2メッセージを使う合成publisherで検証しました。
制限のある実行環境ではROS_LOG_DIRを書き込み可能な場所へ設定し、ローカルDDS通信を許可して実行します。

```bash
cd ~/lab_navigation_ws
source install/local_setup.bash
PYTHONPATH="$PWD/src/sfm_capture:$PYTHONPATH" /usr/bin/python3 -m pytest -q tests/sfm_capture
ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/sfm_capture/smoke_capture.py --bag-images
ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/sfm_capture/smoke_topic.py
ROS_DOMAIN_ID=229 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/sfm_capture/smoke_topic.py --simulation
ROS_DOMAIN_ID=230 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/sfm_capture/smoke_topic.py --simulation --reset-clock
```

smoke_captureはUSBノードだけを合成publisherに置き換えます。
smoke_topicはlab_navigationのlaunchをそのまま使い、外部publisherから合成画像を配信します。
シミュレーション試験は加速したclock・3秒の一時停止・clock逆行・bagの時刻を検査します。
試験データは `validation/` に保存され、SfM地図には使えません。
USB実機の撮影品質や、実際のGazebo画像によるSfM復元品質は別途確認が必要です。
