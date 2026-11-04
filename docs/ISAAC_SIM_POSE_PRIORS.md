# Isaac Sim のカメラ ground truth で位置制約付き SfM を行う

この手順では、Isaac Sim の**撮影カメラ prim のワールド位置**を画像の撮影時刻に対応付けます。
ロボットの `/odom` や LiDAR localization の `map → odom` は ground truth として扱いません。
出力は既存の [POSE_PRIORS.md](POSE_PRIORS.md) と同じ `poses.csv` 形式で、
`import_pose_priors` と COLMAP 4.2 系の `pose_prior_mapper` をそのまま使えます。
以下は収集・復元の手順です。**Isaac Sim の実シーンにおける画像と姿勢の同期精度、
カメラ内部パラメータ、復元精度はまだ検証していません。**

```text
Isaac Sim: RGB Image + CameraInfo + /clock + camera prim の world 姿勢
                                       ↓ /sfm_gt_tf (専用 TF トピック)
sfm_capture_isaacsim.launch.py → images/ + timestamps.csv + bag/
                                       ↓ export_isaac_gt_poses
                                  poses.csv → import_pose_priors
                                            → matcher → pose_prior_mapper
```

## 1. Isaac Sim 側の設定

1. ROS 2 Bridge を有効にし、ROS 2 側と同じ `ROS_DOMAIN_ID` を使います。既存の
   Action Graph で `/clock` を Isaac Read Simulation Time から配信します。
2. SfM で使う**実際の撮影カメラ prim**から、非圧縮 RGB の `sensor_msgs/msg/Image` と
   `sensor_msgs/msg/CameraInfo` を配信します。両方は同じカメラ、解像度、frame ID にします。
   以下の例は `/camera/image_raw` と `/camera/camera_info` を使います。既存の名前が違えば
   収集 launch の `image_topic` / `camera_info_topic` を変更します。
   ROS2 Camera Helper（Camera Info Helper を使う場合はそれも）の `useSystemTime` は
   **false** にし、画像・CameraInfo のヘッダ時刻をシミュレーション時刻に揃えます。
   `/clock` が配信されていても、画像がシステム時刻なら TF とは対応しません。
   Isaac Sim 6.0 以降では、カメラの配信周期は sensor prim の
   `omni:sensor:tickRate` で設定し、helper の `frameSkipCount` は 0 にします。
3. そのカメラ prim の位置を、**専用**の `/sfm_gt_tf` (`tf2_msgs/msg/TFMessage`) に
   動的 TF として配信する Action Graph を追加します。ナビゲーション用 `/tf` とは別にします。
   カメラではなく `base_link` を対象にすると、カメラ取り付けオフセットだけ位置がずれます。

作業記録に Isaac Sim のバージョン、USD シーンの版、撮影カメラ prim のパス、
画像と CameraInfo のトピック・frame ID、GT の child frame、stage の長さ単位、
カメラ配信周期、ロボットの移動速度を残します。後から時刻ずれの影響を評価するためです。

時刻設定の根拠: [Camera Helper の `useSystemTime`](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.ros2.nodes/docs/ogn/OgnROS2CameraHelper.html)、
[6.0 以降のセンサ時刻と周期](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensor_timing_and_rtx_lidar_asset_migration.html)。

### Isaac Sim 6.0 以降の TF graph

`/World` の下に Action Graph を作り、次を配置します。

| ノード | 設定と接続 |
| --- | --- |
| On Playback Tick | `tick` → Isaac Compute Transform Tree の `execIn` |
| Isaac Read Simulation Time | `simulationTime` → ROS2 Publish Transform Tree の `timeStamp` |
| Isaac Compute Transform Tree | `targetPrims` に**撮影カメラ prim を1個だけ**指定。`parentPrim` は空欄にして world 基準にする |
| ROS2 Publish Transform Tree | `topicName=/sfm_gt_tf`, `staticPublisher=false`。Compute の `execOut`, `parentFrames`, `childFrames`, `translations`, `orientations` を同名入力へ接続 |
| ROS2 Context | 必要なら `context` を publisher へ接続。通常は既定の context でもよい |

### Isaac Sim 5.1 以前の TF graph

ROS2 Publish Transform Tree ノード自身の `targetPrims` に撮影カメラ prim を1個指定し、
`parentPrim` は空欄、`topicName=/sfm_gt_tf`、`staticPublisher=false` にします。
On Playback Tick の `tick` を `execIn` へ、Isaac Read Simulation Time の
`simulationTime` を `timeStamp` へ接続します。ROS2 Context は必要に応じて接続します。

両バージョンとも、カメラの相対姿勢ではなく、**`world → 撮影カメラ` の直接の動的 TF**が
出ていることを確認します。child frame 名は prim と graph の設定によって変わるため、
実際の `child_frame_id` を抽出コマンドの `--camera-frame` に指定します。
`ROS2 Publish Raw Transform Tree` を使う場合も、camera prim から計算した並進・回転と
simulationTime を入力してください。固定のゼロ変換は真値姿勢にはなりません。

Play を押した状態で、ROS 2 端末から確認します。

```bash
ros2 topic info /clock
ros2 topic info /camera/image_raw
ros2 topic info /camera/camera_info
ros2 topic info /sfm_gt_tf
ros2 topic echo /sfm_gt_tf --once
ros2 topic echo /camera/image_raw --field header --once
```

`/sfm_gt_tf` の `header.frame_id` が `world`、`child_frame_id` がカメラ用フレーム、
時刻が画像と同じシミュレーション時計で進むことを確認します。
画像の `header.frame_id` と真値 TF の child 名は異なっても構いませんが、
**同じ物理カメラ中心**を指すことを Stage 上で確認してください。
Stage の長さ単位と ROS のメートルが一致していることも確認します。

参考: [NVIDIA の TF 手順 (6.0 以降)](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_series/tutorial_ros2_tf.html)、
[5.1 の TF 手順](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.html)、
[カメラ画像配信](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/advanced_sensors/tutorial_ros2_camera_publishing.html)。

## 2. 画像と真値 TF を同じセッションに収集

```bash
cd ~/lab_navigation_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select sfm_capture lab_navigation
source install/local_setup.bash

sfm_capture_dir="$PWD/captures/isaac_001"
ros2 launch lab_navigation sfm_capture_isaacsim.launch.py \
  output_dir:="$sfm_capture_dir" \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  gt_tf_topic:=/sfm_gt_tf \
  save_fps:=2.0 duration_sec:=60.0
```

出力先は存在しないディレクトリにします。`duration_sec:=0.0` なら Ctrl+C まで収集します。
この launch は画像 PNG、ヘッダ時刻、CameraInfo、`/clock`、`/sfm_gt_tf` と
通常の `/tf`/`/tf_static` を保存します。画像と真値の対応には**画像のヘッダ時刻**を使い、
ROS bag の受信時刻は使いません。`/sfm_gt_tf` が記録されない場合、抽出は失敗します。

```bash
ros2 run sfm_capture check_dataset "$sfm_capture_dir"
python3 -m json.tool "$sfm_capture_dir/bag_report.json"
python3 -m json.tool "$sfm_capture_dir/camera_info.json"
```

`bag_report.json` の `/sfm_gt_tf` の `message_count` が正であること、
CameraInfo の `k[0]` と `k[4]` が正であることを確認します。

### 収集後の時刻検査

1. `timestamps.csv` の画像時刻、`/sfm_gt_tf` の各 `header.stamp`、`/clock` が
   いずれも同じシミュレーション時刻の範囲で、逆行なく進むことを確認します。
   `ros2 topic hz` は壁時計上の配信速度なので、シミュレーション時刻での周期と混同しません。
2. 第3節の抽出後、`poses_report.json` の `valid == images`、`missing == 0`、
   `tf.dynamic_edges` の親・子とサンプル数、`max_valid_tf_gap_sec` を確認します。
   欠測や補間間隔が大きい場合は GT の配信周期・bag の記録状態を先に調べます。
3. **この検査は時刻の対応可能性を示すだけ**です。Image ヘッダが実際の描画時刻を
   表すかは保証しません。まず静止中にカメラ prim のワールド位置と TF の値が一致するか、
   次に既知の速度で短距離を走らせ、既知位置の目印が写る画像とその時刻のカメラ姿勢を照合します。
   必要な精度に対してずれが無視できない場合は、Isaac Sim 内で同じレンダリングフレームの
   RGB・カメラ姿勢・時刻を一緒に保存する方式を検討します。

例えば 0.5 m/s で走行中に 33 ms の対応ずれがあると、並進だけで約 1.65 cm の差です。
これは実測値ではなく、`速度 × 時刻差` による影響の目安です。
[Isaac Sim の時刻モデル](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_multitick_rendering.html)も参照してください。

## 3. 各画像のカメラ中心位置を抽出

```bash
sfm_pose_dir="$sfm_capture_dir/poses_world_camera_gt"
ros2 run sfm_capture export_isaac_gt_poses \
  "$sfm_capture_dir" "$sfm_pose_dir" \
  --camera-frame sfm_gt_camera \
  --world-frame world --tf-topic /sfm_gt_tf \
  --position-std 0.05 0.05 0.05

python3 -m json.tool "$sfm_pose_dir/poses_report.json"
```

`sfm_gt_camera` は例です。実際の TF の `child_frame_id` に置き換えます。
抽出は、カメラフレームが専用トピック上で `world` の**直接の動的 child**であることを検査し、
一般の `/tf` や推定された `map → odom → base_link` へ自動的にフォールバックしません。
TF の時刻を画像時刻へ補間します。既定では補間に使う前後の TF 間隔が 0.2 秒を超える画像を
欠測とします。必要なら `--max-tf-gap-sec` を実際の配信周期に合わせて変更します。
欠測があるとコマンドは失敗終了し、行ごとの理由を CSV とレポートに残します。

`--position-std 0.05` は**例として選んだ 5 cm の標準偏差**です。
シミュレーション真値であっても画像時刻と露光時刻のずれ、カメラ prim と投影中心のずれ、
地図座標の対応に誤差があり得ます。実測・検証に応じて各軸の値を調整し、0 は指定しません。
この値は GT 自体の測定誤差を断定するものではなく、COLMAP に与える位置制約の
共分散と最適化の重みに関わります。研究比較では、同じ画像・特徴・照合条件で
例えば 0.01 m、0.05 m、0.20 m を別々の `poses.csv`・DB・復元先に設定して比較します。
一つの DB の事前情報を上書きしながら比較しないでください。
COLMAP へ登録するのは `x,y,z` のカメラ中心位置とその共分散です。
CSV のクォータニオンは記録しますが、このインポーターは向きの事前制約を登録しません。
クォータニオンの軸は真値 TF の child frame に従い、カメラ光学座標と一致するとは限りません。

## 4. COLMAP で位置制約付き SfM

[POSE_PRIORS.md の第3〜6節](POSE_PRIORS.md) をそのまま使います。主要コマンドは次です。
画像名を特徴抽出後に変更せず、COLMAP 4.2 系 CLI と対応する PyCOLMAP 4.2 系を使います。

```bash
sfm_colmap_dir="$sfm_capture_dir/colmap_gt"
mkdir -p "$sfm_colmap_dir/sparse"
camera_params="$(python3 - "$sfm_capture_dir/camera_info.json" <<'PY'
import json
import sys

info = json.load(open(sys.argv[1]))
k = info['k']
if k[0] <= 0 or k[4] <= 0 or any(abs(value) > 1e-12 for value in info['d']):
    raise SystemExit('This PINHOLE example requires positive fx/fy and zero distortion')
print(','.join(str(k[index]) for index in (0, 4, 2, 5)))
PY
)"

colmap feature_extractor \
  --database_path "$sfm_colmap_dir/features.db" \
  --image_path "$sfm_capture_dir/images" \
  --ImageReader.single_camera 1 \
  --ImageReader.camera_model PINHOLE \
  --ImageReader.camera_params "$camera_params" \
  --FeatureExtraction.use_gpu 0

ros2 run sfm_capture import_pose_priors \
  "$sfm_pose_dir/poses.csv" "$sfm_colmap_dir/features.db" \
  --output-database "$sfm_colmap_dir/database.db"

colmap spatial_matcher \
  --database_path "$sfm_colmap_dir/database.db" \
  --FeatureMatching.use_gpu 0 \
  --SpatialMatching.ignore_z 0 \
  --SpatialMatching.max_distance 3.0 \
  --SpatialMatching.max_num_neighbors 30 \
  --SpatialMatching.min_num_neighbors 0

colmap pose_prior_mapper \
  --database_path "$sfm_colmap_dir/database.db" \
  --image_path "$sfm_capture_dir/images" \
  --output_path "$sfm_colmap_dir/sparse" \
  --use_robust_loss_on_prior_position 1
```

`camera_params` は `camera_info.json` の `k[0],k[4],k[2],k[5]` から作ります。
PINHOLE は歪みなし画像の例です。Isaac の CameraInfo にレンズ歪みがある場合は
[COLMAP のカメラモデル](https://colmap.github.io/cameras.html)と画像の補正状態に合わせて変更します。
歪み係数がゼロでも、CameraInfo の K が実際の描画モデルと一致する保証はありません。
`check_dataset` で画像と CameraInfo の解像度・frame ID の一致を確認した後、
既知の寸法・位置の目印を複数の画像に写し、K とカメラ姿勢で投影した位置が
画像上の位置に合うか確認します。カメラの画角・解像度・投影方式を途中で変えた場合は
別セッションとして収集します。

CameraInfo が正確だと確認できた比較実験では、初回の復元で内部パラメータを固定すると
位置制約の影響を見やすくできます。別の出力先で次を実行します。

```bash
mkdir -p "$sfm_colmap_dir/sparse_fixed_intrinsics"
colmap pose_prior_mapper \
  --database_path "$sfm_colmap_dir/database.db" \
  --image_path "$sfm_capture_dir/images" \
  --output_path "$sfm_colmap_dir/sparse_fixed_intrinsics" \
  --use_robust_loss_on_prior_position 1 \
  --Mapper.ba_refine_focal_length 0 \
  --Mapper.ba_refine_principal_point 0 \
  --Mapper.ba_refine_extra_params 0
```

これらは手元の COLMAP 4.2.1 で `colmap pose_prior_mapper -h` に表示されます。
実行環境でも確認してください。主点の refinement はこの版では既定で 0、
焦点距離と追加パラメータは既定で 1 です。PINHOLE には歪みの追加パラメータはありません。
K が不正確なまま固定すると復元に誤差が入るため、固定・可変の結果も比較します。
参考: [COLMAP の Bundle Adjustment 設定](https://colmap.github.io/pycolmap/pycolmap.html)。
照合半径 3 m と近傍 30 枚も例です。`sequential_matcher` を併用する手順は
[POSE_PRIORS.md](POSE_PRIORS.md) を参照してください。

この SfM の座標系は Isaac Sim の `world` です。Nav2 の `map` で利用する場合は
`world` と `map` の変換を別途確定し、モデルまたは後段の位置をその変換で揃えます。
画像を出す graph と GT graph が別の camera prim を参照していた場合、
CSV の件数検査は通っても位置は正しくありません。最初は短い走行で軌跡と画像を目視照合します。

## 5. 研究用の比較・評価

GT を位置制約に使ったモデルを、**同じ画像の同じ GT との位置誤差だけで評価しない**でください。
位置事前情報が強ければ、画像特徴の対応が弱くてもカメラ位置だけは GT に近づけられます。
比較の際は画像・特徴抽出・照合条件・カメラ内部パラメータを揃え、各実験の DB と出力を分けます。
**画像特徴だけの基準実験**では `sequential_matcher` 等で作った照合結果を使います。
`spatial_matcher` は GT 位置で画像ペアを選ぶため、それを通常の `mapper` に渡しても
「GT を一切使わない」条件にはなりません。復元時の prior の効果を比べるときは
同じ照合結果を両 mapper に渡し、位置を使ったペア選択の効果は別の実験で比較します。

| 条件 | 目的 |
| --- | --- |
| 通常の `mapper`、GT prior なし | 画像特徴による復元の基準性能 |
| `pose_prior_mapper`、GT カメラ位置 | 位置制約の効果 |
| `pose_prior_mapper`、同じ GT に人工ノイズを加えた位置 | 位置誤差への頑健性 |
| GT 位置を同じにして位置標準偏差を変更 | 制約の重みへの感度 |

登録画像率、再投影誤差、復元モデルの接続・3D 点数を記録します。
カメラ位置誤差は、**prior に使わない画像や別走行の GT**で評価します。
通常の `mapper` のモデルは尺度と座標が任意なので、評価前に位置合わせが必要です。
位置合わせに使う画像と評価画像を分け、同じ変換を評価画像へ適用します。
3D 点の絶対精度を主張する場合は、対応する既知の 3D 点やシーン形状との比較が別途必要です。

今回の ROS 経由方式は、既存の収集・COLMAP パイプラインを使うための出発点です。
画像と姿勢の同期誤差が要求精度を超えると判明した場合は、Isaac Sim 内部で同じ描画フレームから
画像・カメラ姿勢・内部パラメータを一緒に保存する方式へ切り替えます。
