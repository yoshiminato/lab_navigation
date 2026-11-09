# Isaac Sim pose prior SfMからVisual LocalizationとNav2へ接続する

この文書は、Isaac Simのカメラ真値に沿って作成したSfMモデルを、
HLocのオンラインVisual LocalizationとNav2へ接続する現在の運用手順です。
作業順とSIFT / SuperPoint + LightGlueの選択は
[VISUAL_LOCALIZATION_WORKFLOW.md](VISUAL_LOCALIZATION_WORKFLOW.md)、文書全体の索引は
[README.md](README.md) を参照してください。

## 1. 処理の全体像

```text
Isaac Sim RGB + CameraInfo + camera world pose + /clock
    -> sfm_capture_isaacsim.launch.py
    -> images/ + poses_world_camera_gt/poses.csv
    -> COLMAP pose_prior_mapper
    -> maps/sift/sparse/0
    -> COLMAP SIFTをHLoc形式へ変換
    -> maps/sift/reference_features.h5
    -> 参照画像のNetVLADを抽出
    -> retrieval/netvlad.h5
    -> hloc_localizer
    -> /hloc/camera_pose
    -> hloc_tf_publisher
       T_map_camera = T_map_world * T_world_camera
    -> map -> odom -> base_link
    -> Nav2
```

画像収集とpose prior付きSfMまでは、次の順に参照します。

1. [SFM_CAPTURE.md](SFM_CAPTURE.md)
2. [ISAAC_SIM_POSE_PRIORS.md](ISAAC_SIM_POSE_PRIORS.md)
3. [POSE_PRIORS.md](POSE_PRIORS.md)

## 2. 作業パスを設定

各作業端末で最初に次を実行し、データセット名は `SESSION_NAME` だけ変更します。
新しい端末では変数を再設定してください。シェル変数は端末間では共有されません。

```bash
WORKSPACE_DIR="$HOME/lab_navigation_ws"
SESSION_NAME="isaac_001"
CAPTURE_DIR="$WORKSPACE_DIR/captures/$SESSION_NAME"
SIFT_DIR="$CAPTURE_DIR/maps/sift"
SUPERPOINT_DIR="$CAPTURE_DIR/maps/superpoint"
IMAGE_DIR="$CAPTURE_DIR/images"

test -d "$IMAGE_DIR" || { echo "images not found: $IMAGE_DIR"; exit 1; }
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

新しいセッションを使う場合は、収集先の `SESSION_NAME` に変更します。
収集をホームディレクトリから開始した場合は、`CAPTURE_DIR="$HOME/captures/$SESSION_NAME"` とします。

## 3. 現在のIsaac Simデータ

現在のデータセットは次です。

```text
captures/isaac_001/
├── images/                         908画像
├── poses_world_camera_gt/
├── maps/
│   ├── sift/
│   │   ├── database.db
│   │   ├── sparse/0/               908登録画像、190,006点
│   │   └── reference_features.h5   SIFT参照特徴量
│   └── superpoint/
│       ├── database.db
│       ├── sparse/0/               908登録画像、110,578点
│       └── reference_features.h5   SuperPoint参照特徴量
└── retrieval/netvlad.h5           共通の検索特徴量
```

両方式に必要なオンライン参照データはすでに存在します。
HLocの`data_path`にはセッションルートの `$CAPTURE_DIR` を指定します。
`isaac_gt_test_002` は SIFT のみなので、SuperPointを選ぶ場合は先にその地図を作ります。

## 4. 仮想環境を使う場面

HLoc環境の初回構築は
[lab_navigation_ws_setup_guide.md](../src/hloc_localizer/lab_navigation_ws_setup_guide.md)の
第4〜7節を使用します。

| 作業 | `.venv`のactivate | 理由 |
| --- | --- | --- |
| COLMAP特徴量のHLoc変換 | 必要 | `h5py`, `pycolmap`を使用 |
| NetVLAD・SuperPoint・LightGlue生成 | 必要 | HLocとPyTorchを使用 |
| オフラインlocalization | 必要 | HLoc Python APIを直接使用 |
| `colcon build` | 不要。activate済みなら`deactivate` | ROS 2のビルド環境を使用 |
| `ros2 run hloc_localizer ...` | 不要 | パッケージのラッパーが`.venv/bin/python3`を直接使用 |
| `navigation_isaacsim_visual.launch.py` | 不要 | 同じラッパー経由で起動 |

オフライン処理では次の順に読み込みます。

```bash
source "$WORKSPACE_DIR/.venv/bin/activate"
```

環境確認:

```bash
python3 - <<'PY'
import h5py
import hloc
import pycolmap
import torch

print('HLoc: OK')
print('PyCOLMAP:', pycolmap.__version__)
print('Torch CUDA:', torch.cuda.is_available())
PY
```

## 5. SfM後に追加で必要なHLoc参照データ

オンラインSIFT方式は、SfMモデルだけでは起動できません。次の3種類が必要です。

| パス（`data_path`基準） | 用途 |
| --- | --- |
| `maps/sift/sparse/0/` | SIFTのカメラ姿勢、内部パラメータ、3D点 |
| `maps/sift/reference_features.h5` | 登録済み参照画像のSIFT特徴量と3D点の対応 |
| `retrieval/netvlad.h5` | オンライン画像から参照候補を検索する共通の大域特徴量 |

現在は生成済みなので、SfMモデルや登録画像を変更しない限り再生成は不要です。
再生成する場合は、既存ファイルを上書きせず新しい出力先で検証してから切り替えます。

### 5.1 SIFT参照特徴量とNetVLADを生成・検証

```bash
source "$WORKSPACE_DIR/.venv/bin/activate"
python3 script/prepare_localization.py "$CAPTURE_DIR" --method sift
```

### 5.2 SuperPoint参照データを検証

```bash
python3 script/prepare_localization.py "$CAPTURE_DIR" --method superpoint --verify-only
```

SuperPointの地図がまだないセッションでは、この検証はスキップします。
作成手順は[SuperPoint SfM](../src/hloc_localizer/README_hloc_superpoint_colmap_sfm.md)を参照してください。

初回はNetVLADのモデル重みを取得するため、ネットワーク接続と保存領域が必要です。
query側のNetVLADファイルやpairファイルはオフライン評価では必要ですが、
オンラインノードは受信画像のNetVLADをその場で計算するため必須ではありません。

詳細なSIFTオフライン検証は
[lab_navigation_ws_setup_guide.md](../src/hloc_localizer/lab_navigation_ws_setup_guide.md)の
第10〜20節を参照してください。

## 6. ビルド

```bash
deactivate 2>/dev/null || true
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash

colcon build --packages-select hloc_localizer lab_navigation \
  --symlink-install

source install/setup.bash
```

## 7. 段階的に起動する

最初はNav2の自律移動を無効にします。Isaac SimをPlayしてから実行します。

```bash
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

ros2 launch lab_navigation navigation_isaacsim_visual.launch.py \
  start_navigation:=false \
  local_method:=sift \
  data_path:="$CAPTURE_DIR"
```

SuperPoint地図を使う場合は、上の起動を終了してから方式を変えます。

```bash
ros2 launch lab_navigation navigation_isaacsim_visual.launch.py \
  start_navigation:=false \
  local_method:=superpoint \
  data_path:="$CAPTURE_DIR"
```

`world -> map`補正は`map_name`に対応する次のファイルから読み込みます。

```text
src/lab_navigation/map/warehouse_multiple_shelves/world_to_map.yaml
```

現在の内容は次の暫定値です。

```yaml
hloc_tf_publisher:
  ros__parameters:
    world_to_map_x: 5.0
    world_to_map_y: 10.0
    world_to_map_z: 0.0
    world_to_map_yaw_deg: 0.0
```

別の補正ファイルを試す場合は、元ファイルを上書きせず次のように指定します。

```bash
ros2 launch lab_navigation navigation_isaacsim_visual.launch.py \
  start_navigation:=false \
  alignment_yaml:=/absolute/path/to/world_to_map.yaml
```

`(+5,+10,0), yaw=0°`は原点不一致を確認するための暫定値です。最終値は
Isaac `world`とGrid Map `map`の同じ基準点・方位から校正します。

HLoc localizer、HLoc TF、障害物検出、Nav2、RVizはGNOME Terminalの
個別タブで起動します。`start_hloc:=false`、`start_navigation:=false`、
`start_rviz:=false`を指定したサブシステムのタブは起動しません。

## 8. 起動後の確認順序

### 8.1 入力トピック

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /velodyne_points
ros2 topic hz /odom
ros2 topic echo /clock --once
```

### 8.2 HLocノードと出力

```bash
ros2 node list | grep hloc
ros2 topic info /hloc/camera_pose -v
ros2 topic echo /hloc/camera_pose --once
ros2 topic echo /hloc/localization_status --once
```

`/hloc/camera_pose`のPublisher countが0なら、HLoc localizerタブの例外を確認します。
よくある原因は、`data_path`が地図フォルダや`sparse/0`を指していること、
HLoc参照ファイルが不足していること、Python依存関係の不整合です。

### 8.3 TF

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

TFの関係は次です。

```text
map --hloc_tf_publisher--> odom --wheel odometry--> base_link
                                              └--> camera_optical_frame
```

`map -> odom`はHLocだけから配信します。AMCL、LiDAR localization、別のglobal EKFから
同じTFを同時に配信しません。`/sfm_gt_tf`は評価専用で、Nav2へ接続しません。

### 8.4 Nav2を有効化

静止状態と低速手動走行で位置関係を確認した後、`start_navigation:=true`で再起動します。
RVizでoccupancy map、robot footprint、LiDAR障害物、推定位置が重なることを確認してから
短い目標を与えます。

## 9. 精度評価

TFが存在するだけでは推定精度を判定できません。HLocとGround Truthを比較するときは、
次を揃えます。

- 座標系: `T_map_world`を適用し、両方を`map`座標にする
- 対象: カメラ同士、または`base_link`同士で比較する
- 時刻: 画像の`header.stamp`と同じシミュレーション時刻を使う
- 指標: XY誤差、Yaw誤差、RMSE、中央値、95パーセンタイル、成功率、更新頻度

動作中に最新GTと古いHLoc推定を比較すると、位置誤差に処理遅延分の移動量が混ざります。
最初はNav2を止め、静止、低速直進、旋回、経路走行の順に評価します。

## 10. 切り分けコマンド

```bash
ros2 run tf2_ros tf2_monitor map odom
ros2 topic hz /tf
ros2 topic hz /camera/image_raw
ros2 topic hz /velodyne_points
ros2 node info /hloc_localizer
ros2 run tf2_tools view_frames
```

| 症状 | 最初に確認する箇所 |
| --- | --- |
| `/hloc/camera_pose`のPublisherが0 | HLoc localizerタブ、`ros2 node list`、実行ファイル |
| HLocは起動するがすぐ終了 | `data_path`と第5節の参照成果物 |
| HLoc poseが更新されない | 画像のHz、QoS、localization status、候補画像数 |
| `map -> odom`がない | HLoc pose、`odom -> base_link`、カメラ固定TF |
| TFが揺れる・競合する | `tf2_monitor`で`map -> odom`の配信元を確認 |
| 地図上で一定量ずれる | `world_to_map.yaml`の4パラメータを校正 |
| 移動時だけ誤差が増える | 画像時刻、TF時刻、処理遅延を同時に確認 |

HLocノード単体の詳細は
[README_HLoc_ROS2.md](../src/hloc_localizer/README_HLoc_ROS2.md)を参照してください。
