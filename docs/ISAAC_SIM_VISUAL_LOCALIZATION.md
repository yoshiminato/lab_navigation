# Isaac Sim pose prior SfMからVisual LocalizationとNav2へ接続する

この文書は、Isaac Simのカメラ真値に沿って作成したSfMモデルを、
HLocのオンラインVisual LocalizationとNav2へ接続する現在の運用手順です。
文書全体の関係は [README.md](README.md) を参照してください。

## 1. 処理の全体像

```text
Isaac Sim RGB + CameraInfo + camera world pose + /clock
    -> sfm_capture_isaacsim.launch.py
    -> images/ + poses_world_camera_gt/poses.csv
    -> COLMAP pose_prior_mapper
    -> colmap_gt/sparse/0
    -> COLMAP SIFTをHLoc形式へ変換
    -> hloc_output/feats-shift.h5
    -> 参照画像のNetVLADを抽出
    -> hloc_output/global-ref-netvlad.h5
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

## 2. 現在のIsaac Simデータ

現在のデータセットは次です。

```text
captures/isaac_gt_test_002/
├── images/                         604画像
├── poses_world_camera_gt/
└── colmap_gt/
    ├── database.db
    ├── sparse/0/                   602登録画像、97,408点
    └── hloc_output/
        ├── feats-shift.h5          602登録画像分
        └── global-ref-netvlad.h5   NetVLAD参照特徴量
```

SIFT方式に必要なオンライン参照データはすでに存在します。
HLocの`data_path`には、収集ルートや`sparse/0`ではなく、次を指定します。

```text
/home/user/lab_navigation_ws/captures/isaac_gt_test_002/colmap_gt
```

現在のIsaacデータには`hloc_output_superpoint_lightglue/`がないため、
`local_method:=superpoint_lightglue`を選ぶ前にSP + LG用モデルを別途作成する必要があります。

## 3. 仮想環境を使う場面

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
cd ~/lab_navigation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate
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

## 4. SfM後に追加で必要なHLoc参照データ

オンラインSIFT方式は、SfMモデルだけでは起動できません。次の3種類が必要です。

| パス（`data_path`基準） | 用途 |
| --- | --- |
| `sparse/0/` | カメラ姿勢、内部パラメータ、3D点 |
| `hloc_output/feats-shift.h5` | 登録済み参照画像のSIFT特徴量と3D点の対応 |
| `hloc_output/global-ref-netvlad.h5` | オンライン画像から参照候補を検索する大域特徴量 |

現在は生成済みなので、SfMモデルや登録画像を変更しない限り再生成は不要です。
再生成する場合は、既存ファイルを上書きせず新しい出力先で検証してから切り替えます。

### 4.1 COLMAP SIFTをHLoc形式へ変換

```bash
cd ~/lab_navigation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate

export capture_path="$PWD/captures/isaac_gt_test_002"
export data_path="$capture_path/colmap_gt"
export image_dir="$capture_path/images"
export out="$data_path/hloc_output"

mkdir -p "$out"

python3 script/colmap_db_to_hloc.py \
  --database "$data_path/database.db" \
  --model "$data_path/sparse/0" \
  --output "$out/feats-shift.h5"
```

### 4.2 登録済み画像一覧を作る

```bash
python3 - <<'PY'
import os
from pathlib import Path
import pycolmap

data = Path(os.environ['data_path'])
out = Path(os.environ['out'])
model = pycolmap.Reconstruction(data / 'sparse/0')
names = sorted(image.name for image in model.images.values())
(out / 'reference.txt').write_text('\n'.join(names) + '\n')
print('registered images:', len(names))
PY
```

### 4.3 参照画像のNetVLADを生成

```bash
python3 -m hloc.extract_features \
  --image_dir "$image_dir" \
  --export_dir "$out" \
  --conf netvlad \
  --image_list "$out/reference.txt" \
  --feature_path "$out/global-ref-netvlad.h5" \
  --as_half
```

初回はNetVLADのモデル重みを取得するため、ネットワーク接続と保存領域が必要です。
query側のNetVLADファイルやpairファイルはオフライン評価では必要ですが、
オンラインノードは受信画像のNetVLADをその場で計算するため必須ではありません。

詳細なSIFTオフライン検証は
[lab_navigation_ws_setup_guide.md](../src/hloc_localizer/lab_navigation_ws_setup_guide.md)の
第10〜20節を参照してください。

## 5. ビルド

```bash
deactivate 2>/dev/null || true
cd ~/lab_navigation_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select hloc_localizer lab_navigation \
  --symlink-install

source install/setup.bash
```

## 6. 段階的に起動する

最初はNav2の自律移動を無効にします。Isaac SimをPlayしてから実行します。

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 launch lab_navigation navigation_isaacsim_visual.launch.py \
  start_navigation:=false \
  local_method:=sift \
  data_path:=/home/user/lab_navigation_ws/captures/isaac_gt_test_002/colmap_gt
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

## 7. 起動後の確認順序

### 7.1 入力トピック

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /velodyne_points
ros2 topic hz /odom
ros2 topic echo /clock --once
```

### 7.2 HLocノードと出力

```bash
ros2 node list | grep hloc
ros2 topic info /hloc/camera_pose -v
ros2 topic echo /hloc/camera_pose --once
ros2 topic echo /hloc/localization_status --once
```

`/hloc/camera_pose`のPublisher countが0なら、HLoc localizerタブの例外を確認します。
よくある原因は、`data_path`が収集ルートまたは`sparse/0`を指していること、
HLoc参照ファイルが不足していること、Python依存関係の不整合です。

### 7.3 TF

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

### 7.4 Nav2を有効化

静止状態と低速手動走行で位置関係を確認した後、`start_navigation:=true`で再起動します。
RVizでoccupancy map、robot footprint、LiDAR障害物、推定位置が重なることを確認してから
短い目標を与えます。

## 8. 精度評価

TFが存在するだけでは推定精度を判定できません。HLocとGround Truthを比較するときは、
次を揃えます。

- 座標系: `T_map_world`を適用し、両方を`map`座標にする
- 対象: カメラ同士、または`base_link`同士で比較する
- 時刻: 画像の`header.stamp`と同じシミュレーション時刻を使う
- 指標: XY誤差、Yaw誤差、RMSE、中央値、95パーセンタイル、成功率、更新頻度

動作中に最新GTと古いHLoc推定を比較すると、位置誤差に処理遅延分の移動量が混ざります。
最初はNav2を止め、静止、低速直進、旋回、経路走行の順に評価します。

## 9. 切り分けコマンド

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
| HLocは起動するがすぐ終了 | `data_path`と第4節の3成果物 |
| HLoc poseが更新されない | 画像のHz、QoS、localization status、候補画像数 |
| `map -> odom`がない | HLoc pose、`odom -> base_link`、カメラ固定TF |
| TFが揺れる・競合する | `tf2_monitor`で`map -> odom`の配信元を確認 |
| 地図上で一定量ずれる | `world_to_map.yaml`の4パラメータを校正 |
| 移動時だけ誤差が増える | 画像時刻、TF時刻、処理遅延を同時に確認 |

HLocノード単体の詳細は
[README_HLoc_ROS2.md](../src/hloc_localizer/README_HLoc_ROS2.md)を参照してください。
