# lab_navigation_ws 環境構築手順書

## 対象

別PCで `lab_navigation_ws` を clone 済みの状態から、HLoc + ROS 2 Humble + Visual Localization の動作確認環境を構築する。

前提:

- Ubuntu
- ROS 2 Humble
- Python 3.10
- ワークスペース: `~/lab_navigation_ws`
- Visual Localization 単独試験では HLoc が `map -> odom` を発行する
- `lidar_localization_ros2` と HLoc から `map -> odom` を同時配信しない

---

## 1. リポジトリ更新

```bash
cd ~/lab_navigation_ws

git status
git branch
git pull
```

開発ブランチを使う場合:

```bash
git checkout feature/hloc-nav2-integration
git pull
```

Submodule がある場合:

```bash
git submodule update --init --recursive
```

---

## 2. ROS 2 環境を読み込む

```bash
source /opt/ros/humble/setup.bash
```

確認:

```bash
ros2 --help
```

---

## 3. ROS 依存パッケージをインストール

```bash
cd ~/lab_navigation_ws

sudo apt update
rosdep update

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

`rosdep` で解決できないカスタム依存がある場合は個別にインストールする。

---

## 4. HLoc を配置する

HLoc は ROS パッケージではないため、`src/external/` ではなく `third_party/` に置く。

推奨構成:

```text
lab_navigation_ws/
├── .venv/
├── src/
│   ├── hloc_localizer/
│   ├── lab_navigation/
│   └── external/
├── third_party/
│   └── Hierarchical-Localization/
├── script/
│   └── colmap_db_to_hloc.py
├── captures/
├── build/
├── install/
└── log/
```

HLoc がまだ無い場合:

```bash
cd ~/lab_navigation_ws
mkdir -p third_party
cd third_party

git clone --recursive \
  https://github.com/cvg/Hierarchical-Localization.git

cd ~/lab_navigation_ws
```

確認:

```bash
ls third_party/Hierarchical-Localization
```

---

## 5. Python 仮想環境を作成

ROS 2 の `rclpy` や `cv_bridge` も同じ Python から利用できるよう、`--system-site-packages` を付ける。

```bash
cd ~/lab_navigation_ws

python3 -m venv \
  --system-site-packages \
  .venv

source .venv/bin/activate
```

確認:

```bash
which python3
```

例:

```text
/home/user/lab_navigation_ws/.venv/bin/python3
```

---

## 6. HLoc をインストール

```bash
cd ~/lab_navigation_ws

python3 -m pip install -e \
  third_party/Hierarchical-Localization
```

今回動作確認できたバージョンへ固定:

```bash
python3 -m pip install \
  "numpy==1.26.4" \
  "setuptools<80"
```

確認:

```bash
python3 - <<'PY'
import numpy
import setuptools
import hloc
import pycolmap
import rclpy
from cv_bridge import CvBridge

print("numpy     :", numpy.__version__)
print("setuptools:", setuptools.__version__)
print("HLoc      : OK")
print("PyCOLMAP  : OK")
print("rclpy      : OK")
print("cv_bridge  : OK")
PY
```

今回の確認値:

```text
numpy     : 1.26.4
setuptools: 79.0.1
HLoc      : OK
PyCOLMAP  : OK
rclpy      : OK
cv_bridge  : OK
```

### 注意

HLoc 実行時に SciPy / NumPy / OpenCV の依存警告が出る場合がある。今回の構成では localization まで正常動作を確認済みだが、長期運用では依存関係を整理する。

---

## 7. ROS ワークスペースをビルド

ビルド時はいったん venv を抜ける。

```bash
deactivate
source /opt/ros/humble/setup.bash

cd ~/lab_navigation_ws
colcon build --symlink-install

source install/setup.bash
```

---

## 8. データセットをコピー

データセットは次に配置する。

```text
~/lab_navigation_ws/captures/
```

注意: 新環境では `capture` ではなく `captures`。

旧PCからコピーする例:

```bash
mkdir -p ~/lab_navigation_ws/captures

rsync -av --progress \
  --exclude='dense/' \
  takushikai@<旧PCのIP>:/home/takushikai/lab_navigation_ws/capture/ \
  ~/lab_navigation_ws/captures/
```

`dense/` は今回の HLoc localization には不要なので除外する。

---

## 9. データセット確認

今回使用したデータ:

```text
captures/hv_building_lab/20261010/
```

確認:

```bash
cd ~/lab_navigation_ws

ls captures/hv_building_lab/20261010
ls captures/hv_building_lab/20261010/images | head
ls captures/hv_building_lab/20261010/sparse/0
```

`sparse/0` には最低限以下が必要:

```text
cameras.bin
images.bin
points3D.bin
```

---

## 10. HLoc 実行用の環境を準備

HLoc 実行時は次の順で source する。

```bash
cd ~/lab_navigation_ws

source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash
source ~/lab_navigation_ws/.venv/bin/activate
```

環境変数:

```bash
export data_path=$PWD/captures/hv_building_lab/20261010
export out=${data_path}/hloc_output

echo $data_path
echo $out
```

---

## 11. COLMAP feature を HLoc 形式へ変換

```bash
mkdir -p ${out}

python3 script/colmap_db_to_hloc.py \
  --database ${data_path}/database.db \
  --model ${data_path}/sparse/0 \
  --output ${out}/feats-shift.h5
```

今回の正常値:

```text
Exported 1370 registered images
```

確認:

```bash
ls -lh ${out}/feats-shift.h5
```

今回のファイルサイズは約 9.2 GB。

---

## 12. Query 画像を確認

```bash
cat ${data_path}/query.txt
```

今回:

```text
query/test1.png
query/test2.png
```

対応する画像:

```text
images/query/test1.png
images/query/test2.png
```

---

## 13. Query の SIFT 特徴抽出

```bash
python3 -m hloc.extract_features \
  --image_dir ${data_path}/images \
  --export_dir ${out} \
  --conf sift \
  --image_list ${data_path}/query.txt \
  --feature_path ${out}/feats-query-sift.h5
```

---

## 14. Reference 画像一覧を作成

COLMAP に登録済みの画像だけを Reference として使用する。

```bash
python3 - <<'PY'
import os
import pycolmap

data_path = os.environ["data_path"]
out = os.environ["out"]

model = pycolmap.Reconstruction(
    f"{data_path}/sparse/0"
)

names = sorted(
    image.name for image in model.images.values()
)

with open(f"{out}/reference.txt", "w") as f:
    for name in names:
        f.write(name + "\n")

print("registered images:", len(names))
print("first:", names[:3])
print("last :", names[-3:])
PY
```

今回の正常値:

```text
registered images: 1370
first: ['00000046.png', '00000047.png', '00000048.png']
last : ['00001898.png', '00001899.png', '00001900.png']
```

---

## 15. NetVLAD 特徴抽出

### Query 側

```bash
python3 -m hloc.extract_features \
  --image_dir ${data_path}/images \
  --export_dir ${out} \
  --conf netvlad \
  --image_list ${data_path}/query.txt \
  --feature_path ${out}/global-query-netvlad.h5 \
  --as_half
```

初回は NetVLAD weights 約 529 MB がダウンロードされる。

### Reference 側

```bash
python3 -m hloc.extract_features \
  --image_dir ${data_path}/images \
  --export_dir ${out} \
  --conf netvlad \
  --image_list ${out}/reference.txt \
  --feature_path ${out}/global-ref-netvlad.h5 \
  --as_half
```

---

## 16. NetVLAD retrieval

Query ごとに上位20枚を取得する。

```bash
python3 -m hloc.pairs_from_retrieval \
  --descriptors ${out}/global-query-netvlad.h5 \
  --db_descriptors ${out}/global-ref-netvlad.h5 \
  --db_model ${data_path}/sparse/0 \
  --query_list ${data_path}/query.txt \
  --output ${out}/pairs-query-netvlad20.txt \
  --num_matched 20
```

今回の正常値:

```text
Found 40 pairs.
```

---

## 17. SIFT matching

```bash
python3 - <<'PY'
import os
from pathlib import Path
from hloc import match_features

out = Path(os.environ["out"])

conf = match_features.confs["NN-ratio"]

match_features.main(
    conf,
    pairs=out / "pairs-query-netvlad20.txt",
    features=out / "feats-query-sift.h5",
    features_ref=out / "feats-shift.h5",
    matches=out / "matches-query-sift.h5",
    overwrite=True,
)

print("Output:", out / "matches-query-sift.h5")
PY
```

設定:

```text
ratio_threshold = 0.8
mutual_check = True
```

---

## 18. Query 用カメラ内部パラメータ生成

COLMAP map で Bundle Adjustment されたカメラパラメータを Query に使用する。

```bash
python3 - <<'PY'
import os
import pycolmap
from pathlib import Path

data = Path(os.environ["data_path"])
out_dir = Path(os.environ["out"])

model = pycolmap.Reconstruction(data / "sparse/0")
cam = next(iter(model.cameras.values()))

queries = [
    q.strip()
    for q in (data / "query.txt").read_text().splitlines()
    if q.strip()
]

params = " ".join(f"{x:.12g}" for x in cam.params)

path = out_dir / "queries_with_intrinsics.txt"

with open(path, "w") as f:
    for q in queries:
        f.write(
            f"{q} {cam.model.name} "
            f"{cam.width} {cam.height} "
            f"{params}\n"
        )

print(path.read_text())
PY
```

今回のカメラ:

```text
model: OPENCV
resolution: 1920 x 1080
fx = 603.018990602
fy = 611.737919589
cx = 1012.52586
cy = 488.124656
```

---

## 19. HLoc localization

```bash
python3 -m hloc.localize_sfm \
  --reference_sfm ${data_path}/sparse/0 \
  --queries ${out}/queries_with_intrinsics.txt \
  --features ${out}/feats-query-sift.h5 \
  --matches ${out}/matches-query-sift.h5 \
  --retrieval ${out}/pairs-query-netvlad20.txt \
  --results ${out}/localization_results.txt
```

正常なら:

```text
Localized 2 / 2 images.
```

確認:

```bash
cat ${out}/localization_results.txt
```

---

## 20. PnP 成功確認

```bash
python3 - <<'PY'
import os
import pickle

path = os.path.join(
    os.environ["out"],
    "localization_results.txt_logs.pkl"
)

with open(path, "rb") as f:
    logs = pickle.load(f)

for qname, log in logs["loc"].items():
    print(f"\n== {qname} ==")
    print("num_matches:", log.get("num_matches"))

    ret = log.get("PnP_ret")

    if ret is None:
        print("PnP: FAILED")
    else:
        print("PnP: SUCCESS")
        print("num_inliers:", ret["num_inliers"])
PY
```

今回の再現基準:

```text
== query/test1.png ==
num_matches: 463
PnP: SUCCESS
num_inliers: 94

== query/test2.png ==
num_matches: 74
PnP: SUCCESS
num_inliers: 17
```

ここまで一致すれば HLoc 環境の移行は成功と判断できる。

---

## 21. hloc_localizer をビルド

```bash
deactivate
source /opt/ros/humble/setup.bash

cd ~/lab_navigation_ws

colcon build \
  --packages-select hloc_localizer \
  --symlink-install

source install/setup.bash
```

---

## 22. robot_state_publisher

`diffbot_custom.launch.py` では既に `robot_state_publisher` を起動しているため、別途追加不要。

URDF の TF 構造:

```text
base_link
└── camera_link
    └── camera_optical_frame
```

現在のカメラ取付値:

```text
base_link -> camera_link
xyz = 0.290 0.000 0.338
rpy = 0 0 0

camera_link -> camera_optical_frame
xyz = 0 0 0
rpy = -pi/2 0 -pi/2
```

カメラ取付位置を変更する場合は、HLoc ノード側へ値をハードコードせず、URDF だけを変更する。

HLoc ノードは TF から `base_link -> camera_optical_frame` を取得する。

---

## 23. Mock hardware で TF 確認

実機や micro-ROS を使わず確認する場合:

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 launch lab_navigation diffbot_custom.launch.py \
  use_mock_hardware:=true \
  start_micro_ros_agent:=false \
  gui:=false
```

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 node list | grep robot_state
```

正常なら:

```text
/robot_state_publisher
```

カメラ TF 確認:

```bash
ros2 run tf2_ros tf2_echo \
  base_link camera_optical_frame
```

Translation は概ね:

```text
x = 0.290
y = 0.000
z = 0.338
```

---

## 24. odom -> base_link を確認

```bash
ros2 run tf2_ros tf2_echo \
  odom base_link
```

値が連続表示されれば OK。

---

## 25. HLoc から map -> odom を発行

現段階では Visual Localization 単独試験として HLoc が `map -> odom` を発行する。

TF 全体:

```text
map
└── odom
    └── base_link
        ├── camera_link
        │   └── camera_optical_frame
        ├── imu_link
        └── velodyne
```

HLoc ノード実行前に:

```bash
cd ~/lab_navigation_ws

source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash
source ~/lab_navigation_ws/.venv/bin/activate
```

実行:

```bash
ros2 run hloc_localizer hloc_tf_publisher \
  --ros-args \
  -p result_file:=/home/user/lab_navigation_ws/captures/hv_building_lab/20261010/hloc_output/localization_results.txt \
  -p query_name:=test1.png
```

確認:

```bash
ros2 run tf2_ros tf2_echo map odom
```

さらに:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

両方取得できれば、以下が接続できている。

```text
map -> odom -> base_link
```

### 注意

現在は `test1.png` の固定 HLoc 姿勢を使った確認段階なので、テスト中はロボットを動かさない。

本番では、新しいカメラ画像の HLoc 結果と、その画像取得時刻の `odom -> base_link` を使って `map -> odom` を更新する。

---

## 26. lidar_localization_ros2 との競合に注意

`lidar_localization_ros2` も `map -> odom` を発行する。

したがって以下を同時に実行しない。

```text
lidar_localization_ros2 -> map -> odom
HLoc                    -> map -> odom
```

Visual 単独試験:

```text
HLoc -> map -> odom
```

LiDAR 単独試験:

```text
lidar_localization_ros2 -> map -> odom
```

将来的には Visual / LiDAR の pose を統合し、最終的な `map -> odom` の発行元を1つにする。

---

## 27. 実機で micro-ROS を使う場合

`micro_ros_agent` が別 workspace にある場合は source 順が重要。

```bash
source /opt/ros/humble/setup.bash
source ~/micro_ros_ws/install/setup.bash
source ~/lab_navigation_ws/install/setup.bash
```

確認:

```bash
ros2 pkg prefix micro_ros_agent
```

HLoc も使う場合は最後に:

```bash
source ~/lab_navigation_ws/.venv/bin/activate
```

実機運用時の基本順序:

```bash
source /opt/ros/humble/setup.bash
source ~/micro_ros_ws/install/setup.bash
source ~/lab_navigation_ws/install/setup.bash
source ~/lab_navigation_ws/.venv/bin/activate
```

---

## 28. 最終チェックリスト

- [ ] `ros2` が使用できる
- [ ] `lab_navigation_ws` が `colcon build` できる
- [ ] `.venv` 内で `hloc` import OK
- [ ] `pycolmap` import OK
- [ ] `rclpy` import OK
- [ ] `cv_bridge` import OK
- [ ] `captures/.../sparse/0` が存在する
- [ ] `feats-shift.h5` を1370画像で生成できる
- [ ] NetVLAD retrieval が40 pairs
- [ ] test1: 463 matches / 94 inliers
- [ ] test2: 74 matches / 17 inliers
- [ ] `robot_state_publisher` が起動する
- [ ] `base_link -> camera_optical_frame` が取得できる
- [ ] `odom -> base_link` が取得できる
- [ ] HLoc から `map -> odom` が取得できる
- [ ] `map -> base_link` が取得できる

---

## 29. 重要事項まとめ

1. データフォルダは `capture` ではなく `captures`。
2. HLoc 実行時は `ROS -> workspace -> .venv` の順で source する。
3. `colcon build` 時は `.venv` を `deactivate` する。
4. カメラ取付位置は HLoc ノードへハードコードせず URDF で管理する。
5. `map -> odom` は HLoc と `lidar_localization_ros2` から同時配信しない。
6. 現段階の HLoc TF は固定 Query による確認用。本番ではリアルタイム画像入力と時刻同期を実装する。
