# HLoc ROS 2 Localization - 実行手順

このREADMEでは、現在の構成でHLocによる画像自己位置推定を行い、

- `/hloc/camera_pose`
- `map -> odom`
- `map -> base_link`

まで確認する手順をまとめます。

---

## 1. 現在の構成

処理の流れは次の通りです。

```text
カメラ画像
    ↓
/camera/image_raw
    ↓
hloc_localizer
    ├─ NetVLADで候補画像検索
    ├─ SIFT + NN-ratio
    │      または
    ├─ SuperPoint + LightGlue
    └─ PnP
    ↓
/hloc/camera_pose
    ↓
hloc_tf_publisher
    ↓
map -> odom
    ↓
odom -> base_link
    ↓
map -> base_link
```

ローカル特徴量方式は実行時に切り替えられます。

```text
sift
superpoint_lightglue
```

---

## 2. ROS 2環境を読み込む

各ターミナルで最初に実行します。

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash
```

### `.venv` のactivateについて

通常の実行では、

```bash
source ~/lab_navigation_ws/.venv/bin/activate
```

は不要です。

現在の `hloc_localizer` パッケージは、ROS 2から起動されたラッパースクリプトが直接、

```text
/home/user/lab_navigation_ws/.venv/bin/python3
```

を使用する構成になっています。

したがって、

```bash
ros2 run hloc_localizer ...
```

で起動すればHLocは自動的にvenvのPythonで実行されます。

---

# 3. 起動手順

HLocによるlocalizationを行う場合は、基本的に以下の順番で起動します。

---

## Terminal 1: ロボットのTF / odometryを起動

`hloc_tf_publisher` が動作するためには、少なくとも以下のTFが必要です。

```text
odom -> base_link
base_link -> camera_optical_frame
```

実機の場合は、通常使用しているロボット側launchを起動してください。

例:

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 launch lab_navigation diffbot_custom.launch.py
```

テスト用のmock hardwareを使用する場合は、現在のlaunch設定に応じてmock hardwareで起動します。

TFが存在するか確認できます。

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

別ターミナルで:

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

両方ともTransformが表示されればOKです。

---

## Terminal 2: HLoc TF publisherを起動

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 run hloc_localizer hloc_tf_publisher
```

正常なら、HLocの姿勢を待つ状態になります。

概念的には:

```text
Waiting for HLoc pose: /hloc/camera_pose
```

となります。

このノードは、

```text
/hloc/camera_pose
```

を受信して、

```text
map -> odom
```

を計算・配信します。

---

## Terminal 3: HLoc localizerを起動

### SuperPoint + LightGlueを使用する場合

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=superpoint_lightglue
```

正常起動時には、例えば以下のようなログが表示されます。

```text
Local method: superpoint_lightglue
Loading SuperPoint extractor...
Loading LightGlue matcher...
Registered images: 1370
3D points: 227522
NetVLAD reference database: 1370 images
Waiting for images: /camera/image_raw
```

---

### SIFT + NN-ratioを使用する場合

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=sift
```

つまり、切り替えるのは

```bash
-p local_method:=...
```

だけです。

---

# 4. テスト画像でlocalizationする

現在のテスト画像:

```text
/home/user/lab_navigation_ws/captures/hv_building_lab/20261010/images/query/test1.png
```

を送る場合は、別ターミナルで:

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 run hloc_localizer image_file_publisher \
  --ros-args \
  -p image_path:=/home/user/lab_navigation_ws/captures/hv_building_lab/20261010/images/query/test1.png
```

画像は

```text
/camera/image_raw
```

に1回だけpublishされます。

---

## 繰り返し送信する場合

例えば0.5 Hzで繰り返す場合:

```bash
ros2 run hloc_localizer image_file_publisher \
  --ros-args \
  -p image_path:=/home/user/lab_navigation_ws/captures/hv_building_lab/20261010/images/query/test1.png \
  -p publish_rate:=0.5
```

速度測定や連続動作確認に使用できます。

---

# 5. localization結果を確認する

HLoc localizerが成功すると、次のようなログが出ます。

```text
Method     : superpoint_lightglue
Result     : SUCCESS
Keypoints  : 1020
Candidates : 20
3D matches : 6303
PnP corr.  : 1815
Inliers    : 1100以上
```

成功すると、

```text
/hloc/camera_pose
```

にカメラ姿勢がpublishされます。

確認:

```bash
ros2 topic echo /hloc/camera_pose --once
```

---

# 6. TFを確認する

## map -> odom

```bash
ros2 run tf2_ros tf2_echo map odom
```

Transformが連続して表示されれば、`hloc_tf_publisher` によるTF発行は成功です。

---

## map -> base_link

```bash
ros2 run tf2_ros tf2_echo map base_link
```

これも表示されれば、

```text
map
 └─ odom
     └─ base_link
         └─ camera_link
             └─ camera_optical_frame
```

というTFツリーがつながっています。

---

# 7. 実カメラでlocalizationする場合

実カメラが既に

```text
/camera/image_raw
```

をpublishしている場合、

```text
image_file_publisher
```

は起動しません。

必要なのは、

```text
1. ロボット / odometry / camera TF
2. hloc_tf_publisher
3. hloc_localizer
4. 実カメラpublisher
```

だけです。

SuperPoint + LightGlueの場合:

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=superpoint_lightglue
```

実カメラから `/camera/image_raw` が届くたびにHLocが処理します。

---

# 8. 画像トピックを変更する場合

カメラが別のトピックを使用している場合は、

```bash
-p image_topic:=/別の画像トピック
```

を指定します。

例:

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=superpoint_lightglue \
  -p image_topic:=/camera/color/image_raw
```

---

# 9. NetVLAD候補数を変更する

現在のデフォルトは:

```text
Candidates = 20
```

です。

例えば10枚に減らす場合:

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=superpoint_lightglue \
  -p num_matched:=10
```

SuperPoint + LightGlueではLightGlue matchingが現在の主な処理時間なので、候補数を減らすことで高速化できる可能性があります。

まずは20のままで動作確認することを推奨します。

---

# 10. 最小のテスト手順まとめ

## Terminal 1

ロボット / odometry / camera TFを起動。

確認:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

---

## Terminal 2

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 run hloc_localizer hloc_tf_publisher
```

---

## Terminal 3

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=superpoint_lightglue
```

---

## Terminal 4

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash

ros2 run hloc_localizer image_file_publisher \
  --ros-args \
  -p image_path:=/home/user/lab_navigation_ws/captures/hv_building_lab/20261010/images/query/test1.png
```

---

## Terminal 5

```bash
ros2 run tf2_ros tf2_echo map odom
```

または:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

---

# 11. 正常時の判定

以下がすべて成立すれば、HLoc localizationからTF発行まで正常です。

```text
[OK] /camera/image_raw が届く

[OK] hloc_localizer が
     Result : SUCCESS

[OK] /hloc/camera_pose がpublishされる

[OK] hloc_tf_publisher が姿勢を受信する

[OK] map -> odom がpublishされる

[OK] map -> base_link が取得できる
```

---

# 12. 現在のSuperPoint + LightGlue性能

現在の `test1.png` の連続実行では、おおよそ:

```text
SuperPoint : 約 0.02 s
NetVLAD    : 約 0.06 s
LightGlue  : 約 0.40 s
PnP        : 約 0.03 s

TOTAL      : 約 0.52 s
Rate       : 約 1.9 Hz
```

また、同じテスト画像では:

```text
PnP correspondences : 約1815
Inliers             : 約1120～1140
Success             : 11/11
```

となっています。

現在の主なボトルネックはLightGlue matchingです。

---

# 13. よく使うコマンド

SIFT:

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=sift
```

SuperPoint + LightGlue:

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=superpoint_lightglue
```

HLoc TF:

```bash
ros2 run hloc_localizer hloc_tf_publisher
```

HLoc pose確認:

```bash
ros2 topic echo /hloc/camera_pose --once
```

TF確認:

```bash
ros2 run tf2_ros tf2_echo map odom
```

```bash
ros2 run tf2_ros tf2_echo map base_link
```

ROSノード確認:

```bash
ros2 node list
```

画像トピック確認:

```bash
ros2 topic hz /camera/image_raw
```

TFツリー生成:

```bash
ros2 run tf2_tools view_frames
```


---

# 14. 既存SIFT地図を残したまま SuperPoint + LightGlue 用地図を作る

ここでは、最初に **SIFT特徴量で作成済みのCOLMAP/HLoc地図しかない状態** から、参照画像・カメラ姿勢・世界座標系はそのまま使い、SuperPoint + LightGlue用の特徴量と3D点群を作り直す手順をまとめます。

今回使用したデータセットは:

```text
/home/user/lab_navigation_ws/captures/hv_building_lab/20261010
```

元のSIFTモデルは:

```text
sparse/0
```

にあります。

SuperPoint + LightGlue用の出力は、SIFT版を消さずに別ディレクトリへ保存します。

```text
hloc_output_superpoint_lightglue/
```

このため、SIFT版とSuperPoint + LightGlue版を後から切り替えて比較できます。

## 14.1 「地図をそのまま使う」の意味

SuperPointの特徴点をSIFTモデルへ単純に差し替えるわけではありません。SIFTで作られた元モデルには、各参照画像のカメラ姿勢、カメラ内部パラメータ、SIFT特徴点と3D点の対応が含まれています。

SuperPointを抽出し直すと、特徴点の位置・個数・順番がSIFTとは変わります。そのため、SIFTの3D点IDをSuperPoint特徴点へそのまま対応させることはできません。

今回の方法では:

```text
元の sparse/0
    ↓
カメラ姿勢・カメラ内部パラメータ・世界座標系を再利用

SuperPoint
    ↓
新しい2D特徴点

LightGlue
    ↓
参照画像間の新しい対応

Triangulation
    ↓
SuperPoint用の新しい3D点群
```

とします。つまり、**世界座標系とカメラ姿勢は元の地図と同じまま、3D特徴点群だけをSuperPoint + LightGlue用に再構築**します。

---

# 15. 作業前の環境設定

HLocの処理は仮想環境で実行します。

```bash
cd ~/lab_navigation_ws
source /opt/ros/humble/setup.bash
source .venv/bin/activate
```

HLocソースは:

```text
~/lab_navigation_ws/third_party/Hierarchical-Localization
```

にあります。

確認:

```bash
python3 - <<'PY2'
import hloc
import pycolmap
import torch

print("HLoc import: OK")
print("CUDA:", torch.cuda.is_available())
print("PyCOLMAP CUDA:", pycolmap.has_cuda)
PY2
```

---

# 16. パスを整理する

```bash
data_path=~/lab_navigation_ws/captures/hv_building_lab/20261010
spout=${data_path}/hloc_output_superpoint_lightglue
sfm_ref=${data_path}/sparse/0
mkdir -p "${spout}"
```

参照画像ディレクトリは、元のSfMモデル作成時に使用した参照画像ディレクトリを指定します。以下では例として:

```bash
image_dir=${data_path}/images
```

と表記します。実際のデータセット構造で参照画像が別サブディレクトリにある場合は、そのパスへ読み替えてください。

---

# 17. Step 1: 参照画像からSuperPoint特徴量を抽出

HLocの `superpoint_aachen` 設定を使用します。今回の設定は概ね:

```text
SuperPoint
最大keypoint数: 4096
resize_max: 1024
grayscale: True
```

です。

```bash
python3 - <<'PY2'
from pathlib import Path
from hloc import extract_features

data_path = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data_path / "hloc_output_superpoint_lightglue"
image_dir = data_path / "images"

conf = extract_features.confs["superpoint_aachen"]

extract_features.main(
    conf,
    image_dir,
    feature_path=spout / "feats-ref-superpoint.h5",
    overwrite=True,
)
PY2
```

作成されるファイル:

```text
hloc_output_superpoint_lightglue/
└── feats-ref-superpoint.h5
```

確認:

```bash
ls -lh "${spout}/feats-ref-superpoint.h5"
```

## 17.1 参照画像だけを抽出する場合

`images/` の中にquery画像なども入っている場合は、元COLMAPモデルに登録されている参照画像だけを対象にします。今回のSuperPoint地図では、元モデルと同じ **1370枚** の参照画像を使用しました。

---

# 18. Step 2: 元SIFT地図からcovisibility pairを作る

元の `sparse/0` にあるcovisibility情報を使い、各画像について上位20枚の参照画像pairを作成しました。

```bash
python3 -m hloc.pairs_from_covisibility \
  --model "${data_path}/sparse/0" \
  --output "${spout}/pairs-db-covis20.txt" \
  --num_matched 20
```

今回のデータでは **27400 pairs** が生成されました。

確認:

```bash
wc -l "${spout}/pairs-db-covis20.txt"
```

---

# 19. Step 3: 参照画像同士をLightGlueでマッチング

先ほど作成したSuperPoint特徴量を使い、SuperPoint + LightGlueで参照画像同士を対応付けします。

```bash
python3 - <<'PY2'
from pathlib import Path
from hloc import match_features

data_path = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data_path / "hloc_output_superpoint_lightglue"

conf = match_features.confs["superpoint+lightglue"]

match_features.main(
    conf,
    spout / "pairs-db-covis20.txt",
    features=spout / "feats-ref-superpoint.h5",
    matches=spout / "matches-ref-superpoint-lightglue.h5",
    overwrite=True,
)
PY2
```

作成されるファイル:

```text
hloc_output_superpoint_lightglue/
├── feats-ref-superpoint.h5
├── pairs-db-covis20.txt
└── matches-ref-superpoint-lightglue.h5
```

---

# 20. Step 4: SuperPoint特徴点で3D点群を再triangulationする

入力は:

```text
元モデル:
    sparse/0

新特徴量:
    feats-ref-superpoint.h5

新match:
    matches-ref-superpoint-lightglue.h5

pair:
    pairs-db-covis20.txt
```

です。

出力モデルは:

```text
sfm-superpoint-lightglue-colmapgv/
```

とします。

## 20.1 最初に試した通常triangulation

最初はHLoc標準のgeometry verificationを使用してtriangulationしました。概念的には:

```python
triangulation.main(
    ...,
    estimate_two_view_geometries=False,
)
```

です。

この段階では、database内に大量のmatch / two-view情報が存在しているにもかかわらず、最終的なtriangulation結果が:

```text
3D points = 0
```

になりました。

この作業では原因を断定していません。重要なのは、matchが存在しなかったわけではなく、**databaseにはmatchおよびtwo-view情報が入っていたが、後段のtriangulationで3D点が生成されなかった**という点です。

## 20.2 成功した方法: COLMAPでtwo-view geometryを再推定

そこで:

```text
estimate_two_view_geometries=True
```

を指定し、COLMAP / PyCOLMAP側でtwo-view geometryを再推定してからtriangulationしました。

```bash
python3 - <<'PY2'
from pathlib import Path
from hloc import triangulation

data_path = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data_path / "hloc_output_superpoint_lightglue"

reference_model = data_path / "sparse/0"
image_dir = data_path / "images"
output_model = spout / "sfm-superpoint-lightglue-colmapgv"

reconstruction = triangulation.main(
    sfm_dir=output_model,
    reference_model=reference_model,
    image_dir=image_dir,
    pairs=spout / "pairs-db-covis20.txt",
    features=spout / "feats-ref-superpoint.h5",
    matches=spout / "matches-ref-superpoint-lightglue.h5",
    estimate_two_view_geometries=True,
    verbose=True,
)

print(reconstruction.summary())
PY2
```

今回、0点問題を回避できた重要な設定が:

```python
estimate_two_view_geometries=True
```

です。

---

# 21. 成功したSuperPoint + LightGlueモデルの統計

完成したモデル:

```text
hloc_output_superpoint_lightglue/
└── sfm-superpoint-lightglue-colmapgv/
```

の統計は:

```text
num_points3D                  227522
num_observations             1345764
mean_track_length            5.91487
mean_observations_per_image  982.309
mean_reprojection_error      1.48239
```

でした。

比較として元のSIFTモデルは:

```text
num_points3D                  362259
num_observations             2589705
mean_track_length            7.14877
mean_observations_per_image  1890.3
mean_reprojection_error      0.812313
```

でした。

SuperPointモデルの方が3D点数や観測数は少ないですが、localization時には非常に多くの有効対応を得られました。

---

# 22. モデルが正常に作成されたか確認する

```bash
python3 - <<'PY2'
from pathlib import Path
import pycolmap

model = (
    Path.home()
    / "lab_navigation_ws/captures/hv_building_lab/20261010"
    / "hloc_output_superpoint_lightglue"
    / "sfm-superpoint-lightglue-colmapgv"
)

rec = pycolmap.Reconstruction(model)

print(rec.summary())
print("images   :", len(rec.images))
print("points3D :", len(rec.points3D))
PY2
```

今回と同じなら概ね:

```text
images   : 1370
points3D : 227522
```

となります。少なくとも `points3D > 0` であることを必ず確認します。

---

# 23. Query画像でもSuperPoint + LightGlueを確認する

これはROSのオンラインlocalizationには必須ではありませんが、地図を作り直した直後の動作確認として有効です。

今回のquery画像:

```text
query/test1.png
query/test2.png
```

に対してSuperPoint特徴量を抽出し、次のファイルを作りました。

```text
feats-query-superpoint.h5
matches-query-superpoint-lightglue.h5
```

## 23.1 Query特徴量抽出

```bash
python3 - <<'PY2'
from pathlib import Path
from hloc import extract_features

data_path = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data_path / "hloc_output_superpoint_lightglue"
query_dir = data_path / "images/query"

conf = extract_features.confs["superpoint_aachen"]

extract_features.main(
    conf,
    query_dir,
    feature_path=spout / "feats-query-superpoint.h5",
    overwrite=True,
)
PY2
```

## 23.2 NetVLAD top20 pairを使う

Global retrievalには、既存SIFT版でも使用していたNetVLADを再利用できます。

```text
Local feature:
    SIFT → SuperPointへ変更

Global retrieval:
    NetVLADはそのまま
```

既にNetVLAD top20のquery-reference pairファイルがある場合は、そのpairをそのまま利用します。以下では例として:

```text
pairs-query-netvlad20.txt
```

とします。

## 23.3 Query-referenceをLightGlueでmatchする

```bash
python3 - <<'PY2'
from pathlib import Path
from hloc import match_features

data_path = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data_path / "hloc_output_superpoint_lightglue"

conf = match_features.confs["superpoint+lightglue"]

match_features.main(
    conf,
    spout / "pairs-query-netvlad20.txt",
    features=spout / "feats-query-superpoint.h5",
    features_ref=spout / "feats-ref-superpoint.h5",
    matches=spout / "matches-query-superpoint-lightglue.h5",
    overwrite=True,
)
PY2
```

---

# 24. 今回のオフラインlocalization結果

SuperPoint + LightGlueで作り直した地図を使った結果:

## test1

```text
matches      : 6271
inliers      : 1134
inlier ratio : 約18.1%
```

## test2

```text
matches      : 5644
inliers      : 805
inlier ratio : 約14.3%
```

元のSIFT版では:

```text
test1: 463 matches / 94 inliers
test2: 74 matches / 17 inliers
```

でした。

match数の定義や処理方式が異なるため、raw match数だけを完全に同条件で比較することはできません。ただし、PnPの絶対inlier数はSuperPoint + LightGlueで大きく増加しました。

---

# 25. ROSオンラインlocalizerへ接続する

SuperPoint + LightGlue地図が完成した後は、ROS側で:

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=superpoint_lightglue
```

とするだけで、SuperPoint + LightGlue側のモデルが選択されます。

SIFTへ戻す場合:

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=sift
```

現在は:

```text
                ┌─ SIFT map + SIFT + NN-ratio
image → NetVLAD ┤
                └─ SP map + SuperPoint + LightGlue
                         ↓
                        PnP
                         ↓
                /hloc/camera_pose
```

という切替構成になっています。

---

# 26. 今回のROSオンラインテスト結果

`test1.png` をROS Imageとして繰り返し送信した結果:

```text
Keypoints  : 1020
Candidates : 20
3D matches : 6303
PnP corr.  : 1815
Inliers    : 約1120～1146
Success    : 11/11
```

ウォームアップ後の処理時間は概ね:

```text
SuperPoint : 約0.02 s
NetVLAD    : 約0.06 s
Retrieval  : ほぼ0 s
LightGlue  : 約0.40 s
PnP        : 約0.03 s
TOTAL      : 約0.52 s
Rate       : 約1.9 Hz
```

でした。

---

# 27. SIFTしかない状態からSP+LGへ移行する最短手順

```text
1. sparse/0 のSIFTモデルを残す

2. 同じ参照画像から
   feats-ref-superpoint.h5
   を作る

3. sparse/0 のcovisibilityを使って
   pairs-db-covis20.txt
   を作る

4. SuperPoint特徴量同士をLightGlueでmatch
   matches-ref-superpoint-lightglue.h5
   を作る

5. sparse/0 のカメラ姿勢を固定したまま
   SuperPoint + LightGlue対応から3D点をtriangulation

6. 通常triangulationで3D点が0だったため、
   estimate_two_view_geometries=True
   でtwo-view geometryを再推定

7. sfm-superpoint-lightglue-colmapgv
   が完成

8. ROSでは
   -p local_method:=superpoint_lightglue
   で使用
```

重要なのは、**元のSIFT地図を削除・上書きしない**ことです。

最終的には:

```text
sparse/0
    → SIFT用

hloc_output_superpoint_lightglue/
    → SuperPoint + LightGlue用
```

の2系統を共存させます。

---

# 28. 作成物一覧

最終的な主要ファイルは:

```text
20261010/
├── sparse/
│   └── 0/
│       └── 元SIFT SfMモデル
│
├── hloc_output/
│   ├── feats-shift.h5
│   └── global-ref-netvlad.h5
│
└── hloc_output_superpoint_lightglue/
    ├── feats-ref-superpoint.h5
    ├── pairs-db-covis20.txt
    ├── matches-ref-superpoint-lightglue.h5
    ├── feats-query-superpoint.h5
    ├── matches-query-superpoint-lightglue.h5
    └── sfm-superpoint-lightglue-colmapgv/
```

この構成にしておけば、SIFTとSuperPoint + LightGlueを同じROSノードから実行時パラメータで切り替えられます。
