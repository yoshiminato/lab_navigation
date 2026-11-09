# HLoc ROS 2 Localization - 実行手順

このREADMEでは、現在の構成でHLocによる画像自己位置推定を行い、

- `/hloc/camera_pose`
- `/hloc/localization_status` (処理した画像ごとの成功・失敗。評価用)
- `map -> odom`
- `map -> base_link`

まで確認する手順をまとめます。

---

## 1. 現在の構成

`data_path` は収集セッションのルートを指定します。SIFT地図は
`maps/sift/{sparse/0,reference_features.h5}`、SuperPoint地図は
`maps/superpoint/{sparse/0,reference_features.h5}`、共通検索特徴量は
`retrieval/netvlad.h5` にあります。`local_method` で使用地図を切り替えます。
新規地図の生成・検証は [作業フロー](../../docs/VISUAL_LOCALIZATION_WORKFLOW.md) を参照してください。

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
superpoint
```

`superpoint` はSuperPoint特徴量とLightGlue照合を使います。旧指定の
`superpoint_lightglue` も同じ方式の別名として受け付けます。

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

ros2 launch lab_navigation diffbot_control.launch.py
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

HLoc姿勢がIsaac Simの`world`座標で、Grid Mapの`map`と原点や方位が異なる場合は、
`T_map_world`をパラメータで指定します。

```bash
ros2 run hloc_localizer hloc_tf_publisher \
  --ros-args \
  -p world_to_map_x:=5.0 \
  -p world_to_map_y:=10.0 \
  -p world_to_map_z:=0.0 \
  -p world_to_map_yaw_deg:=0.0
```

既定値はすべて0で無補正です。Isaac Simの暫定値と校正方法は
[../../docs/ISAAC_SIM_VISUAL_LOCALIZATION.md](../../docs/ISAAC_SIM_VISUAL_LOCALIZATION.md)
を参照してください。

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
  -p local_method:=superpoint
```

正常起動時には、例えば以下のようなログが表示されます。

```text
Local method: superpoint
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
Method     : superpoint
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
  -p local_method:=superpoint
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
  -p local_method:=superpoint \
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
  -p local_method:=superpoint \
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
  -p local_method:=superpoint
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
  -p local_method:=superpoint
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

# 14. 参照地図を準備・変更する場合

この文書の第1〜13節は、作成済みの参照地図をROS 2で運用する手順です。
参照地図そのものを準備するときは、目的に応じて次を参照してください。

- 環境構築、SIFT特徴量の変換、NetVLAD生成:
  [lab_navigation_ws_setup_guide.md](lab_navigation_ws_setup_guide.md)
- 既存SIFT地図からSuperPoint + LightGlue用地図を追加:
  [README_HLoc_ROS2_SP_LG.md](README_HLoc_ROS2_SP_LG.md)
- Pose prior、spatial pairを使ってSuperPoint + LightGlueでSfM:
  [README_hloc_superpoint_colmap_sfm.md](README_hloc_superpoint_colmap_sfm.md)
- Isaac Simでの一連の手順:
  [../../docs/ISAAC_SIM_VISUAL_LOCALIZATION.md](../../docs/ISAAC_SIM_VISUAL_LOCALIZATION.md)

文書全体の関係は [../../docs/README.md](../../docs/README.md) を参照してください。
