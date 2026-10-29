# HLoc ROS 2 Localization 手順書
## SIFT地図から SuperPoint + LightGlue 地図を追加し、ROS 2で使用する

このREADMEは、すでにSIFTで作成したCOLMAP/HLoc地図がある状態から、

1. 元のSIFT地図を残す
2. 同じ参照画像からSuperPoint特徴量を抽出する
3. LightGlueで参照画像同士を対応付けする
4. 元の地図のカメラ姿勢・座標系を維持したままSuperPoint用3D点群を再構築する
5. ROS 2で `sift` / `superpoint_lightglue` を切り替えてlocalizationする

までを、上から順番に実行できる形でまとめたものです。

---

# 1. 前提

ワークスペース:

```text
~/lab_navigation_ws
```

HLoc:

```text
~/lab_navigation_ws/third_party/Hierarchical-Localization
```

データ:

```text
~/lab_navigation_ws/captures/hv_building_lab/20261010
```

元のSIFT SfMモデル:

```text
~/lab_navigation_ws/captures/hv_building_lab/20261010/sparse/0
```

参照画像を含むディレクトリ:

```text
~/lab_navigation_ws/captures/hv_building_lab/20261010/images
```

query画像例:

```text
~/lab_navigation_ws/captures/hv_building_lab/20261010/images/query/test1.png
~/lab_navigation_ws/captures/hv_building_lab/20261010/images/query/test2.png
```

SuperPoint + LightGlue用の出力先:

```text
~/lab_navigation_ws/captures/hv_building_lab/20261010/hloc_output_superpoint_lightglue
```

---

# 2. 何を残し、何を作り直すのか

元のSIFTモデル `sparse/0` は削除しない。

再利用するもの:

```text
・参照画像
・カメラ内部パラメータ
・各参照画像のカメラ姿勢
・map座標系
・NetVLAD global descriptor
```

作り直すもの:

```text
・SuperPoint 2D特徴点
・LightGlueによる参照画像間match
・SuperPoint特徴点に対応する3D点群
```

重要:

```text
SIFTの3D点IDをSuperPoint特徴点へそのまま流用することはできない。
```

SuperPointでは特徴点位置・個数・順番がSIFTと異なるため、元のカメラ姿勢を固定して3D点を再triangulationする。

---

# 3. 作業用ターミナルを準備

以下を最初に実行する。

```bash
cd ~/lab_navigation_ws

source /opt/ros/humble/setup.bash
source .venv/bin/activate

export PYTHONPATH=$HOME/lab_navigation_ws/third_party/Hierarchical-Localization:$PYTHONPATH
```

パス変数:

```bash
export DATA=$HOME/lab_navigation_ws/captures/hv_building_lab/20261010
export SPOUT=$DATA/hloc_output_superpoint_lightglue
export REF_MODEL=$DATA/sparse/0
export IMAGE_DIR=$DATA/images

mkdir -p "$SPOUT"
```

確認:

```bash
echo "$DATA"
echo "$SPOUT"
echo "$REF_MODEL"
echo "$IMAGE_DIR"
```

---

# 4. HLoc / CUDAが使えるか確認

```bash
python3 - <<'PY'
import torch
import hloc
import pycolmap

print("torch CUDA :", torch.cuda.is_available())
print("HLoc       : OK")
print("pycolmap   :", pycolmap.__version__)
PY
```

RTX 3060 Ti環境では:

```text
torch CUDA : True
```

になればよい。

---

# 5. 元のSIFTモデルに登録されている参照画像一覧を作る

`images/` 以下には `query/` も存在するため、ディレクトリ内の全画像をSuperPoint抽出対象にはしない。

元の `sparse/0` に登録されている画像名だけを取り出す。

```bash
python3 - <<'PY'
from pathlib import Path
import pycolmap

data = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
model = pycolmap.Reconstruction(data / "sparse/0")

names = sorted(image.name for image in model.images.values())

out = data / "hloc_output_superpoint_lightglue/reference_images.txt"
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text("\n".join(names) + "\n")

print("registered images:", len(names))
print("output:", out)
print("first 5:")
for n in names[:5]:
    print(" ", n)
PY
```

今回の地図なら:

```text
registered images: 1370
```

になる。

確認:

```bash
wc -l "$SPOUT/reference_images.txt"
```

期待:

```text
1370
```

---

# 6. 参照画像のSuperPoint特徴量を抽出

HLoc標準設定:

```text
superpoint_aachen
```

を使用する。

設定は主に:

```text
max_keypoints = 4096
resize_max    = 1024
grayscale     = True
```

HLoc標準の `superpoint_aachen` がこの設定を持つ。

実行:

```bash
python3 - <<'PY'
from pathlib import Path
from hloc import extract_features

data = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data / "hloc_output_superpoint_lightglue"

image_dir = data / "images"
image_list = spout / "reference_images.txt"
feature_path = spout / "feats-ref-superpoint.h5"

conf = extract_features.confs["superpoint_aachen"]

extract_features.main(
    conf=conf,
    image_dir=image_dir,
    image_list=image_list,
    feature_path=feature_path,
    overwrite=True,
)

print("created:", feature_path)
PY
```

成功後:

```bash
ls -lh "$SPOUT/feats-ref-superpoint.h5"
```

---

# 7. SuperPoint特徴量の画像数を確認

```bash
python3 - <<'PY'
from pathlib import Path
import h5py

p = (
    Path.home()
    / "lab_navigation_ws/captures/hv_building_lab/20261010"
    / "hloc_output_superpoint_lightglue"
    / "feats-ref-superpoint.h5"
)

with h5py.File(p, "r") as f:
    print("feature images:", len(f.keys()))
    names = list(f.keys())
    print("first groups:", names[:5])
PY
```

今回なら:

```text
feature images: 1370
```

が目安。

---

# 8. 元のSIFT地図からcovisibility pairを作る

元のSIFT地図には、どの画像同士が同じ3D点を観測しているかという情報がある。

これを利用して、各参照画像についてcovisibility上位20画像を選ぶ。

```bash
python3 -m hloc.pairs_from_covisibility \
  --model "$REF_MODEL" \
  --output "$SPOUT/pairs-db-covis20.txt" \
  --num_matched 20
```

確認:

```bash
wc -l "$SPOUT/pairs-db-covis20.txt"
```

今回:

```text
27400
```

程度。

先頭確認:

```bash
head "$SPOUT/pairs-db-covis20.txt"
```

形式:

```text
00000001.png 00000002.png
00000001.png 00000003.png
...
```

---

# 9. 参照画像同士をLightGlueでmatch

SuperPoint特徴量に対してLightGlueを使用する。

```bash
python3 - <<'PY'
from pathlib import Path
from hloc import match_features

data = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data / "hloc_output_superpoint_lightglue"

conf = match_features.confs["superpoint+lightglue"]

match_features.main(
    conf=conf,
    pairs=spout / "pairs-db-covis20.txt",
    features=spout / "feats-ref-superpoint.h5",
    matches=spout / "matches-ref-superpoint-lightglue.h5",
    overwrite=True,
)

print("created:", spout / "matches-ref-superpoint-lightglue.h5")
PY
```

成功後:

```bash
ls -lh "$SPOUT/matches-ref-superpoint-lightglue.h5"
```

---

# 10. LightGlue matchファイルを確認

```bash
python3 - <<'PY'
from pathlib import Path
import h5py

p = (
    Path.home()
    / "lab_navigation_ws/captures/hv_building_lab/20261010"
    / "hloc_output_superpoint_lightglue"
    / "matches-ref-superpoint-lightglue.h5"
)

with h5py.File(p, "r") as f:
    print("stored pair groups:", len(f.keys()))
    print("first groups:")
    for name in list(f.keys())[:5]:
        print(" ", name)
PY
```

---

# 11. SuperPoint用3Dモデルを再triangulation

元の:

```text
sparse/0
```

から以下は維持する:

```text
カメラ内部パラメータ
参照画像のカメラ姿勢
世界座標系
```

一方、3D点はSuperPoint + LightGlue対応から作り直す。

今回、最初に通常のHLoc geometric verificationで実行したところ、matchは存在していたにもかかわらず:

```text
3D points = 0
```

になった。

そのため、最終的には:

```python
estimate_two_view_geometries=True
```

を使用し、COLMAP/PyCOLMAP側でtwo-view geometryを再推定してからtriangulationした。

---

# 12. 以前の失敗モデルがあれば削除

SuperPoint用出力だけを消す。

元のSIFT:

```text
$DATA/sparse/0
```

は絶対に消さない。

```bash
rm -rf "$SPOUT/sfm-superpoint-lightglue-colmapgv"
```

---

# 13. 成功したtriangulationコマンド

```bash
python3 - <<'PY'
from pathlib import Path
from hloc import triangulation

data = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data / "hloc_output_superpoint_lightglue"

output_model = spout / "sfm-superpoint-lightglue-colmapgv"

reconstruction = triangulation.main(
    sfm_dir=output_model,
    reference_model=data / "sparse/0",
    image_dir=data / "images",
    pairs=spout / "pairs-db-covis20.txt",
    features=spout / "feats-ref-superpoint.h5",
    matches=spout / "matches-ref-superpoint-lightglue.h5",

    # 今回重要だった設定
    estimate_two_view_geometries=True,

    verbose=True,
)

print()
print("===== RESULT =====")
print(reconstruction.summary())
PY
```

ここは処理に時間がかかる。

`estimate_two_view_geometries=True` のとき、HLocはimportしたmatchに対してCOLMAP/PyCOLMAPのgeometric verificationを実行してからtriangulationする。

---

# 14. 作成されたモデルを確認

```bash
python3 - <<'PY'
from pathlib import Path
import pycolmap

model_path = (
    Path.home()
    / "lab_navigation_ws/captures/hv_building_lab/20261010"
    / "hloc_output_superpoint_lightglue"
    / "sfm-superpoint-lightglue-colmapgv"
)

rec = pycolmap.Reconstruction(model_path)

print(rec.summary())
print("registered images:", len(rec.images))
print("3D points        :", len(rec.points3D))
PY
```

今回完成したモデル:

```text
Registered images              1370
Points3D                       227522
Observations                   1345764
Mean track length              5.91487
Mean observations per image    982.309
Mean reprojection error        1.48239
```

少なくとも:

```text
registered images = 1370
3D points > 0
```

であることを確認する。

---

# 15. 元のSIFTモデルとSuperPointモデルの両方が残っていることを確認

```bash
ls "$DATA/sparse/0"
ls "$SPOUT/sfm-superpoint-lightglue-colmapgv"
```

最終的に:

```text
20261010/
├── sparse/
│   └── 0/
│       └── SIFTで作った元モデル
│
└── hloc_output_superpoint_lightglue/
    └── sfm-superpoint-lightglue-colmapgv/
        └── SuperPoint + LightGlue用モデル
```

となる。

この2つは別物なので、SIFTモデルは上書きされない。

---

# 16. Query画像でもSuperPoint特徴量を作る

ROSオンライン処理ではquery特徴量はその場で抽出するため、このHDF5は必須ではない。

ただし、ROS導入前にオフラインで確認するには便利。

queryディレクトリ:

```text
images/query/
```

実行:

```bash
python3 - <<'PY'
from pathlib import Path
from hloc import extract_features

data = Path.home() / "lab_navigation_ws/captures/hv_building_lab/20261010"
spout = data / "hloc_output_superpoint_lightglue"

conf = extract_features.confs["superpoint_aachen"]

extract_features.main(
    conf=conf,
    image_dir=data / "images/query",
    feature_path=spout / "feats-query-superpoint.h5",
    overwrite=True,
)

print("created:", spout / "feats-query-superpoint.h5")
PY
```

確認:

```bash
ls -lh "$SPOUT/feats-query-superpoint.h5"
```

---

# 17. NetVLADは作り直さなくてよい

SuperPointへ変更したのはlocal featureだけ。

Global retrievalには引き続きNetVLADを使用する。

つまり:

```text
query image
   │
   ├── NetVLAD
   │      ↓
   │   top 20 reference images
   │
   └── SuperPoint
          ↓
       LightGlue
          ↓
         PnP
```

となる。

既存の:

```text
hloc_output/global-ref-netvlad.h5
```

はそのまま再利用する。

---

# 18. ROS 2 localizerでSuperPoint + LightGlueを使う

地図作成が終わったら、ROS 2側では以下だけでよい。

各ターミナルで:

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash
```

現在はROS実行ラッパーが `.venv/bin/python3` を直接呼ぶので、ROSノード起動時は `.venv` のactivateは不要。

---

# 19. TF publisherを起動

```bash
ros2 run hloc_localizer hloc_tf_publisher
```

---

# 20. SuperPoint + LightGlue localizerを起動

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=superpoint_lightglue
```

正常時:

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

# 21. SIFTに戻す場合

```bash
ros2 run hloc_localizer hloc_localizer \
  --ros-args \
  -p local_method:=sift
```

つまりlocalizer側は:

```text
-p local_method:=sift
```

または:

```text
-p local_method:=superpoint_lightglue
```

だけで切り替える。

---

# 22. test1.pngをROS画像として送る

別ターミナル:

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/setup.bash
```

実行:

```bash
ros2 run hloc_localizer image_file_publisher \
  --ros-args \
  -p image_path:=/home/user/lab_navigation_ws/captures/hv_building_lab/20261010/images/query/test1.png
```

連続テスト:

```bash
ros2 run hloc_localizer image_file_publisher \
  --ros-args \
  -p image_path:=/home/user/lab_navigation_ws/captures/hv_building_lab/20261010/images/query/test1.png \
  -p publish_rate:=0.5
```

---

# 23. 今回のROS実測結果

`test1.png`:

```text
Method     : superpoint_lightglue
Keypoints  : 1020
Candidates : 20
3D matches : 6303
PnP corr.  : 1815
Inliers    : 約1120～1146
Success    : 11/11
```

ウォームアップ後:

```text
SuperPoint : 約0.02 s
NetVLAD    : 約0.06 s
Matching   : 約0.40 s
PnP        : 約0.03 s

TOTAL      : 約0.52 s
Rate       : 約1.9 Hz
```

---

# 24. pose確認

```bash
ros2 topic echo /hloc/camera_pose --once
```

---

# 25. TF確認

`map -> odom`:

```bash
ros2 run tf2_ros tf2_echo map odom
```

`map -> base_link`:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

---

# 26. 最短実行手順

SIFTしかない状態からSP+LG地図を作る場合:

```text
Step 1
元sparse/0を残す

Step 2
reference_images.txtを作る

Step 3
feats-ref-superpoint.h5を作る

Step 4
pairs-db-covis20.txtを作る

Step 5
matches-ref-superpoint-lightglue.h5を作る

Step 6
estimate_two_view_geometries=Trueでtriangulation

Step 7
sfm-superpoint-lightglue-colmapgvを確認

Step 8
ROSで local_method:=superpoint_lightglue
```

---

# 27. 最終ファイル構成

```text
20261010/
├── images/
│   ├── 00000001.png
│   ├── ...
│   └── query/
│       ├── test1.png
│       └── test2.png
│
├── sparse/
│   └── 0/
│       └── 元SIFT SfMモデル
│
├── hloc_output/
│   ├── feats-shift.h5
│   └── global-ref-netvlad.h5
│
└── hloc_output_superpoint_lightglue/
    ├── reference_images.txt
    ├── feats-ref-superpoint.h5
    ├── pairs-db-covis20.txt
    ├── matches-ref-superpoint-lightglue.h5
    ├── feats-query-superpoint.h5
    └── sfm-superpoint-lightglue-colmapgv/
```

---

# 28. 注意点

## 元のSIFTモデルを消さない

以下は元地図:

```text
$DATA/sparse/0
```

SuperPoint用は必ず:

```text
$SPOUT/sfm-superpoint-lightglue-colmapgv
```

へ作る。

---

## query画像を参照特徴量へ混ぜない

`images/` を再帰的にそのままSuperPoint抽出すると:

```text
images/query/test1.png
images/query/test2.png
```

まで対象になる可能性がある。

そのため、このREADMEでは:

```text
reference_images.txt
```

を元のCOLMAPモデルから生成して、その1370枚だけを抽出対象にしている。

---

## triangulationが0点の場合

今回実際に発生した。

通常のverification経路では3D点が0になったため:

```python
estimate_two_view_geometries=True
```

に変更した。

この設定ではCOLMAP/PyCOLMAP側でtwo-view geometryを再推定してからtriangulationする。

---

# 29. 今回の構成の考え方

最終的には:

```text
                       ┌─ SIFT + NN-ratio
                       │   └─ sparse/0
Image ── NetVLAD ──────┤
                       │
                       └─ SuperPoint + LightGlue
                           └─ sfm-superpoint-lightglue-colmapgv
```

という2系統を持つ。

NetVLADは共通。

ROS側では:

```bash
-p local_method:=sift
```

または:

```bash
-p local_method:=superpoint_lightglue
```

で切り替える。
