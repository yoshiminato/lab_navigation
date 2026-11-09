# 既存SIFT地図の姿勢を使ってSuperPoint地図を追加する

SIFTのカメラ姿勢を固定し、SuperPoint特徴点とLightGlue照合から3D点を再三角測量します。SIFTの2D点と3D点の対応をそのまま流用することはできません。SuperPointだけで位置制約付きSfMを行う場合は [SuperPoint SfM手順](README_hloc_superpoint_colmap_sfm.md) を参照してください。

## 準備

```bash
WORKSPACE_DIR="$HOME/lab_navigation_ws"
SESSION_NAME="20261010"
CAPTURE_DIR="$WORKSPACE_DIR/captures/hv_building_lab/$SESSION_NAME"
IMAGE_DIR="$CAPTURE_DIR/images"
SIFT_DIR="$CAPTURE_DIR/maps/sift"
SUPERPOINT_DIR="$CAPTURE_DIR/maps/superpoint"
RUN_NAME="triangulated_run02"
RUN_DIR="$CAPTURE_DIR/runs/superpoint/$RUN_NAME"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
source .venv/bin/activate
export PYTHONPATH="$WORKSPACE_DIR/third_party/Hierarchical-Localization:${PYTHONPATH:-}"
export IMAGE_DIR SIFT_DIR SUPERPOINT_DIR RUN_DIR

test -d "$SIFT_DIR/sparse/0" && test -d "$IMAGE_DIR" || { echo "SIFT地図または画像がありません"; exit 1; }
test ! -e "$SUPERPOINT_DIR" && test ! -e "$RUN_DIR" || { echo "出力先が既にあります"; exit 1; }
mkdir -p "$RUN_DIR" "$SUPERPOINT_DIR/sparse"
```

既に `maps/superpoint` があるセッションでは、既存の完成地図を保持し、セッション全体の作業コピーで実行してください。

## 参照画像一覧と特徴抽出

SIFTモデルに登録済みの画像だけを対象にします。

```bash
python3 - <<'PY'
import os
from pathlib import Path
import pycolmap

model = pycolmap.Reconstruction(Path(os.environ['SIFT_DIR']) / 'sparse/0')
names = sorted(image.name for image in model.images.values())
path = Path(os.environ['RUN_DIR']) / 'reference_images.txt'
path.write_text('\n'.join(names) + '\n')
print(len(names), path)
PY

python3 -m hloc.extract_features \
  --image_dir "$IMAGE_DIR" --export_dir "$SUPERPOINT_DIR" \
  --image_list "$RUN_DIR/reference_images.txt" \
  --feature_path "$SUPERPOINT_DIR/reference_features.h5" \
  --conf superpoint_aachen --as_half
```

## CovisibilityペアとLightGlue照合

```bash
python3 -m hloc.pairs_from_covisibility \
  --model "$SIFT_DIR/sparse/0" \
  --output "$RUN_DIR/pairs_covis20.txt" --num_matched 20

python3 - <<'PY'
import os
from pathlib import Path
from hloc import match_features

run = Path(os.environ['RUN_DIR'])
features = Path(os.environ['SUPERPOINT_DIR']) / 'reference_features.h5'
match_features.main(
    match_features.confs['superpoint+lightglue'],
    run / 'pairs_covis20.txt', features,
    matches=run / 'matches_lightglue.h5',
)
PY
```

## カメラ姿勢を固定して再三角測量

```bash
python3 - <<'PY'
import os
from pathlib import Path
from hloc import triangulation

session_images = Path(os.environ['IMAGE_DIR'])
sift = Path(os.environ['SIFT_DIR'])
superpoint = Path(os.environ['SUPERPOINT_DIR'])
run = Path(os.environ['RUN_DIR'])
result = triangulation.main(
    sfm_dir=superpoint / 'sparse/0',
    reference_model=sift / 'sparse/0',
    image_dir=session_images,
    pairs=run / 'pairs_covis20.txt',
    features=superpoint / 'reference_features.h5',
    matches=run / 'matches_lightglue.h5',
    estimate_two_view_geometries=True,
    verbose=True,
)
print(result.summary())
PY
```

`estimate_two_view_geometries=True` は、登録したマッチからtwo-view geometryを推定して3D点を作るために使います。SIFT参照モデルの座標系とカメラ姿勢を維持します。再三角測量の作業DBが `sparse/0/database.db` に生成された場合は、モデルの検証後に `$SUPERPOINT_DIR/database.db` へ移動してください。生成されない構成ではDBはオンライン測位に不要です。

## 検証と起動

```bash
python3 script/prepare_localization.py "$CAPTURE_DIR" --method superpoint
ros2 launch lab_navigation navigation_isaacsim_visual.launch.py \
  data_path:="$CAPTURE_DIR" local_method:=superpoint \
  start_navigation:=false
```

この検証は、モデルの登録画像とH5の特徴点数が一致するか、共通の `retrieval/netvlad.h5` に検索特徴量が揃っているかを確認します。オンラインlocalizerは `data_path` にセッションルートを受け取り、`local_method` で `maps/sift` と `maps/superpoint` を選びます。
