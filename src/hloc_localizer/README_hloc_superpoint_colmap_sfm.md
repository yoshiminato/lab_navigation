# SuperPoint + LightGlue で位置制約付き SfM 地図を作る

全体の作業順は [Visual Localization 作業フロー](../../docs/VISUAL_LOCALIZATION_WORKFLOW.md) を参照してください。この手順は画像・カメラ情報・画像ごとのカメラ中心位置が揃った収集セッションから、新しいSuperPoint地図を作ります。既存SIFT地図の姿勢を再利用する場合は [再三角測量手順](README_HLoc_ROS2_SP_LG.md) を使用します。

## パスと成果物

最初に `SESSION_NAME` だけ設定します。別端末で続ける場合は変数と `source` を再実行します。

```bash
WORKSPACE_DIR="$HOME/lab_navigation_ws"
SESSION_NAME="isaac_001"
CAPTURE_DIR="$WORKSPACE_DIR/captures/$SESSION_NAME"
IMAGE_DIR="$CAPTURE_DIR/images"
POSE_DIR="$CAPTURE_DIR/poses_world_camera_gt"
SUPERPOINT_DIR="$CAPTURE_DIR/maps/superpoint"
RETRIEVAL_DIR="$CAPTURE_DIR/retrieval"
RUN_NAME="run01"
RUN_DIR="$CAPTURE_DIR/runs/superpoint/$RUN_NAME"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
source .venv/bin/activate
export PYTHONPATH="$WORKSPACE_DIR/third_party/Hierarchical-Localization:${PYTHONPATH:-}"

test -d "$IMAGE_DIR" && test -f "$POSE_DIR/poses.csv" && test -f "$CAPTURE_DIR/camera_info.json"
```

```text
<session>/
├── images/                         撮影画像
├── camera_info.json
├── poses_world_camera_gt/poses.csv
├── maps/
│   ├── sift/                       SIFT地図があれば並存
│   └── superpoint/
│       ├── database.db             pose prior付きCOLMAP DB
│       ├── sparse/0/               採用したSfMモデル
│       └── reference_features.h5   モデルと同じSuperPoint特徴点
├── retrieval/netvlad.h5           両方式で共有する検索特徴量
└── runs/superpoint/<run名>/         pairs、matches、作業用DB
```

## 一括生成

`maps/superpoint` と `runs/superpoint/$RUN_NAME` が存在しない新しいセッションで実行します。両方の出力先が既にある場合は停止し、上書きしません。地図は最初にrun内で組み立て、COLMAPがモデルを作った後で `maps/superpoint` に移します。途中で失敗した場合は中間ファイルを確認し、原因を直してから別のrun名で再実行できます。

```bash
python3 script/build_superpoint_map.py "$CAPTURE_DIR" --run-name "$RUN_NAME"
```

このスクリプトは、`poses.csv` で位置が有効かつ画像が存在する参照画像だけを選び、SuperPoint抽出 → 位置に基づく画像ペア作成 → LightGlue照合 → COLMAP DBへのkeypoints・matches登録と幾何検証 → 位置prior登録 → `pose_prior_mapper` → 参照データ検証・NetVLAD生成を順に実行します。`camera_info.json` の焦点距離と主点から `PINHOLE` を設定します。歪みがゼロでない画像はエラーにします。別のカメラモデルが必要な場合は、補正画像・CameraInfoを確認してから対応する生成方法を選んでください。

既定の空間ペア条件は半径3 m、各画像の近傍30枚です。順序上の近傍も追加する例:

```bash
python3 script/build_superpoint_map.py "$CAPTURE_DIR" \
  --run-name "$RUN_NAME" --max-distance 3.0 \
  --max-neighbors 30 --sequential-overlap 5
```

`poses.csv` に位置があっても、ペア選択だけではメートル単位の復元になりません。この手順は位置をCOLMAP DBのpriorとして登録し、`pose_prior_mapper`で復元に使います。`poses.csv`の位置は車体原点ではなく撮影カメラ中心である必要があります。

## 既にある地図を検証

`isaac_001` には既存のSuperPoint地図があります。生成スクリプトを再実行せず、次の検証を使います。

```bash
python3 script/prepare_localization.py "$CAPTURE_DIR" --method superpoint --verify-only
```

登録画像数、3D点数、モデルの各画像の2D点数とH5のkeypoints数、NetVLADの欠損数を確認します。`Missing NetVLAD descriptors: 0` が必要です。不足するNetVLADだけ追加する場合は `--verify-only` を外します。

```bash
python3 script/prepare_localization.py "$CAPTURE_DIR" --method superpoint
```

## 処理段階を個別に実行する場合

一括生成スクリプトの各段階は [build_superpoint_map.py](../../script/build_superpoint_map.py) で確認できます。主要な作業用CLIは [make_spatial_pairs.py](../../script/superpoint_sfm/make_spatial_pairs.py) と [build_hloc_colmap_db.py](../../script/superpoint_sfm/build_hloc_colmap_db.py) です。これらは入力・出力パスを明示的に受け取るため、特徴量を `$SUPERPOINT_DIR/reference_features.h5`、ペアとマッチを `$RUN_DIR`、最終DBとモデルを `$SUPERPOINT_DIR` に指定します。

COLMAPで `No images with matches` と出たら、実際に指定したDBの `images` と `two_view_geometries` を調べます。シェル変数が別端末で未設定、DBのパスが違う、マッチ登録や幾何検証が失敗した、といった場合に0件になります。`Creating SIFT CPU feature matcher` というログだけでは、HLocが登録したSuperPoint特徴点とLightGlueの対応がSIFTに置き換わったことを意味しません。

```bash
python3 - "$SUPERPOINT_DIR/database.db" <<'PY'
import sqlite3
import sys
with sqlite3.connect(sys.argv[1]) as db:
    for table in ('images', 'keypoints', 'matches', 'two_view_geometries', 'pose_priors'):
        print(table, db.execute(f'SELECT COUNT(*) FROM {table}').fetchone()[0])
PY
```

## ROS 2で使用

`data_path` は地図フォルダではなくセッションルートです。方式だけ切り替えます。

```bash
ros2 launch lab_navigation navigation_isaacsim_visual.launch.py \
  data_path:="$CAPTURE_DIR" local_method:=superpoint \
  start_navigation:=false
```

SIFTとSuperPointは2D特徴点の座標・順序が異なるため、地図とH5を方式間で混ぜないでください。
