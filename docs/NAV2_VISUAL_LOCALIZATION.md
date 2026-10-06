# COLMAP の単眼 SfM 地図から Visual Localization を行い、Nav2 に接続する

調査日: 2026-09-23。対象: 単眼 USB カメラ、ROS 2 Humble、既存の Nav2。

この文書は、画像収集と SfM の次に行う作業の手順書です。このワークスペースで提供する収集システムは画像と時刻の記録を担当します。**オンラインの Visual Localization ノード、Nav2 用 TF 接続ノード、自動走行機能は今回の収集パッケージには含まれていません。** 以下では、既存ツールで試せるオフライン検証と、実機に接続するための実装を区別します。

## 1. 採用する構成

最初に COLMAP の `image_registrator` で、地図に使っていない画像の姿勢が求まることを確認します。その後、画像検索、特徴点照合、PnP を組み合わせる [hloc](https://github.com/cvg/Hierarchical-Localization) と [PyCOLMAP](https://colmap.github.io/pycolmap/pycolmap.html) をオンライン処理の部品として使います。COLMAP 自体にも既存地図への画像登録があり、下記の検証に利用できます。[COLMAP の新規画像登録](https://colmap.github.io/faq.html#register-localize-new-images-into-an-existing-reconstruction)

```text
現在の単眼画像 + 内部パラメータ + 撮影時刻
  → 参照画像の検索 → 2D 特徴点の照合
  → 参照画像の特徴点に対応する 3D 点を取得
  → 2D–3D PnP / RANSAC → カメラ姿勢と品質情報
  → 尺度・座標・取り付け位置の変換
  → 撮影時刻の odom を使って map → odom を計算

車輪オドメトリ / IMU など → odom → base_link を連続更新
Nav2 → map → odom → base_link → camera_optical_frame を参照
```

**単眼カメラで、既知の 3D 地図に対する 6 自由度の姿勢推定は可能です。** ステレオや RGB-D はこの方式の必須条件ではありません。ただし単眼 SfM の尺度は任意なので、Nav2 に渡す前にメートル単位に合わせます。また、画像照合の間をつなぐ連続的なオドメトリは別途必要です。

| 必要なもの | 準備・確認内容 |
|---|---|
| SfM 地図 | 参照画像、元の `database.db`、疎な復元モデル一式。PLY 点群だけでは不足 |
| 検証用画像 | 地図作成とは別の走行で収集した画像。まず 10〜30 枚程度で検証 |
| カメラ内部パラメータ | 運用する解像度・フォーカス・歪みモデルに一致したもの |
| 取り付け位置 | `base_link → camera_optical_frame`。実測または校正した固定変換 |
| メートル尺度・地図座標 | SfM 地図と Nav2 の `map` の対応。後述の相似変換 |
| ローカルオドメトリ | `odom → base_link` と `nav_msgs/Odometry`。現在の構成を確認する |
| Nav2 の地図・障害物検出 | 既存の occupancy map と LiDAR 等の障害物情報を確認する |

ROS 2 の TF に画像の特徴量や点群を渡す必要はありません。Nav2 が参照する座標系に、適切な姿勢を提供します。[Nav2 の TF 要件](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/)

## 2. COLMAP だけで、地図に使っていない画像を位置推定する

### 2.1 作業用コピーを用意する

この節は **SfM を行った別 PC** で実行します。まず [SfM 手順](COLMAP_SFM.md) に従って復元を完了してください。以下は収集データの `images/` と、SfM 手順で作った `colmap_radial_run01/database.db`、`colmap_radial_run01/sparse/0/` を使う例です。複数の復元モデルがある場合は、使用する経路を含むモデルを選びます。

`HOLDOUT_IMAGES` は別走行の `images` ディレクトリです。地図作成に使用済みの画像を指定しないでください。パスは実際の保存先に変更します。この節のコマンドは同じ Bash ターミナルで順に実行します。

```bash
set -e
SFM_PROJECT="$HOME/datasets/building_cw_01"
SFM_RESULT="$SFM_PROJECT/colmap_radial_run01"
HOLDOUT_IMAGES="$HOME/datasets/building_test/images"
LOC_PROJECT="$HOME/datasets/building_localization_test"
REF_MODEL="$SFM_RESULT/sparse/0"

test -f "$SFM_RESULT/database.db"
test -d "$REF_MODEL"
test -d "$HOLDOUT_IMAGES"
test ! -e "$LOC_PROJECT"
mkdir -p "$LOC_PROJECT/images/query" "$LOC_PROJECT/reference_txt" \
  "$LOC_PROJECT/registered" "$LOC_PROJECT/registered_txt"
cp "$SFM_RESULT/database.db" "$LOC_PROJECT/database.db"
cp -a "$SFM_PROJECT/images/." "$LOC_PROJECT/images/"
cp -a "$HOLDOUT_IMAGES/." "$LOC_PROJECT/images/query/"
colmap model_converter --input_path "$REF_MODEL" \
  --output_path "$LOC_PROJECT/reference_txt" --output_type TXT
export LOC_PROJECT
```

ここでコピーする `database.db` は、その SfM モデルを作ったデータベースです。異なる実行で特徴点を抽出し直した DB に置き換えないでください。モデル内の `image_id`、特徴点の配列番号、3D 点の対応を保つ必要があります。[COLMAP のデータベース形式](https://colmap.github.io/database.html)

以下は登録済み参照画像と query のペアを作ります。query は参照画像と同名にならないよう `query/` 以下に置きます。**この全 query × 全参照画像の照合は最初の小規模検証用**です。数千枚への連続照合には、画像検索による候補選択を使います。

```bash
python3 - <<'PY'
import os
from pathlib import Path

p = Path(os.environ['LOC_PROJECT'])
exts = {'.jpg', '.jpeg', '.png'}
queries = sorted(f.relative_to(p / 'images').as_posix()
                 for f in (p / 'images/query').rglob('*')
                 if f.suffix.lower() in exts)
# images.txt は画像行と観測点行の 2 行組。空の観測点行を消さない。
lines = [line for line in (p / 'reference_txt/images.txt').read_text().splitlines()
         if not line.startswith('#')]
references = [line.split(maxsplit=9)[9] for line in lines[::2]]
assert queries and references
assert not set(queries) & set(references), 'query と参照画像の名前が重複'
assert not any(any(c.isspace() for c in name) for name in queries + references)
(p / 'queries.txt').write_text('\n'.join(queries) + '\n')
with (p / 'query_reference_pairs.txt').open('w') as f:
    for q in queries:
        for r in references:
            f.write(f'{q} {r}\n')
print(f'{len(queries)} queries, {len(references)} references')
PY
```

### 2.2 query の特徴抽出・照合・登録

`reference_txt/cameras.txt` を開きます。各行は `CAMERA_ID MODEL WIDTH HEIGHT PARAMS...` です。同じ設定の単眼カメラで撮った場合、その最終モデルの `MODEL` と `PARAMS` を使います。次の処理はカメラが 1 種類の場合にそれらを取得します。複数種類あれば停止するため、query と一致するものを選んで設定してください。query の実解像度も `WIDTH HEIGHT` と一致させます。

```bash
read -r QUERY_MODEL QUERY_WIDTH QUERY_HEIGHT QUERY_PARAMS < <(python3 - <<'PY'
import os
from pathlib import Path
p = Path(os.environ['LOC_PROJECT']) / 'reference_txt/cameras.txt'
rows = [s.split() for s in p.read_text().splitlines() if s and not s.startswith('#')]
assert len(rows) == 1, 'query と同じ設定の CAMERA_ID を明示的に選んでください'
r = rows[0]
print(r[1], r[2], r[3], ','.join(r[4:]))
PY
)
test -n "$QUERY_PARAMS"
```

以下の CPU 指定は、旧版と新版で異なるオプション名を `-h` から選びます。これは速度より動作確認を優先する設定です。GPU を使う際はインストールした版のヘルプを確認してください。元の地図が SIFT で作られていることを前提とします。

```bash
extract_help="$(colmap feature_extractor -h 2>&1)"
match_help="$(colmap matches_importer -h 2>&1)"
if [[ "$extract_help" == *FeatureExtraction.use_gpu* ]]; then
  extract_gpu=(--FeatureExtraction.use_gpu 0)
else
  extract_gpu=(--SiftExtraction.use_gpu 0)
fi
if [[ "$match_help" == *FeatureMatching.use_gpu* ]]; then
  match_gpu=(--FeatureMatching.use_gpu 0)
else
  match_gpu=(--SiftMatching.use_gpu 0)
fi

colmap feature_extractor \
  --database_path "$LOC_PROJECT/database.db" \
  --image_path "$LOC_PROJECT/images" \
  --image_list_path "$LOC_PROJECT/queries.txt" \
  --ImageReader.single_camera 1 \
  --ImageReader.camera_model "$QUERY_MODEL" \
  --ImageReader.camera_params "$QUERY_PARAMS" \
  "${extract_gpu[@]}"

colmap matches_importer \
  --database_path "$LOC_PROJECT/database.db" \
  --match_list_path "$LOC_PROJECT/query_reference_pairs.txt" \
  --match_type pairs "${match_gpu[@]}"

colmap image_registrator \
  --database_path "$LOC_PROJECT/database.db" \
  --input_path "$REF_MODEL" \
  --output_path "$LOC_PROJECT/registered" \
  --Mapper.abs_pose_refine_focal_length 0 \
  --Mapper.abs_pose_refine_extra_params 0

colmap model_converter --input_path "$LOC_PROJECT/registered" \
  --output_path "$LOC_PROJECT/registered_txt" --output_type TXT
colmap model_analyzer --path "$LOC_PROJECT/registered"
```

query の出力結果は `registered_txt/images.txt` の `query/...` の行です。登録されなかった query は位置推定に失敗しています。元モデルを固定した位置推定の評価では、ここで `mapper` や全体の bundle adjustment を実行しません。それらは地図そのものを変更し得ます。`image_registrator` の実装は、未登録画像の姿勢を追加する処理です。[COLMAP 実装](https://github.com/colmap/colmap/blob/3.13.0/src/colmap/exe/image.cc)

確認項目は、登録率、実測した位置・向きとの差、逆方向からの成功率、処理時間です。特徴点がよく似た別の場所に誤って登録されることもあるため、登録成功だけを合格条件にしません。

## 3. hloc を使う場合の準備と具体例

### 3.1 別 PC の専用 Python 環境にインストール

hloc は ROS 2 の完成済み localization ノードではなく、Python の処理部品と評価用 pipeline です。ROS の Python 環境と分離してオフライン検証から始めます。以下は調査で確認した hloc の commit を固定した導入例です。**このワークスペース上で hloc・モデル重み・GPU 環境のインストールや実データ実行はしていません。**

```bash
sudo apt install git python3-venv
python3 -m venv "$HOME/venvs/hloc"
source "$HOME/venvs/hloc/bin/activate"
python -m pip install --upgrade pip
mkdir -p "$HOME/src"
git clone --recursive https://github.com/cvg/Hierarchical-Localization.git \
  "$HOME/src/Hierarchical-Localization"
cd "$HOME/src/Hierarchical-Localization"
git checkout c13273bd0ecc2917a35910fd843712a1c6243193
git submodule update --init --recursive
python -m pip install 'pycolmap==3.13.0'
python -m pip install -e .
python -m pip check
python -c 'import hloc, pycolmap, torch; print(pycolmap.__version__); print("CUDA:", torch.cuda.is_available())'
python -m pip freeze > environment-resolved.txt
git rev-parse HEAD > hloc-commit.txt
```

この commit の依存条件は `pycolmap>=3.13.0` です。上記では API を揃えるため 3.13.0 を指定しています。PyTorch/CUDA は GPU・ドライバーに合う構成で別途準備し、解決された依存バージョンを保存します。初回の特徴抽出時にはモデル重みのダウンロードが発生します。[hloc の依存条件](https://github.com/cvg/Hierarchical-Localization/blob/c13273bd0ecc2917a35910fd843712a1c6243193/requirements.txt)、[公式導入手順](https://github.com/cvg/Hierarchical-Localization#installation)

### 3.2 SIFT 地図に SuperPoint の特徴点を直接接続しない

SfM モデルの各 3D 点は、参照画像中の**特徴点配列の番号**に対応しています。COLMAP SIFT と hloc SuperPoint では、その位置・数・配列番号が異なります。SIFT 点群を残したまま SuperPoint の番号を参照すると、誤った 3D 点へ対応付けてしまいます。同じ SIFT という名前でも、画像サイズや抽出条件を変えて再抽出した配列をそのまま使うことはできません。

既存のカメラ姿勢を活用する場合は、参照画像から SuperPoint を抽出し、参照画像間を LightGlue で照合して、**既存の撮影姿勢から新しい特徴点を再三角測量**します。hloc の `triangulation.main` がこの作業を担当します。別の方法は、hloc で最初から一貫した特徴量を使って SfM をやり直すことです。[hloc の再三角測量](https://github.com/cvg/Hierarchical-Localization/blob/c13273bd0ecc2917a35910fd843712a1c6243193/hloc/triangulation.py)、[既存 SIFT 姿勢を使う公式 pipeline](https://github.com/cvg/Hierarchical-Localization/blob/c13273bd0ecc2917a35910fd843712a1c6243193/hloc/pipelines/Aachen/pipeline.py)

### 3.3 手元の画像で再三角測量と query localization を行う例

2 節で用意した作業ディレクトリを使います。仮想環境を有効にし、`LOC_PROJECT` と `REF_MODEL` を設定・export したターミナルで実行します。小規模データから試し、出力は必ず新しいディレクトリにします。

```bash
export LOC_PROJECT REF_MODEL
python - <<'PY'
import os
from pathlib import Path
import pycolmap
from hloc import extract_features, match_features, pairs_from_covisibility
from hloc import pairs_from_retrieval, triangulation, localize_sfm

p = Path(os.environ['LOC_PROJECT'])
reference = Path(os.environ['REF_MODEL'])
images = p / 'images'
out = p / 'hloc'
out.mkdir(exist_ok=False)
model = pycolmap.Reconstruction(reference)
refs = sorted(im.name for im in model.images.values())
queries = (p / 'queries.txt').read_text().splitlines()
assert len(refs) >= 2 and queries
assert len(model.cameras) == 1, 'query に対応する校正を明示的に選んでください'
camera = next(iter(model.cameras.values()))
query_calibration = out / 'queries_with_intrinsics.txt'
with query_calibration.open('w') as f:
    params = ' '.join(map(str, camera.params))
    for name in queries:
        f.write(f'{name} {camera.model_name} {camera.width} {camera.height} {params}\n')

feature_conf = extract_features.confs['superpoint_aachen']
matcher_conf = match_features.confs['superpoint+lightglue']
features = extract_features.main(
    feature_conf, images, out, as_half=False, image_list=refs + queries)

map_pairs = out / 'pairs-map.txt'
pairs_from_covisibility.main(reference, map_pairs, num_matched=20)
map_matches = match_features.main(
    matcher_conf, map_pairs, features, matches=out / 'matches-map.h5')
hloc_model = out / 'reference_sfm'
triangulation.main(hloc_model, reference, images, map_pairs, features, map_matches)

# 候補は復元モデルに含まれる参照画像だけから選ぶ。
descriptors = extract_features.main(
    extract_features.confs['netvlad'], images, out, image_list=refs + queries)
query_pairs = out / 'pairs-query.txt'
pairs_from_retrieval.main(
    descriptors, query_pairs, num_matched=min(20, len(refs)),
    query_list=queries, db_list=refs)
query_matches = match_features.main(
    matcher_conf, query_pairs, features, matches=out / 'matches-query.h5')
localize_sfm.main(
    hloc_model, query_calibration, query_pairs, features, query_matches,
    out / 'query-poses.txt', ransac_thresh=4,
    covisibility_clustering=True)
PY
```

この例は参照モデルの相対画像名と実ファイルを保つこと、query が同じ内部パラメータ・解像度であることが前提です。内部で縮小して特徴抽出した座標は hloc が元画像座標へ戻します。`ransac_thresh=4` は元画像の画素単位の試験初期値であり、実データで調整してください。[特徴抽出の座標処理](https://github.com/cvg/Hierarchical-Localization/blob/c13273bd0ecc2917a35910fd843712a1c6243193/hloc/extract_features.py)、[画像検索 API](https://github.com/cvg/Hierarchical-Localization/blob/c13273bd0ecc2917a35910fd843712a1c6243193/hloc/pairs_from_retrieval.py)

**`query-poses.txt` に姿勢があるだけでは、実機へ入力しないでください。** 調査した hloc には、`covisibility_clustering=False` のとき PnP が失敗すると最近傍参照画像の姿勢を出力する fallback があります。上の例では clustering を有効にしていますが、それでも最低 inlier 数などの運用上の条件は別途必要です。自身が生成した `query-poses.txt_logs.pkl` の `PnP_ret`、`num_inliers`、inlier 対応点を確認します。オンライン化では `QueryLocalizer.localize` の `None` を失敗として扱い、成功時だけ品質検査へ進めます。[hloc の姿勢推定・fallback 実装](https://github.com/cvg/Hierarchical-Localization/blob/c13273bd0ecc2917a35910fd843712a1c6243193/hloc/localize_sfm.py)

## 4. SfM 座標をメートル単位の Nav2 地図に合わせる

単眼 SfM の座標 `p_sfm` と Nav2 の座標 `p_map` は、一般に次の相似変換で対応します。

```text
p_map = scale × R_map_sfm × p_sfm + translation_map_sfm
```

実測距離が 1 本あれば尺度を定められますが、それだけで原点と方位は決まりません。既存の Nav2 地図を使う場合は、地図上の座標が分かる複数地点で、カメラ光学中心の位置を対応付けます。最低 3 点を一直線に置かず、経路全体に分散した複数点を使い、合わせ込みに使わない地点でも誤差を検証します。`base_link` の位置を使う場合は取り付けオフセットを考慮してカメラ中心へ変換します。

対応ファイルの各行は以下の形式です。`x y z` は Nav2 の map 座標系で測ったカメラ中心の位置、単位は m です。

```text
00000001.png x1 y1 z1
00000200.png x2 y2 z2
00000400.png x3 y3 z3
```

数値を実測値に置き換えて `camera_centers_map.txt` に保存したら、例えば以下で新しいモデルに変換します。0.10 m は対応点の誤差を除外するための例で、測定精度に合わせて変更します。

```bash
align_help="$(colmap model_aligner -h 2>&1)"
if [[ "$align_help" == *robust_alignment_max_error* ]]; then
  align_error=(--robust_alignment_max_error 0.10)
else
  align_error=(--alignment_max_error 0.10)
fi
mkdir -p "$SFM_RESULT/sparse_map"
colmap model_aligner --input_path "$REF_MODEL" \
  --output_path "$SFM_RESULT/sparse_map" \
  --ref_images_path "$SFM_PROJECT/camera_centers_map.txt" \
  --ref_is_gps 0 --alignment_type custom "${align_error[@]}" \
  --transform_path "$SFM_RESULT/sfm_to_map.txt"
```

以後この `sparse_map` を位置推定の参照モデルにすれば、出力を map 座標で扱えます。hloc の再三角測量後のモデルを整列する場合は、そのモデルを `--input_path` にします。変換前後のモデルと変換ファイルを両方保存してください。[COLMAP の整列実装](https://github.com/colmap/colmap/blob/main/src/colmap/exe/model.cc)

**TF は回転と並進を表すもので、尺度を保持できません。** 任意尺度のモデルを static TF で `map` につなぐだけでは不十分です。地図全体を変換するか、推定したカメラ中心に上の尺度を適用してから TF を構成します。回転行列やカメラ取り付け寸法を尺度倍しないでください。カメラ中心は尺度倍し、姿勢の回転は `R_map_sfm` で回転します。

## 5. カメラ姿勢を ROS のロボット姿勢へ変換する

`T_A_B` を「B 座標の点を A 座標へ変換する行列」と定義します。COLMAP の `images.txt` の姿勢および PyCOLMAP の `cam_from_world` は `T_camera_world` です。カメラの世界座標での姿勢が必要なときは逆行列を使います。

```text
COLMAP:        p_camera = R × p_world + t
カメラ中心:    C_world = -Rᵀ × t
カメラ姿勢:    T_world_camera = inverse(T_camera_world)
```

COLMAP のカメラ軸は x 右・y 下・z 前です。ROS の `camera_optical_frame` と対応させ、x 前・y 左・z 上の `base_link` に直接読み替えないでください。COLMAP テキストの quaternion は `qw qx qy qz`、ROS メッセージは `x y z w` です。[COLMAP 姿勢の定義](https://colmap.github.io/format.html#images-txt)

メートル尺度へ合わせた後、固定変換 `T_base_camera` を使います。

```text
T_map_base(t_image) = T_map_camera(t_image) × inverse(T_base_camera)
T_map_odom          = T_map_base(t_image) × inverse(T_odom_base(t_image))
```

**最後の odom は、推論完了時刻ではなく画像の撮影時刻の値**です。たとえば画像処理に 0.8 秒かかったとき、最新 odom と過去の画像姿勢を組み合わせると、その間の移動が誤差になります。TF バッファには最大推論遅延より長い履歴を保持します。カメラドライバーの stamp が露光時刻とどの程度一致するかも評価してください。

## 6. オンライン ROS 2 ノードとして実装する内容

以下はこれから作るノードの入出力仕様です。**この名前の実行ファイルや launch が既に提供されているという意味ではありません。** hloc のファイル処理例を全フレームでそのまま起動するのではなく、地図・画像検索用特徴量・ネットワーク重みを一度読み込む常駐処理にします。

| 入出力 | 内容 |
|---|---|
| 入力 image | `sensor_msgs/Image` または `CompressedImage`。画像の `header.stamp` を保存 |
| 入力 calibration | 対応する `CameraInfo` と COLMAP の投影モデル。未校正の K=0 を使わない |
| 入力 TF | 固定 `base_link → camera_optical_frame`、時刻付き `odom → base_link` |
| 地図 | 整列済み疎モデル、参照画像名、特徴点・特徴量・3D 対応、検索用特徴量 |
| 出力 pose | map 内の **base_link の姿勢**、元画像の stamp、成功/失敗と品質情報 |
| 出力 TF | ノードが担当する構成では `map → odom` のみ |

広角では、画像、投影モデル、内部パラメータの組を揃えます。魚眼画像に通常の pinhole 歪みモデルをそのまま使わず、校正したモデルで PnP を解きます。歪み補正するなら、補正後の画像とその新しい内部パラメータを地図・query の双方で一貫して扱います。OpenCV 系と COLMAP の画素中心規約の違いは、hloc の標準処理と重複して補正しないようにします。[COLMAP のカメラモデル](https://colmap.github.io/cameras.html)

処理は次のように構成します。

1. 推論中に次々と画像を積まず、最新の未処理画像を選ぶ。まず 1〜2 Hz で性能を測る。
2. 画像検索で参照候補を取得し、特徴点照合から 2D–3D 対応を作る。
3. PyCOLMAP の姿勢推定と refinement を行い、成功結果と inlier mask を受け取る。
4. 下記の品質判定を通った結果だけ、map 座標の base_link 姿勢へ変換する。
5. 撮影時刻の odom から `map → odom` を更新する。時刻の逆転・過度に古い結果は捨てる。
6. 最後に採用した `map → odom` の値をタイマーで継続配信する。画像の pose メッセージは元の撮影時刻を保つ。

TF の再配信時刻は現在の ROS 時刻にし、補正値は最後に検証した値を保持します。その間、現在の `odom → base_link` が移動を表します。推定結果の時刻を現在に書き換えて「現在の base_link の実測姿勢」として配信する処理とは異なります。最後の成功時刻は別途監視し、長時間の失敗を単なる再配信で隠さないでください。bag 評価ではノードと Nav2 の双方を `use_sim_time:=true` に揃えます。

品質判定では、少なくとも次を確認します。数値は環境・地図・解像度・移動速度で変わるため、検証データに基づいて設定します。

- PnP 成功、十分な inlier 数と inlier 比率、再投影誤差の中央値・上位分位点。
- inlier が画像の一角や一本の線に集中していないこと、3D 点の空間的な分布。
- カメラの高さ、地面に対する向き、予測位置との差が運用条件から外れていないこと。
- 撮影からの経過時間、最後に採用した結果より新しいこと、TF 履歴が存在すること。
- 初期位置が未知のときと追跡中を分けること。初期化時は odom 原点への近さを条件にせず、複数画像による一貫性などで確認する。

失敗時は未検証の補正を入力せず、短時間は既存の odom でつなぎます。許容時間を超えたら navigation を停止・キャンセルできる監視を別途設け、再局在化を行います。必要な更新頻度は固定値ではなく、odom のドリフト、走行速度、許容誤差、推論遅延の計測から決めます。

### robot_localization を使う場合

直接 TF を更新する代わりに、map 内の base_link 姿勢を `geometry_msgs/PoseWithCovarianceStamped` で公開し、global EKF に融合する構成もあります。その場合は EKF を `map → odom` の唯一の配信者にし、Visual Localization ノードの TF 配信を止めます。local EKF または既存の odometry ノードが `odom → base_link` を担当します。

既存の global EKF 設定へ追加する部分の例です。これは完成した EKF 設定ではなく、既存の odom/IMU 入力と合わせて調整する断片です。

```yaml
ekf_global:
  ros__parameters:
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: map
    publish_tf: true
    two_d_mode: true   # 平面走行の場合
    pose0: /visual_localization/pose
    pose0_config: [true, true, false,
                   false, false, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    pose0_differential: false
    pose0_relative: false
    pose0_queue_size: 5
```

covariance をゼロや適当な小さい数で埋めないでください。画像残差から得た推定共分散を座標変換し、尺度・取り付け校正・地図誤差も考慮するか、独立した検証走行の誤差から保守的に校正します。遅れて届く測定は元の stamp を維持し、利用する robot_localization の履歴再処理設定と必要な履歴長を確認します。検証できていない共分散で融合するより先に、pose を記録して精度を評価します。[robot_localization の Humble 設定例](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml)

## 7. 既存の Nav2 に接続する順序

### 7.1 現在の TF と自己位置推定を確認する

実機の既存起動後に確認します。

```bash
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list -t
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_tools view_frames
```

フレーム名は実機に合わせます。AMCL、SLAM Toolbox、global EKF などが既に `map → odom` を配信しているなら、その担当を決めてから切り替えます。画像方式と AMCL の両方から同じ TF を同時配信しません。まず既存 localization を使ったまま、画像推定結果を pose トピックとログだけに出し、位置誤差とジャンプを評価するのが実装を検証しやすい手順です。

### 7.2 地図と costmap を準備する

疎な SfM 点群は「点が存在しない場所が通行可能」という意味ではありません。Nav2 の既存 occupancy map、ロボット footprint、障害物観測、costmap を利用し、その map 座標に視覚地図を整列します。単眼カメラしかなく障害物検出・メートル単位の odom・走行可能領域が未整備なら、その準備も必要です。自己位置推定の導入だけでそれらが生成されるわけではありません。[Nav2 の mapping / localization](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/)

### 7.3 TF の担当を切り替えて navigation を起動する

接続ノードの検証が完了したら、既存の localization の起動を外すか TF 配信を止めます。map server は別途継続し、ローカル odom、センサー、URDF、Visual Localization、TF の順に確認します。

Humble の `navigation_launch.py` は navigation の各サーバーを起動し、AMCL や map server は起動しません。既存のパラメータファイルを使う起動例は以下です。既存の navigation が起動済みなら二重に実行しません。[Humble の launch 実装](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py)

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/absolute/path/to/your_nav2_params.yaml \
  use_sim_time:=false
```

このコマンドの前に、既存 map server の lifecycle が active であること、`map → odom → base_link` が継続して得られることを確認します。`global_costmap.global_frame` は通常 `map`、`local_costmap.global_frame` は通常 `odom`、robot base frame と odom トピックは実機の名前に揃えます。既存の controller、障害物センサー、速度制限などの設定は引き継ぎます。

### 7.4 完了判定

- 静止中と低速移動中の両方で、独立した参照に対する位置・方位の誤差を測定した。
- 初期位置が未知でも局在化でき、逆方向、曲がり角、照明差でも成功率を確認した。
- 画像が途切れたとき、誤照合したとき、推論が遅れたときの動作を確認した。
- `map → odom` の配信者が 1 つで、ローカル odom に画像補正による不連続を入れていない。
- RViz でロボット footprint、occupancy map、障害物観測の重なりが妥当である。
- 上記を記録データと手動走行で確認してから、短い目標への Nav2 走行を試す。

## 8. 保存しておく成果物

運用モデルごとに、整列前後の COLMAP モデル、元画像、元 DB、hloc 特徴量と対応、query の校正、SfM→map 変換、カメラ取り付け変換、ソフトウェア版、品質判定の設定、独立した検証結果をまとめます。特徴点ファイルと 3D 地図を別々の実験から混ぜないよう、モデル一式を同じ版として扱ってください。
