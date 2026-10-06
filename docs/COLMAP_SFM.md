# 収集した単眼画像から COLMAP で SfM 地図を作る

確認日: **2026-09-23**。実行場所は画像を転送した**別のPC**を想定します。この文書は手順書であり、このワークスペースで建物の SfM が完了したという意味ではありません。

収集方法は [SFM_CAPTURE](SFM_CAPTURE.md)、SfM 後の自己位置推定と Nav2 接続は [NAV2_VISUAL_LOCALIZATION.md](NAV2_VISUAL_LOCALIZATION.md) を参照してください。

## 1. 今回作るものと保存形式

COLMAP の SfM は画像の特徴抽出 → 対応付け → カメラ姿勢・3D点の復元という処理です。Visual Localization の初期検証には**疎な SfM モデル**を使います。密な点群やメッシュを作る MVS は、この手順の必須工程ではありません。[公式チュートリアル](https://colmap.github.io/tutorial.html#structure-from-motion)

推奨する入力構成は次のとおりです。

```text
session/
├── images/
│   ├── 00000001.png
│   ├── 00000002.png
│   └── ...
├── timestamps.csv
└── カメラ設定・キャリブレーション等のメタデータ
```

- JPEG/PNG の通常の静止画を使います。専用動画形式や ROS bag を直接 COLMAP に渡す必要はありません。
- **ゼロ埋めしたファイル名の辞書順を撮影順にします。** Sequential Matching が使う順番はファイル名であり、ファイル更新日時、CSVの時刻、データベースの登録順ではありません。
- `timestamps.csv` はオドメトリ対応付けや記録確認用です。COLMAP が自動的に読み込む入力ではありません。
- 画像は同じ解像度・フォーカス・ズームで、未補正の状態を保持します。切り抜き、電子手ぶれ補正、歪み補正を混在させません。
- COLMAP 内の画像名は `images/` からの相対パスです。特徴抽出後はファイル名・相対配置を変えません。

この収集システムの既定値は **PNG の連番保存**です。カメラの MJPEG をデコードし、保存時の追加の非可逆圧縮を避けます。元の MJPEG 圧縮で失われた情報が PNG 化で戻るわけではありません。COLMAP にとって JPEG/PNG のどちらも有効な入力です。[公式: 入力構成と Sequential Matching](https://colmap.github.io/tutorial.html#data-structure)

## 2. 別PCの環境構築とバージョン確認

Ubuntu の最初の検証はパッケージ版と CPU 処理で始められます。

```bash
sudo apt update
sudo apt install colmap python3
colmap -h
colmap feature_extractor -h
colmap sequential_matcher -h
colmap mapper -h
```

`apt` 版のバージョンは Ubuntu によって異なります。公式によるとディストリビューション標準パッケージには CUDA 対応が含まれないため、GPU で大規模に処理する際は、別PCに合う公式バイナリ・Docker・ソースビルドを選びます。CPU でも疎な SfM は実行できます。GPU 環境の構築方法は [公式インストール手順](https://colmap.github.io/install.html) を参照してください。

**オンラインの最新マニュアルと手元の実行ファイルのオプション名が一致するとは限りません。** 確認時のオンライン文書は `4.3.0.dev0` です。旧版の代表として公式 `3.8` ソースも確認しています。

| 設定 | 旧版 3.8 の名前 | 確認時の現行文書の名前 |
|---|---|---|
| 特徴抽出を CPU で実行 | `SiftExtraction.use_gpu` | `FeatureExtraction.use_gpu` |
| 特徴照合を CPU で実行 | `SiftMatching.use_gpu` | `FeatureMatching.use_gpu` |
| 地図の座標合わせで使う RANSAC 許容誤差 | `robust_alignment_max_error` | `alignment_max_error` |

以下では `-h` の出力を判定して GPU 指定名を選びます。版番号だけで名前を推測しません。[現行 CLI](https://colmap.github.io/cli.html)、[旧版オプション定義](https://github.com/colmap/colmap/blob/3.8/src/util/option_manager.cc)、[旧版モデル変換コマンド](https://github.com/colmap/colmap/blob/3.8/src/exe/model.cc)

## 3. 作業フォルダを準備する

以下のコードは **Bash の同じターミナル**で順番に実行します。`sfm_project` は転送済みのセッションの絶対パスに置き換えてください。`sfm_result` は未使用の名前にし、設定を比較するときは新しい名前に変えます。

```bash
sfm_project="$HOME/datasets/building_cw_01"
sfm_result="$sfm_project/colmap_radial_run01"
export sfm_project sfm_result
test -d "$sfm_project/images" || { echo "images がありません"; exit 1; }
test ! -e "$sfm_result" || { echo "結果フォルダは新しい名前にしてください"; exit 1; }
mkdir -p "$sfm_result/logs" "$sfm_result/sparse"

colmap -h > "$sfm_result/logs/colmap_help.txt" 2>&1
colmap feature_extractor -h > "$sfm_result/logs/feature_extractor_help.txt" 2>&1
colmap sequential_matcher -h > "$sfm_result/logs/sequential_matcher_help.txt" 2>&1
colmap mapper -h > "$sfm_result/logs/mapper_help.txt" 2>&1

if grep -q -- 'FeatureExtraction.use_gpu' "$sfm_result/logs/feature_extractor_help.txt"; then
  sfm_extract_gpu='--FeatureExtraction.use_gpu'
elif grep -q -- 'SiftExtraction.use_gpu' "$sfm_result/logs/feature_extractor_help.txt"; then
  sfm_extract_gpu='--SiftExtraction.use_gpu'
else
  echo "GPU 指定名が不明です。保存したヘルプを確認してください"
  exit 1
fi

if grep -q -- 'FeatureMatching.use_gpu' "$sfm_result/logs/sequential_matcher_help.txt"; then
  sfm_match_gpu='--FeatureMatching.use_gpu'
elif grep -q -- 'SiftMatching.use_gpu' "$sfm_result/logs/sequential_matcher_help.txt"; then
  sfm_match_gpu='--SiftMatching.use_gpu'
else
  echo "GPU 指定名が不明です。保存したヘルプを確認してください"
  exit 1
fi
set -o pipefail
```

各工程がエラーになったら次の工程には進まず、`logs/` を確認します。ここで生成した DB を別のカメラモデルの比較に使い回さないでください。既存画像の特徴が再抽出されず、指定変更が反映されない原因になります。

まずは曲がり角を含む短い区間の数百枚で確認し、その後に建物全周へ広げる運用を推奨します。これは本ワークスペースでの作業方針で、COLMAP の枚数上限ではありません。

## 4. 単眼・広角カメラのモデルを選ぶ

**「広角」と「魚眼」は同義ではありません。** カメラ仕様とキャリブレーションで投影モデルを決めます。レンズの型番が不明な現段階では、どちらかを確定できません。

| 入力画像・レンズ | 候補 | COLMAP のパラメータ順 |
|---|---|---|
| 通常の広角、放射方向の歪みをまず近似、校正値なし | `RADIAL` | `f,cx,cy,k1,k2` |
| 通常のレンズで OpenCV 型の校正値あり | `OPENCV` | `fx,fy,cx,cy,k1,k2,p1,p2` |
| 魚眼の OpenCV fisheye 校正値あり | `OPENCV_FISHEYE` | `fx,fy,cx,cy,k1,k2,k3,k4` |
| 別工程で歪み補正済み、その出力の内部パラメータが既知 | `PINHOLE` | `fx,fy,cx,cy` |

`RADIAL` は検証の出発点です。魚眼レンズなら魚眼モデルを使います。複雑なモデルほど常に良いわけではありません。1台で撮影した画像に `--ImageReader.single_camera 1` を指定し、内部パラメータを共有します。これはフォーカスなどが撮影中に固定されていることを前提とします。[カメラモデルの公式説明](https://colmap.github.io/cameras.html)、[モデルごとのパラメータ順](https://github.com/colmap/colmap/blob/3.8/src/base/camera_models.h)

未校正で、通常の広角レンズとして試す場合:

```bash
sfm_camera_model='RADIAL'
sfm_intrinsics_args=()
```

校正済みの場合は、上の2行の代わりに以下の要領で設定します。`実測値をカンマ区切りで記入` は必ず実数に置き換えます。

```bash
sfm_camera_model='OPENCV'
sfm_intrinsics_args=(--ImageReader.camera_params '実測値をカンマ区切りで記入')
```

OpenCV の通常モデルと fisheye モデルでは、同じ `k1` という名前でも定義が異なります。また一般的な OpenCV `plumb_bob` の5係数 `k1,k2,p1,p2,k3` を `OPENCV` の8パラメータにそのまま追加できません。`k3` 等が有意なら、適切な `FULL_OPENCV` モデルへの対応付けや校正モデルを再検討します。解像度変更・切り抜き後は `fx,fy,cx,cy` も変化するため、その画像に合う値を使用します。[OpenCV の通常校正モデル](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)、[OpenCV fisheye モデル](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)

USBカメラの画像にレンズ情報の EXIF がなくても処理できます。ただし、初期焦点距離の推測に任せるより、同じ解像度・フォーカスで校正しておくほうが検証しやすくなります。

## 5. SIFT 特徴を抽出する

```bash
colmap feature_extractor \
  --database_path "$sfm_result/database.db" \
  --image_path "$sfm_project/images" \
  --ImageReader.single_camera 1 \
  --ImageReader.camera_model "$sfm_camera_model" \
  "${sfm_intrinsics_args[@]}" \
  "$sfm_extract_gpu" 0 \
  2>&1 | tee "$sfm_result/logs/01_extract.log"
```

この手順では標準の SIFT を使用します。CPU の検証が終わり、CUDA 等を使えるビルドであることが確認できたら `"$sfm_extract_gpu" 0` と照合側の `"$sfm_match_gpu" 0` をそれぞれ `1` にできます。[公式 CLI](https://colmap.github.io/cli.html)

ロボットの車体が大きく写る場合は、その領域を黒にしたグレースケール PNG を別途作り、特徴抽出時に `--ImageReader.camera_mask_path /絶対パス/mask.png` を追加できます。全画像で同じ画素領域をマスクする用途です。元画像自体に黒塗り加工する必要はありません。[公式: マスク](https://colmap.github.io/faq.html#mask-image-regions)

## 6. 撮影順の画像を照合する

まず近接する画像を照合します。

```bash
colmap sequential_matcher \
  --database_path "$sfm_result/database.db" \
  --SequentialMatching.overlap 10 \
  --SequentialMatching.quadratic_overlap 0 \
  --SequentialMatching.loop_detection 0 \
  "$sfm_match_gpu" 0 \
  2>&1 | tee "$sfm_result/logs/02_sequential.log"
```

`overlap=10` は近傍画像の照合範囲の初期値であり、「画像が10%重なる」という設定ではありません。撮影間隔や速度で調整します。遠いフレームも候補に加える quadratic overlap と、画像検索による loop detection は別の機能です。[旧版の SequentialMatching 定義](https://github.com/colmap/colmap/blob/3.8/src/feature/matching.h)

### 一周した始点と終点を結ぶ

上の設定だけでは、画像列の始点・終点が離れているため、周回の閉じ合いを十分に照合できません。次のいずれかを**mapper の前**に追加します。

**A. 数百枚の試験では全組合せを追加照合する。** 計算量はおおよそ枚数の2乗なので、数千枚全体には安易に使いません。

```bash
colmap exhaustive_matcher \
  --database_path "$sfm_result/database.db" \
  "$sfm_match_gpu" 0 \
  2>&1 | tee "$sfm_result/logs/03_exhaustive.log"
```

**B. 周回の開始・終了画像が同じ景色を写している場合は、その区間の候補を指定する。** 以下は先頭20枚と末尾20枚の候補リストを作る例です。連番ファイルを画像フォルダ直下に置いた単一走行を対象とします。実際の重複区間が違う場合は対象画像を変更します。

```bash
python3 - <<'PY'
import os
from pathlib import Path

root = Path(os.environ['sfm_project']) / 'images'
names = sorted(p.name for p in root.iterdir()
               if p.suffix.lower() in {'.jpg', '.jpeg', '.png'})
if len(names) < 40:
    raise SystemExit('この例は40枚以上用です。少数なら全組合せ照合を使ってください。')
pairs = sorted({tuple(sorted((a, b)))
                for a in names[:20] for b in names[-20:] if a != b})
out = Path(os.environ['sfm_result']) / 'loop_pairs.txt'
out.write_text(''.join(f'{a} {b}\n' for a, b in pairs), encoding='utf-8')
print(f'{len(pairs)} pairs -> {out}')
PY

colmap matches_importer \
  --database_path "$sfm_result/database.db" \
  --match_list_path "$sfm_result/loop_pairs.txt" \
  --match_type pairs \
  "$sfm_match_gpu" 0 \
  2>&1 | tee "$sfm_result/logs/03_loop_pairs.log"
```

`match_type pairs` は候補画像ペアを受け取り、特徴の照合と幾何検証を実行します。ペアリストに書くだけで正しい対応やループ閉じ合いが保証されるわけではありません。[公式の実装](https://github.com/colmap/colmap/blob/3.8/src/exe/feature.cc#L231)

**C. 数千枚・複数周回では Vocabulary Tree による画像検索を使う。** 利用している版に対応した木を用意し、次のように loop detection を有効にします。

```bash
sfm_vocab='/絶対パス/利用するCOLMAP版に対応したvocab_tree.bin'
colmap sequential_matcher \
  --database_path "$sfm_result/database.db" \
  --SequentialMatching.overlap 10 \
  --SequentialMatching.loop_detection 1 \
  --SequentialMatching.loop_detection_period 10 \
  --SequentialMatching.loop_detection_num_images 50 \
  --SequentialMatching.vocab_tree_path "$sfm_vocab" \
  "$sfm_match_gpu" 0 \
  2>&1 | tee "$sfm_result/logs/03_loop_detection.log"
```

現行版は木の自動取得に対応していますが、旧3.8ではパスが既定で空です。また古い木と Faiss 形式の木を混同しないでください。手元の `colmap sequential_matcher -h` と[その版の文書](https://colmap.github.io/legacy.html)を確認し、[公式配布ページ](https://demuc.de/colmap/)から適合する木を選択します。まず A または B を使えば、木をダウンロードせずに試せます。

### 時計回りと反時計回りの画像を一緒に使う場合

撮影途中で180度反転すると前向きカメラの見え方は大きく変わります。同じ場所の画像でも対応付かないことがあります。角や広場で**並進を伴う中間の向きの画像**を追加します。回転のみでは3D点の三角測量に必要な視差が足りません。[公式の撮影推奨](https://github.com/colmap/colmap/blob/3.8/doc/tutorial.rst#structure-from-motion)

複数セッションは、例えば `images/01_cw/00000001.png`、`images/02_ccw/00000001.png` として名前の衝突を避けます。各フォルダ内の順番は維持されますが、Sequential Matching が自動的に「別走行」を理解して接続するわけではありません。時間近傍以外も候補にする C、全画像への `vocab_tree_matcher`、または両走行間の候補を記載した B を追加します。B のペアは例えば `01_cw/00000420.png 02_ccw/00000615.png` の形式です。

1つの `single_camera` を共有できるのは、両方とも同じ解像度・内部パラメータの場合です。異なる設定のセッションを無理に共有させず、カメラIDを分けて管理します。

## 7. 疎な SfM を実行する

```bash
colmap mapper \
  --database_path "$sfm_result/database.db" \
  --image_path "$sfm_project/images" \
  --output_path "$sfm_result/sparse" \
  2>&1 | tee "$sfm_result/logs/04_mapper.log"
```

校正値が十分信頼でき、固定して比較する場合は、このコマンドに以下を追加します。通常の未校正の試行では追加せず、焦点距離・歪みの最適化を許可します。

```text
--Mapper.ba_refine_focal_length 0
--Mapper.ba_refine_principal_point 0
--Mapper.ba_refine_extra_params 0
```

校正値を入力することと、最適化で固定することは別です。オプションの存在は手元の `mapper -h` でも確認してください。[公式: 内部パラメータの固定](https://colmap.github.io/faq.html#fix-intrinsic-camera-parameters)

出力は `sparse/0/`、場合によっては `sparse/1/` 以降です。複数フォルダは、すべてが同じ座標系に接続できたという意味ではありません。

## 8. 結果を確認・書き出す

まず全モデルを調べ、登録画像数と経路の範囲を確認します。

```bash
for sfm_model in "$sfm_result"/sparse/*; do
  test -d "$sfm_model" || continue
  colmap model_analyzer --path "$sfm_model"
done
```

モデル `0` を調べる例:

```bash
sfm_model="$sfm_result/sparse/0"
colmap model_analyzer --path "$sfm_model" \
  2>&1 | tee "$sfm_result/logs/05_analyzer.log"
mkdir -p "$sfm_result/model_txt"
colmap model_converter \
  --input_path "$sfm_model" \
  --output_path "$sfm_result/model_txt" \
  --output_type TXT
colmap gui
```

GUI の `File > Import model` で `sparse/0` を開き、カメラの並びと3D点を確認します。選ぶモデル番号は登録率だけでなく、必要な経路がつながっているかで判断します。

確認する項目:

- **登録画像数 / 入力画像数**。モデル間の重複画像を足して登録率にしないこと。
- 曲がり角や反対方向を含め、必要な経路が1つのモデルに接続されているか。
- カメラ軌跡が飛ぶ、建物が折れ曲がる、周回の始点・終点が二重になる等がないか。
- 平均再投影誤差、1点を観測する画像数（track length）、画像あたりの観測点数。
- `cameras.txt` の推定焦点距離・歪みがレンズ仕様や校正値から大きく外れていないか。
- **別走行の検証画像**でも経路全体で Localization できるか。SfM に使った画像での再現だけでは汎化の評価にならない。

再投影誤差が小さくても、メートル単位の位置精度が保証されるわけではありません。誤差や track length の合否値は一律に決めず、検証画像・実測位置との比較で定めます。[モデル解析コマンド](https://github.com/colmap/colmap/blob/3.8/src/exe/model.cc#L405)

### よくある問題

| 現象 | 最初に確認すること |
|---|---|
| 初期の2枚が決まらない | ブレ、暗さ、白壁、純回転、並進不足、間引き過多、レンズモデルの誤り |
| 途中までしか復元できない | その区間の画像重複、露出変化、角での急旋回。照合範囲と追加撮影を検討 |
| 往路と復路が別モデル | 同じ静止特徴が見えているか。方向間を結ぶ画像と走行間の照合候補を追加 |
| 周回が閉じない | 時間近傍以外の照合を追加したか。始点・終点で同じ向き・景色が写っているか |
| カメラモデルを変えても結果が同じ | 新しい `sfm_result` と DB を使ったか |
| GPU/画面関連のエラー | CPU 指定のオプション名、GPU 対応ビルド・ドライバを確認 |
| 大部分は良いが細長い経路が歪む | 校正値の信頼性、周回間の接続、平面・前進だけの観測への偏りを確認 |

これらは調査の入口です。閾値を緩めて登録画像数だけを増やす前に、元画像と対応付けを確認してください。

## 9. 残す成果物と次の工程

以下をセットで保管します。

```text
session/
├── images/                    # 元画像・相対名を維持
├── timestamps.csv
├── カメラ設定・校正データ
└── colmap_radial_run01/
    ├── database.db            # SIFT特徴、対応付けなど
    ├── sparse/0/              # 採用したカメラ・画像姿勢・3D点・観測関係
    ├── model_txt/             # 人が点検できる書き出し
    └── logs/                  # バージョン/ヘルプ/処理結果
```

新しい版では `rigs.bin`・`frames.bin` もモデルに含まれます。モデルディレクトリ全体を保持します。**PLY 点群だけの保存では、2D特徴と3D点の対応やカメラ情報を失います。** [公式出力形式](https://colmap.github.io/format.html)

単眼 SfM の尺度・原点・軸方向は、そのまま Nav2 の `map` 座標にはなりません。`p_map = s R p_sfm + t` の相似変換を求め、メートル単位の地図に合わせます。複数の実測したカメラ中心、または既存地図内の信頼できる位置との対応を残してください。単一の既知距離は尺度には使えても、原点と全軸方向を決定しません。参照位置は一直線上だけで選ばず、経路全体に分散させます。

COLMAP の `images.txt` にある `q,t` は**世界座標からカメラ座標への変換**で、`t` 自体は世界座標内のカメラ位置ではありません。カメラ中心は `C = -Rᵀt` です。さらにロボット中心との取り付け変換が必要です。座標合わせの `model_aligner` と Nav2 用 TF への接続は [NAV2_VISUAL_LOCALIZATION.md](NAV2_VISUAL_LOCALIZATION.md) に続きます。[出力座標の定義](https://colmap.github.io/format.html#images-txt)、[公式の座標合わせ](https://colmap.github.io/faq.html#geo-registration)

後段で hloc + SuperPoint/LightGlue を使う場合、**SIFT地図の特徴点番号と SuperPoint の特徴点番号は一致しません。** 本手順の SIFT 地図を読み込んで特徴抽出器だけ差し替える構成にはしません。SIFT を維持して登録を検証するか、同じ参照画像・採用した姿勢を使い、後段と同じ特徴量で hloc の再三角測量／再構築を行います。各特徴量と3D点の対応を揃える手順も次の文書に示します。[hloc公式: 既存 SfM の利用](https://github.com/cvg/Hierarchical-Localization#localization-with-a-custom-dataset)

## 参照した一次資料

上記リンクの確認日はすべて **2026-09-23** です。最新ページは更新されるため、再現実験では実際に使った COLMAP の版・`-h` 出力・カメラ設定・画像集合を一緒に残します。

- [COLMAP Tutorial](https://colmap.github.io/tutorial.html)
- [COLMAP CLI](https://colmap.github.io/cli.html)
- [COLMAP Installation](https://colmap.github.io/install.html)
- [COLMAP Camera Models](https://colmap.github.io/cameras.html)
- [COLMAP Output Format](https://colmap.github.io/format.html)
- [COLMAP FAQ](https://colmap.github.io/faq.html)
- [COLMAP 3.8 公式ソース](https://github.com/colmap/colmap/tree/3.8)
- [COLMAP 過去バージョンの文書](https://colmap.github.io/legacy.html)
- [hloc 公式リポジトリ](https://github.com/cvg/Hierarchical-Localization)
