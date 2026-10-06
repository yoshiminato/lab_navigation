# 撮影時刻のTFを抽出し、COLMAPの位置事前情報へ登録する

追加日: 2026-10-06。処理は `export_poses` と `import_pose_priors` の2つです。
実機にカメラTFがない場合も、`map → base_link` 等を取得し、取り付け位置を補正できます。
ROSノードやbagの再生は起動せず、保存済みのデータを処理します。

```text
captures/SESSION/{images/, timestamps.csv, bag/}
    → export_poses → poses.csv + poses_report.json
    → import_pose_priors + 特徴抽出済みDB
    → 新しいdatabase.db + database.db.pose_priors.json
    → 特徴照合 → pose_prior_mapper → sparse/
```

## 1. ビルドと環境

```bash
cd ~/lab_navigation_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select sfm_capture
source install/local_setup.bash
ros2 run sfm_capture export_poses --help
ros2 run sfm_capture import_pose_priors --help
```

TF抽出はROS 2の `rosbag2_py`、`tf2_py`、`tf2_msgs` を使用します。
画像自体は読み込まないので、画像を別PCへ転送した後でも `timestamps.csv` とbagだけで抽出できます。
DB登録時だけ、実行するPython環境にNumPyとPyCOLMAP 4.2系が必要です。
検証環境は COLMAP CLI **4.2.1**、PyCOLMAP **4.2.0** です。
COLMAP 3.xや4.2より前のDB形式は、このインポーターの対象外です。

```bash
python3 -c 'import pycolmap; print(pycolmap.__version__)'
colmap -h
colmap pose_prior_mapper -h
```

別PCでDB登録する場合はROSをインストールする必要はありません。
画像、位置CSV、特徴抽出DBと `src/sfm_capture` を転送し、専用環境にインストールできます。

```bash
python3 -m venv .venv-colmap
source .venv-colmap/bin/activate
python -m pip install './src/sfm_capture[colmap]'
import_pose_priors --help
```

`PYTHONNOUSERSITE=1` が設定された端末では、ユーザー領域のPyCOLMAPを読み込めません。
その場合はDBコマンドを `env -u PYTHONNOUSERSITE ros2 run ...` で実行するか、上の専用環境を使用します。
COLMAPとPyCOLMAPのDB形式が適合する環境を使ってください。

## 2. TFから画像と一対一のCSVを作る

以下ではBashの同じ端末で変数を設定します。出力先は新しい名前を指定してください。

```bash
sfm_capture_dir="$PWD/captures/sim_002"
sfm_pose_dir="$sfm_capture_dir/poses_map_base"
sfm_colmap_dir="$sfm_capture_dir/colmap_with_priors"

ros2 run sfm_capture export_poses \
  "$sfm_capture_dir" "$sfm_pose_dir" \
  --world-frame map --source-frame base_link \
  --position-std 1.0 1.0 1.0
```

この例は `base_link` 原点の位置・姿勢を保存します。
CSVとレポートの `position_kind=source_origin` は、カメラ中心への補正がないことを表します。
取り付け寸法の測定前に画像対応付けとDB登録を確認する用途に使えます。
COLMAPに登録した場合、車体原点をカメラ中心の近似値として使用することになります。

### カメラTFがなく、取り付け位置を補正する場合

カメラの位置を **sourceフレーム内のメートル単位**で指定します。
カメラが車体前方0.37m、上方0.10mにある場合の例:

```bash
ros2 run sfm_capture export_poses \
  "$sfm_capture_dir" "$sfm_capture_dir/poses_map_camera_offset" \
  --world-frame map --source-frame base_link \
  --camera-translation 0.37 0.0 0.10 \
  --camera-quaternion -0.5 0.5 -0.5 0.5 \
  --position-std 1.0 1.0 1.0
```

**この値は現在のシミュレーションURDFに対応します。実機では実測値に置き換えてください。**
`--camera-quaternion` の順序は **qx,qy,qz,qw** です。
固定変換 `source_from_camera` は、カメラ座標をsource座標へ変換する向きで指定します。

```text
world_from_camera = world_from_source × source_from_camera
p_world_camera = p_world_source + R_world_source × p_source_camera
```

カメラ中心の位置だけに必要なのは `--camera-translation` です。
クォータニオンを省略すると、保存する向きはsourceと同じ向きとして扱います。
向きも正しく保存したい場合は、カメラ光学座標の取り付け回転を指定してください。
`camera_orientation_provided` が向き指定の有無を示します。
`position_kind=camera_center` は指定した取り付け変換で補正したことを表し、測定精度を保証しません。

### カメラの光学フレームがTFにある場合

```bash
ros2 run sfm_capture export_poses \
  "$sfm_capture_dir" "$sfm_capture_dir/poses_map_camera_tf" \
  --world-frame map --source-frame camera_optical_frame \
  --source-is-camera --position-std 1.0 1.0 1.0
```

この場合はTFが取り付け位置・回転を含むため、固定変換の追加指定は不要です。
`--source-is-camera` と取り付け変換の同時指定は、二重補正を防ぐため拒否します。
基準をオドメトリ座標にする場合は `--world-frame odom` に変更できます。
実機用URDFへのカメラフレーム追加は、取り付け寸法・向きの測定後に行います。

### 抽出時の検査

- `Image.header.stamp` と各 `TransformStamped.header.stamp` を対応付けます。
  CSVの `received_ns`、bag記録時刻、ファイル更新日時は使用しません。
- tf2による並進の線形補間と回転の球面線形補間を使用します。
  TFバッファの保持時間を記録期間に合わせるため、10秒以上の収集も処理できます。
- **TFの前後サンプル間隔は既定で0.2秒以内**です。経路内の各動的TFについて確認します。
  変更する場合は `--max-tf-gap-sec 0.5` 等を指定します。
  時刻外への外挿や、最新TFへの置き換えは行いません。
- 指定経路の親フレームの変更、静的TFの変更、TFツリーの循環を拒否します。
  他の枝で検出した矛盾は `tf.ignored_tf_issues` に記録し、使用しません。
- 同時刻に異なる動的TFがある場合は `tf.conflicting_dynamic_stamps` に記録します。
  そのサンプルを参照・補間する画像を欠測とし、撮影前などの使用しない時刻の矛盾は他の画像に影響させません。
- 不足した画像もCSVに残し、`status=tf_unavailable` または `gap_too_large` と理由を記録します。
  位置・姿勢は空欄になります。既定では終了コード1です。
  `--allow-missing` で一部欠測を許容できますが、全件欠測は常に失敗します。
- `--bag /別の/bag`、`--tf-topic /別の/tf`、`--static-topic /別の/tf_static` も指定できます。
  ストレージ形式はbagのmetadataから自動判定します。

## 3. 特徴抽出DBを作る

すでにDBがある場合、この工程は省略して次へ進めます。
以下は検証したCOLMAP 4.2.1のCPU実行例です。
実機のレンズモデルと校正値は [COLMAP_SFM.md](COLMAP_SFM.md) に従って指定してください。

```bash
mkdir -p "$sfm_colmap_dir"
colmap feature_extractor \
  --database_path "$sfm_colmap_dir/features.db" \
  --image_path "$sfm_capture_dir/images" \
  --ImageReader.single_camera 1 \
  --ImageReader.camera_model OPENCV \
  --FeatureExtraction.use_gpu 0
```

`RADIAL` は未校正の通常レンズで試す場合の例です。
魚眼や校正済みカメラでは、適切なモデル・内部パラメータを指定してください。

## 4. 位置事前情報をDBへ登録する

```bash
ros2 run sfm_capture import_pose_priors \
  "$sfm_pose_dir/poses.csv" "$sfm_colmap_dir/features.db" \
  --output-database "$sfm_colmap_dir/database.db"
```

登録先には**新しいファイル名**が必要です。元の `features.db` は保持します。
SQLiteのbackup APIでコピーし、特徴・対応付け・カメラ情報などを引き継ぎます。
登録・読み戻し検証が成功したDBだけを指定先に保存します。
既存の位置事前情報は、同じカメラ・画像に対応するものを更新します。
既存の重力情報は保持し、対象外の画像・センサの事前情報は変更しません。

画像の対応は `image_name` とDBの `images.name` の**完全一致**です。
連番のindexから画像IDを推測したり、ファイルのbasenameだけで照合したりはしません。
複数セッションを `images/01_run/` 等にまとめた場合は `--image-prefix 01_run` を指定できます。

CSVに欠測がある場合や、DBに対応する画像がない場合は、既定では登録を拒否します。
検査後に有効な画像だけを登録する場合は `--skip-missing` を明示します。
除外画像と理由はDB登録レポートに保存します。

### 位置の不確かさ

`--position-std SX SY SZ` は**worldフレームの各軸方向の標準偏差（m）**です。
既定は各1mで、TFから推定した精度ではありません。
DBには `diag(SX², SY², SZ²)` を位置共分散（m²）として保存します。
0、NaN、無限大、二乗すると0/無限大になる値は拒否します。
信頼できる測定・評価に基づいて値を設定してください。

DBには `CARTESIAN` の位置制約だけを登録します。
CSVのクォータニオンは保存されますが、今回の位置事前情報には登録しません。
COLMAPの `images.txt` のtranslationとは異なり、ここで登録する `x,y,z` はworld内のカメラ中心です。

## 5. 保存先と検証

| 保存先 | 内容 |
|---|---|
| 指定した位置出力ディレクトリの `poses.csv` | 全画像と一対一の時刻・位置・姿勢・標準偏差・状態・補間間隔 |
| 同ディレクトリの `poses_report.json` | 入力、ハッシュ、指定フレーム、固定変換、TF一覧、成功・欠測数 |
| 指定した `database.db` | 元DBのコピーに位置事前情報を登録したもの |
| `database.db.pose_priors.json` | 追加・更新・検証件数、除外画像、CSVのハッシュ、PyCOLMAPの版 |
| `sparse/` | 下記 `pose_prior_mapper` の出力 |

撮影データ自体の整合性も確認できます。

```bash
ros2 run sfm_capture check_dataset "$sfm_capture_dir"
python3 -m json.tool "$sfm_pose_dir/poses_report.json"
python3 -m json.tool "$sfm_colmap_dir/database.db.pose_priors.json"

ros2 run sfm_capture import_pose_priors \
  "$sfm_pose_dir/poses.csv" "$sfm_colmap_dir/database.db" --verify-only
```

DB登録時も自動検証しますが、`--verify-only` で後から再確認できます。
SQLiteを読み取り専用で開き、画像との対応、Cartesian指定、位置、共分散をCSVと照合します。
この検証だけならROS・NumPy・PyCOLMAPは不要で、Python標準ライブラリで動作します。
部分登録の場合は検証時も同じ `--skip-missing`・`--image-prefix` を指定します。

完全成功の目安は、位置レポートの `valid == images`、`missing == 0`、
DBレポートの `verified == valid`、`skipped == []`、再検証の `status == verified` です。
取り付け補正済みなら `position_kind=camera_center` も確認します。
これらは時刻対応とDB登録の整合性検査です。位置の実精度は別途評価します。

## 6. 位置で照合ペアを選び、位置制約を使ってSfMを実行する

位置事前情報には、**照合する画像ペアを選ぶ**用途と、**SfMで推定する位置に制約を与える**用途があります。
DBへ登録しただけで、すべてのmatcherが位置を使うわけではありません。

| コマンド | 役割 | 登録した位置の使用 |
|---|---|---|
| `sequential_matcher` | ファイル名順で近い画像を照合する | ペア選択には使用しない |
| `spatial_matcher` | 撮影位置の距離・近傍数で画像ペアを選んで照合する | ペア選択に使用する |
| `pose_prior_mapper` | 照合結果からSfMを実行する | 復元時の位置制約に使用する |

従来の `sequential_matcher → pose_prior_mapper` でも、復元時の位置制約は有効です。
**登録位置でマッチング対象も絞りたい場合は `spatial_matcher → pose_prior_mapper` を使用します。**
正確な位置が得られる場合のspatial matchingは、[COLMAP公式の推奨](https://colmap.github.io/tutorial.html#feature-matching-and-geometric-verification)でもあります。

### 位置で画像ペアを絞る場合

特徴照合は位置登録済みのDBに対して行います。
今回の `CARTESIAN` の位置事前情報は、COLMAP 4.2.1の `spatial_matcher` がそのまま参照できます。
緯度・経度やEXIFのGPSへ変換する必要はありません。

```bash
colmap spatial_matcher \
  --database_path "$sfm_colmap_dir/database.db" \
  --FeatureMatching.use_gpu 0 \
  --SpatialMatching.ignore_z 0 \
  --SpatialMatching.max_distance 3.0 \
  --SpatialMatching.max_num_neighbors 30 \
  --SpatialMatching.min_num_neighbors 0
```

- `ignore_z 0`: XYZの距離を使います。既定の `1` はZを無視します。
- `max_distance 3.0`: この例では登録位置の距離が3m以内の画像を候補にします。
  TF位置をメートルで保存しているため、距離もメートルです。COLMAPの既定は100です。
- `max_num_neighbors 30`: 各画像から検索する近傍画像数の上限です。
  他の画像側から候補になることもあるため、最終的な接続数が必ず30以下になるわけではありません。
- `min_num_neighbors 0`: 距離上限を超える候補を、最低近傍数の確保のために追加しません。

**3m・30近傍は設定方法の例で、今回のデータで最適化した値ではありません。**
画角、被写体までの距離、間引き後の撮影間隔、位置の誤差に合わせて調整してください。
位置の誤差に対して半径が小さすぎると、見た目が重なる画像も候補から外れます。
位置共分散やカメラの向きは、この距離によるペア選択には使われません。
近い位置でも壁を挟む・逆方向を撮る等の場合は画像が重ならないため、特徴照合と幾何検証が必要です。
有効な位置事前情報がない画像はspatialの候補から外れます。

### 連続撮影の隣接画像も照合したい場合

連続走行で撮影した画像なら、`sequential_matcher` は位置の誤差に左右されず、
ファイル名順に近い画像を照合できます。位置の精度を評価する段階の比較対象にも使えます。
以下をspatialの代わりに実行するか、同じDBへ追加で実行してください。

```bash
colmap sequential_matcher \
  --database_path "$sfm_colmap_dir/database.db" \
  --FeatureMatching.use_gpu 0 \
  --SequentialMatching.overlap 10 \
  --SequentialMatching.quadratic_overlap 0 \
  --SequentialMatching.loop_detection 0
```

両方を実行すると、連続する画像の接続と、再訪時などの空間的に近い画像の接続を追加できます。
**併用は候補の和集合です。sequentialの候補をspatialの距離条件で絞り直す処理ではありません。**
既存の照合結果も残ります。距離で候補を限定したい場合は、照合前のDBにspatialだけを実行します。
方式を比較する場合も、同じ位置登録済み・照合前DBから別々のコピーを作ってください。
第4節で照合済みDBを入力した場合は、コピー先にも照合結果が引き継がれる点に注意してください。

画像を間引いても、名前を保持していれば位置の対応は変わりません。
spatialは残った画像の実際の登録位置を使い、sequentialは残った画像のファイル名順を使います。
sequentialの `overlap` は残った画像の順序上の間隔であり、元の連番の差や撮影時間ではありません。
撮影時刻・TFの対応付け自体は、第2節のCSVとヘッダ時刻を使う処理です。
大幅な間引きで画像の見た目の重なりが失われた場合は、どちらの方式でも復元が難しくなります。

### 照合後に位置制約付きの復元を行う

上記のどの照合方式でも、復元時の位置制約を使うには `pose_prior_mapper` を実行します。

```bash
mkdir -p "$sfm_colmap_dir/sparse"
colmap pose_prior_mapper \
  --database_path "$sfm_colmap_dir/database.db" \
  --image_path "$sfm_capture_dir/images" \
  --output_path "$sfm_colmap_dir/sparse" \
  --use_robust_loss_on_prior_position 1
```

通常の `mapper` では今回の位置制約を有効にしません。
`--overwrite_priors_covariance` を有効にすると、CSVから登録した不確かさを上書きするため、
ここでは既定のfalseで使います。
周回の照合追加やモデル品質の確認は [COLMAP_SFM.md](COLMAP_SFM.md) を参照してください。

`map` は自己位置推定の補正時に跳ぶ場合があり、`odom` は長距離でドリフトします。
急な位置補正の付近では補間結果と軌跡を確認してください。
画像ヘッダの時刻と実際の露光時刻のずれも、TF補間だけでは解消しません。

## 7. 自動テスト

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/sfm_capture:$PYTHONPATH" /usr/bin/python3 -m pytest -q tests/sfm_capture
```

合成TFによる補間・回転する取り付け位置・長時間のバッファ保持・欠測・補間間隔の検査と、
実際のPyCOLMAP DBによる画像ID対応・共分散・既存情報保持・失敗時の出力防止を検証します。
ROSやPyCOLMAPがない環境では、依存するテストはskipします。

## 8. このワークスペースでの検証結果

2026-10-06に確認した結果です。検証用出力は
`validation/pose_pipeline_20261006/` に保存しました。

| 確認 | 結果 |
|---|---|
| sfm_captureのビルド・2つのROSコマンド | 成功 |
| 既存テストと新規テスト | **51件成功、skipなし** |
| sim_001のmap上のカメラ位置抽出 | **969/969件成功** |
| sim_002のmap上のカメラ位置抽出 | **1,121/1,121件成功** |
| sim_002のodom上のbase_link位置抽出 | インストール済みコマンドで**1,121/1,121件成功** |
| 取り付け補正と直接camera TFの比較 | sim_001全件で一致。最大位置差約 **2.14×10⁻¹⁴m** |
| sim_002画像と撮影メタデータの整合性 | check_dataset成功 |
| COLMAP 4.2.1で特徴抽出した24画像のDBへ登録 | **24件登録・読み戻し照合成功** |
| 元DBの保持 | SHA-256一致、5,675個の特徴をコピー先にも保持 |
| ROS/NumPy/PyCOLMAPを読み込まないDB再検証 | `python3 -S` で成功 |

実データのDB検証では、`feature_images.txt` に指定した24画像だけを特徴抽出しました。
全1,121行の位置CSVから `--skip-missing` で対応する24件を登録し、
残り1,097件は `image_not_in_database` としてレポートに記録しています。
この検証用DBを建物全体のSfM入力として使わず、実運用では必要な全画像の特徴抽出を行います。

`sim_001` は元CSVの先頭99画像が現フォルダにありません。
位置抽出は可能ですが、現状の画像一式では `check_dataset` による全件整合性検査に通りません。
`sim_002` は1,121画像すべてが存在します。
bag内の指定経路以外の静的TF矛盾は、位置レポートに記録されています。
sim_002には撮影前の同時刻TFの矛盾もありますが、画像時刻の補間には使われていません。

実機の取り付け校正と、位置制約を使ったSfMの復元精度は、この検証には含みません。

参考: [COLMAP位置事前情報](https://colmap.github.io/faq.html#reconstruction-with-pose-priors-gps)、
[PosePrior API](https://colmap.github.io/pycolmap/pycolmap.html#pycolmap.PosePrior)。
