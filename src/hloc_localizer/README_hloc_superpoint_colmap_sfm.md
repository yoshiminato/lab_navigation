# HLoc + SuperPoint + LightGlue + COLMAP SfM 手順
## 画像間引き対応 / Spatial Pairs / COLMAP DB変換 / Pose Prior対応

更新日: 2026-10-14

---

## 0. この手順で行うこと

この手順では、COLMAP の SIFT + matcher の代わりに、

- SuperPoint で特徴抽出
- LightGlue で特徴マッチング
- HLoc から COLMAP database へ keypoints / matches を登録
- COLMAP で幾何検証
- 必要に応じて pose prior を登録
- `mapper` または `pose_prior_mapper` で SfM

を行います。

処理全体は次のようになります。

```text
images/
   │
   ↓
SuperPoint
   │
   ↓
features.h5
   │
   ├───────────────┐
   │               │
   │           poses.csv
   │               │
   └───────┬───────┘
           ↓
make_spatial_pairs.py
           │
           ↓
pairs_spatial.txt
           │
           ↓
       LightGlue
           │
           ↓
       matches.h5
           │
           ↓
build_hloc_colmap_db.py
           │
           ↓
database_superpoint_raw.db
           │
           ├── 通常SfM
           │      ↓
           │    mapper
           │
           │
           └── pose priorを使用
                  ↓
             import_pose_priors
                  ↓
             pose_prior_mapper
```

---

# 1. 前提

例として、作業ディレクトリを次の場所とします。

```bash
cd ~/lab_navigation_ws/captures/hv_building_lab/20261010
```

想定ディレクトリ:

```text
20261010/
├── images/
│   ├── 00000001.png
│   ├── 00000004.png
│   └── ...
├── poses_map_base/
│   └── poses.csv
├── make_spatial_pairs.py
├── build_hloc_colmap_db.py
└── hloc_superpoint/
```

HLoc の仮想環境を有効化します。

```bash
source ~/lab_navigation_ws/.venv/bin/activate
```

環境に合わせて `.venv` の場所は変更してください。

出力ディレクトリを設定します。

```bash
OUT="$PWD/hloc_superpoint"
mkdir -p "$OUT"
```

---

# 2. SuperPoint 特徴量を抽出

```bash
python -m hloc.extract_features \
    --image_dir images \
    --export_dir "$OUT" \
    --conf superpoint_aachen \
    --as_half
```

生成例:

```text
$OUT/feats-superpoint-n4096-r1024.h5
```

確認:

```bash
ls -lh "$OUT/feats-superpoint-n4096-r1024.h5"
```

---

# 3. 画像を間引いている場合の注意

`poses.csv` には全撮影画像が残っていても、`images/` は間引かれている場合があります。

例:

```text
poses.csv      : 2000枚
images/        : 1400枚
SuperPoint H5  : 1372枚
```

この場合、`poses.csv` だけからペアを作ると、SuperPoint の `.h5` に存在しない画像名が `pairs.txt` に混ざります。

その状態で LightGlue を実行すると、

```text
KeyError: object '00001729.png' doesn't exist
```

のようなエラーになります。

このため、**SuperPoint H5 に実際に存在する画像だけを先に抽出し、その画像群だけで近傍探索を行います。**

---

# 4. `make_spatial_pairs.py`

以下を `make_spatial_pairs.py` として保存します。

```python
#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import h5py
import numpy as np


def get_feature_image_names(features_path: Path) -> set[str]:
    """HLoc features.h5 に実際に特徴量が存在する画像名を取得する。"""
    names = set()

    with h5py.File(features_path, "r") as f:
        def visitor(name, obj):
            if isinstance(obj, h5py.Group) and "keypoints" in obj:
                names.add(name)

        f.visititems(visitor)

    return names


def main():
    parser = argparse.ArgumentParser(
        description=(
            "poses.csv の位置情報から HLoc 用画像ペアを生成する。"
            "SuperPoint 特徴量が存在する画像だけを使用するため、"
            "画像を間引いていても動作する。"
        )
    )

    parser.add_argument("poses_csv", type=Path)
    parser.add_argument("output", type=Path)

    parser.add_argument(
        "--features",
        type=Path,
        required=True,
        help="HLoc の SuperPoint 特徴量 .h5",
    )

    parser.add_argument(
        "--image-dir",
        type=Path,
        default=None,
        help="指定時は実画像ファイルの存在も確認する",
    )

    parser.add_argument(
        "--max-distance",
        type=float,
        default=3.0,
        help="Spatial matching 最大距離[m]",
    )

    parser.add_argument(
        "--max-neighbors",
        type=int,
        default=30,
        help="各画像の Spatial 近傍数上限",
    )

    parser.add_argument(
        "--ignore-z",
        action="store_true",
        help="Zを無視してXY距離だけで近傍探索",
    )

    parser.add_argument(
        "--sequential-overlap",
        type=int,
        default=0,
        help="間引き後の画像順で前後N枚もペアに追加",
    )

    args = parser.parse_args()

    if not args.poses_csv.is_file():
        raise FileNotFoundError(args.poses_csv)

    if not args.features.is_file():
        raise FileNotFoundError(args.features)

    if args.image_dir is not None and not args.image_dir.is_dir():
        raise NotADirectoryError(args.image_dir)

    feature_names = get_feature_image_names(args.features)

    print(f"SuperPoint特徴量あり画像: {len(feature_names)}")

    records = []
    csv_total = 0
    status_ng = 0
    invalid_position = 0
    missing_feature = 0
    missing_image = 0
    duplicate = 0

    already_seen = set()

    with args.poses_csv.open(
        "r",
        newline="",
        encoding="utf-8",
    ) as f:

        reader = csv.DictReader(f)

        required_columns = {
            "image_name",
            "status",
            "x",
            "y",
            "z",
        }

        if reader.fieldnames is None:
            raise RuntimeError("CSVヘッダがありません")

        missing_columns = required_columns - set(reader.fieldnames)

        if missing_columns:
            raise RuntimeError(
                "poses.csv に必要な列がありません: "
                + ", ".join(sorted(missing_columns))
            )

        for row in reader:
            csv_total += 1

            if row["status"] != "ok":
                status_ng += 1
                continue

            image_name = row["image_name"].strip()

            if not image_name:
                continue

            # 間引かれて SuperPoint 特徴量が存在しない画像を除外
            if image_name not in feature_names:
                missing_feature += 1
                continue

            if args.image_dir is not None:
                image_path = args.image_dir / image_name

                if not image_path.is_file():
                    missing_image += 1
                    continue

            try:
                x = float(row["x"])
                y = float(row["y"])
                z = float(row["z"])
            except (ValueError, TypeError):
                invalid_position += 1
                continue

            if not np.all(np.isfinite([x, y, z])):
                invalid_position += 1
                continue

            if image_name in already_seen:
                duplicate += 1
                continue

            already_seen.add(image_name)

            records.append(
                {
                    "name": image_name,
                    "position": [x, y, z],
                }
            )

    print(f"CSV総行数            : {csv_total}")
    print(f"status != ok         : {status_ng}")
    print(f"位置不正             : {invalid_position}")
    print(f"SuperPoint特徴量なし : {missing_feature}")

    if args.image_dir is not None:
        print(f"画像ファイルなし     : {missing_image}")

    print(f"重複                 : {duplicate}")
    print(f"使用画像数           : {len(records)}")

    if len(records) < 2:
        raise RuntimeError("有効画像が2枚未満です")

    names = [r["name"] for r in records]

    positions = np.asarray(
        [r["position"] for r in records],
        dtype=np.float64,
    )

    if args.ignore_z:
        search_positions = positions[:, :2]
    else:
        search_positions = positions

    num_images = len(names)
    pairs = set()

    # Spatial pairs
    if args.max_neighbors > 0:

        for i in range(num_images):

            delta = search_positions - search_positions[i]
            distances = np.linalg.norm(delta, axis=1)

            valid_mask = (
                (np.arange(num_images) != i)
                & (distances <= args.max_distance)
            )

            candidates = np.where(valid_mask)[0]

            candidates = candidates[
                np.argsort(distances[candidates])
            ]

            candidates = candidates[:args.max_neighbors]

            for j in candidates:

                if i < j:
                    pair = (names[i], names[j])
                else:
                    pair = (names[j], names[i])

                pairs.add(pair)

    # Sequential pairs
    if args.sequential_overlap > 0:

        for i in range(num_images):

            end = min(
                num_images,
                i + args.sequential_overlap + 1,
            )

            for j in range(i + 1, end):
                pairs.add((names[i], names[j]))

    name_to_index = {
        name: i for i, name in enumerate(names)
    }

    sorted_pairs = sorted(
        pairs,
        key=lambda p: (
            name_to_index[p[0]],
            name_to_index[p[1]],
        ),
    )

    args.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with args.output.open(
        "w",
        encoding="utf-8",
    ) as f:

        for name1, name2 in sorted_pairs:
            f.write(f"{name1} {name2}\n")

    print()
    print(f"max_distance       : {args.max_distance} m")
    print(f"max_neighbors      : {args.max_neighbors}")
    print(f"ignore_z           : {args.ignore_z}")
    print(f"sequential_overlap : {args.sequential_overlap}")
    print(f"最終ペア数         : {len(sorted_pairs)}")
    print(f"出力               : {args.output}")


if __name__ == "__main__":
    main()
```

---

# 5. Spatial + Sequential ペアを生成

推奨初期値:

```text
max_distance       = 3.0 m
max_neighbors      = 10
sequential_overlap = 5
```

実行:

```bash
python make_spatial_pairs.py \
    poses_map_base/poses.csv \
    "$OUT/pairs_spatial.txt" \
    --features "$OUT/feats-superpoint-n4096-r1024.h5" \
    --image-dir images \
    --max-distance 3.0 \
    --max-neighbors 10 \
    --sequential-overlap 5
```

確認:

```bash
wc -l "$OUT/pairs_spatial.txt"
head "$OUT/pairs_spatial.txt"
```

画像数が 1372 枚の場合、exhaustive では、

```text
1372 × 1371 / 2 = 940,506 pairs
```

になるため、全組合せは避けます。

---

# 6. LightGlue でマッチング

```bash
python -m hloc.match_features \
    --pairs "$OUT/pairs_spatial.txt" \
    --export_dir "$OUT" \
    --features feats-superpoint-n4096-r1024 \
    --conf superpoint+lightglue
```

生成されたファイルを確認します。

```bash
ls -lh "$OUT"/*lightglue*.h5
```

例:

```text
feats-superpoint-n4096-r1024_matches-superpoint-lightglue_pairs_spatial.h5
```

変数に入れます。

```bash
MATCHES=$(find "$OUT" -maxdepth 1 -type f \
    -name '*matches-superpoint-lightglue*pairs_spatial*.h5' \
    | head -n 1)

echo "$MATCHES"
```

---

# 7. HLoc の特徴点・マッチを COLMAP DB へ登録

HLoc の `.h5` を `pose_prior_mapper` や `mapper` が直接読むことはできません。

そのため、

```text
SuperPoint keypoints
LightGlue matches
        ↓
COLMAP database
```

へ変換します。

---

# 8. `build_hloc_colmap_db.py`

以下を `build_hloc_colmap_db.py` として保存します。

```python
#!/usr/bin/env python3

import argparse
from pathlib import Path

import pycolmap

from hloc.reconstruction import (
    create_empty_db,
    import_images,
    get_image_ids,
)

from hloc.triangulation import (
    import_features,
    import_matches,
    estimation_and_geometric_verification,
)

from hloc.utils.io import list_h5_names


def main():

    parser = argparse.ArgumentParser(
        description=(
            "HLoc SuperPoint features + LightGlue matches を "
            "COLMAP database に変換する"
        )
    )

    parser.add_argument(
        "--image-dir",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--features",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--pairs",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--matches",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--database",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--camera-model",
        type=str,
        default="OPENCV",
    )

    parser.add_argument(
        "--camera-params",
        type=str,
        default=None,
        help="例: fx,fy,cx,cy,k1,k2,p1,p2",
    )

    parser.add_argument(
        "--skip-geometric-verification",
        action="store_true",
    )

    args = parser.parse_args()

    for path in [
        args.image_dir,
        args.features,
        args.pairs,
        args.matches,
    ]:
        if not path.exists():
            raise FileNotFoundError(path)

    args.database.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    image_list = sorted(
        list_h5_names(args.features)
    )

    valid_images = []

    for name in image_list:
        if (args.image_dir / name).is_file():
            valid_images.append(name)

    print()
    print("----------------------------------------")
    print("Input")
    print("----------------------------------------")
    print(f"SuperPoint images : {len(image_list)}")
    print(f"Existing images   : {len(valid_images)}")
    print(f"features          : {args.features}")
    print(f"pairs             : {args.pairs}")
    print(f"matches           : {args.matches}")
    print(f"database          : {args.database}")

    if len(valid_images) == 0:
        raise RuntimeError("使用可能な画像がありません")

    create_empty_db(args.database)

    image_options = {
        "camera_model": args.camera_model,
    }

    if args.camera_params is not None:
        image_options["camera_params"] = args.camera_params

    import_images(
        args.image_dir,
        args.database,
        pycolmap.CameraMode.SINGLE,
        valid_images,
        image_options,
    )

    image_ids = get_image_ids(
        args.database
    )

    print()
    print(f"COLMAP images     : {len(image_ids)}")

    with pycolmap.Database.open(
        args.database
    ) as db:

        import_features(
            image_ids,
            db,
            args.features,
        )

        import_matches(
            image_ids,
            db,
            args.pairs,
            args.matches,
            min_match_score=None,
            skip_geometric_verification=(
                args.skip_geometric_verification
            ),
        )

    if not args.skip_geometric_verification:

        estimation_and_geometric_verification(
            args.database,
            args.pairs,
            verbose=True,
        )

    print()
    print("----------------------------------------")
    print("Finished")
    print("----------------------------------------")
    print(f"Database: {args.database}")


if __name__ == "__main__":
    main()
```

---

# 9. DB変換を実行

```bash
DB_RAW="$OUT/database_superpoint_raw.db"
```

実行:

```bash
python build_hloc_colmap_db.py \
    --image-dir images \
    --features "$OUT/feats-superpoint-n4096-r1024.h5" \
    --pairs "$OUT/pairs_spatial.txt" \
    --matches "$MATCHES" \
    --database "$DB_RAW" \
    --camera-model OPENCV
```

実行例:

```text
SuperPoint images : 1372
Existing images   : 1372

COLMAP images     : 1372

Importing features...
1372/1372

Importing matches...
11219/11219

Performing geometric verification...
```

---

# 10. `Creating SIFT CPU feature matcher` と表示される件

幾何検証時に、

```text
Performing geometric verification of the matches...
Creating SIFT CPU feature matcher
```

と表示される場合があります。

**この表示だけで、SuperPoint + LightGlue のマッチが SIFT に置き換えられたわけではありません。**

この処理の直前に HLoc は、

```text
import_features()
```

で SuperPoint の keypoints を DB へ登録し、

```text
import_matches()
```

で LightGlue の matches を DB へ登録しています。

その後、

```python
pycolmap.verify_matches(...)
```

を呼び出し、COLMAP の two-view geometric verification を行います。

つまり基本構成は、

```text
SuperPoint keypoints
        +
LightGlue raw matches
        ↓
COLMAP geometric verification
        ↓
two_view_geometries
```

です。

COLMAP の内部実装上、`verify_matches` 実行時に SIFT matcher の worker が初期化されたというログが出ますが、ここで SuperPoint descriptor を SIFT descriptor に変換したり、画像から SIFT 特徴を再抽出したりしているわけではありません。

HLoc の公式 `reconstruction.py` も同じく、

```text
import_features
→ import_matches
→ estimation_and_geometric_verification
```

の順で処理しています。

## DBで確認する

幾何検証終了後:

```bash
sqlite3 "$DB_RAW" \
'SELECT COUNT(*) FROM images;'
```

```bash
sqlite3 "$DB_RAW" \
'SELECT COUNT(*) FROM keypoints;'
```

```bash
sqlite3 "$DB_RAW" \
'SELECT COUNT(*) FROM matches;'
```

```bash
sqlite3 "$DB_RAW" \
'SELECT COUNT(*) FROM two_view_geometries;'
```

さらに descriptor を確認する場合:

```bash
sqlite3 "$DB_RAW" \
'SELECT COUNT(*) FROM descriptors;'
```

HLoc のこの構成では、COLMAP DB に SIFT descriptor を入れていないため、descriptor 行数は通常 0 です。

LightGlue の raw match は `matches`、幾何検証後の inlier 情報は `two_view_geometries` に保存されます。

---

# 11. Pose prior は必須か

## 結論

**SfMそのものには必須ではありません。**

SuperPoint + LightGlue の DB が完成していれば、通常の COLMAP `mapper` だけでも 3D reconstruction は可能です。

```bash
SFM_NO_PRIOR="$OUT/sparse_no_prior"

mkdir -p "$SFM_NO_PRIOR"

colmap mapper \
    --database_path "$DB_RAW" \
    --image_path images \
    --output_path "$SFM_NO_PRIOR"
```

---

# 12. Pose prior がない場合のスケール

単眼 SfM のみでは、復元結果には絶対スケールがありません。

つまり、

```text
COLMAP上の距離 1.0
```

が、

```text
1 m
```

である保証はありません。

復元形状は、一般に

- 回転
- 並進
- スケール

を含む 3D similarity transform の自由度を持ちます。

したがって、`poses.csv` を **pairs の選択に使っただけ**では、SfMモデルにメートルスケールが入るわけではありません。

今回の `make_spatial_pairs.py` は、

```text
3 m以内の画像をマッチ候補にする
```

ために位置を使っているだけです。

SfM optimizer にはその数値を位置制約として渡していません。

---

# 13. メートルスケールを与える方法

大きく2通りあります。

## 方法A: SfM中に pose prior を使用

```text
poses.csv
   ↓
import_pose_priors
   ↓
pose_prior_mapper
```

とします。

位置priorがメートル単位で正しく登録されれば、SfMをその座標系・スケールへ拘束できます。

### priorを登録

HLoc DB には間引き後の画像しかないため、`--skip-missing` を付けます。

```bash
source /opt/ros/humble/setup.bash
source ~/lab_navigation_ws/install/local_setup.bash
```

```bash
DB_PRIOR="$OUT/database_superpoint_prior.db"
```

```bash
ros2 run sfm_capture import_pose_priors \
    poses_map_base/poses.csv \
    "$DB_RAW" \
    --output-database "$DB_PRIOR" \
    --skip-missing
```

確認:

```bash
ros2 run sfm_capture import_pose_priors \
    poses_map_base/poses.csv \
    "$DB_PRIOR" \
    --verify-only \
    --skip-missing
```

### pose_prior_mapper

```bash
SFM_PRIOR="$OUT/sparse_superpoint_prior"

mkdir -p "$SFM_PRIOR"

colmap pose_prior_mapper \
    --database_path "$DB_PRIOR" \
    --image_path images \
    --output_path "$SFM_PRIOR" \
    --use_robust_loss_on_prior_position 1
```

---

## 方法B: 通常SfM後にモデルを実世界座標へ合わせる

先に通常の `mapper` で SfM を行い、

```text
arbitrary scale SfM
        ↓
camera center correspondences
        ↓
model_aligner
        ↓
metric / map coordinates
```

とする方法です。

COLMAP の `model_aligner` は、SfMで推定されたカメラ中心と実測カメラ中心の対応から 3D similarity transform を推定できます。

つまり、**pose prior を SfM中に使わなくても、後からスケールを与えることは可能**です。

この方法は、

- pose prior がやや粗い
- SfMそのものの性能を純粋に評価したい
- prior に SfM形状を引っ張られたくない

場合に適しています。

---

# 14. 今回のデータでの注意: base_link と camera center

`poses_map_base/poses.csv` の位置が `base_link` 原点の場合、その XYZ はカメラ中心そのものではありません。

SfM の pose prior や `model_aligner` に理想的なのは、

```text
world frame 上の camera center
```

です。

車体原点とカメラ位置に数十cmのオフセットがある場合、そのまま強い prior として使うと誤差要因になります。

可能なら、

```text
map → camera_optical_frame
```

を直接取得するか、

```text
map → base_link
+
base_link → camera
```

の固定変換でカメラ中心へ補正した位置を使用します。

---

# 15. どちらを選ぶべきか

## 地図作成品質を比較したい場合

まずは prior なし:

```text
SuperPoint
+
LightGlue
+
mapper
```

を実行する方法が分かりやすいです。

その後、必要なら `model_aligner` で実世界座標へ合わせます。

これなら特徴量・マッチング・SfMだけの性能を評価しやすくなります。

## 最初から map 座標・メートルスケールで作りたい場合

```text
SuperPoint
+
LightGlue
+
position prior
+
pose_prior_mapper
```

を使います。

ただし、prior 位置はできるだけ **実際のcamera center** に補正しておくことを推奨します。

---

# 16. 今回のおすすめ比較

同じ `database_superpoint_raw.db` から2系統作ると比較しやすくなります。

```text
database_superpoint_raw.db
        │
        ├───────────────┐
        │               │
        ↓               ↓
     mapper        import_pose_priors
        │               │
        ↓               ↓
 arbitrary          pose_prior_mapper
 scale model             │
        │                ↓
        │          metric/map model
        ↓
 model_aligner
        │
        ↓
 metric/map model
```

比較対象:

```text
A. SuperPoint + LightGlue + mapper + 後段alignment
B. SuperPoint + LightGlue + pose_prior_mapper
```

この比較をすると、pose prior が reconstruction 自体にどの程度影響しているかも確認できます。

---

# 17. DB変換後のチェック

現在の実行ログで、

```text
SuperPoint images : 1372
Existing images   : 1372
COLMAP images     : 1372
Importing features: 1372/1372
Importing matches : 11219/11219
```

まで進んでいる場合、SuperPoint と LightGlue の DB import は正常に進んでいます。

幾何検証終了後に次を確認します。

```bash
sqlite3 "$DB_RAW" <<'SQL'
SELECT 'images', COUNT(*) FROM images;
SELECT 'keypoints', COUNT(*) FROM keypoints;
SELECT 'descriptors', COUNT(*) FROM descriptors;
SELECT 'matches', COUNT(*) FROM matches;
SELECT 'two_view_geometries', COUNT(*) FROM two_view_geometries;
SQL
```

重要なのは、

```text
two_view_geometries > 0
```

になっていることです。

---

# 18. 次に実行するコマンド

## priorを使わずまずSfMする場合

```bash
SFM_NO_PRIOR="$OUT/sparse_no_prior"

mkdir -p "$SFM_NO_PRIOR"

colmap mapper \
    --database_path "$DB_RAW" \
    --image_path images \
    --output_path "$SFM_NO_PRIOR"
```

## priorありでSfMする場合

```bash
DB_PRIOR="$OUT/database_superpoint_prior.db"

ros2 run sfm_capture import_pose_priors \
    poses_map_base/poses.csv \
    "$DB_RAW" \
    --output-database "$DB_PRIOR" \
    --skip-missing
```

```bash
SFM_PRIOR="$OUT/sparse_superpoint_prior"

mkdir -p "$SFM_PRIOR"

colmap pose_prior_mapper \
    --database_path "$DB_PRIOR" \
    --image_path images \
    --output_path "$SFM_PRIOR" \
    --use_robust_loss_on_prior_position 1
```

---

# 19. まとめ

今回の構成では、

```text
特徴抽出       : SuperPoint
特徴マッチ     : LightGlue
候補ペア選択   : poses.csv + Sequential
幾何検証       : COLMAP / PyCOLMAP
SfM            : COLMAP mapper または pose_prior_mapper
```

です。

幾何検証時に `Creating SIFT CPU feature matcher` と表示されても、HLoc が登録した SuperPoint keypoints / LightGlue matches が SIFT 特徴へ置き換わったことを意味しません。

また、pose prior は SfM の成立自体には必須ではありません。

ただし、単眼 SfM に **実世界のメートルスケール**を与えたい場合は、

- pose prior を使う
- SfM後に model alignment する
- 既知距離など別の尺度を使う

のいずれかが必要です。
