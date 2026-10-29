# HLoc用 Spatial Pairs 生成手順（画像間引き対応）

## 概要

この手順では、`poses.csv` に保存された各画像の位置情報を利用して、HLoc の `match_features` で使用する `pairs_spatial.txt` を生成します。

画像を間引いている場合でも動作するように、**SuperPoint の特徴量ファイル `.h5` に実際に存在する画像だけを対象として近傍探索**します。

これにより、`poses.csv` には存在するが、間引き後の画像群には存在しない画像が `pairs.txt` に混入して、

```text
KeyError: object '00001729.png' doesn't exist
```

のようなエラーになることを防ぎます。

---

## 想定ディレクトリ

```text
20261010/
├── images/
│   ├── 00000001.png
│   ├── 00000004.png
│   └── ...
├── poses_map_base/
│   └── poses.csv
├── make_spatial_pairs.py
└── hloc_superpoint/
    └── feats-superpoint-n4096-r1024.h5
```

`poses.csv` に撮影時の全画像が残っていても問題ありません。

例:

```text
poses.csv      : 2000枚分
images/        : 1400枚
SuperPoint H5  : 1372枚
```

この場合、H5 に実際に特徴量が存在する 1372 枚だけを使ってペアを生成します。

---

## 1. HLoc環境を有効化

例:

```bash
cd ~/lab_navigation_ws
source .venv/bin/activate
```

---

## 2. SuperPoint特徴量を準備

まだ抽出していない場合:

```bash
OUT="$PWD/hloc_superpoint"
mkdir -p "$OUT"

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
ls "$OUT/feats-superpoint-n4096-r1024.h5"
```

---

## 3. `make_spatial_pairs.py` を作成

以下を `make_spatial_pairs.py` として保存します。

```python
#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import h5py
import numpy as np


def get_feature_image_names(features_path: Path) -> set[str]:
    """
    HLocのfeatures.h5に実際に特徴量が保存されている画像名を取得する。
    """
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
            "poses.csvの位置情報からHLoc用画像ペアを生成する。"
            "SuperPoint特徴量が存在する画像だけを使用するため、"
            "画像を間引いていても動作する。"
        )
    )

    parser.add_argument("poses_csv", type=Path)
    parser.add_argument("output", type=Path)

    parser.add_argument(
        "--features",
        type=Path,
        required=True,
        help="HLocのSuperPoint特徴量 .h5 ファイル",
    )

    parser.add_argument(
        "--image-dir",
        type=Path,
        default=None,
        help="指定した場合、画像ファイルの存在も確認する",
    )

    parser.add_argument(
        "--max-distance",
        type=float,
        default=3.0,
        help="Spatial matchingの最大距離[m]",
    )

    parser.add_argument(
        "--max-neighbors",
        type=int,
        default=30,
        help="各画像のSpatial近傍数上限",
    )

    parser.add_argument(
        "--ignore-z",
        action="store_true",
        help="Zを無視し、XY距離だけで近傍探索する",
    )

    parser.add_argument(
        "--sequential-overlap",
        type=int,
        default=0,
        help="間引き後の画像順で前後N枚もペアに追加する",
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

            # 間引かれてSuperPoint特徴量が存在しない画像を除外
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

## 4. Spatial pairs を生成

まずは以下を推奨します。

- 最大距離: 3 m
- Spatial近傍: 最大10枚
- Sequential: 前後5枚
- Z座標も使用

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

---

## 5. 生成結果を確認

ペア数:

```bash
wc -l "$OUT/pairs_spatial.txt"
```

先頭:

```bash
head "$OUT/pairs_spatial.txt"
```

例:

```text
00000001.png 00000004.png
00000001.png 00000007.png
00000004.png 00000007.png
```

---

## 6. LightGlue でマッチング

```bash
python -m hloc.match_features \
    --pairs "$OUT/pairs_spatial.txt" \
    --export_dir "$OUT" \
    --features feats-superpoint-n4096-r1024 \
    --conf superpoint+lightglue
```

処理の流れ:

```text
images/
   │
   ↓
SuperPoint
   │
   ↓
features.h5
   │
   ├──────────────┐
   │              │
   │          poses.csv
   │              │
   └──────┬───────┘
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
```

---

## なぜ exhaustive matching を使わないのか

1372枚を全組み合わせすると、

```text
1372 × 1371 / 2 = 940,506 pairs
```

となります。

`pairs_from_exhaustive` で生成した約94万ペアを LightGlue で処理すると非常に時間がかかります。

そこで、位置情報を使って比較対象を近傍画像だけに限定します。

---

## 画像を間引いていても動く理由

このスクリプトでは、以下の順番で対象画像を決めます。

```text
poses.csv
   ↓
status == ok
   ↓
SuperPoint H5 に特徴量が存在するか
   ↓
画像ファイルが存在するか
   ↓
残った画像だけでXYZ近傍探索
   ↓
pairs_spatial.txt
```

重要なのは、**近傍探索をする前に間引かれた画像を除外すること**です。

例えば、先に30近傍を選んでから存在しない20枚を削除すると、実質10近傍しか残らない可能性があります。

このスクリプトでは、実際に使用可能な画像だけで改めて近傍10枚・30枚を選ぶため、この問題を避けられます。

---

## Sequentialとの併用

連続走行で撮影した画像では、Spatial だけでなく Sequential も併用することを推奨します。

```bash
--sequential-overlap 5
```

を指定すると、**間引き後に残った画像の順番**で前後5枚をペアに追加します。

例えば、

```text
元画像:
00000001
00000002
00000003
00000004
00000005
00000006

間引き後:
00000001
00000004
00000006
```

の場合、連番差ではなく、

```text
00000001 → 00000004 → 00000006
```

を連続画像として扱います。

---

## パラメータの目安

最初は、

```text
max_distance       = 3.0 m
max_neighbors      = 10
sequential_overlap = 5
```

を推奨します。

SfM の接続が不足する場合は、

```text
max_neighbors:
10 → 20 → 30
```

の順に増やします。

位置誤差が大きい場合は `max_distance` も広げます。

---

## Zを無視する場合

床面を移動するロボットなどで XY 距離だけを使いたい場合:

```bash
python make_spatial_pairs.py \
    poses_map_base/poses.csv \
    "$OUT/pairs_spatial.txt" \
    --features "$OUT/feats-superpoint-n4096-r1024.h5" \
    --image-dir images \
    --max-distance 3.0 \
    --max-neighbors 10 \
    --sequential-overlap 5 \
    --ignore-z
```

---

## 次の工程

LightGlue のマッチング後は、

```text
SuperPoint keypoints
+
LightGlue matches
+
位置 prior
        ↓
COLMAP database
        ↓
pose_prior_mapper
        ↓
sparse model
```

という流れで、SIFT の代わりに SuperPoint + LightGlue を利用した位置制約付き SfM に接続します。
