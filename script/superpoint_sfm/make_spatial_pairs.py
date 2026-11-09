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
