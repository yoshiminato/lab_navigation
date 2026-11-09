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
