#!/usr/bin/env python3

import argparse
import csv
import os
import subprocess
import time
from pathlib import Path

import pycolmap


def run_command(cmd, name):
    print()
    print("=" * 70)
    print(name)
    print("=" * 70)

    start = time.perf_counter()

    result = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    elapsed = time.perf_counter() - start

    print(result.stdout)

    if result.returncode != 0:
        raise RuntimeError(
            f"{name} failed "
            f"(returncode={result.returncode})"
        )

    return elapsed


def create_query_intrinsics(
    query_name,
    model_path,
    output_path,
):
    reconstruction = pycolmap.Reconstruction(
        str(model_path)
    )

    camera = next(
        iter(reconstruction.cameras.values())
    )

    params = " ".join(
        f"{x:.12g}"
        for x in camera.params
    )

    line = (
        f"{query_name} "
        f"{camera.model.name} "
        f"{camera.width} "
        f"{camera.height} "
        f"{params}\n"
    )

    output_path.write_text(line)


def read_result(
    result_file,
    query_name,
):
    if not result_file.exists():
        return None

    for line in result_file.read_text().splitlines():

        values = line.split()

        if len(values) != 8:
            continue

        if values[0] == query_name:
            return values[1:]

    return None


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--data_path",
        required=True,
        help="Dataset root",
    )

    parser.add_argument(
        "--query_list",
        required=True,
        help="Query image list",
    )

    parser.add_argument(
        "--output_dir",
        default=None,
    )

    parser.add_argument(
        "--num_matched",
        type=int,
        default=20,
    )

    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="0 = all images",
    )

    args = parser.parse_args()

    data_path = Path(args.data_path)

    image_dir = data_path / "images"
    sfm_model = data_path / "sparse" / "0"

    base_hloc_dir = data_path / "hloc_output"

    reference_local_features = (
        base_hloc_dir / "feats-shift.h5"
    )

    reference_global_features = (
        base_hloc_dir / "global-ref-netvlad.h5"
    )

    if args.output_dir is None:
        output_dir = (
            base_hloc_dir /
            "sequential_benchmark"
        )
    else:
        output_dir = Path(
            args.output_dir
        )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    query_list_path = Path(
        args.query_list
    )

    query_names = [
        line.strip()
        for line
        in query_list_path.read_text().splitlines()
        if line.strip()
    ]

    if args.limit > 0:
        query_names = query_names[
            :args.limit
        ]

    print(
        f"Number of queries: "
        f"{len(query_names)}"
    )

    print(
        f"Image directory: "
        f"{image_dir}"
    )

    print(
        f"Reference SfM: "
        f"{sfm_model}"
    )

    print(
        f"Reference local features: "
        f"{reference_local_features}"
    )

    print(
        f"Reference global features: "
        f"{reference_global_features}"
    )

    csv_path = (
        output_dir /
        "benchmark.csv"
    )

    rows = []

    for index, query_name in enumerate(
        query_names,
        start=1,
    ):

        print()
        print()
        print("#" * 70)
        print(
            f"[{index}/{len(query_names)}] "
            f"{query_name}"
        )
        print("#" * 70)

        query_dir = (
            output_dir /
            f"query_{index:04d}"
        )

        query_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        one_query_list = (
            query_dir /
            "query.txt"
        )

        one_query_list.write_text(
            query_name + "\n"
        )

        query_local_features = (
            query_dir /
            "feats-query-sift.h5"
        )

        query_global_features = (
            query_dir /
            "global-query-netvlad.h5"
        )

        retrieval_pairs = (
            query_dir /
            "pairs-query-netvlad.txt"
        )

        matches_file = (
            query_dir /
            "matches-query-sift.h5"
        )

        query_intrinsics = (
            query_dir /
            "queries_with_intrinsics.txt"
        )

        localization_results = (
            query_dir /
            "localization_results.txt"
        )

        # =====================================
        # 1. SIFT extraction
        # =====================================

        t_sift = run_command(
            [
                "python3",
                "-m",
                "hloc.extract_features",
                "--image_dir",
                str(image_dir),
                "--export_dir",
                str(query_dir),
                "--conf",
                "sift",
                "--image_list",
                str(one_query_list),
                "--feature_path",
                str(query_local_features),
            ],
            "1. Query SIFT extraction",
        )

        # =====================================
        # 2. NetVLAD extraction
        # =====================================

        t_netvlad = run_command(
            [
                "python3",
                "-m",
                "hloc.extract_features",
                "--image_dir",
                str(image_dir),
                "--export_dir",
                str(query_dir),
                "--conf",
                "netvlad",
                "--image_list",
                str(one_query_list),
                "--feature_path",
                str(query_global_features),
                "--as_half",
            ],
            "2. Query NetVLAD extraction",
        )

        # =====================================
        # 3. Retrieval
        # =====================================

        t_retrieval = run_command(
            [
                "python3",
                "-m",
                "hloc.pairs_from_retrieval",
                "--descriptors",
                str(query_global_features),
                "--db_descriptors",
                str(reference_global_features),
                "--db_model",
                str(sfm_model),
                "--query_list",
                str(one_query_list),
                "--output",
                str(retrieval_pairs),
                "--num_matched",
                str(args.num_matched),
            ],
            "3. NetVLAD retrieval",
        )

        # =====================================
        # 4. SIFT matching
        # =====================================

        match_script = f"""
from pathlib import Path
from hloc import match_features

conf = match_features.confs["NN-ratio"]

match_features.main(
    conf,
    pairs=Path(r"{retrieval_pairs}"),
    features=Path(r"{query_local_features}"),
    features_ref=Path(r"{reference_local_features}"),
    matches=Path(r"{matches_file}"),
    overwrite=True,
)
"""

        t_matching = run_command(
            [
                "python3",
                "-c",
                match_script,
            ],
            "4. SIFT matching",
        )

        # =====================================
        # 5. Query intrinsics
        # =====================================

        create_query_intrinsics(
            query_name,
            sfm_model,
            query_intrinsics,
        )

        # =====================================
        # 6. PnP localization
        # =====================================

        t_localization = run_command(
            [
                "python3",
                "-m",
                "hloc.localize_sfm",
                "--reference_sfm",
                str(sfm_model),
                "--queries",
                str(query_intrinsics),
                "--features",
                str(query_local_features),
                "--matches",
                str(matches_file),
                "--retrieval",
                str(retrieval_pairs),
                "--results",
                str(localization_results),
            ],
            "5. PnP localization",
        )

        total = (
            t_sift
            + t_netvlad
            + t_retrieval
            + t_matching
            + t_localization
        )

        pose = read_result(
            localization_results,
            query_name,
        )

        success = (
            pose is not None
        )

        print()
        print("=" * 70)
        print("RESULT")
        print("=" * 70)

        print(
            f"SIFT       : "
            f"{t_sift:.3f} s"
        )

        print(
            f"NetVLAD    : "
            f"{t_netvlad:.3f} s"
        )

        print(
            f"Retrieval  : "
            f"{t_retrieval:.3f} s"
        )

        print(
            f"Matching   : "
            f"{t_matching:.3f} s"
        )

        print(
            f"PnP        : "
            f"{t_localization:.3f} s"
        )

        print(
            f"TOTAL      : "
            f"{total:.3f} s"
        )

        print(
            f"Localization: "
            f"{'SUCCESS' if success else 'FAILED'}"
        )

        if success:

            qw, qx, qy, qz, tx, ty, tz = pose

            print(
                "Pose:"
            )

            print(
                f"  q = "
                f"({qw}, {qx}, "
                f"{qy}, {qz})"
            )

            print(
                f"  t = "
                f"({tx}, {ty}, {tz})"
            )

        rows.append({
            "query": query_name,
            "success": success,
            "sift_s": t_sift,
            "netvlad_s": t_netvlad,
            "retrieval_s": t_retrieval,
            "matching_s": t_matching,
            "pnp_s": t_localization,
            "total_s": total,
        })

        with csv_path.open(
            "w",
            newline="",
        ) as f:

            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "query",
                    "success",
                    "sift_s",
                    "netvlad_s",
                    "retrieval_s",
                    "matching_s",
                    "pnp_s",
                    "total_s",
                ],
            )

            writer.writeheader()
            writer.writerows(rows)

    # =========================================
    # Summary
    # =========================================

    if not rows:
        return

    successful = [
        row
        for row in rows
        if row["success"]
    ]

    print()
    print()
    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)

    print(
        f"Queries    : "
        f"{len(rows)}"
    )

    print(
        f"Successful : "
        f"{len(successful)}"
    )

    if successful:

        avg_total = sum(
            row["total_s"]
            for row in successful
        ) / len(successful)

        min_total = min(
            row["total_s"]
            for row in successful
        )

        max_total = max(
            row["total_s"]
            for row in successful
        )

        print(
            f"Average    : "
            f"{avg_total:.3f} s"
        )

        print(
            f"Minimum    : "
            f"{min_total:.3f} s"
        )

        print(
            f"Maximum    : "
            f"{max_total:.3f} s"
        )

        print(
            f"Rate       : "
            f"{1.0 / avg_total:.3f} Hz"
        )

    print()
    print(
        f"CSV: {csv_path}"
    )


if __name__ == "__main__":
    main()