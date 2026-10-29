# colmap_db_to_hloc.py

import argparse
import sqlite3

import h5py
import numpy as np
import pycolmap


def main(database_path, model_path, output_path):
    reconstruction = pycolmap.Reconstruction(model_path)

    registered_names = {
        image.name
        for image in reconstruction.images.values()
    }

    conn = sqlite3.connect(database_path)
    cur = conn.cursor()

    rows = cur.execute("""
        SELECT
            images.image_id,
            images.name,
            cameras.width,
            cameras.height,
            keypoints.rows,
            keypoints.cols,
            keypoints.data,
            descriptors.rows,
            descriptors.cols,
            descriptors.data
        FROM images
        JOIN cameras
            ON images.camera_id = cameras.camera_id
        JOIN keypoints
            ON images.image_id = keypoints.image_id
        JOIN descriptors
            ON images.image_id = descriptors.image_id
    """)

    count = 0

    with h5py.File(output_path, "w") as h5:
        for (
            image_id,
            name,
            width,
            height,
            kp_rows,
            kp_cols,
            kp_blob,
            desc_rows,
            desc_cols,
            desc_blob,
        ) in rows:

            if name not in registered_names:
                continue

            keypoints = np.frombuffer(
                kp_blob, dtype=np.float32
            ).reshape(kp_rows, kp_cols)

            descriptors = np.frombuffer(
                desc_blob, dtype=np.uint8
            ).reshape(desc_rows, desc_cols)

            if kp_rows != desc_rows:
                raise RuntimeError(
                    f"{name}: keypoints={kp_rows}, "
                    f"descriptors={desc_rows}"
                )

            # COLMAP → HLoc pixel convention
            keypoints_xy = keypoints[:, :2].copy()
            keypoints_xy -= 0.5

            # uint8 COLMAP descriptor → float unit descriptor
            descriptors = descriptors.astype(np.float32)

            norm = np.linalg.norm(
                descriptors, axis=1, keepdims=True
            )
            descriptors /= np.maximum(norm, 1e-8)

            group = h5.create_group(name)

            group.create_dataset(
                "keypoints",
                data=keypoints_xy.astype(np.float32),
            )

            # HLoc expects D x N
            group.create_dataset(
                "descriptors",
                data=descriptors.T.astype(np.float32),
            )

            group.create_dataset(
                "image_size",
                data=np.array(
                    [width, height],
                    dtype=np.int32,
                ),
            )

            count += 1

    conn.close()

    print(f"Exported {count} registered images")
    print(f"Output: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--database", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", required=True)

    args = parser.parse_args()

    main(
        args.database,
        args.model,
        args.output,
    )
