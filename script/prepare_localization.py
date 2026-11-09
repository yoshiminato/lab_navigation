#!/usr/bin/env python3
"""Prepare and validate local features and shared NetVLAD for a capture session.

Run with the workspace HLoc virtual environment.  Paths follow
captures/<session>/maps/<method>/ and captures/<session>/retrieval/.
"""

import argparse
import subprocess
import sys
from pathlib import Path

import h5py
import pycolmap

from hloc import extract_features
from hloc.utils.io import list_h5_names


METHODS = {'sift': 'sift', 'superpoint': 'superpoint'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--method', choices=METHODS, required=True)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()

    session = args.session.resolve(strict=True)
    images = session / 'images'
    method_dir = session / 'maps' / args.method
    model_path = method_dir / 'sparse' / '0'
    local_path = method_dir / 'reference_features.h5'
    global_path = session / 'retrieval' / 'netvlad.h5'

    if not images.is_dir():
        raise FileNotFoundError(images)
    model = pycolmap.Reconstruction(model_path)
    names = sorted(image.name for image in model.images.values())
    if not names:
        raise ValueError(f'No registered images in {model_path}')
    missing_images = [name for name in names if not (images / name).is_file()]
    if missing_images:
        raise FileNotFoundError(f'{len(missing_images)} model images missing: {missing_images[:5]}')

    if args.method == 'sift' and not local_path.exists() and not args.verify_only:
        database = method_dir / 'database.db'
        if not database.is_file():
            raise FileNotFoundError(database)
        converter = Path(__file__).with_name('colmap_db_to_hloc.py')
        subprocess.run([
            sys.executable, str(converter), '--database', str(database),
            '--model', str(model_path), '--output', str(local_path),
        ], check=True)

    if not local_path.is_file():
        raise FileNotFoundError(local_path)
    with h5py.File(local_path, 'r') as features:
        local_names = set(list_h5_names(local_path))
        missing_local = set(names) - local_names
        if missing_local:
            raise ValueError(f'{len(missing_local)} registered images lack {args.method} features')
        for image in model.images.values():
            count = features[image.name]['keypoints'].shape[0]
            if count != len(image.points2D):
                raise ValueError(
                    f'{image.name}: feature keypoints={count}, model points2D={len(image.points2D)}'
                )

    global_names = set(list_h5_names(global_path)) if global_path.exists() else set()
    missing_global = sorted(set(names) - global_names)
    if missing_global and not args.verify_only:
        global_path.parent.mkdir(parents=True, exist_ok=True)
        extract_features.main(
            extract_features.confs['netvlad'], images,
            export_dir=global_path.parent, as_half=True,
            image_list=missing_global, feature_path=global_path,
        )
        global_names = set(list_h5_names(global_path))
        missing_global = sorted(set(names) - global_names)

    print(f'Session: {session}')
    print(f'Method: {args.method}')
    print(f'Registered images: {len(names)}')
    print(f'3D points: {len(model.points3D)}')
    print(f'Local features: {local_path}')
    print(f'NetVLAD: {global_path}')
    print(f'Missing NetVLAD descriptors: {len(missing_global)}')
    if missing_global:
        raise ValueError('Run without --verify-only to create missing NetVLAD descriptors')


if __name__ == '__main__':
    main()
