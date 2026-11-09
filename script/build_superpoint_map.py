#!/usr/bin/env python3
"""Build a pose-prior SuperPoint/LightGlue map in the canonical session layout.

Run with the workspace .venv after sourcing ROS 2 and install/local_setup.bash.
The destination map and run must not exist; existing maps are never replaced.
"""

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

from hloc import extract_features, match_features


def run(*command):
    print('+', ' '.join(map(str, command)), flush=True)
    subprocess.run([str(part) for part in command], check=True)


def camera_params(camera_info):
    info = json.loads(camera_info.read_text())
    k = info['k']
    if k[0] <= 0 or k[4] <= 0 or any(abs(value) > 1e-12 for value in info['d']):
        raise ValueError('PINHOLE requires positive focal lengths and zero distortion')
    return ','.join(str(k[index]) for index in (0, 4, 2, 5))


def usable_images(poses_csv, image_dir):
    names = set()
    with poses_csv.open(newline='') as stream:
        reader = csv.DictReader(stream)
        if not {'image_name', 'status'} <= set(reader.fieldnames or []):
            raise ValueError(f'Missing image_name or status in {poses_csv}')
        for row in reader:
            name = row['image_name'].strip()
            if row['status'] == 'ok' and name and (image_dir / name).is_file():
                names.add(name)
    if len(names) < 2:
        raise ValueError(f'Fewer than two valid camera poses with images in {poses_csv}')
    return sorted(names)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--run-name', default='run01')
    parser.add_argument('--max-distance', type=float, default=3.0)
    parser.add_argument('--max-neighbors', type=int, default=30)
    parser.add_argument('--sequential-overlap', type=int, default=0)
    args = parser.parse_args()

    session = args.session.resolve(strict=True)
    images = session / 'images'
    poses = session / 'poses_world_camera_gt' / 'poses.csv'
    info = session / 'camera_info.json'
    destination = session / 'maps' / 'superpoint'
    work = session / 'runs' / 'superpoint' / args.run_name
    for path in (images, poses, info):
        if not path.exists():
            raise FileNotFoundError(path)
    if destination.exists() or work.exists():
        raise FileExistsError(f'Map or run already exists: {destination}, {work}')
    params = camera_params(info)
    image_names = usable_images(poses, images)
    script_dir = Path(__file__).resolve().parent
    staged_map = work / 'map'
    features = staged_map / 'reference_features.h5'
    pairs = work / 'pairs_spatial.txt'
    matches = work / 'matches_lightglue.h5'
    raw_db = work / 'database_raw.db'
    database = staged_map / 'database.db'
    sparse = staged_map / 'sparse'
    work.mkdir(parents=True)
    staged_map.mkdir()
    sparse.mkdir()

    extract_features.main(
        extract_features.confs['superpoint_aachen'], images,
        feature_path=features, image_list=image_names, as_half=True,
    )
    run(sys.executable, script_dir / 'superpoint_sfm' / 'make_spatial_pairs.py',
        poses, pairs, '--features', features, '--image-dir', images,
        '--max-distance', args.max_distance, '--max-neighbors', args.max_neighbors,
        '--sequential-overlap', args.sequential_overlap)
    match_features.main(
        match_features.confs['superpoint+lightglue'], pairs, features,
        matches=matches,
    )
    run(sys.executable, script_dir / 'superpoint_sfm' / 'build_hloc_colmap_db.py',
        '--image-dir', images, '--features', features, '--pairs', pairs,
        '--matches', matches, '--database', raw_db,
        '--camera-model', 'PINHOLE', '--camera-params', params)
    run('ros2', 'run', 'sfm_capture', 'import_pose_priors', poses, raw_db,
        '--output-database', database, '--skip-missing')
    run('colmap', 'pose_prior_mapper', '--database_path', database,
        '--image_path', images, '--output_path', sparse,
        '--use_robust_loss_on_prior_position', '1')
    if not (sparse / '0' / 'images.bin').is_file():
        raise RuntimeError(f'No model 0 created in {sparse}; inspect COLMAP output')
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    staged_map.rename(destination)
    report_path = destination / 'database.db.pose_priors.json'
    if report_path.is_file():
        report = json.loads(report_path.read_text())
        report['output_database'] = str(destination / 'database.db')
        report_path.write_text(json.dumps(report, indent=2) + '\n')
    run(sys.executable, script_dir / 'prepare_localization.py', session,
        '--method', 'superpoint')
    print(f'Complete: {destination}')


if __name__ == '__main__':
    main()
