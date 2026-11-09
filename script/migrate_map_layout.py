#!/usr/bin/env python3
"""Move known capture sessions to maps/<method>/ and retrieval/.

The default is a dry run.  Every move is refused if its destination exists;
there is no copying, deleting, or implicit overwrite of research data.
"""

import argparse
from pathlib import Path


def move(root, source, destination, apply):
    src, dst = root / source, root / destination
    if not apply:
        print(f'{src} -> {dst}')
        return
    if not src.exists():
        if dst.exists():
            return  # already migrated
        raise FileNotFoundError(src)
    if dst.exists():
        raise FileExistsError(f'Refusing to replace {dst}')
    print(f'{src} -> {dst}')
    dst.parent.mkdir(parents=True, exist_ok=True)
    src.rename(dst)


def cleanup(root, relative, apply):
    directory = root / relative
    if directory.is_dir() and not any(directory.iterdir()):
        print(f'empty directory: {directory}')
        if apply:
            directory.rmdir()


def migrate_isaac_001(root, apply):
    move(root, 'colmap_gt', 'maps/sift', apply)
    move(root, 'hloc_output_superpoint_lightglue_run01',
         'runs/superpoint/run01', apply)
    move(root, 'runs/superpoint/run01/sparse_superpoint_prior/0',
         'maps/superpoint/sparse/0', apply)
    move(root, 'runs/superpoint/run01/database_superpoint_prior.db',
         'maps/superpoint/database.db', apply)
    move(root, 'runs/superpoint/run01/database_superpoint_prior.db.pose_priors.json',
         'maps/superpoint/database.db.pose_priors.json', apply)
    move(root, 'runs/superpoint/run01/feats-superpoint-n4096-r1024.h5',
         'maps/superpoint/reference_features.h5', apply)
    cleanup(root, 'runs/superpoint/run01/sparse_superpoint_prior', apply)
    move(root, 'maps/sift/gt_comparison',
         'runs/sift/gt_comparison', apply)
    move(root, 'maps/sift/sparse_txt',
         'runs/sift/model_text', apply)
    move(root, 'maps/sift/sparse_no_robust',
         'runs/sift/no_robust/sparse', apply)
    move(root, 'maps/sift/no_robust.log',
         'runs/sift/no_robust/mapper.log', apply)


def migrate_isaac_gt(root, apply):
    move(root, 'colmap_gt', 'maps/sift', apply)
    move(root, 'maps/sift/hloc_output/feats-shift.h5',
         'maps/sift/reference_features.h5', apply)
    move(root, 'maps/sift/hloc_output/global-ref-netvlad.h5',
         'retrieval/netvlad.h5', apply)
    cleanup(root, 'maps/sift/hloc_output', apply)


def migrate_hv_20261010(root, apply):
    move(root, 'sparse/0', 'maps/sift/sparse/0', apply)
    move(root, 'database.db', 'maps/sift/database.db', apply)
    move(root, 'features.db', 'maps/sift/features.db', apply)
    move(root, 'hloc_output', 'runs/sift/legacy', apply)
    move(root, 'runs/sift/legacy/feats-shift.h5',
         'maps/sift/reference_features.h5', apply)
    move(root, 'runs/sift/legacy/global-ref-netvlad.h5',
         'retrieval/netvlad.h5', apply)
    move(root, 'hloc_output_superpoint_lightglue',
         'runs/superpoint/triangulated', apply)
    move(root, 'runs/superpoint/triangulated/sfm-superpoint-lightglue-colmapgv',
         'maps/superpoint/sparse/0', apply)
    move(root, 'maps/superpoint/sparse/0/database.db',
         'maps/superpoint/database.db', apply)
    move(root, 'runs/superpoint/triangulated/feats-ref-superpoint.h5',
         'maps/superpoint/reference_features.h5', apply)
    move(root, 'hloc_superpoint', 'runs/superpoint/pose_prior_experiment', apply)
    move(root, 'sparse_pure', 'runs/sift/no_prior', apply)
    cleanup(root, 'sparse', apply)


def migrate_hv_20261007(root, apply):
    move(root, 'sparse', 'maps/sift/sparse', apply)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', type=Path,
                        help='Workspace captures directory')
    parser.add_argument('--apply', action='store_true',
                        help='Perform the checked renames')
    args = parser.parse_args()
    captures = args.captures.resolve(strict=True)
    sessions = (
        ('isaac_001', migrate_isaac_001),
        ('isaac_gt_test_002', migrate_isaac_gt),
        ('hv_building_lab/20261010', migrate_hv_20261010),
        ('hv_building_lab/20261007', migrate_hv_20261007),
    )
    for relative, migrate in sessions:
        root = captures / relative
        if not root.is_dir():
            raise FileNotFoundError(root)
        print(f'\n[{relative}]')
        migrate(root, args.apply)
    print('\nMigration complete.' if args.apply else '\nDry run only.')


if __name__ == '__main__':
    main()
