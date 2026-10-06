"""Copy a COLMAP 4.2+ database, import Cartesian position priors, and verify them.

This module is independent of ROS. Image names are relative to COLMAP image_path,
never matched by CSV row numbers or basenames. The feature database is retained.
"""
import argparse
from contextlib import closing
import json
import math
import os
from pathlib import Path
import sqlite3
import struct
import tempfile

from .pose_io import load_pose_rows, relative_name, sha256_file, write_json


def open_readonly(path):
    path = Path(path).resolve()
    if not path.is_file():
        raise ValueError(f'Database does not exist: {path}; run feature_extractor first')
    return sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)


def database_inventory(connection):
    columns = {row[1] for row in connection.execute('PRAGMA table_info(pose_priors)')}
    required = {'pose_prior_id', 'corr_data_id', 'corr_sensor_id', 'corr_sensor_type',
                'position', 'position_covariance', 'coordinate_system'}
    if not required.issubset(columns):
        raise ValueError('Expected a COLMAP 4.2+ pose_priors schema; use a matching COLMAP/PyCOLMAP version')
    images = {name: dict(image_id=image_id, camera_id=camera_id)
              for image_id, name, camera_id in connection.execute('SELECT image_id,name,camera_id FROM images')}
    if not images:
        raise ValueError('Database has no images; run feature_extractor first')
    return images


def select_rows(rows, images, image_prefix='', skip_missing=False):
    if image_prefix:
        image_prefix = relative_name(image_prefix.rstrip('/')) + '/'
    selected, skipped = [], []
    for row in rows:
        name = relative_name(image_prefix + row['image_name'])
        reason = row['status'] if row['status'] != 'ok' else None
        if reason is None and name not in images:
            reason = 'image_not_in_database'
        if reason:
            skipped.append(dict(image_name=name, reason=reason))
        else:
            selected.append(dict(row, database_name=name, **images[name]))
    if skipped and not skip_missing:
        example = skipped[0]
        raise ValueError(f'{len(skipped)} images cannot be imported; {example}. '
                         'Inspect failures, or explicitly use --skip-missing')
    if not selected:
        raise ValueError('No valid CSV images match the database')
    return selected, skipped


def read_priors(connection):
    priors = {}
    for row in connection.execute(
            'SELECT pose_prior_id,corr_data_id,corr_sensor_id,corr_sensor_type,'
            'position,position_covariance,coordinate_system FROM pose_priors'):
        # SensorType.CAMERA = 0 in the supported COLMAP schema.
        key = (row[3], row[2], row[1])
        priors.setdefault(key, []).append(row)
    return priors


def verify_selected(connection, selected):
    priors = read_priors(connection)
    for row in selected:
        matching = priors.get((0, row['camera_id'], row['image_id']), [])
        if len(matching) != 1:
            raise ValueError(f'Expected one camera prior for {row["database_name"]}; found {len(matching)}')
        prior_id, _, _, _, position_blob, covariance_blob, system = matching[0]
        if system != 1:  # PosePriorCoordinateSystem.CARTESIAN
            raise ValueError(f'Prior {prior_id} is not CARTESIAN')
        if position_blob is None or len(position_blob) != 24 or covariance_blob is None or len(covariance_blob) != 72:
            raise ValueError(f'Invalid position/covariance blob for {row["database_name"]}')
        position = struct.unpack('<3d', position_blob)
        covariance = struct.unpack('<9d', covariance_blob)
        expected_cov = tuple(row['std'][i // 3]**2 if i // 3 == i % 3 else 0. for i in range(9))
        if not all(math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-10)
                   for a, b in zip((*position, *covariance), (*row['position'], *expected_cov))):
            raise ValueError(f'Position/covariance mismatch for {row["database_name"]}')
    return len(selected)


def import_priors(poses_csv, database, output_database=None, verify_only=False,
                  skip_missing=False, image_prefix=''):
    poses_csv, database = Path(poses_csv).resolve(), Path(database).resolve()
    rows = load_pose_rows(poses_csv)
    with closing(open_readonly(database)) as connection:
        images = database_inventory(connection)
        selected, skipped = select_rows(rows, images, image_prefix, skip_missing)
        if verify_only:
            if output_database is not None:
                raise ValueError('--verify-only takes the database to check, without --output-database')
            verified = verify_selected(connection, selected)
            return dict(status='verified', database=str(database), csv_images=len(rows),
                        database_images=len(images), verified=verified, skipped=skipped,
                        source_origin_priors=sum(row['position_kind'] == 'source_origin' for row in selected))
    if output_database is None:
        raise ValueError('--output-database is required for import; the input database is preserved')
    output = Path(output_database).resolve()
    report_path = output.with_name(output.name + '.pose_priors.json')
    if output == database or output.exists() or report_path.exists():
        raise FileExistsError('Output database/report must be new and different from the input database')
    try:
        import numpy as np
        import pycolmap
    except ImportError as error:
        raise ValueError('Import requires PyCOLMAP 4.2+ and numpy in this Python environment') from error
    if not hasattr(pycolmap.PosePrior(), 'corr_data_id'):
        raise ValueError('This importer requires the PyCOLMAP 4.2+ corr_data_id API')
    output.parent.mkdir(parents=True, exist_ok=True)
    written, updated = 0, 0
    with tempfile.TemporaryDirectory(prefix='.pose_import_', dir=output.parent) as temporary:
        working = Path(temporary) / 'database.db'
        # SQLite backup also includes committed WAL contents; file copying does not.
        source = open_readonly(database)
        destination = sqlite3.connect(working)
        try:
            source.backup(destination)
        finally:
            destination.close()
            source.close()
        with pycolmap.Database.open(working) as db:
            existing = {}
            for prior in db.read_all_pose_priors():
                data_id = prior.corr_data_id
                key = (data_id.sensor_id.type.value, data_id.sensor_id.id, data_id.id)
                existing.setdefault(key, []).append(prior)
            # Check ambiguity before any write. Existing gravity is retained on update.
            for row in selected:
                if len(existing.get((0, row['camera_id'], row['image_id']), [])) > 1:
                    raise ValueError(f'Multiple existing camera priors for {row["database_name"]}')
            with pycolmap.DatabaseTransaction(db):
                for row in selected:
                    matching = existing.get((0, row['camera_id'], row['image_id']), [])
                    prior = matching[0] if matching else pycolmap.PosePrior()
                    prior.corr_data_id = pycolmap.data_t(
                        pycolmap.sensor_t(pycolmap.SensorType.CAMERA, row['camera_id']), row['image_id'])
                    prior.position = np.array(row['position'])
                    prior.position_covariance = np.diag(np.square(row['std']))
                    prior.coordinate_system = pycolmap.PosePriorCoordinateSystem.CARTESIAN
                    if matching:
                        db.update_pose_prior(prior)
                        updated += 1
                    else:
                        db.write_pose_prior(prior)
                        written += 1
        checkpoint = sqlite3.connect(working)
        try:
            if checkpoint.execute('PRAGMA wal_checkpoint(TRUNCATE)').fetchone()[0] != 0:
                raise ValueError('Could not checkpoint the imported database')
        finally:
            checkpoint.close()
        connection = open_readonly(working)
        try:
            verified = verify_selected(connection, selected)
        finally:
            connection.close()
        # Publish only the verified copy, and never replace an existing file.
        os.link(working, output)
    report = dict(schema_version=1, status='complete', input_database=str(database),
                  output_database=str(output), poses_csv=str(poses_csv), poses_sha256=sha256_file(poses_csv),
                  pycolmap_version=pycolmap.__version__, world_frame=rows[0]['world_frame'],
                  coordinate_system='CARTESIAN', csv_images=len(rows), database_images=len(images),
                  inserted=written, updated=updated, verified=verified, skipped=skipped,
                  image_prefix=image_prefix,
                  source_origin_priors=sum(row['position_kind'] == 'source_origin' for row in selected),
                  orientation_imported=False,
                  uncertainty='CSV std_x/y/z_m squared into diagonal covariance in world coordinates (m^2)')
    write_json(report_path, report)
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('poses_csv', type=Path)
    parser.add_argument('database', type=Path, help='Existing feature database; or database to verify')
    parser.add_argument('--output-database', type=Path, help='New database copy for import (required unless verifying)')
    parser.add_argument('--verify-only', action='store_true', help='Read-only comparison of CSV and stored priors')
    parser.add_argument('--skip-missing', action='store_true', help='Skip failed CSV poses and absent DB images')
    parser.add_argument('--image-prefix', default='', help='Relative prefix when combining capture sessions in images/')
    try:
        report = import_priors(**vars(parser.parse_args(args)))
    except (OSError, ValueError, KeyError, RuntimeError, sqlite3.Error) as error:
        parser.exit(1, f'FAIL: {error}\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
