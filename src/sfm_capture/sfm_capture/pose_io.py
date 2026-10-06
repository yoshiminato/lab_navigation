"""ROS-independent contract for timestamped poses and COLMAP image names."""
import csv
import hashlib
import json
import math
from pathlib import Path, PurePosixPath


POSE_FIELDS = ['index', 'filename', 'image_name', 'stamp_ns', 'image_frame',
               'world_frame', 'source_frame', 'position_kind', 'status', 'error',
               'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
               'std_x_m', 'std_y_m', 'std_z_m', 'max_tf_gap_ns']


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def relative_name(name):
    if (not name or name.startswith('/') or '\\' in name or
            any(part in ('', '.', '..') for part in name.split('/'))):
        raise ValueError(f'Expected a relative image path without traversal: {name!r}')
    return PurePosixPath(name).as_posix()


def finite_vector(values, count, label, positive=False):
    values = tuple(float(value) for value in values)
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f'{label} must contain {count} finite numbers')
    if positive and not all(value > 0 and 0 < value * value < math.inf for value in values):
        raise ValueError(f'{label} must be positive with finite squared values')
    return values


def unit_quaternion(values):
    values = finite_vector(values, 4, 'Quaternion (x,y,z,w)')
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Quaternion must have nonzero norm')
    return tuple(value / norm for value in values)


def load_pose_rows(path):
    with Path(path).open(newline='') as handle:
        reader = csv.DictReader(handle)
        required = set(POSE_FIELDS) - {'index', 'filename', 'image_frame', 'error', 'max_tf_gap_ns'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f'Pose CSV is missing fields: {sorted(required - set(reader.fieldnames or []))}')
        rows = list(reader)
    if not rows:
        raise ValueError('Pose CSV contains no images')
    names = set()
    worlds = set()
    for row in rows:
        row['image_name'] = relative_name(row['image_name'])
        if row['image_name'] in names:
            raise ValueError(f'Duplicate image_name: {row["image_name"]}')
        names.add(row['image_name'])
        if not row['world_frame'] or not row['source_frame']:
            raise ValueError('Pose frames must not be empty')
        worlds.add(row['world_frame'])
        if int(row['stamp_ns']) <= 0:
            raise ValueError('Image stamp_ns must be positive')
        if row['position_kind'] not in ('source_origin', 'camera_center'):
            raise ValueError(f'Unknown position_kind: {row["position_kind"]}')
        if row['status'] not in ('ok', 'tf_unavailable', 'gap_too_large'):
            raise ValueError(f'Unknown pose status: {row["status"]}')
        if row['status'] == 'ok':
            row['position'] = finite_vector([row[key] for key in ('x', 'y', 'z')], 3, 'Position')
            unit_quaternion([row[key] for key in ('qx', 'qy', 'qz', 'qw')])
            row['std'] = finite_vector([row[key] for key in ('std_x_m', 'std_y_m', 'std_z_m')],
                                       3, 'Position standard deviation', positive=True)
    if len(worlds) != 1:
        raise ValueError('Pose CSV mixes world frames; transform them to one frame first')
    return rows
