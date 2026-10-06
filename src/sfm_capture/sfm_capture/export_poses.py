"""Extract one pose per saved image from timestamped rosbag2 TF (no live ROS)."""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import csv
import math
from pathlib import Path

from .pose_io import (POSE_FIELDS, finite_vector, relative_name, sha256_file,
                      unit_quaternion, write_json)


class TFConnectionError(ValueError):
    pass


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def compose_pose(transform, translation, quaternion):
    t, r = transform.transform.translation, transform.transform.rotation
    q = unit_quaternion((r.x, r.y, r.z, r.w))
    rotated = quaternion_product(quaternion_product(q, (*translation, 0.0)),
                                 (-q[0], -q[1], -q[2], q[3]))[:3]
    position = finite_vector((t.x + rotated[0], t.y + rotated[1], t.z + rotated[2]),
                             3, 'Camera position')
    return position, unit_quaternion(quaternion_product(q, quaternion))


def load_manifest(capture):
    with (capture / 'timestamps.csv').open(newline='') as handle:
        reader = csv.DictReader(handle)
        if not {'filename', 'stamp_ns', 'frame_id'}.issubset(reader.fieldnames or []):
            raise ValueError('timestamps.csv needs filename, stamp_ns, frame_id')
        rows = list(reader)
    if not rows:
        raise ValueError('timestamps.csv contains no images')
    previous = 0
    names = set()
    for row in rows:
        filename = relative_name(row['filename'])
        if not filename.startswith('images/'):
            raise ValueError(f'Image is not under images/: {filename}')
        row['image_name'] = relative_name(filename[len('images/'):])
        if row['image_name'] in names:
            raise ValueError(f'Duplicate image: {row["image_name"]}')
        names.add(row['image_name'])
        stamp = int(row['stamp_ns'])
        if stamp <= previous:
            raise ValueError('Image timestamps must be positive and strictly increasing')
        previous = stamp
    return rows


def load_tf(bag, tf_topic='/tf', static_topic='/tf_static', storage_id=None,
            world_frame='map', source_frame='base_link'):
    # Imports stay local so CSV/DB tools do not require a ROS installation.
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from tf2_msgs.msg import TFMessage
    from tf2_py import BufferCore
    import yaml

    if tf_topic == static_topic:
        raise ValueError('Dynamic and static TF topics must be different')
    if storage_id is None:
        metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())
        storage_id = metadata['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    selected = [topic for topic in (tf_topic, static_topic) if topic in topics]
    if not selected or any(topics[topic] != 'tf2_msgs/msg/TFMessage' for topic in selected):
        raise ValueError('Bag must contain TFMessage on the selected TF topics')
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    parents, static, dynamic = {}, {}, defaultdict(dict)
    issues = defaultdict(set)
    conflicting_stamps = defaultdict(set)
    message_counts = Counter()
    while reader.has_next():
        topic, data, _recorded_ns = reader.read_next()
        message_counts[topic] += 1
        for transform in deserialize_message(data, TFMessage).transforms:
            child, parent = transform.child_frame_id, transform.header.frame_id
            if not child or not parent or child == parent:
                issues[child].add('TF must have distinct, nonempty parent and child frames')
                continue
            if child in parents and parents[child] != parent:
                issues[child].add(f'TF parent changed for {child}; use a single frame tree/session')
            parents.setdefault(child, parent)
            t, q = transform.transform.translation, transform.transform.rotation
            try:
                finite_vector((t.x, t.y, t.z), 3, 'TF translation')
                unit_quaternion((q.x, q.y, q.z, q.w))
            except ValueError as error:
                issues[child].add(str(error))
                continue
            if abs(math.hypot(q.x, q.y, q.z, q.w) - 1.0) > 1e-3:
                issues[child].add(f'TF quaternion is not normalized: {parent} -> {child}')
                continue
            if topic == static_topic:
                if child in static and static[child].transform != transform.transform:
                    issues[child].add(f'Static TF changed for {child}')
                static[child] = transform
            else:
                stamp = transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec
                if stamp < 0:
                    issues[child].add('Dynamic TF timestamp must be nonnegative')
                    continue
                if stamp in dynamic[child] and dynamic[child][stamp].transform != transform.transform:
                    conflicting_stamps[child].add(stamp)
                dynamic[child][stamp] = transform
    del reader
    for child in set(static) & set(dynamic):
        issues[child].add('A TF child appears in both dynamic and static topics')
    try:
        used = set(path_children(world_frame, source_frame, parents))
    except TFConnectionError:
        used = set()  # Missing connections become one failed row per image.
    for child in sorted(used):
        if issues[child]:
            raise ValueError('; '.join(sorted(issues[child])))
    all_stamps = [stamp for child in used for stamp in dynamic.get(child, {})]
    span = max(all_stamps, default=0) - min(all_stamps, default=0)
    # The default 10 second buffer would discard the beginning of a long run.
    buffer = BufferCore(Duration(nanoseconds=max(10**10, span + 10**9)))
    for child, transform in static.items():
        if child in used:
            buffer.set_transform_static(transform, 'sfm_capture_offline')
    stamps_by_child = {}
    for child, transforms in dynamic.items():
        stamps_by_child[child] = sorted(transforms)
        if child in used:
            for stamp in stamps_by_child[child]:
                buffer.set_transform(transforms[stamp], 'sfm_capture_offline')
    inventory = dict(storage_id=storage_id, message_counts=dict(message_counts),
                     used_children=sorted(used),
                     conflicting_dynamic_stamps={c: sorted(values) for c, values in sorted(conflicting_stamps.items())},
                     ignored_tf_issues=[dict(child=c, reasons=sorted(errors))
                                        for c, errors in sorted(issues.items()) if errors and c not in used],
                     static_edges=[dict(parent=parents[c], child=c) for c in sorted(static)],
                     dynamic_edges=[dict(parent=parents[c], child=c, samples=len(stamps),
                                         first_stamp_ns=stamps[0], last_stamp_ns=stamps[-1])
                                    for c, stamps in sorted(stamps_by_child.items())])
    return buffer, parents, stamps_by_child, inventory


def path_children(target, source, parents):
    def ancestors(frame):
        frames = [frame]
        while frame in parents:
            frame = parents[frame]
            if frame in frames:
                raise ValueError(f'TF tree contains a cycle at {frame}')
            frames.append(frame)
        return frames
    target_path, source_path = ancestors(target), ancestors(source)
    common = next((frame for frame in source_path if frame in target_path), None)
    if common is None:
        raise TFConnectionError(f'No TF connection between {target} and {source}')
    return target_path[:target_path.index(common)] + source_path[:source_path.index(common)]


def interpolation_gap(children, stamps_by_child, stamp, conflicting_stamps=None):
    gap = 0
    for child in children:
        if child not in stamps_by_child:
            continue
        stamps = stamps_by_child[child]
        index = bisect_left(stamps, stamp)
        if index < len(stamps) and stamps[index] == stamp:
            if stamp in (conflicting_stamps or {}).get(child, ()):
                raise ValueError(f'Conflicting TF at timestamp {stamp} for {child}')
            continue
        if index == 0 or index == len(stamps):
            raise ValueError(f'TF timestamp is outside recorded range for {child}')
        if any(value in (conflicting_stamps or {}).get(child, ()) for value in (stamps[index - 1], stamps[index])):
            raise ValueError(f'Conflicting TF in interpolation bracket for {child}')
        gap = max(gap, stamps[index] - stamps[index - 1])
    return gap


def export_poses(capture, output, world_frame='map', source_frame='base_link', bag=None,
                 camera_translation=None, camera_quaternion=None, source_is_camera=False,
                 position_std=(1., 1., 1.), max_tf_gap_sec=0.2,
                 tf_topic='/tf', static_topic='/tf_static', storage_id=None):
    from rclpy.clock import ClockType
    from rclpy.time import Time
    from tf2_py import TransformException

    capture, output = Path(capture).resolve(), Path(output).resolve()
    bag = Path(bag).resolve() if bag is not None else capture / 'bag'
    if not world_frame or not source_frame:
        raise ValueError('world_frame and source_frame must not be empty')
    std = finite_vector(position_std, 3, 'Position standard deviation', positive=True)
    if not math.isfinite(max_tf_gap_sec) or max_tf_gap_sec <= 0:
        raise ValueError('max_tf_gap_sec must be finite and positive')
    has_extrinsic = camera_translation is not None or camera_quaternion is not None
    if source_is_camera and has_extrinsic:
        raise ValueError('Use source_is_camera or a camera extrinsic, not both')
    translation = finite_vector(camera_translation if camera_translation is not None else (0., 0., 0.),
                                 3, 'Camera translation in source frame')
    quaternion = unit_quaternion(camera_quaternion if camera_quaternion is not None else (0., 0., 0., 1.))
    rows = load_manifest(capture)
    if output.exists():
        raise FileExistsError(f'Output directory already exists: {output}')
    buffer, parents, stamps, inventory = load_tf(bag, tf_topic, static_topic, storage_id,
                                               world_frame, source_frame)
    conflicts = {child: set(values) for child, values in inventory['conflicting_dynamic_stamps'].items()}
    output.mkdir(parents=True, exist_ok=False)
    counts = Counter()
    max_valid_gap = 0
    kind = 'camera_center' if source_is_camera or has_extrinsic else 'source_origin'
    try:
        children = path_children(world_frame, source_frame, parents)
        connection_error = None
    except ValueError as error:
        children, connection_error = [], str(error)
    with (output / 'poses.csv').open('x', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=POSE_FIELDS)
        writer.writeheader()
        for index, image in enumerate(rows, 1):
            row = dict(index=index, filename=image['filename'], image_name=image['image_name'],
                       stamp_ns=image['stamp_ns'], image_frame=image['frame_id'],
                       world_frame=world_frame, source_frame=source_frame, position_kind=kind,
                       std_x_m=std[0], std_y_m=std[1], std_z_m=std[2], status='ok', error='')
            try:
                if connection_error:
                    raise ValueError(connection_error)
                stamp = int(image['stamp_ns'])
                gap = interpolation_gap(children, stamps, stamp, conflicts)
                row['max_tf_gap_ns'] = gap
                if gap > round(max_tf_gap_sec * 10**9):
                    row.update(status='gap_too_large', error=f'TF bracket gap {gap / 1e9:.6f}s exceeds limit')
                else:
                    transform = buffer.lookup_transform_core(
                        world_frame, source_frame, Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME))
                    position, rotation = compose_pose(transform, translation, quaternion)
                    row.update(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), (*position, *rotation)))
            except (TransformException, ValueError) as error:
                row.update(status='tf_unavailable', error=str(error))
            counts[row['status']] += 1
            if row['status'] == 'ok':
                max_valid_gap = max(max_valid_gap, row['max_tf_gap_ns'])
            writer.writerow(row)
    valid = counts['ok']
    report = dict(schema_version=1, status='complete' if valid == len(rows) else 'partial',
                  capture=str(capture), bag=str(bag), poses_csv=str(output / 'poses.csv'),
                  timestamps_sha256=sha256_file(capture / 'timestamps.csv'),
                  poses_sha256=sha256_file(output / 'poses.csv'), images=len(rows), valid=valid,
                  missing=len(rows) - valid, counts=dict(counts), world_frame=world_frame,
                  source_frame=source_frame, position_kind=kind,
                  source_from_camera=dict(translation=list(translation), quaternion_xyzw=list(quaternion)),
                  camera_orientation_provided=source_is_camera or camera_quaternion is not None,
                  position_std_m=list(std), position_std_source='user_configured; not estimated from TF',
                  max_tf_gap_sec=max_tf_gap_sec, max_valid_tf_gap_sec=max_valid_gap / 1e9,
                  tf_topics=[tf_topic, static_topic], tf=inventory,
                  pose_convention='world_from_source; with extrinsic: world_from_camera; quaternion xyzw',
                  timestamp_semantics='Image.header.stamp matched to TransformStamped.header.stamp')
    write_json(output / 'poses_report.json', report)
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path, help='Capture directory with timestamps.csv (image files are not read)')
    parser.add_argument('output', type=Path, help='New directory for poses.csv and poses_report.json')
    parser.add_argument('--bag', type=Path, help='Default: CAPTURE/bag')
    parser.add_argument('--world-frame', default='map')
    parser.add_argument('--source-frame', required=True)
    parser.add_argument('--source-is-camera', action='store_true', help='The source frame is the camera optical frame')
    parser.add_argument('--camera-translation', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--camera-quaternion', nargs=4, type=float, metavar=('QX', 'QY', 'QZ', 'QW'))
    parser.add_argument('--position-std', nargs=3, type=float, default=(1., 1., 1.), metavar=('SX', 'SY', 'SZ'))
    parser.add_argument('--max-tf-gap-sec', type=float, default=0.2)
    parser.add_argument('--tf-topic', default='/tf')
    parser.add_argument('--static-topic', default='/tf_static')
    parser.add_argument('--storage-id', help='Default: auto-detect from bag metadata')
    parser.add_argument('--allow-missing', action='store_true', help='Exit successfully with some missing poses')
    options = vars(parser.parse_args(args))
    allow_missing = options.pop('allow_missing')
    try:
        report = export_poses(**options)
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as error:
        parser.exit(1, f'FAIL: {error}\n')
    print(f"{report['poses_csv']}: {report['valid']}/{report['images']} valid; {report['missing']} missing; "
          f"position_kind={report['position_kind']}")
    if report['valid'] == 0 or (report['missing'] and not allow_missing):
        parser.exit(1, 'Pose failures are preserved in CSV/report; inspect them before importing.\n')


if __name__ == '__main__':
    main()
