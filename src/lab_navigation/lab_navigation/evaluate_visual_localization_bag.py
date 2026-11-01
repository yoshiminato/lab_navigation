"""Compare recorded HLoc poses with timestamp-interpolated LiDAR poses."""

import argparse
import bisect
from collections import Counter, deque
import csv
import json
import math
from pathlib import Path
import statistics

import yaml

from ament_index_python.packages import get_package_share_directory
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def quaternion_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1e-12:
        raise ValueError('Encountered a zero-length quaternion')
    return tuple(value / norm for value in q)


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(q, vector):
    q = quaternion_normalize(q)
    vector_q = (vector[0], vector[1], vector[2], 0.0)
    inverse_q = (-q[0], -q[1], -q[2], q[3])
    rotated = quaternion_multiply(
        quaternion_multiply(q, vector_q), inverse_q
    )
    return rotated[:3]


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_values(pose):
    position = pose.position
    orientation = pose.orientation
    return (
        (float(position.x), float(position.y), float(position.z)),
        quaternion_normalize((
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )),
    )


def make_pose(position, orientation):
    return position, quaternion_normalize(orientation)


def slerp(q0, q1, ratio):
    q0 = quaternion_normalize(q0)
    q1 = quaternion_normalize(q1)
    dot = sum(a * b for a, b in zip(q0, q1))

    if dot < 0.0:
        q1 = tuple(-value for value in q1)
        dot = -dot

    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quaternion_normalize(tuple(
            a + ratio * (b - a) for a, b in zip(q0, q1)
        ))

    angle = math.acos(dot)
    sin_angle = math.sin(angle)
    weight0 = math.sin((1.0 - ratio) * angle) / sin_angle
    weight1 = math.sin(ratio * angle) / sin_angle
    return tuple(
        weight0 * a + weight1 * b for a, b in zip(q0, q1)
    )


def interpolate_pose(samples, target_ns, max_gap_ns):
    stamps = [sample[0] for sample in samples]
    index = bisect.bisect_left(stamps, target_ns)

    if index < len(samples) and stamps[index] == target_ns:
        return samples[index][1], 0.0

    if index == 0 or index >= len(samples):
        return None, None

    left_stamp, left_pose = samples[index - 1]
    right_stamp, right_pose = samples[index]
    gap = right_stamp - left_stamp
    if gap <= 0 or gap > max_gap_ns:
        return None, None

    ratio = (target_ns - left_stamp) / gap
    position = tuple(
        a + ratio * (b - a)
        for a, b in zip(left_pose[0], right_pose[0])
    )
    orientation = slerp(left_pose[1], right_pose[1], ratio)
    return make_pose(position, orientation), gap / 1_000_000_000.0


def pose_from_transform(transform):
    translation = transform.translation
    rotation = transform.rotation
    return make_pose(
        (float(translation.x), float(translation.y), float(translation.z)),
        (
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        ),
    )


def inverse_pose(pose):
    position, orientation = pose
    inverse_orientation = (
        -orientation[0], -orientation[1], -orientation[2], orientation[3]
    )
    inverse_position_rotated = rotate_vector(
        inverse_orientation, position
    )
    return (
        tuple(-value for value in inverse_position_rotated),
        inverse_orientation,
    )


def compose_pose(first, second):
    first_position, first_orientation = first
    second_position, second_orientation = second
    translated = rotate_vector(first_orientation, second_position)
    position = tuple(a + b for a, b in zip(first_position, translated))
    orientation = quaternion_multiply(first_orientation, second_orientation)
    return make_pose(position, orientation)


def lookup_transform(transforms, parent_frame, child_frame):
    """Compose a static TF path and return T_parent_child."""
    parent_frame = parent_frame.lstrip('/')
    child_frame = child_frame.lstrip('/')
    if parent_frame == child_frame:
        return make_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    graph = {}
    for parent, child, parent_from_child in transforms:
        graph.setdefault(parent, []).append((child, parent_from_child))
        graph.setdefault(child, []).append((parent, inverse_pose(parent_from_child)))

    identity = make_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    queue = deque([(parent_frame, identity)])
    visited = {parent_frame}

    while queue:
        frame, parent_from_frame = queue.popleft()
        for next_frame, frame_from_next in graph.get(frame, []):
            if next_frame in visited:
                continue
            parent_from_next = compose_pose(parent_from_frame, frame_from_next)
            if next_frame == child_frame:
                return parent_from_next
            visited.add(next_frame)
            queue.append((next_frame, parent_from_next))

    return None


def align_hloc_pose(pose, alignment):
    position, orientation = pose
    scale = float(alignment['scale'])
    translation = tuple(float(value) for value in alignment['translation_m'])
    rotation = quaternion_normalize(
        tuple(float(value) for value in alignment['rotation_xyzw'])
    )
    scaled_position = tuple(scale * value for value in position)
    aligned_position = rotate_vector(rotation, scaled_position)
    aligned_position = tuple(
        value + offset for value, offset in zip(aligned_position, translation)
    )
    aligned_orientation = quaternion_multiply(rotation, orientation)
    return make_pose(aligned_position, aligned_orientation)


def yaw_from_quaternion(q):
    x, y, z, w = quaternion_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values, percentage):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentage / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (index - lower) * (
        ordered[upper] - ordered[lower]
    )


def read_config(config_path):
    with config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    alignment = config['map_alignment']
    frames = config['frames']
    sync_gap = float(
        config['synchronization']['max_lidar_bracket_gap_sec']
    )
    if sync_gap <= 0.0:
        raise ValueError('max_lidar_bracket_gap_sec must be positive')

    return config, alignment, frames, sync_gap


def default_config_path():
    package_share = Path(get_package_share_directory('lab_navigation'))
    return package_share / 'params' / 'visual_localization_evaluation.yaml'


def read_bag(bag_path, frames):
    storage_options = StorageOptions(
        uri=str(bag_path), storage_id='sqlite3'
    )
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    expected_types = {
        '/pcl_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
        '/hloc/camera_pose': 'geometry_msgs/msg/PoseStamped',
        '/hloc/localization_status': 'std_msgs/msg/String',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
    }
    required_topics = {'/hloc/localization_status'}
    for topic, expected_type in expected_types.items():
        actual_type = topic_types.get(topic)
        if actual_type is None:
            if topic in required_topics:
                raise ValueError(
                    f'Required topic {topic} is missing from the bag'
                )
            continue
        if actual_type != expected_type:
            raise ValueError(
                f'{topic} has type {actual_type}; expected {expected_type}'
            )

    message_classes = {
        topic: get_message(message_type)
        for topic, message_type in topic_types.items()
        if topic in expected_types
    }

    lidar_samples = []
    hloc_poses = []
    statuses = []
    static_transforms = []

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in message_classes:
            continue

        message = deserialize_message(serialized, message_classes[topic])
        if topic == '/pcl_pose':
            frame_id = message.header.frame_id.lstrip('/')
            if frame_id != frames['lidar_map'].lstrip('/'):
                raise ValueError(
                    f'/pcl_pose frame is {frame_id}, expected '
                    f"{frames['lidar_map']}"
                )
            lidar_samples.append((
                stamp_to_ns(message.header.stamp),
                pose_values(message.pose.pose),
            ))
        elif topic == '/hloc/camera_pose':
            frame_id = message.header.frame_id.lstrip('/')
            if frame_id != frames['hloc_map'].lstrip('/'):
                raise ValueError(
                    f'/hloc/camera_pose frame is {frame_id}, expected '
                    f"{frames['hloc_map']}"
                )
            hloc_poses.append((
                stamp_to_ns(message.header.stamp),
                pose_values(message.pose),
            ))
        elif topic == '/hloc/localization_status':
            try:
                statuses.append(json.loads(message.data))
            except (TypeError, json.JSONDecodeError) as error:
                raise ValueError(
                    'Invalid JSON in /hloc/localization_status'
                ) from error
        elif topic == '/tf_static':
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip('/')
                child = transform.child_frame_id.lstrip('/')
                if not parent or not child:
                    continue
                static_transforms.append((
                    parent,
                    child,
                    pose_from_transform(transform.transform),
                ))

    if not statuses:
        raise ValueError(
            'The bag contains no HLoc localization status messages; '
            'the HLoc success rate cannot be calculated'
        )

    base_camera_transform = None
    if hloc_poses:
        if not lidar_samples:
            raise ValueError('The bag contains no LiDAR poses on /pcl_pose')
        base_camera_transform = lookup_transform(
            static_transforms,
            frames['base'],
            frames['camera'],
        )
        if base_camera_transform is None:
            raise ValueError(
                'Could not find a transform path in /tf_static from '
                f"{frames['base']} -> {frames['camera']}"
            )

    lidar_samples.sort(key=lambda item: item[0])
    hloc_poses.sort(key=lambda item: item[0])
    return lidar_samples, hloc_poses, statuses, base_camera_transform


def summarize(values):
    if not values:
        return {
            'mean': None,
            'rmse': None,
            'median': None,
            'p95': None,
            'max': None,
        }
    return {
        'mean': statistics.fmean(values),
        'rmse': math.sqrt(statistics.fmean(value * value for value in values)),
        'median': statistics.median(values),
        'p95': percentile(values, 95.0),
        'max': max(values),
    }


def validate_status_pose_timestamps(statuses, hloc_poses):
    successful_status_stamps = Counter()
    for status in statuses:
        if not isinstance(status, dict):
            raise ValueError(
                'Each /hloc/localization_status message must contain a JSON object'
            )

        success = status.get('success')
        if not isinstance(success, bool):
            raise ValueError(
                'Each HLoc status must contain a boolean "success" field'
            )
        if not success:
            continue

        try:
            stamp_sec = int(status['stamp_sec'])
            stamp_nanosec = int(status['stamp_nanosec'])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                'Successful HLoc status is missing a valid image timestamp'
            ) from error

        if not 0 <= stamp_nanosec < 1_000_000_000:
            raise ValueError(
                'Successful HLoc status has an invalid nanosecond timestamp'
            )
        successful_status_stamps[
            stamp_sec * 1_000_000_000 + stamp_nanosec
        ] += 1

    pose_stamps = Counter(stamp_ns for stamp_ns, _ in hloc_poses)
    missing_poses = successful_status_stamps - pose_stamps
    poses_without_success = pose_stamps - successful_status_stamps
    if missing_poses or poses_without_success:
        raise ValueError(
            'HLoc status/pose timestamp mismatch: '
            f'{sum(missing_poses.values())} successful status timestamp(s) '
            'have no matching camera pose; '
            f'{sum(poses_without_success.values())} camera pose timestamp(s) '
            'have no matching successful status'
        )

    return sum(successful_status_stamps.values())


def analyze(bag_path, output_dir, config_path):
    _, alignment, frames, max_gap_sec = read_config(config_path)
    lidar_samples, hloc_poses, statuses, base_camera = read_bag(
        bag_path, frames
    )
    successful = validate_status_pose_timestamps(statuses, hloc_poses)

    if hloc_poses:
        if not alignment.get('verified', False):
            raise ValueError(
                'Map alignment is not marked verified. Confirm the fixed '
                'SfM-to-LiDAR map transform using independent control points, '
                'then update map_alignment in the evaluation YAML.'
            )
        if float(alignment['scale']) <= 0.0:
            raise ValueError('map_alignment.scale must be positive')
        base_from_camera = inverse_pose(base_camera)
    else:
        base_from_camera = None

    max_gap_ns = int(max_gap_sec * 1_000_000_000)
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / 'pose_errors.csv'
    rows = []
    unmatched = 0

    for stamp_ns, raw_camera_pose in hloc_poses:
        lidar_pose, bracket_gap = interpolate_pose(
            lidar_samples, stamp_ns, max_gap_ns
        )
        if lidar_pose is None:
            unmatched += 1
            continue

        aligned_camera_pose = align_hloc_pose(raw_camera_pose, alignment)
        hloc_base_pose = compose_pose(aligned_camera_pose, base_from_camera)
        lidar_position, lidar_orientation = lidar_pose
        hloc_position, hloc_orientation = hloc_base_pose
        dx = hloc_position[0] - lidar_position[0]
        dy = hloc_position[1] - lidar_position[1]
        dz = hloc_position[2] - lidar_position[2]
        xy_error = math.hypot(dx, dy)
        xyz_error = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw_error = abs(wrapped_angle(
            yaw_from_quaternion(hloc_orientation)
            - yaw_from_quaternion(lidar_orientation)
        ))

        rows.append({
            'stamp_ns': stamp_ns,
            'bracket_gap_sec': bracket_gap,
            'lidar_x': lidar_position[0],
            'lidar_y': lidar_position[1],
            'lidar_z': lidar_position[2],
            'hloc_base_x': hloc_position[0],
            'hloc_base_y': hloc_position[1],
            'hloc_base_z': hloc_position[2],
            'dx': dx,
            'dy': dy,
            'dz': dz,
            'xy_error_m': xy_error,
            'xyz_error_m': xyz_error,
            'yaw_error_rad': yaw_error,
            'yaw_error_deg': math.degrees(yaw_error),
        })

    fieldnames = list(rows[0].keys()) if rows else [
        'stamp_ns', 'bracket_gap_sec', 'lidar_x', 'lidar_y', 'lidar_z',
        'hloc_base_x', 'hloc_base_y', 'hloc_base_z', 'dx', 'dy', 'dz',
        'xy_error_m', 'xyz_error_m', 'yaw_error_rad', 'yaw_error_deg',
    ]
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    processed = len(statuses)
    failed = processed - successful
    failure_reasons = Counter(
        status.get('reason', 'unspecified')
        for status in statuses
        if not status.get('success')
    )
    xy_errors = [row['xy_error_m'] for row in rows]
    xyz_errors = [row['xyz_error_m'] for row in rows]
    yaw_errors = [row['yaw_error_deg'] for row in rows]
    summary = {
        'bag': str(bag_path),
        'alignment_config': str(config_path),
        'reference': 'LiDAR Localization estimate (/pcl_pose)',
        'frames_processed_by_hloc': processed,
        'hloc_successes': successful,
        'hloc_failures': failed,
        'hloc_failure_reasons': dict(failure_reasons),
        'hloc_success_rate': successful / processed if processed else None,
        'successful_hloc_poses': len(hloc_poses),
        'poses_matched_to_lidar': len(rows),
        'matched_pose_rate': len(rows) / len(hloc_poses) if hloc_poses else None,
        'poses_without_lidar_bracket': unmatched,
        'max_lidar_bracket_gap_sec': max_gap_sec,
        'xy_error_m': summarize(xy_errors),
        'xyz_error_m': summarize(xyz_errors),
        'yaw_error_deg': summarize(yaw_errors),
        'csv': str(csv_path),
    }
    summary_path = output_dir / 'summary.json'
    with summary_path.open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
        stream.write('\n')

    print(json.dumps(summary, indent=2, ensure_ascii=False))


def main(args=None):
    parser = argparse.ArgumentParser(
        description=(
            'Compare HLoc camera localization with the LiDAR reference '
            'trajectory recorded by visual_localization_evaluation.launch.py.'
        )
    )
    parser.add_argument('bag', type=Path, help='Path to a ROS 2 bag directory')
    parser.add_argument(
        '--output-dir', type=Path,
        help='Directory for pose_errors.csv and summary.json (defaults to bag)',
    )
    parser.add_argument(
        '--config', type=Path,
        help='Evaluation YAML containing frame names and fixed map alignment',
    )
    parsed = parser.parse_args(args=args)
    bag_path = parsed.bag.expanduser().resolve()
    if not bag_path.is_dir():
        parser.error(f'Bag directory does not exist: {bag_path}')
    output_dir = (
        parsed.output_dir.expanduser().resolve()
        if parsed.output_dir else bag_path
    )
    config_path = (
        parsed.config.expanduser().resolve()
        if parsed.config else default_config_path()
    )

    try:
        analyze(bag_path, output_dir, config_path)
    except (KeyError, TypeError, ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
