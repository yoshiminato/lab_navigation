"""Time interpolation, camera offsets, image identity, and actual COLMAP DB writes."""
import csv
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import pytest

from sfm_capture.export_poses import export_poses, main as export_main
from sfm_capture.import_pose_priors import import_priors
from sfm_capture.pose_io import POSE_FIELDS, sha256_file


def read_rows(path):
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def write_rows(path, rows):
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=POSE_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def make_capture(root, stamps):
    (root / 'images').mkdir(parents=True)
    with (root / 'timestamps.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=['filename', 'stamp_ns', 'frame_id'])
        writer.writeheader()
        for index, stamp in enumerate(stamps, 1):
            filename = f'images/{index:08d}.png'
            (root / filename).write_bytes(b'placeholder: pose extraction does not decode images')
            writer.writerow(dict(filename=filename, stamp_ns=stamp, frame_id='camera_optical_frame'))
    return root


def transform(parent, child, seconds, xyz=(0., 0., 0.), yaw=0.):
    pytest.importorskip('geometry_msgs')
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec, t.header.stamp.nanosec = divmod(round(seconds * 1e9), 10**9)
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


def make_tf_bag(path, messages):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from tf2_msgs.msg import TFMessage
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for topic in ('/tf', '/tf_static'):
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type='tf2_msgs/msg/TFMessage',
                                                    serialization_format='cdr'))
    for index, (topic, t) in enumerate(messages):
        # Receipt times deliberately differ from both image and TF source times.
        writer.write(topic, serialize_message(TFMessage(transforms=[t])), 100 * 10**9 + index)
    del writer


def test_source_time_interpolation_and_rotated_camera_offset(tmp_path):
    capture = make_capture(tmp_path / 'capture', [2 * 10**9])
    make_tf_bag(capture / 'bag', [
        ('/tf', transform('map', 'base_link', 3, (2., 0., 0.), math.pi / 2)),
        ('/tf_static', transform('base_link', 'camera_optical_frame', 0, (1., 0., 0.))),
        ('/tf', transform('map', 'base_link', 1)),  # Late/out-of-order TF publication.
    ])
    output = tmp_path / 'poses'
    report = export_poses(capture, output, camera_translation=(1., 0., 0.),
                          camera_quaternion=(0., 0., 1., 0.), max_tf_gap_sec=3.)
    assert report['valid'] == report['images'] == 1
    row, = read_rows(output / 'poses.csv')
    np.testing.assert_allclose([float(row[k]) for k in ('x', 'y', 'z')],
                               [1 + math.sqrt(.5), math.sqrt(.5), 0.], atol=1e-12)
    np.testing.assert_allclose([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')],
                               [0., 0., math.cos(math.pi / 8), -math.sin(math.pi / 8)], atol=1e-12)
    assert row['image_name'] == '00000001.png'
    assert row['position_kind'] == 'camera_center'
    assert row['max_tf_gap_ns'] == str(2 * 10**9)
    direct = tmp_path / 'direct'
    export_poses(capture, direct, source_frame='camera_optical_frame', source_is_camera=True,
                 max_tf_gap_sec=3.)
    other, = read_rows(direct / 'poses.csv')
    np.testing.assert_allclose([float(other[k]) for k in ('x', 'y', 'z')],
                               [float(row[k]) for k in ('x', 'y', 'z')])


def test_long_session_retains_early_tf_and_source_origin_label(tmp_path):
    capture = make_capture(tmp_path / 'capture', [10**9, 61 * 10**9])
    make_tf_bag(capture / 'bag', [('/tf', transform('map', 'base_link', 1, (1., 2., 3.))),
                                ('/tf', transform('map', 'base_link', 61, (4., 5., 6.)))])
    # Image transfer/deletion does not prevent independent CSV + TF processing.
    for path in (capture / 'images').iterdir():
        path.unlink()
    report = export_poses(capture, tmp_path / 'poses')
    assert report['valid'] == 2
    assert report['position_kind'] == 'source_origin'
    assert report['position_std_source'].startswith('user_configured')


def test_missing_and_large_gap_keep_one_row_per_image_and_nonzero_cli_exit(tmp_path):
    capture = make_capture(tmp_path / 'capture', [10**9, 2 * 10**9, 4 * 10**9])
    make_tf_bag(capture / 'bag', [('/tf', transform('map', 'base_link', 1)),
                                ('/tf', transform('map', 'base_link', 3))])
    output = tmp_path / 'poses'
    with pytest.raises(SystemExit) as error:
        export_main([str(capture), str(output), '--source-frame', 'base_link'])
    assert error.value.code == 1
    rows = read_rows(output / 'poses.csv')
    assert [row['status'] for row in rows] == ['ok', 'gap_too_large', 'tf_unavailable']
    assert rows[1]['x'] == rows[2]['x'] == ''
    assert json.loads((output / 'poses_report.json').read_text())['missing'] == 2
    missing = export_poses(capture, tmp_path / 'missing', source_frame='not_a_frame')
    assert missing['valid'] == 0 and missing['images'] == 3


@pytest.mark.parametrize('messages,reason', [
    ([('/tf', ('map', 'base_link', 1)), ('/tf', ('odom', 'base_link', 2))], 'parent changed'),
    ([('/tf', ('map', 'base_link', 1)), ('/tf_static', ('map', 'base_link', 0))], 'both dynamic'),
])
def test_ambiguous_frame_tree_is_rejected(tmp_path, messages, reason):
    capture = make_capture(tmp_path / 'capture', [10**9])
    make_tf_bag(capture / 'bag', [(topic, transform(*args)) for topic, args in messages])
    with pytest.raises(ValueError, match=reason):
        export_poses(capture, tmp_path / 'poses')
    assert not (tmp_path / 'poses').exists()


def test_unrelated_static_conflict_is_reported_without_changing_selected_pose(tmp_path):
    capture = make_capture(tmp_path / 'capture', [10**9])
    make_tf_bag(capture / 'bag', [
        ('/tf', transform('map', 'base_link', 1, (1., 2., 3.))),
        ('/tf_static', transform('base_link', 'lidar', 0, (0., 0., .1))),
        ('/tf_static', transform('base_link', 'lidar', 0, (0., 0., .2))),
    ])
    report = export_poses(capture, tmp_path / 'poses')
    assert report['valid'] == 1
    assert report['tf']['ignored_tf_issues'] == [dict(child='lidar', reasons=['Static TF changed for lidar'])]
    with pytest.raises(ValueError, match='Static TF changed'):
        export_poses(capture, tmp_path / 'bad', source_frame='lidar')


def test_conflicting_dynamic_tf_only_invalidates_images_using_that_sample(tmp_path):
    capture = make_capture(tmp_path / 'capture', [10**9, 2 * 10**9, 3 * 10**9, 5 * 10**9])
    make_tf_bag(capture / 'bag', [
        ('/tf', transform('map', 'base_link', 1)),
        ('/tf', transform('map', 'base_link', 1, (1., 0., 0.))),
        ('/tf', transform('map', 'base_link', 3)),
        ('/tf', transform('map', 'base_link', 6)),
    ])
    report = export_poses(capture, tmp_path / 'poses', max_tf_gap_sec=4.)
    rows = read_rows(tmp_path / 'poses/poses.csv')
    assert [row['status'] for row in rows] == ['tf_unavailable', 'tf_unavailable', 'ok', 'ok']
    assert report['tf']['conflicting_dynamic_stamps'] == {'base_link': [10**9]}
    assert report['valid'] == 2


def pose_csv(path, names=('session/first.png', 'session/second.png')):
    rows = []
    for index, name in enumerate(names, 1):
        rows.append(dict(index=index, filename=f'images/{name}', image_name=name,
                         stamp_ns=index * 10**9, image_frame='camera', world_frame='map',
                         source_frame='base_link', position_kind='camera_center', status='ok', error='',
                         x=index * 1.25, y=-2., z=.5, qx=0., qy=0., qz=0., qw=1.,
                         std_x_m=.3, std_y_m=.4, std_z_m=1.2, max_tf_gap_ns=0))
    write_rows(path, rows)
    return rows


def feature_db(path):
    pycolmap = pytest.importorskip('pycolmap')
    if not hasattr(pycolmap.PosePrior(), 'corr_data_id'):
        pytest.skip('Requires PyCOLMAP 4.2+')
    with pycolmap.Database.open(path) as db:
        camera_id = db.write_camera(pycolmap.Camera(model='PINHOLE', width=32, height=16,
                                                    params=[100., 100., 16., 8.]))
        # IDs and insertion order deliberately differ from CSV index/order.
        for name, image_id in [('session/second.png', 42), ('session/first.png', 17)]:
            db.write_image(pycolmap.Image(name=name, camera_id=camera_id, image_id=image_id), use_image_id=True)
            db.write_keypoints(image_id, np.array([[2., 3.], [4., 5.]], dtype=np.float32))
        existing = pycolmap.PosePrior(
            corr_data_id=pycolmap.data_t(pycolmap.sensor_t(pycolmap.SensorType.CAMERA, camera_id), 17),
            gravity=np.array([0., 1., 0.]))
        db.write_pose_prior(existing)
    return pycolmap


def test_real_pycolmap_insert_update_preserve_features_and_readonly_verify(tmp_path):
    source, output = tmp_path / 'features.db', tmp_path / 'database.db'
    pycolmap = feature_db(source)
    poses = tmp_path / 'poses.csv'
    pose_csv(poses)
    before = sha256_file(source)
    report = import_priors(poses, source, output)
    assert report['inserted'] == report['updated'] == 1
    assert report['verified'] == 2
    assert sha256_file(source) == before
    with pycolmap.Database.open(output) as db:
        for prior in db.read_all_pose_priors():
            if prior.corr_data_id.id == 17:
                np.testing.assert_allclose(prior.position, [1.25, -2., .5])
                np.testing.assert_allclose(prior.gravity, [0., 1., 0.])
            else:
                assert prior.corr_data_id.id == 42
                np.testing.assert_allclose(prior.position, [2.5, -2., .5])
            np.testing.assert_allclose(prior.position_covariance, np.diag([.09, .16, 1.44]))
        np.testing.assert_array_equal(db.read_keypoints(17)[:, :2], [[2., 3.], [4., 5.]])
    output_before = sha256_file(output)
    assert import_priors(poses, output, verify_only=True)['verified'] == 2
    assert sha256_file(output) == output_before
    assert json.loads(Path(str(output) + '.pose_priors.json').read_text())['verified'] == 2
    rows = read_rows(poses)
    rows[0]['x'] = '99'
    write_rows(poses, rows)
    with pytest.raises(ValueError, match='mismatch'):
        import_priors(poses, output, verify_only=True)
    assert sha256_file(output) == output_before


def test_missing_database_images_and_failed_poses_are_explicit(tmp_path):
    source = tmp_path / 'features.db'
    feature_db(source)
    poses = tmp_path / 'poses.csv'
    rows = pose_csv(poses, ('session/first.png', 'session/absent.png', 'session/second.png'))
    rows[-1]['status'] = 'tf_unavailable'
    for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'):
        rows[-1][key] = ''
    write_rows(poses, rows)
    output = tmp_path / 'database.db'
    with pytest.raises(ValueError, match='2 images cannot'):
        import_priors(poses, source, output)
    assert not output.exists()
    report = import_priors(poses, source, output, skip_missing=True)
    assert report['verified'] == 1
    assert [row['reason'] for row in report['skipped']] == ['image_not_in_database', 'tf_unavailable']


@pytest.mark.parametrize('key,value,reason', [
    ('image_name', '../first.png', 'traversal'), ('x', 'nan', 'finite'),
    ('std_x_m', '0', 'positive'), ('std_x_m', '1e-300', 'positive'),
    ('qw', '0', 'Quaternion'), ('world_frame', 'odom', 'mixes world'),
])
def test_invalid_csv_fails_before_output_or_source_changes(tmp_path, key, value, reason):
    source = tmp_path / 'features.db'
    feature_db(source)
    poses = tmp_path / 'poses.csv'
    rows = pose_csv(poses)
    rows[0][key] = value
    write_rows(poses, rows)
    before = sha256_file(source)
    with pytest.raises(ValueError, match=reason):
        import_priors(poses, source, tmp_path / 'output.db')
    assert not (tmp_path / 'output.db').exists()
    assert sha256_file(source) == before


def test_image_prefix_exact_name_matching_and_existing_output_guard(tmp_path):
    source = tmp_path / 'features.db'
    feature_db(source)
    poses = tmp_path / 'poses.csv'
    pose_csv(poses, ('first.png', 'second.png'))
    output = tmp_path / 'output.db'
    assert import_priors(poses, source, output, image_prefix='session')['verified'] == 2
    before = sha256_file(output)
    with pytest.raises(FileExistsError):
        import_priors(poses, source, output, image_prefix='session')
    assert sha256_file(output) == before


def test_db_failure_does_not_publish_partial_database(tmp_path):
    source = tmp_path / 'features.db'
    feature_db(source)
    connection = sqlite3.connect(source)
    connection.execute("CREATE TRIGGER reject_prior BEFORE INSERT ON pose_priors "
                       "BEGIN SELECT RAISE(ABORT, 'injected write failure'); END")
    connection.commit()
    connection.close()
    poses = tmp_path / 'poses.csv'
    pose_csv(poses)
    before = sha256_file(source)
    with pytest.raises(RuntimeError, match='constraint failed'):
        import_priors(poses, source, tmp_path / 'output.db')
    assert sha256_file(source) == before
    assert not (tmp_path / 'output.db').exists()
