"""Capture invariants, asynchronous backpressure, and auxiliary bag integrity."""
from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from sfm_capture.bag_support import inspect_bag, recording_topics
from sfm_capture.camera_contract import CameraContract
from sfm_capture.check_dataset import check
from sfm_capture.dataset import Dataset
from sfm_capture.image_writer import ImageWriter


def camera_info():
    return dict(header=dict(frame_id='camera', stamp=dict(sec=1, nanosec=0)),
        width=32, height=16, distortion_model='plumb_bob', d=[0.] * 5,
        k=[100., 0., 16., 0., 100., 8., 0., 0., 1.], r=[0.] * 9, p=[0.] * 12,
        binning_x=0, binning_y=0,
        roi=dict(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def test_camera_info_timestamp_changes_but_intrinsics_must_stay_fixed():
    contract = CameraContract()
    contract.observe_image(32, 16, 'rgb8', 'camera')
    info = camera_info()
    contract.observe_info(info)
    info['header']['stamp']['sec'] = 2
    contract.observe_info(info)
    info['k'][0] = 101.
    with pytest.raises(ValueError, match='calibration/frame_id changed'):
        contract.observe_info(info)


@pytest.mark.parametrize('signature', [(31, 16, 'rgb8', 'camera'),
    (32, 16, 'bgr8', 'camera'), (32, 16, 'rgb8', 'other')])
def test_mixed_image_stream_is_rejected_even_before_thinning(signature):
    contract = CameraContract()
    contract.observe_image(32, 16, 'rgb8', 'camera')
    with pytest.raises(ValueError, match='changed'):
        contract.observe_image(*signature)


@pytest.mark.parametrize('info_first', [True, False])
def test_wrong_calibration_resolution_is_rejected_in_either_callback_order(info_first):
    contract = CameraContract()
    first = lambda: contract.observe_info(camera_info())
    second = lambda: contract.observe_image(64, 32, 'rgb8', 'camera')
    if not info_first:
        first, second = second, first
    first()
    with pytest.raises(ValueError, match='resolution does not match'):
        second()


def test_calibration_frame_mismatch_and_nonfinite_values():
    contract = CameraContract()
    contract.observe_image(32, 16, 'rgb8', 'other')
    with pytest.raises(ValueError, match='frame_id do not match'):
        contract.observe_info(camera_info())
    info = camera_info()
    info['d'][0] = float('nan')
    with pytest.raises(ValueError, match='non-finite'):
        CameraContract().observe_info(info)


def test_16_bit_source_is_not_silently_reduced_to_8_bit():
    with pytest.raises(ValueError, match='Unsupported source encoding'):
        CameraContract().observe_image(32, 16, 'mono16', 'camera')


def test_selected_frames_keep_order_and_spacing_while_disk_is_busy(tmp_path):
    entered, release = Event(), Event()
    pixels = np.arange(32 * 16 * 3, dtype=np.uint8).reshape(16, 32, 3)
    message = SimpleNamespace(header=SimpleNamespace(frame_id='camera'), encoding='bgr8')

    def convert(message):
        entered.set()
        assert release.wait(5), 'test writer release timed out'
        return pixels

    dataset = Dataset(tmp_path / 'run', save_fps=2.)
    writer = ImageWriter(dataset, convert, queue_size=4)
    try:
        assert dataset.eligible(1_000_000_000)
        writer.submit(message, 1_000_000_000, 2_000_000_000)
        assert entered.wait(5)
        assert not dataset.eligible(1_100_000_000)
        for stamp in (1_500_000_000, 2_000_000_000):
            assert dataset.eligible(stamp)
            writer.submit(message, stamp, stamp + 1)
    finally:
        release.set()
        writer.close()
        dataset.close()
    assert check(dataset.path)['images'] == 3
    assert dataset.stats['thinned'] == 1
    assert dataset.last_saved == 2_000_000_000


def test_backpressure_fails_explicitly_instead_of_silently_dropping(tmp_path):
    entered, release = Event(), Event()
    pixels = np.zeros((16, 32, 3), np.uint8)
    message = SimpleNamespace(header=SimpleNamespace(frame_id='camera'), encoding='bgr8')

    def convert(message):
        entered.set()
        assert release.wait(5)
        return pixels

    dataset = Dataset(tmp_path / 'run')
    writer = ImageWriter(dataset, convert, queue_size=1)
    try:
        writer.submit(message, 1_000_000_000, 1)
        assert entered.wait(5)
        writer.submit(message, 2_000_000_000, 2)
        with pytest.raises(RuntimeError, match='queue is full'):
            writer.submit(message, 3_000_000_000, 3)
    finally:
        release.set()
        writer.close()
        dataset.close('failed', 'queue overflow')
    assert dataset.stats['saved'] == 2
    with pytest.raises(ValueError, match='failed'):
        check(dataset.path)


def test_disk_errors_are_propagated_on_close(tmp_path):
    dataset = Dataset(tmp_path / 'run')

    def convert(message):
        raise OSError('disk error')

    writer = ImageWriter(dataset, convert)
    writer.submit(None, 1, 1)
    with pytest.raises(RuntimeError, match='disk error'):
        writer.close()
    dataset.close('failed', str(writer.error))


def test_bag_defaults_keep_sensors_and_static_tf_without_raw_images():
    topics = recording_topics()
    assert '/camera/image_raw' not in topics
    assert {'/camera/camera_info', '/tf', '/tf_static', '/odom', '/imu/data',
            '/imu/data_raw', '/gps/fix'} == set(topics)
    assert '/camera/image_raw' in recording_topics(True)
    assert recording_topics(False, '/odom /odom').count('/odom') == 1
    with pytest.raises(ValueError, match='absolute ROS topic'):
        recording_topics(False, '--all')


def test_custom_camera_topics_and_simulation_clock_are_recorded():
    topics = recording_topics(True, '/odom /clock', '/front/image', '/front/info', True)
    assert {'/front/image', '/front/info', '/tf', '/tf_static', '/clock', '/odom'} == set(topics)
    assert topics.count('/clock') == 1
    assert '/camera/image_raw' not in topics
    assert '/camera/camera_info' not in topics
    with pytest.raises(ValueError, match='absolute ROS topic'):
        recording_topics(False, '', 'relative/image')


def test_bag_inventory_reports_missing_topics_and_rejects_missing_storage(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    storage = bag / 'bag_0.db3'
    storage.touch()
    metadata = dict(storage_identifier='sqlite3', relative_file_paths=[storage.name],
        message_count=4, topics_with_message_count=[dict(message_count=4,
            topic_metadata=dict(name='/imu/data', type='sensor_msgs/msg/Imu'))])
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=metadata)))
    report = inspect_bag(bag, ['/imu/data', '/tf_static'])
    assert report['missing_or_empty_topics'] == ['/tf_static']
    storage.unlink()
    with pytest.raises(ValueError, match='storage file is missing'):
        inspect_bag(bag)
