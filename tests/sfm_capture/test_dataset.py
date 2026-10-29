"""Functional tests of the image dataset and offline ROS bag path (no camera)."""

import csv
import hashlib
import json
import sys

import cv2
import numpy as np
import pytest

from sfm_capture.check_dataset import check
from sfm_capture.dataset import Dataset


@pytest.fixture
def pixels():
    # Include all intensity values and distinct channels to catch lossy encoding
    # and channel-order mistakes, rather than testing a constant image only.
    return np.arange(16 * 32 * 3, dtype=np.uint16).reshape(16, 32, 3).astype(np.uint8)


def read_rows(directory):
    with (directory / 'timestamps.csv').open(newline='') as handle:
        return list(csv.DictReader(handle))


def write_single(directory, pixels, stamp=1_234_567_890):
    dataset = Dataset(directory)
    try:
        assert dataset.eligible(stamp)
        dataset.save(pixels, stamp, stamp + 12_345)
    finally:
        dataset.close()
    return directory


def test_png_roundtrip_manifest_and_completed_session(tmp_path, pixels):
    directory = write_single(tmp_path / 'run', pixels)
    path = directory / 'images/00000001.png'
    assert path.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
    np.testing.assert_array_equal(cv2.imread(str(path), cv2.IMREAD_UNCHANGED), pixels)
    row, = read_rows(directory)
    assert row['stamp_ns'] == '1234567890'
    assert row['stamp_sec'] == '1'
    assert row['stamp_nanosec'] == '234567890'
    assert row['received_ns'] == str(1_234_567_890 + 12_345)
    assert row['sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    metadata = json.loads((directory / 'session.json').read_text())
    assert metadata['status'] == 'complete'
    assert metadata['counters']['saved'] == 1
    assert check(directory) == dict(images=1, width=32, height=16, span_sec=0, average_fps=0)


def test_source_time_thinning_duplicates_and_filename_order(tmp_path, pixels):
    dataset = Dataset(tmp_path / 'run', save_fps=2.0)
    stamps = [1_000_000_000, 1_000_000_000, 1_499_999_999,
              1_500_000_000, 1_999_999_999, 2_000_000_000, 3_100_000_000]
    chosen = []
    try:
        for stamp in stamps:
            if dataset.eligible(stamp):
                chosen.append(stamp)
                dataset.save(pixels, stamp, stamp + 999)
    finally:
        dataset.close()
    assert chosen == [1_000_000_000, 1_500_000_000, 2_000_000_000, 3_100_000_000]
    assert dataset.stats == dict(received=7, saved=4, thinned=2, duplicates=1)
    rows = read_rows(dataset.path)
    assert [int(row['stamp_ns']) for row in rows] == chosen
    assert [row['filename'] for row in rows] == [f'images/{i:08d}.png' for i in range(1, 5)]
    assert check(dataset.path)['span_sec'] == 2.1


@pytest.mark.parametrize('stamp', [0, -1, -1_000_000_000])
def test_nonpositive_source_timestamp_is_rejected(tmp_path, stamp):
    dataset = Dataset(tmp_path / 'run')
    try:
        with pytest.raises(ValueError, match='zero/negative'):
            dataset.eligible(stamp)
    finally:
        dataset.close('failed', 'invalid timestamp')


def test_regression_is_detected_even_for_frames_that_would_be_thinned(tmp_path, pixels):
    dataset = Dataset(tmp_path / 'run')
    try:
        assert dataset.eligible(2_000_000_000)
        dataset.save(pixels, 2_000_000_000, 2_000_000_001)
        assert not dataset.eligible(2_100_000_000)
        with pytest.raises(ValueError, match='backwards'):
            dataset.eligible(2_050_000_000)
    finally:
        dataset.close('failed', 'clock moved backwards')
    with pytest.raises(ValueError, match='failed'):
        check(dataset.path)


@pytest.mark.parametrize('fps', [0, -1, float('inf'), float('-inf'), float('nan')])
def test_invalid_save_fps_does_not_create_output(tmp_path, fps):
    directory = tmp_path / 'run'
    with pytest.raises(ValueError, match='finite and positive'):
        Dataset(directory, save_fps=fps)
    assert not directory.exists()


def test_existing_dataset_is_not_overwritten(tmp_path, pixels):
    directory = write_single(tmp_path / 'run', pixels)
    before = {path.relative_to(directory): path.read_bytes()
              for path in directory.rglob('*') if path.is_file()}
    with pytest.raises(FileExistsError):
        Dataset(directory)
    after = {path.relative_to(directory): path.read_bytes()
             for path in directory.rglob('*') if path.is_file()}
    assert before == after
    assert check(directory)['images'] == 1


def test_camera_process_failure_rejects_finalized_partial_dataset(tmp_path, pixels):
    directory = write_single(tmp_path / 'run', pixels)
    (directory / 'capture_failure.json').write_text(
        json.dumps({'process': 'camera', 'returncode': 1, 'error': 'Camera disconnected'}))
    with pytest.raises(ValueError, match='Capture process failed'):
        check(directory)


def test_resolution_change_is_rejected_without_second_file(tmp_path, pixels):
    dataset = Dataset(tmp_path / 'run')
    try:
        assert dataset.eligible(1_000_000_000)
        dataset.save(pixels, 1_000_000_000, 1_000_000_001)
        assert dataset.eligible(2_000_000_000)
        with pytest.raises(ValueError, match='resolution changed'):
            dataset.save(pixels[:-1], 2_000_000_000, 2_000_000_001)
        assert dataset.stats['saved'] == 1
        assert not (dataset.path / 'images/00000002.png').exists()
    finally:
        dataset.close('failed', 'resolution changed')


def test_same_size_file_corruption_is_detected(tmp_path, pixels):
    directory = write_single(tmp_path / 'run', pixels)
    path = directory / 'images/00000001.png'
    damaged = bytearray(path.read_bytes())
    damaged[len(damaged) // 2] ^= 0x01
    path.write_bytes(damaged)
    with pytest.raises(ValueError, match='size/hash mismatch'):
        check(directory)


def test_extra_image_or_leftover_temporary_file_is_rejected(tmp_path, pixels):
    directory = write_single(tmp_path / 'run', pixels)
    (directory / 'images/00000002.png.tmp').write_bytes(b'incomplete')
    with pytest.raises(ValueError, match='Extra/missing'):
        check(directory)


def test_manifest_timestamp_tampering_is_detected(tmp_path, pixels):
    directory = write_single(tmp_path / 'run', pixels)
    path = directory / 'timestamps.csv'
    rows = read_rows(directory)
    rows[0]['stamp_nanosec'] = '1'
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    with pytest.raises(ValueError, match='Inconsistent timestamp fields'):
        check(directory)


def make_synthetic_bag(path, pixels, source_stamps, change_calibration=False):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from cv_bridge import CvBridge
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for name, message_type in [('/camera/image_raw', 'sensor_msgs/msg/Image'),
                               ('/camera/camera_info', 'sensor_msgs/msg/CameraInfo')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=message_type,
                                                    serialization_format='cdr'))
    camera = CameraInfo()
    camera.header.frame_id = 'camera_optical_frame'
    camera.width, camera.height = pixels.shape[1], pixels.shape[0]
    camera.k = [100.0, 0.0, 16.0, 0.0, 100.0, 8.0, 0.0, 0.0, 1.0]
    writer.write('/camera/camera_info', serialize_message(camera), 9_000_000_000)
    bridge = CvBridge()
    for index, stamp in enumerate(source_stamps):
        # An RGB source proves export honors encoding, including channel order.
        message = bridge.cv2_to_imgmsg(pixels[:, :, ::-1].copy(), encoding='rgb8')
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id = 'camera_optical_frame'
        writer.write('/camera/image_raw', serialize_message(message), 10_000_000_000 + index)
    if change_calibration:
        camera.k[0] = 101.0
        writer.write('/camera/camera_info', serialize_message(camera), 11_000_000_000)
    # Destruction closes sqlite and writes metadata before the reader opens it.
    del writer


def test_rosbag_export_preserves_pixels_source_time_and_record_time(tmp_path, pixels, monkeypatch):
    pytest.importorskip('rosbag2_py')
    from sfm_capture.export_bag import main

    bag, output = tmp_path / 'bag', tmp_path / 'export'
    make_synthetic_bag(bag, pixels,
                       [1_000_000_000, 1_100_000_000, 1_500_000_000, 2_100_000_000])
    monkeypatch.setattr(sys, 'argv', ['export_bag', str(bag), str(output), '--save-fps', '2'])
    main()
    assert check(output)['images'] == 3
    rows = read_rows(output)
    assert [int(row['stamp_ns']) for row in rows] == [1_000_000_000, 1_500_000_000, 2_100_000_000]
    assert [int(row['received_ns']) for row in rows] == [10_000_000_000, 10_000_000_002, 10_000_000_003]
    for row in rows:
        np.testing.assert_array_equal(cv2.imread(str(output / row['filename'])), pixels)
    metadata = json.loads((output / 'session.json').read_text())
    assert metadata['source'] == 'rosbag2'
    assert metadata['camera_calibrated'] is True
    camera = json.loads((output / 'camera_info.json').read_text())
    assert camera['width'] == 32 and camera['height'] == 16
    assert camera['k'][0] == 100.0


def test_rosbag_export_stops_on_source_time_regression(tmp_path, pixels, monkeypatch):
    pytest.importorskip('rosbag2_py')
    from sfm_capture.export_bag import main

    bag, output = tmp_path / 'bag', tmp_path / 'export'
    make_synthetic_bag(bag, pixels, [2_000_000_000, 1_000_000_000])
    monkeypatch.setattr(sys, 'argv', ['export_bag', str(bag), str(output)])
    with pytest.raises(ValueError, match='backwards'):
        main()
    metadata = json.loads((output / 'session.json').read_text())
    assert metadata['status'] == 'failed'
    assert metadata['counters']['saved'] == 1


def test_rosbag_export_rejects_calibration_change_instead_of_keeping_only_first(tmp_path, pixels, monkeypatch):
    pytest.importorskip('rosbag2_py')
    from sfm_capture.export_bag import main

    bag, output = tmp_path / 'bag', tmp_path / 'export'
    make_synthetic_bag(bag, pixels, [1_000_000_000], change_calibration=True)
    monkeypatch.setattr(sys, 'argv', ['export_bag', str(bag), str(output)])
    with pytest.raises(ValueError, match='calibration/frame_id changed'):
        main()
    assert json.loads((output / 'session.json').read_text())['status'] == 'failed'
    assert json.loads((output / 'camera_info.json').read_text())['k'][0] == 100.0
