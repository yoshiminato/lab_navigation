import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
from .bag_support import inspect_bag
from .camera_contract import CameraContract


def check(directory):
    directory = Path(directory)
    if (directory / 'capture_failure.json').exists():
        raise ValueError(f"Capture process failed: {(directory / 'capture_failure.json').read_text()}")
    metadata = json.loads((directory / 'session.json').read_text())
    if metadata['status'] != 'complete':
        raise ValueError(f"Session status is {metadata['status']}, not complete")
    with (directory / 'timestamps.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError('Empty dataset')
    last_stamp = 0
    dimensions = None
    names = set()
    contract = CameraContract()
    camera_info = directory / 'camera_info.json'
    if camera_info.exists():
        contract.observe_info(json.loads(camera_info.read_text()))
    for i, row in enumerate(rows, 1):
        expected = f'images/{i:08d}.png'
        if row['filename'] != expected or int(row['index']) != i:
            raise ValueError(f'Unexpected filename/index at row {i}')
        stamp = int(row['stamp_ns'])
        if stamp <= last_stamp:
            raise ValueError(f'Non-increasing timestamp at row {i}')
        if stamp != int(row['stamp_sec']) * 1_000_000_000 + int(row['stamp_nanosec']):
            raise ValueError(f'Inconsistent timestamp fields at row {i}')
        last_stamp = stamp
        path = directory / expected
        data = path.read_bytes()
        if len(data) != int(row['bytes']) or hashlib.sha256(data).hexdigest() != row['sha256']:
            raise ValueError(f'File size/hash mismatch: {path}')
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError(f'Cannot decode: {path}')
        height, width = image.shape[:2]
        if (width, height) != (int(row['width']), int(row['height'])):
            raise ValueError(f'Dimensions mismatch: {path}')
        if dimensions is not None and dimensions != (width, height):
            raise ValueError('Mixed resolutions')
        dimensions = (width, height)
        if row.get('source_encoding'):
            contract.observe_image(width, height, row['source_encoding'], row['frame_id'])
        names.add(path.name)
    actual = {p.name for p in (directory / 'images').iterdir()}
    if actual != names:
        raise ValueError('Extra/missing image or unfinished temporary file')
    if metadata['counters']['saved'] != len(rows):
        raise ValueError('Session count does not match manifest')
    span = (last_stamp - int(rows[0]['stamp_ns'])) / 1e9
    result = dict(images=len(rows), width=dimensions[0], height=dimensions[1],
                  span_sec=span, average_fps=(len(rows)-1)/span if span > 0 else 0)
    setup_path = directory / 'capture_setup.json'
    if setup_path.exists():
        setup = json.loads(setup_path.read_text())
        if str(setup.get('record_bag', False)).lower() == 'true':
            report_path = directory / 'bag_report.json'
            if report_path.exists() and json.loads(report_path.read_text())['status'] == 'failed':
                raise ValueError(f'Bag recording failed: {report_path.read_text()}')
            # Verify independently of the launch event handler, including after
            # interrupted shutdowns where bag_report.json may not be written.
            result['bag'] = inspect_bag(directory / 'bag', setup.get('bag_topics', []))
    return result


def main():
    parser = argparse.ArgumentParser(description='Check images, checksums, chronology and resolution')
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(check(args.directory), indent=2))
    except (ValueError, OSError, KeyError) as error:
        parser.exit(1, f'FAIL: {error}\n')
