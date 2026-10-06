"""On-disk dataset contract shared by live recording and rosbag2 export."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import time
from threading import RLock

import cv2


FIELDS = ['index', 'filename', 'stamp_ns', 'stamp_sec', 'stamp_nanosec',
          'received_ns', 'width', 'height', 'bytes', 'sha256', 'frame_id', 'source_encoding']


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    os.replace(temporary, path)


class Dataset:
    def __init__(self, directory, save_fps=2.0, metadata=None):
        if not math.isfinite(save_fps) or save_fps <= 0:
            raise ValueError('save_fps must be finite and positive')
        self.path = Path(directory).expanduser().resolve()
        self.path.mkdir(parents=True, exist_ok=True)
        # Never overwrite a run; exclusive image directory also protects a live run.
        (self.path / 'images').mkdir(exist_ok=False)
        self.file = (self.path / 'timestamps.csv').open('x', newline='')
        self.csv = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.csv.writeheader()
        self.file.flush()
        self.lock = RLock()
        self.interval = max(1, round(1_000_000_000 / save_fps))
        self.last_seen = None
        self.last_selected = None
        self.last_saved = None
        self.shape = None
        self.stats = {'received': 0, 'saved': 0, 'thinned': 0, 'duplicates': 0}
        self.metadata = dict(metadata or {})
        self.metadata.update(schema_version=2, status='recording', save_fps=save_fps,
                             created_unix_ns=time.time_ns(),
                             stamp_semantics='ROS Image.header.stamp; not exposure hardware synchronization',
                             format='PNG; no resizing, cropping, undistortion or lossy re-encoding')
        self.closed = False
        self.update()

    def eligible(self, stamp):
        with self.lock:
            return self._eligible(stamp)

    def _eligible(self, stamp):
        stamp = int(stamp)
        self.stats['received'] += 1
        if stamp <= 0:
            raise ValueError('Image timestamp is zero/negative; refusing fabricated timestamps')
        if self.last_seen is not None and stamp < self.last_seen:
            raise ValueError('Image.header.stamp went backwards; stop and inspect camera/clock')
        if stamp == self.last_seen:
            self.stats['duplicates'] += 1
            return False
        self.last_seen = stamp
        # Source-time spacing; never duplicate frames to manufacture a constant rate.
        if self.last_selected is not None and stamp - self.last_selected < self.interval:
            self.stats['thinned'] += 1
            return False
        # Selection is independent of when the asynchronous disk writer finishes.
        self.last_selected = stamp
        return True

    def save(self, bgr_image, stamp, received_ns, frame_id='', source_encoding=''):
        shape = bgr_image.shape[:2]
        if self.shape is not None and shape != self.shape:
            raise ValueError(f'Image resolution changed: {self.shape} -> {shape}')
        self.shape = shape
        ok, encoded = cv2.imencode('.png', bgr_image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            raise RuntimeError('PNG encoding failed')
        data = encoded.tobytes()
        index = self.stats['saved'] + 1
        name = f'images/{index:08d}.png'
        target = self.path / name
        temporary = target.with_suffix('.png.tmp')
        with temporary.open('xb') as handle:
            handle.write(data)
        os.replace(temporary, target)
        self.csv.writerow(dict(index=index, filename=name, stamp_ns=stamp,
                               stamp_sec=stamp // 1_000_000_000,
                               stamp_nanosec=stamp % 1_000_000_000,
                               received_ns=received_ns, width=shape[1], height=shape[0],
                               bytes=len(data), sha256=hashlib.sha256(data).hexdigest(),
                               frame_id=frame_id, source_encoding=source_encoding))
        self.file.flush()
        with self.lock:
            self.stats['saved'] = index
            self.last_saved = stamp
        return name

    def update(self, **extra):
        with self.lock:
            self.metadata.update(extra)
            write_json(self.path / 'session.json', dict(self.metadata, counters=dict(self.stats)))

    def close(self, status='complete', error=None):
        if self.closed:
            return
        self.closed = True
        self.file.close()
        self.update(status=status, ended_unix_ns=time.time_ns(), error=error)
