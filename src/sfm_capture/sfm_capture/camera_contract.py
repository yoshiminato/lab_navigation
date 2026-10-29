"""Reject mixed image/calibration streams before sharing COLMAP intrinsics."""
import math
from copy import deepcopy


class CameraContract:
    def __init__(self):
        self.image_signature = None
        self.info = None

    def observe_image(self, width, height, encoding, frame_id):
        if width <= 0 or height <= 0:
            raise ValueError('Image dimensions must be positive')
        # Explicitly avoid silently reducing 16-bit/HDR images to bgr8.
        if encoding not in ('rgb8', 'bgr8', 'mono8'):
            raise ValueError(f'Unsupported source encoding {encoding}; expected an 8-bit decoded image')
        signature = (width, height, encoding, frame_id)
        if self.image_signature is not None and signature != self.image_signature:
            raise ValueError('Image resolution/encoding/frame_id changed; start a new session')
        self.image_signature = signature
        self._check_pair()

    def observe_info(self, info):
        # Acquisition timestamps naturally change; calibration and optical frame must not.
        signature = deepcopy({key: value for key, value in info.items() if key != 'header'})
        signature['frame_id'] = info['header']['frame_id']
        for key in ('d', 'k', 'r', 'p'):
            if not all(math.isfinite(float(value)) for value in signature[key]):
                raise ValueError('CameraInfo contains non-finite calibration values')
        if signature['k'][0] != 0 and (signature['k'][0] <= 0 or signature['k'][4] <= 0):
            raise ValueError('Calibrated CameraInfo must have positive fx and fy')
        if self.info is not None and signature != self.info:
            raise ValueError('CameraInfo calibration/frame_id changed; start a new session')
        self.info = signature
        self._check_pair()

    def _check_pair(self):
        if self.info is None or self.image_signature is None:
            return
        width, height, encoding, frame_id = self.image_signature
        if self.info['frame_id'] != frame_id:
            raise ValueError('Image and CameraInfo frame_id do not match')
        roi = self.info['roi']
        # CameraInfo dimensions describe calibration resolution. With binning/ROI
        # they need not equal the delivered image; leave that conversion explicit.
        full_frame = (self.info['binning_x'] in (0, 1) and
                      self.info['binning_y'] in (0, 1) and
                      all(roi[key] == 0 for key in ('x_offset', 'y_offset', 'width', 'height')))
        if full_frame and self.info['width'] and self.info['height']:
            if (self.info['width'], self.info['height']) != (width, height):
                raise ValueError('CameraInfo resolution does not match image; use matching calibration')
