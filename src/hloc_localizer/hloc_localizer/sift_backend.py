#!/usr/bin/env python3

from collections import defaultdict, OrderedDict
from pathlib import Path

import cv2
import h5py
import numpy as np
import torch

from hloc import extract_features
from hloc import extractors
from hloc import match_features
from hloc import matchers
from hloc.utils.base_model import dynamic_load


class SiftBackend:

    method_name = 'sift'
    timing_name = 'SIFT + NN-ratio'

    def __init__(
        self,
        data_path,
        device,
        cache_size=30,
        logger=None,
    ):
        self.data_path = Path(data_path)
        self.device = device
        self.cache_size = cache_size
        self.logger = logger

        # ----------------------------------------------------
        # Paths
        # ----------------------------------------------------

        self.sfm_path = (
            self.data_path /
            'sparse' /
            '0'
        )

        self.hloc_dir = (
            self.data_path /
            'hloc_output'
        )

        self.reference_local_features = (
            self.hloc_dir /
            'feats-shift.h5'
        )

        if not self.sfm_path.exists():
            raise FileNotFoundError(
                str(self.sfm_path)
            )

        if not self.reference_local_features.exists():
            raise FileNotFoundError(
                str(self.reference_local_features)
            )

        # ----------------------------------------------------
        # SIFT
        # ----------------------------------------------------

        if self.logger:
            self.logger.info(
                'Loading SIFT extractor...'
            )

        self.local_conf = (
            extract_features
            .confs['sift']
        )

        Model = dynamic_load(
            extractors,
            self.local_conf[
                'model'
            ]['name'],
        )

        self.local_model = (
            Model(
                self.local_conf[
                    'model'
                ]
            )
            .eval()
            .to(self.device)
        )

        # ----------------------------------------------------
        # NN-ratio
        # ----------------------------------------------------

        if self.logger:
            self.logger.info(
                'Loading NN-ratio matcher...'
            )

        matcher_conf = (
            match_features
            .confs['NN-ratio']
        )

        Matcher = dynamic_load(
            matchers,
            matcher_conf[
                'model'
            ]['name'],
        )

        self.matcher = (
            Matcher(
                matcher_conf[
                    'model'
                ]
            )
            .eval()
            .to(self.device)
        )

        # ----------------------------------------------------
        # Reference feature file
        # ----------------------------------------------------

        self.ref_local_h5 = h5py.File(
            str(
                self.reference_local_features
            ),
            'r',
            libver='latest',
        )

        self.descriptor_cache = (
            OrderedDict()
        )

    # ========================================================
    # Resize
    # ========================================================

    @staticmethod
    def _resize_max(
        image,
        resize_max,
    ):
        height, width = image.shape[:2]

        if (
            resize_max is None
            or max(width, height)
            <= resize_max
        ):
            return image

        scale = (
            resize_max /
            max(width, height)
        )

        new_width = int(
            round(width * scale)
        )

        new_height = int(
            round(height * scale)
        )

        return cv2.resize(
            image,
            (
                new_width,
                new_height,
            ),
            interpolation=cv2.INTER_AREA,
        )

    # ========================================================
    # Query feature extraction
    # ========================================================

    def extract(
        self,
        image_bgr,
    ):
        original_height, original_width = (
            image_bgr.shape[:2]
        )

        gray = cv2.cvtColor(
            image_bgr,
            cv2.COLOR_BGR2GRAY,
        )

        resized = self._resize_max(
            gray,
            self.local_conf[
                'preprocessing'
            ].get(
                'resize_max'
            ),
        )

        resized_height, resized_width = (
            resized.shape[:2]
        )

        tensor = (
            torch.from_numpy(
                resized.astype(
                    np.float32
                )
            )
            .unsqueeze(0)
            .unsqueeze(0)
            / 255.0
        )

        tensor = tensor.to(
            self.device
        )

        with torch.no_grad():
            prediction = (
                self.local_model(
                    {
                        'image': tensor
                    }
                )
            )

        keypoints = (
            prediction[
                'keypoints'
            ][0]
            .detach()
            .cpu()
            .numpy()
            .astype(np.float32)
        )

        descriptors = (
            prediction[
                'descriptors'
            ]
            .detach()
            .float()
            .to(self.device)
        )

        original_size = np.array(
            [
                original_width,
                original_height,
            ],
            dtype=np.float32,
        )

        processed_size = np.array(
            [
                resized_width,
                resized_height,
            ],
            dtype=np.float32,
        )

        scales = (
            original_size /
            processed_size
        )

        keypoints = (
            (keypoints + 0.5)
            * scales[None]
            - 0.5
        )

        return {
            'keypoints':
            keypoints,

            'descriptors':
            descriptors,

            'image_size':
            (
                original_width,
                original_height,
            ),
        }

    # ========================================================
    # Reference descriptor cache
    # ========================================================

    def _get_reference_descriptors(
        self,
        name,
    ):
        if name in self.descriptor_cache:

            value = (
                self.descriptor_cache.pop(
                    name
                )
            )

            self.descriptor_cache[
                name
            ] = value

            return value

        descriptors_np = (
            self.ref_local_h5[
                name
            ]['descriptors'][()]
        )

        descriptors = (
            torch.from_numpy(
                np.asarray(
                    descriptors_np
                )
            )
            .float()
            .unsqueeze(0)
            .to(self.device)
        )

        self.descriptor_cache[
            name
        ] = descriptors

        while (
            len(
                self.descriptor_cache
            )
            > self.cache_size
        ):
            self.descriptor_cache.popitem(
                last=False
            )

        return descriptors

    # ========================================================
    # Matching + 2D -> 3D
    # ========================================================

    def build_correspondences(
        self,
        query_features,
        reference_names,
        ref_point3d_ids,
    ):
        query_descriptors = (
            query_features[
                'descriptors'
            ]
        )

        correspondences = (
            defaultdict(set)
        )

        total_matches = 0

        for ref_name in reference_names:

            if ref_name not in ref_point3d_ids:
                continue

            if ref_name not in self.ref_local_h5:
                continue

            reference_descriptors = (
                self._get_reference_descriptors(
                    ref_name
                )
            )

            with torch.no_grad():
                prediction = (
                    self.matcher(
                        {
                            'descriptors0':
                            query_descriptors,

                            'descriptors1':
                            reference_descriptors,
                        }
                    )
                )

            matches0 = (
                prediction[
                    'matches0'
                ][0]
                .detach()
                .cpu()
                .numpy()
                .astype(np.int64)
            )

            valid_query_indices = (
                np.flatnonzero(
                    matches0 >= 0
                )
            )

            if len(
                valid_query_indices
            ) == 0:
                continue

            reference_indices = (
                matches0[
                    valid_query_indices
                ]
            )

            point3d_table = (
                ref_point3d_ids[
                    ref_name
                ]
            )

            valid_bounds = (
                reference_indices
                < len(point3d_table)
            )

            valid_query_indices = (
                valid_query_indices[
                    valid_bounds
                ]
            )

            reference_indices = (
                reference_indices[
                    valid_bounds
                ]
            )

            point3d_ids = (
                point3d_table[
                    reference_indices
                ]
            )

            valid_3d = (
                point3d_ids != -1
            )

            valid_query_indices = (
                valid_query_indices[
                    valid_3d
                ]
            )

            point3d_ids = (
                point3d_ids[
                    valid_3d
                ]
            )

            total_matches += len(
                point3d_ids
            )

            for (
                query_index,
                point3d_id,
            ) in zip(
                valid_query_indices,
                point3d_ids,
            ):
                correspondences[
                    int(query_index)
                ].add(
                    int(point3d_id)
                )

        query_indices = []
        point3d_ids = []

        for (
            query_index,
            ids,
        ) in correspondences.items():

            for point3d_id in ids:

                query_indices.append(
                    query_index
                )

                point3d_ids.append(
                    point3d_id
                )

        return (
            query_indices,
            point3d_ids,
            total_matches,
        )

    # ========================================================
    # Shutdown
    # ========================================================

    def close(self):

        if hasattr(
            self,
            'ref_local_h5'
        ):
            self.ref_local_h5.close()