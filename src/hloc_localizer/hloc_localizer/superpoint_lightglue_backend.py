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


class SuperPointLightGlueBackend:

    method_name = 'superpoint_lightglue'
    timing_name = 'SuperPoint + LightGlue'

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

        self.hloc_dir = (
            self.data_path /
            'hloc_output_superpoint_lightglue'
        )

        self.sfm_path = (
            self.hloc_dir /
            'sfm-superpoint-lightglue-colmapgv'
        )

        self.reference_local_features = (
            self.hloc_dir /
            'feats-ref-superpoint.h5'
        )

        if not self.sfm_path.exists():
            raise FileNotFoundError(
                str(self.sfm_path)
            )

        if not self.reference_local_features.exists():
            raise FileNotFoundError(
                str(
                    self.reference_local_features
                )
            )

        # ----------------------------------------------------
        # SuperPoint
        # ----------------------------------------------------

        if self.logger:
            self.logger.info(
                'Loading SuperPoint extractor...'
            )

        self.local_conf = (
            extract_features
            .confs[
                'superpoint_aachen'
            ]
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
        # LightGlue
        # ----------------------------------------------------

        if self.logger:
            self.logger.info(
                'Loading LightGlue matcher...'
            )

        self.matcher_conf = (
            match_features
            .confs[
                'superpoint+lightglue'
            ]
        )

        Matcher = dynamic_load(
            matchers,
            self.matcher_conf[
                'model'
            ]['name'],
        )

        self.matcher = (
            Matcher(
                self.matcher_conf[
                    'model'
                ]
            )
            .eval()
            .to(self.device)
        )

        # ----------------------------------------------------
        # Reference features
        # ----------------------------------------------------

        self.ref_local_h5 = h5py.File(
            str(
                self.reference_local_features
            ),
            'r',
            libver='latest',
        )

        self.feature_cache = (
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
    # SuperPoint extraction
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
                        'image':
                        tensor
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
            ][0]
            .detach()
            .float()
            .unsqueeze(0)
            .to(self.device)
        )

        scores = None

        if 'scores' in prediction:

            scores = (
                prediction[
                    'scores'
                ][0]
                .detach()
                .float()
                .unsqueeze(0)
                .to(self.device)
            )

        # HLoc extract_features.py と同じ
        # original image coordinateへ戻す
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

            'scores':
            scores,

            'image_size':
            (
                original_width,
                original_height,
            ),
        }

    # ========================================================
    # Reference feature cache
    # ========================================================

    def _get_reference_features(
        self,
        name,
    ):
        if name in self.feature_cache:

            value = (
                self.feature_cache.pop(
                    name
                )
            )

            self.feature_cache[
                name
            ] = value

            return value

        group = self.ref_local_h5[
            name
        ]

        keypoints = (
            torch.from_numpy(
                np.asarray(
                    group[
                        'keypoints'
                    ][()],
                    dtype=np.float32,
                )
            )
            .float()
            .unsqueeze(0)
            .to(self.device)
        )

        descriptors = (
            torch.from_numpy(
                np.asarray(
                    group[
                        'descriptors'
                    ][()]
                )
            )
            .float()
            .unsqueeze(0)
            .to(self.device)
        )

        scores = None

        if 'scores' in group:

            scores = (
                torch.from_numpy(
                    np.asarray(
                        group[
                            'scores'
                        ][()]
                    )
                )
                .float()
                .unsqueeze(0)
                .to(self.device)
            )

        image_size_np = np.asarray(
            group[
                'image_size'
            ][()]
        )

        image_size = (
            int(image_size_np[0]),
            int(image_size_np[1]),
        )

        value = {
            'keypoints':
            keypoints,

            'descriptors':
            descriptors,

            'scores':
            scores,

            'image_size':
            image_size,
        }

        self.feature_cache[
            name
        ] = value

        while (
            len(
                self.feature_cache
            )
            > self.cache_size
        ):
            self.feature_cache.popitem(
                last=False
            )

        return value

    # ========================================================
    # LightGlue matching + 2D -> 3D
    # ========================================================

    def build_correspondences(
        self,
        query_features,
        reference_names,
        ref_point3d_ids,
    ):
        query_keypoints = (
            query_features[
                'keypoints'
            ]
        )

        query_descriptors = (
            query_features[
                'descriptors'
            ]
        )

        query_scores = (
            query_features[
                'scores'
            ]
        )

        (
            query_width,
            query_height,
        ) = (
            query_features[
                'image_size'
            ]
        )

        query_keypoints_tensor = (
            torch.from_numpy(
                query_keypoints
            )
            .float()
            .unsqueeze(0)
            .to(self.device)
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

            reference = (
                self._get_reference_features(
                    ref_name
                )
            )

            (
                ref_width,
                ref_height,
            ) = reference[
                'image_size'
            ]

            # match_features.FeaturePairsDataset
            # と同等のデータ構造
            data = {
                'image0':
                torch.empty(
                    (
                        1,
                        1,
                        query_height,
                        query_width,
                    ),
                    dtype=torch.float32,
                ),

                'keypoints0':
                query_keypoints_tensor,

                'descriptors0':
                query_descriptors,

                'image_size0':
                torch.tensor(
                    [
                        [
                            query_width,
                            query_height,
                        ]
                    ],
                    dtype=torch.float32,
                    device=self.device,
                ),

                'image1':
                torch.empty(
                    (
                        1,
                        1,
                        ref_height,
                        ref_width,
                    ),
                    dtype=torch.float32,
                ),

                'keypoints1':
                reference[
                    'keypoints'
                ],

                'descriptors1':
                reference[
                    'descriptors'
                ],

                'image_size1':
                torch.tensor(
                    [
                        [
                            ref_width,
                            ref_height,
                        ]
                    ],
                    dtype=torch.float32,
                    device=self.device,
                ),
            }

            if query_scores is not None:
                data[
                    'scores0'
                ] = query_scores

            if (
                reference[
                    'scores'
                ]
                is not None
            ):
                data[
                    'scores1'
                ] = reference[
                    'scores'
                ]

            with torch.no_grad():

                prediction = (
                    self.matcher(
                        data
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