#!/usr/bin/env python3

import math
import time
from pathlib import Path

import cv2
import h5py
import numpy as np
import pycolmap
import torch

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from hloc import extract_features
from hloc import extractors

from hloc.localize_sfm import (
    QueryLocalizer,
)

from hloc.utils.base_model import (
    dynamic_load,
)

from hloc.utils.io import (
    list_h5_names,
)

from .sift_backend import (
    SiftBackend,
)

from .superpoint_lightglue_backend import (
    SuperPointLightGlueBackend,
)


# ============================================================
# Quaternion
# ============================================================

def quat_normalize(q):

    x, y, z, w = q

    n = math.sqrt(
        x*x +
        y*y +
        z*z +
        w*w
    )

    if n == 0.0:
        raise ValueError(
            'Zero-length quaternion'
        )

    return (
        x/n,
        y/n,
        z/n,
        w/n,
    )


def quat_conjugate(q):

    x, y, z, w = q

    return (
        -x,
        -y,
        -z,
        w,
    )


def quat_multiply(
    q1,
    q2,
):

    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return (
        w1*x2
        + x1*w2
        + y1*z2
        - z1*y2,

        w1*y2
        - x1*z2
        + y1*w2
        + z1*x2,

        w1*z2
        + x1*y2
        - y1*x2
        + z1*w2,

        w1*w2
        - x1*x2
        - y1*y2
        - z1*z2,
    )


def rotate_vector(q, v):

    q = quat_normalize(q)

    vq = (
        v[0],
        v[1],
        v[2],
        0.0,
    )

    result = quat_multiply(
        quat_multiply(
            q,
            vq,
        ),
        quat_conjugate(q),
    )

    return (
        result[0],
        result[1],
        result[2],
    )


# ============================================================
# HLoc node
# ============================================================

class HlocLocalizerNode(Node):

    def __init__(self):

        super().__init__(
            'hloc_localizer_node'
        )

        # ====================================================
        # Parameters
        # ====================================================

        self.declare_parameter(
            'data_path',
            '/home/user/'
            'lab_navigation_ws/'
            'captures/'
            'hv_building_lab/'
            '20261010',
        )

        self.declare_parameter(
            'image_topic',
            '/camera/image_raw',
        )

        self.declare_parameter(
            'local_method',
            'sift',
        )

        self.declare_parameter(
            'num_matched',
            20,
        )

        self.declare_parameter(
            'min_inliers',
            12,
        )

        self.declare_parameter(
            'ransac_thresh',
            12.0,
        )

        self.declare_parameter(
            'descriptor_cache_size',
            30,
        )

        self.declare_parameter(
            'map_frame',
            'map',
        )

        self.data_path = Path(
            self.get_parameter(
                'data_path'
            ).value
        )

        self.image_topic = (
            self.get_parameter(
                'image_topic'
            ).value
        )

        self.local_method = (
            self.get_parameter(
                'local_method'
            ).value
        )

        self.num_matched = int(
            self.get_parameter(
                'num_matched'
            ).value
        )

        self.min_inliers = int(
            self.get_parameter(
                'min_inliers'
            ).value
        )

        self.ransac_thresh = float(
            self.get_parameter(
                'ransac_thresh'
            ).value
        )

        self.cache_size = int(
            self.get_parameter(
                'descriptor_cache_size'
            ).value
        )

        self.map_frame = (
            self.get_parameter(
                'map_frame'
            ).value
        )

        # ====================================================
        # Device
        # ====================================================

        self.device = torch.device(
            'cuda'
            if torch.cuda.is_available()
            else 'cpu'
        )

        self.get_logger().info(
            '===================================='
        )

        self.get_logger().info(
            'HLoc localizer'
        )

        self.get_logger().info(
            f'Local method: '
            f'{self.local_method}'
        )

        self.get_logger().info(
            f'PyTorch device: '
            f'{self.device}'
        )

        self.get_logger().info(
            f'PyCOLMAP CUDA: '
            f'{pycolmap.has_cuda}'
        )

        if self.device.type == 'cuda':

            self.get_logger().info(
                'GPU: '
                f'{torch.cuda.get_device_name(0)}'
            )

        # ====================================================
        # Backend
        # ====================================================

        if self.local_method == 'sift':

            self.backend = SiftBackend(
                data_path=self.data_path,
                device=self.device,
                cache_size=self.cache_size,
                logger=self.get_logger(),
            )

        elif (
            self.local_method
            == 'superpoint_lightglue'
        ):

            self.backend = (
                SuperPointLightGlueBackend(
                    data_path=self.data_path,
                    device=self.device,
                    cache_size=self.cache_size,
                    logger=self.get_logger(),
                )
            )

        else:

            raise RuntimeError(
                'Unknown local_method: '
                f'{self.local_method}\n'
                'Available methods: '
                'sift, '
                'superpoint_lightglue'
            )

        # ====================================================
        # Reconstruction
        # ====================================================

        initialization_start = (
            time.perf_counter()
        )

        self.get_logger().info(
            'Loading reconstruction: '
            f'{self.backend.sfm_path}'
        )

        self.reconstruction = (
            pycolmap.Reconstruction(
                self.backend.sfm_path
            )
        )

        self.db_name_to_id = {
            image.name:
            image_id

            for (
                image_id,
                image,
            )
            in self.reconstruction
            .images.items()
        }

        self.get_logger().info(
            'Registered images: '
            f'{len(self.db_name_to_id)}'
        )

        self.get_logger().info(
            '3D points: '
            f'{len(self.reconstruction.points3D)}'
        )

        # ====================================================
        # Camera
        # ====================================================

        if len(
            self.reconstruction.cameras
        ) != 1:

            self.get_logger().warning(
                'Reconstruction contains '
                f'{len(self.reconstruction.cameras)} '
                'cameras. Using the first one.'
            )

        self.query_camera = next(
            iter(
                self.reconstruction
                .cameras
                .values()
            )
        )

        self.get_logger().info(
            'Query camera: '
            f'{self.query_camera.model.name}, '
            f'{self.query_camera.width}x'
            f'{self.query_camera.height}'
        )

        # ====================================================
        # Reference feature index -> point3D
        # ====================================================

        self.get_logger().info(
            'Building reference '
            '2D -> 3D lookup tables...'
        )

        self.ref_point3d_ids = {}

        for (
            image_id,
            image,
        ) in self.reconstruction.images.items():

            point_ids = np.array(
                [
                    point.point3D_id
                    if point.has_point3D()
                    else -1

                    for point
                    in image.points2D
                ],
                dtype=np.int64,
            )

            self.ref_point3d_ids[
                image.name
            ] = point_ids

        # ====================================================
        # NetVLAD
        #
        # Both local methods share this.
        # ====================================================

        self.hloc_dir = (
            self.data_path /
            'hloc_output'
        )

        self.reference_global_features = (
            self.hloc_dir /
            'global-ref-netvlad.h5'
        )

        if not (
            self.reference_global_features
            .exists()
        ):

            raise FileNotFoundError(
                str(
                    self.reference_global_features
                )
            )

        self.get_logger().info(
            'Loading NetVLAD model...'
        )

        self.netvlad_conf = (
            extract_features
            .confs['netvlad']
        )

        NetVLADModel = dynamic_load(
            extractors,
            self.netvlad_conf[
                'model'
            ]['name'],
        )

        self.netvlad_model = (
            NetVLADModel(
                self.netvlad_conf[
                    'model'
                ]
            )
            .eval()
            .to(self.device)
        )

        (
            self.reference_names,
            reference_global,
        ) = (
            self._load_reference_global_features(
                self.reference_global_features
            )
        )

        self.reference_global = (
            torch.from_numpy(
                reference_global
            )
            .float()
            .to(self.device)
        )

        self.reference_global = (
            torch.nn.functional.normalize(
                self.reference_global,
                dim=1,
            )
        )

        self.get_logger().info(
            'NetVLAD reference database: '
            f'{len(self.reference_names)} '
            'images'
        )

        # ====================================================
        # PnP
        # ====================================================

        self.localizer = QueryLocalizer(
            self.reconstruction,
            config={
                'estimation': {
                    'ransac': {
                        'max_error':
                        self.ransac_thresh,
                    }
                }
            },
        )

        # ====================================================
        # ROS
        # ====================================================

        self.bridge = CvBridge()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
            ),
            durability=(
                DurabilityPolicy.VOLATILE
            ),
        )

        # 最新のHLoc poseを後から起動した
        # tf_publisherでも受けられるようにする
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
            ),
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self.subscription = (
            self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                image_qos,
            )
        )

        self.pose_pub = (
            self.create_publisher(
                PoseStamped,
                '/hloc/camera_pose',
                pose_qos,
            )
        )

        self.frame_count = 0
        self.success_count = 0

        self.total_times = []

        initialization_time = (
            time.perf_counter()
            - initialization_start
        )

        self.get_logger().info(
            'Initialization complete: '
            f'{initialization_time:.3f} s'
        )

        self.get_logger().info(
            'Waiting for images: '
            f'{self.image_topic}'
        )

        self.get_logger().info(
            '===================================='
        )

    # ========================================================
    # Utilities
    # ========================================================

    def _sync_cuda(self):

        if self.device.type == 'cuda':
            torch.cuda.synchronize()

    @staticmethod
    def _resize_max(
        image,
        resize_max,
    ):

        height, width = (
            image.shape[:2]
        )

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

        size = (
            int(
                round(width * scale)
            ),
            int(
                round(height * scale)
            ),
        )

        return cv2.resize(
            image,
            size,
            interpolation=cv2.INTER_AREA,
        )

    # ========================================================
    # NetVLAD reference
    # ========================================================

    def _load_reference_global_features(
        self,
        path,
    ):

        names = list_h5_names(
            path
        )

        names = [
            name
            for name in names
            if name in self.db_name_to_id
        ]

        descriptors = []
        valid_names = []

        with h5py.File(
            str(path),
            'r',
            libver='latest',
        ) as file:

            for name in names:

                if (
                    'global_descriptor'
                    not in file[name]
                ):
                    continue

                descriptor = (
                    file[
                        name
                    ][
                        'global_descriptor'
                    ][()]
                )

                descriptors.append(
                    np.asarray(
                        descriptor,
                        dtype=np.float32,
                    )
                )

                valid_names.append(
                    name
                )

        if not descriptors:

            raise RuntimeError(
                'No reference NetVLAD '
                'descriptors found.'
            )

        return (
            valid_names,
            np.stack(
                descriptors,
                axis=0,
            ),
        )

    # ========================================================
    # NetVLAD query
    # ========================================================

    def _extract_netvlad(
        self,
        image_bgr,
    ):

        rgb = cv2.cvtColor(
            image_bgr,
            cv2.COLOR_BGR2RGB,
        )

        rgb = self._resize_max(
            rgb,
            self.netvlad_conf[
                'preprocessing'
            ].get(
                'resize_max'
            ),
        )

        tensor = (
            torch.from_numpy(
                rgb.astype(
                    np.float32
                )
            )
            .permute(
                2,
                0,
                1,
            )
            .unsqueeze(0)
            / 255.0
        )

        tensor = tensor.to(
            self.device
        )

        with torch.no_grad():

            prediction = (
                self.netvlad_model(
                    {
                        'image':
                        tensor
                    }
                )
            )

        descriptor = (
            prediction[
                'global_descriptor'
            ]
            .float()
        )

        return (
            torch.nn.functional.normalize(
                descriptor,
                dim=1,
            )
        )

    # ========================================================
    # Retrieval
    # ========================================================

    def _retrieve(
        self,
        query_descriptor,
    ):

        similarity = (
            query_descriptor
            @ self.reference_global.T
        )

        k = min(
            self.num_matched,
            similarity.shape[1],
        )

        scores, indices = (
            torch.topk(
                similarity,
                k=k,
                dim=1,
                largest=True,
            )
        )

        indices = (
            indices[0]
            .detach()
            .cpu()
            .numpy()
        )

        scores = (
            scores[0]
            .detach()
            .cpu()
            .numpy()
        )

        names = [
            self.reference_names[index]
            for index in indices
        ]

        return (
            names,
            scores,
        )

    # ========================================================
    # Pose publisher
    # ========================================================

    def _publish_pose(
        self,
        msg,
        cam_from_world,
    ):

        quaternion = (
            cam_from_world
            .rotation
            .quat
        )

        translation = (
            cam_from_world
            .translation
        )

        q_cw = quat_normalize(
            (
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            )
        )

        q_wc = quat_conjugate(
            q_cw
        )

        t_wc = rotate_vector(
            q_wc,
            (
                -float(
                    translation[0]
                ),
                -float(
                    translation[1]
                ),
                -float(
                    translation[2]
                ),
            ),
        )

        pose = PoseStamped()

        # 画像撮影時刻を維持
        pose.header.stamp = (
            msg.header.stamp
        )

        pose.header.frame_id = (
            self.map_frame
        )

        pose.pose.position.x = (
            t_wc[0]
        )

        pose.pose.position.y = (
            t_wc[1]
        )

        pose.pose.position.z = (
            t_wc[2]
        )

        pose.pose.orientation.x = (
            q_wc[0]
        )

        pose.pose.orientation.y = (
            q_wc[1]
        )

        pose.pose.orientation.z = (
            q_wc[2]
        )

        pose.pose.orientation.w = (
            q_wc[3]
        )

        self.pose_pub.publish(
            pose
        )

    # ========================================================
    # Image
    # ========================================================

    def image_callback(
        self,
        msg,
    ):

        self.frame_count += 1

        frame_number = (
            self.frame_count
        )

        total_start = (
            time.perf_counter()
        )

        try:

            image_bgr = (
                self.bridge
                .imgmsg_to_cv2(
                    msg,
                    desired_encoding='bgr8',
                )
            )

        except Exception as error:

            self.get_logger().error(
                'cv_bridge error: '
                f'{error}'
            )

            return

        height, width = (
            image_bgr.shape[:2]
        )

        if (
            width
            != self.query_camera.width
            or
            height
            != self.query_camera.height
        ):

            self.get_logger().warning(
                'Input resolution differs '
                'from SfM camera: '
                f'{width}x{height} vs '
                f'{self.query_camera.width}x'
                f'{self.query_camera.height}'
            )

        # ----------------------------------------------------
        # Local features
        # ----------------------------------------------------

        self._sync_cuda()

        start = time.perf_counter()

        query_features = (
            self.backend.extract(
                image_bgr
            )
        )

        self._sync_cuda()

        t_local = (
            time.perf_counter()
            - start
        )

        query_keypoints = (
            query_features[
                'keypoints'
            ]
        )

        # ----------------------------------------------------
        # NetVLAD
        # ----------------------------------------------------

        self._sync_cuda()

        start = time.perf_counter()

        query_global = (
            self._extract_netvlad(
                image_bgr
            )
        )

        self._sync_cuda()

        t_netvlad = (
            time.perf_counter()
            - start
        )

        # ----------------------------------------------------
        # Retrieval
        # ----------------------------------------------------

        start = time.perf_counter()

        (
            reference_names,
            retrieval_scores,
        ) = self._retrieve(
            query_global
        )

        t_retrieval = (
            time.perf_counter()
            - start
        )

        if reference_names:

            self.get_logger().info(
                'Best reference: '
                f'{reference_names[0]} '
                f'({retrieval_scores[0]:.4f})'
            )

        # ----------------------------------------------------
        # Matching
        # ----------------------------------------------------

        self._sync_cuda()

        start = time.perf_counter()

        (
            query_indices,
            point3d_ids,
            num_matches,
        ) = (
            self.backend
            .build_correspondences(
                query_features,
                reference_names,
                self.ref_point3d_ids,
            )
        )

        self._sync_cuda()

        t_matching = (
            time.perf_counter()
            - start
        )

        if not query_indices:

            self.get_logger().warning(
                f'Frame {frame_number}: '
                'no 2D-3D correspondences'
            )

            return

        # ----------------------------------------------------
        # PnP
        # ----------------------------------------------------

        query_keypoints_colmap = (
            query_keypoints
            + 0.5
        )

        start = time.perf_counter()

        try:

            result = (
                self.localizer.localize(
                    query_keypoints_colmap,
                    query_indices,
                    point3d_ids,
                    self.query_camera,
                )
            )

        except Exception as error:

            self.get_logger().error(
                'PnP/localization error: '
                f'{error}'
            )

            return

        t_pnp = (
            time.perf_counter()
            - start
        )

        num_inliers = 0
        success = False

        if result is not None:

            try:

                num_inliers = int(
                    result[
                        'num_inliers'
                    ]
                )

            except Exception:

                num_inliers = 0

            if (
                num_inliers
                >= self.min_inliers
            ):

                success = True

                self.success_count += 1

                self._publish_pose(
                    msg,
                    result[
                        'cam_from_world'
                    ],
                )

        total_time = (
            time.perf_counter()
            - total_start
        )

        if frame_number > 1:

            self.total_times.append(
                total_time
            )

        if self.total_times:

            average_time = (
                sum(self.total_times)
                / len(self.total_times)
            )

            rate = (
                1.0 / average_time
                if average_time > 0
                else 0.0
            )

        else:

            average_time = 0.0
            rate = 0.0

        status = (
            'SUCCESS'
            if success
            else 'FAILED'
        )

        self.get_logger().info(
            '\n'
            '========================================\n'
            f'Method     : {self.local_method}\n'
            f'Frame      : {frame_number}\n'
            f'Result     : {status}\n'
            f'Keypoints  : '
            f'{len(query_keypoints)}\n'
            f'Candidates : '
            f'{len(reference_names)}\n'
            f'3D matches : '
            f'{num_matches}\n'
            f'PnP corr.  : '
            f'{len(point3d_ids)}\n'
            f'Inliers    : '
            f'{num_inliers}\n'
            '----------------------------------------\n'
            f'Local feat : {t_local:.3f} s\n'
            f'NetVLAD    : {t_netvlad:.3f} s\n'
            f'Retrieval  : {t_retrieval:.3f} s\n'
            f'Matching   : {t_matching:.3f} s\n'
            f'PnP        : {t_pnp:.3f} s\n'
            '----------------------------------------\n'
            f'TOTAL      : {total_time:.3f} s\n'
            f'Avg(*)     : {average_time:.3f} s\n'
            f'Rate(*)    : {rate:.2f} Hz\n'
            f'Success    : '
            f'{self.success_count}/'
            f'{self.frame_count}\n'
            '========================================'
        )

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        try:

            if hasattr(
                self,
                'backend'
            ):
                self.backend.close()

        finally:

            super().destroy_node()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = HlocLocalizerNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()