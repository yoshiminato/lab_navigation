#!/usr/bin/env python3

"""
画像からHLocでカメラ位置を推定する
"""

import math
import json
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String

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
from .dataset_layout import retrieval_features


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
            str(
                Path.home()
                / 'lab_navigation_ws'
                / 'captures'
                / 'hv_building_lab'
                / '20261010'
            ),
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

        # PnPカメラ姿勢推定の許容誤差(px)
        self.declare_parameter(
            'ransac_thresh',
            12.0,
        )

        # 参照画像の特徴奇術師を何件までメモリにキャッシュしておくか
        self.declare_parameter(
            'descriptor_cache_size',
            30,
        )

        self.declare_parameter(
            'map_frame',
            'map',
        )

        # /initialpose を受け取った後、半径を段階的に拡大する。
        # 半径は SfM 座標系の単位（通常はスケール調整後の m）。
        self.declare_parameter('prior_radii', [3.0, 5.0, 10.0])
        self.declare_parameter('initialpose_topic', '/initialpose')
        # 類似した別地点への再ジャンプを防ぐため成功後も局所探索を継続。
        self.declare_parameter('continue_local_tracking', True)
        # map -> SfM: p_sfm = scale * R(yaw) * p_map + translation
        # 事前に既知の座標変換で較正すること。初期値は恒等変換。
        self.declare_parameter('sfm_from_map_scale', 1.0)
        self.declare_parameter('sfm_from_map_yaw', 0.0)
        self.declare_parameter('sfm_from_map_x', 0.0)
        self.declare_parameter('sfm_from_map_y', 0.0)
        # base_link からカメラへの水平オフセット（base_link 座標系、m）
        self.declare_parameter('camera_offset_x', 0.0)
        self.declare_parameter('camera_offset_y', 0.0)

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

        self.prior_radii = sorted(set(float(r) for r in
            self.get_parameter('prior_radii').value))
        if not self.prior_radii or any(r <= 0 or not math.isfinite(r)
                                           for r in self.prior_radii):
            raise ValueError('prior_radii must contain finite positive radii')
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.continue_local_tracking = bool(self.get_parameter('continue_local_tracking').value)
        self.sfm_from_map_scale = float(self.get_parameter('sfm_from_map_scale').value)
        self.sfm_from_map_yaw = float(self.get_parameter('sfm_from_map_yaw').value)
        self.sfm_from_map_xy = np.array([
            float(self.get_parameter('sfm_from_map_x').value),
            float(self.get_parameter('sfm_from_map_y').value),
        ], dtype=np.float64)
        self.camera_offset_xy = np.array([
            float(self.get_parameter('camera_offset_x').value),
            float(self.get_parameter('camera_offset_y').value),
        ], dtype=np.float64)
        if self.sfm_from_map_scale <= 0 or not math.isfinite(self.sfm_from_map_scale):
            raise ValueError('sfm_from_map_scale must be finite and positive')
        self.position_prior_sfm = None
        self.prior_radius_index = 0
        self.local_tracking_active = False

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
            in ('superpoint', 'superpoint_lightglue')
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
                'superpoint (alias: superpoint_lightglue)'
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


        # SfM再構成結果の読み込み
        self.reconstruction = (
            pycolmap.Reconstruction(
                self.backend.sfm_path
            )
        )

        # 事前に撮影した画像列に内部IDを割り振る
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

        # カメラモデル(画像サイズ, カメラパラメータ)の取り出し
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

        # 各画像の2D特徴点に対応する3D点IDの一覧を辞書に格納
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

        self.reference_global_features = retrieval_features(self.data_path)

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

        # NetVLAD設定の取得
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

        # 参照画像のNetVLADの特徴量の読み込み
        (
            self.reference_names,
            reference_global,
        ) = (
            self._load_reference_global_features(
                self.reference_global_features
            )
        )

        # numpy配列からPyTorch Tensorに変換
        self.reference_global = (
            torch.from_numpy(
                reference_global
            )
            .float()
            .to(self.device)
        )

        # 正規化
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

        # SfM に登録された参照カメラ中心を NetVLAD の画像順に並べる。
        # projection_center() は SfM world 座標系でのカメラ中心。
        self.reference_xy = np.full((len(self.reference_names), 2),
                                    np.nan, dtype=np.float64)
        for index, name in enumerate(self.reference_names):
            image_id = self.db_name_to_id.get(name)
            if image_id is None:
                continue
            image = self.reconstruction.images[image_id]
            if not image.has_pose:
                continue
            center = np.asarray(image.projection_center(), dtype=np.float64).reshape(-1)
            self.reference_xy[index] = center[:2]
        self.get_logger().info(
            f'SfM positions available: {np.isfinite(self.reference_xy).all(axis=1).sum()}/'
            f'{len(self.reference_names)}'
        )

        # ====================================================
        # PnP
        # ====================================================

        # PnPローカライザの準備
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

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = (
            self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                image_qos,
            )
        )

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10,
        )

        self.pose_pub = (
            self.create_publisher(
                PoseStamped,
                '/hloc/camera_pose',
                pose_qos,
            )
        )

        self.status_pub = (
            self.create_publisher(
                String,
                '/hloc/localization_status',
                status_qos,
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

    # GPUに投げた処理が終わるまでCPUを待機させる
    def _sync_cuda(self):

        if self.device.type == 'cuda':
            torch.cuda.synchronize()

    # 画像リサイズ(計算量削減のため)
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

    # 参照画像ごとの大域特徴量を読み込み, [画像数, 特等次元]の行列にまとめる
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

    # 入力クエリ画像からNetVLADの大域特徴量を作成
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
    # RViz initial pose and position-constrained retrieval
    # ========================================================

    def initialpose_callback(self, msg):
        """RViz の base_link 初期姿勢を SfM カメラ位置の prior に変換。"""
        if msg.header.frame_id.lstrip('/') != self.map_frame.lstrip('/'):
            self.get_logger().error(
                f'Ignoring initialpose: frame_id={msg.header.frame_id!r} '
                f'(expected {self.map_frame!r}); no TF conversion is implemented.'
            )
            return

        pose = msg.pose.pose
        q = pose.orientation
        q_tuple = (float(q.x), float(q.y), float(q.z), float(q.w))
        try:
            qx, qy, qz, qw = quat_normalize(q_tuple)
        except ValueError:
            self.get_logger().error('Ignoring initialpose: zero quaternion')
            return
        # ROS の 2D Pose Estimate が設定するロボット yaw。
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        c, sn = math.cos(yaw), math.sin(yaw)
        base_xy = np.array([pose.position.x, pose.position.y], dtype=np.float64)
        cam_map_xy = base_xy + np.array([
            c * self.camera_offset_xy[0] - sn * self.camera_offset_xy[1],
            sn * self.camera_offset_xy[0] + c * self.camera_offset_xy[1],
        ])
        c, sn = math.cos(self.sfm_from_map_yaw), math.sin(self.sfm_from_map_yaw)
        rot_xy = np.array([
            c * cam_map_xy[0] - sn * cam_map_xy[1],
            sn * cam_map_xy[0] + c * cam_map_xy[1],
        ])
        center = self.sfm_from_map_scale * rot_xy + self.sfm_from_map_xy
        if not np.isfinite(center).all():
            self.get_logger().error('Ignoring non-finite initialpose')
            return
        self.position_prior_sfm = center
        self.prior_radius_index = 0
        self.local_tracking_active = False
        self.get_logger().info(
            f'Initialpose received: SfM camera center XY={center.tolist()}, '
            f'radius={self.prior_radii[0]:.2f} SfM units. '
            'Local search remains active until localization succeeds.'
        )

    def _retrieve(self, query_descriptor):
        """NetVLAD retrieval。prior 中は指定半径内だけを候補とする。"""
        similarity = (query_descriptor @ self.reference_global.T)[0]
        if self.position_prior_sfm is None:
            candidate_indices = torch.arange(
                similarity.shape[0], device=self.device, dtype=torch.long)
        else:
            radius = self.prior_radii[self.prior_radius_index]
            distances = np.linalg.norm(
                self.reference_xy - self.position_prior_sfm[None, :], axis=1)
            nearby = np.flatnonzero(np.isfinite(distances) & (distances <= radius))
            candidate_indices = torch.as_tensor(
                nearby, dtype=torch.long, device=self.device)
            self.get_logger().info(
                f'Prior search radius={radius:.2f}, nearby references={len(nearby)}'
            )

        if candidate_indices.numel() == 0:
            return [], np.empty(0, dtype=np.float32)
        k = min(self.num_matched, candidate_indices.numel())
        scores, ranking = torch.topk(
            similarity[candidate_indices], k=k, largest=True)
        indices = candidate_indices[ranking].detach().cpu().tolist()
        return (
            [self.reference_names[index] for index in indices],
            scores.detach().cpu().numpy(),
        )

    @staticmethod
    def _estimated_camera_xy(cam_from_world):
        # cam_from_world: world -> camera。逆変換してSfM座標内のカメラ中心を得る。
        q = quat_normalize(tuple(float(v) for v in cam_from_world.rotation.quat))
        q_inv = quat_conjugate(q)
        t = tuple(-float(v) for v in cam_from_world.translation)
        return np.asarray(rotate_vector(q_inv, t)[:2], dtype=np.float64)

    def _validate_position_prior(self, result):
        """PnPのインライア数だけでなく、指定地点との幾何的一致も検査。"""
        if not result['success'] or self.position_prior_sfm is None:
            return result
        xy = self._estimated_camera_xy(result['cam_from_world'])
        distance = float(np.linalg.norm(xy - self.position_prior_sfm))
        radius = self.prior_radii[self.prior_radius_index]
        self.get_logger().info(
            f'PnP SfM XY={xy.tolist()}, prior XY={self.position_prior_sfm.tolist()}, '
            f'error={distance:.3f}, tolerance={radius:.3f} SfM units')
        if not np.isfinite(distance) or distance > radius:
            result['success'] = False
            result['reason'] = 'pose_outside_prior_radius'
            self.get_logger().warning('PnP pose rejected: outside local search radius')
        else:
            result['estimated_xy'] = xy
        return result

    def _update_prior_after_attempt(self, result):
        if self.position_prior_sfm is None:
            return
        if result['success']:
            if self.continue_local_tracking:
                self.position_prior_sfm = result['estimated_xy']
                self.prior_radius_index = 0
                if not self.local_tracking_active:
                    self.get_logger().info('Prior accepted; local tracking enabled (global retrieval disabled).')
                self.local_tracking_active = True
            else:
                self.get_logger().info('Prior accepted; returning to global retrieval (may jump to a similar location).')
                self.position_prior_sfm = None
                self.prior_radius_index = 0
                self.local_tracking_active = False
        elif self.prior_radius_index < len(self.prior_radii) - 1:
            self.prior_radius_index += 1
            self.get_logger().warning(
                f'Local localization failed; next radius '
                f'{self.prior_radii[self.prior_radius_index]:.2f} SfM units.')
        else:
            self.get_logger().warning(
                'Local localization failed at maximum radius; keeping local mode. '
                'Use /initialpose again to reset the center.')

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

        result = {
            'success': False,
            'reason': 'unhandled_localization_exception',
            'num_matches': 0,
            'num_inliers': 0,
        }

        try:
            result = self._process_image(msg, frame_number, total_start)
            result = self._validate_position_prior(result)
            if result['success']:
                self._publish_pose(msg, result['cam_from_world'])
                self.success_count += 1
        except Exception as error:
            self.get_logger().error(
                f'Unhandled image localization error: {error}'
            )
            result = {
                'success': False,
                'reason': 'unhandled_localization_exception',
                'num_matches': 0,
                'num_inliers': 0,
            }

        self._update_prior_after_attempt(result)

        self._publish_localization_status(
            msg,
            frame_number,
            result['success'],
            result['reason'],
            num_matches=result['num_matches'],
            num_inliers=result['num_inliers'],
        )

    def _process_image(self, msg, frame_number, total_start):

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

            return {
                'success': False,
                'reason': 'cv_bridge_error',
                'num_matches': 0,
                'num_inliers': 0,
            }

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

        if not reference_names:
            self.get_logger().warning('No reference images inside the prior radius.')
            return {
                'success': False,
                'reason': 'no_prior_candidates',
                'num_matches': 0,
                'num_inliers': 0,
            }

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

            return {
                'success': False,
                'reason': 'no_2d_3d_correspondences',
                'num_matches': num_matches,
                'num_inliers': 0,
            }

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

            return {
                'success': False,
                'reason': 'localization_exception',
                'num_matches': num_matches,
                'num_inliers': 0,
            }

        t_pnp = (
            time.perf_counter()
            - start
        )

        num_inliers = 0
        success = False
        status_reason = 'localization_failed'

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
                status_reason = 'success'

            else:
                status_reason = 'insufficient_inliers'

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
            f'{self.success_count + int(success)}/'
            f'{self.frame_count}\n'
            '========================================'
        )

        outcome = {
            'success': success,
            'reason': status_reason,
            'num_matches': num_matches,
            'num_inliers': num_inliers,
        }
        if success:
            outcome['cam_from_world'] = result['cam_from_world']
        return outcome

    def _publish_localization_status(
        self,
        image_msg,
        frame_number,
        success,
        reason,
        num_matches=0,
        num_inliers=0,
    ):
        """Publish a timestamped machine-readable result for each processed image."""

        status = String()
        status.data = json.dumps(
            {
                'stamp_sec': int(image_msg.header.stamp.sec),
                'stamp_nanosec': int(image_msg.header.stamp.nanosec),
                'frame_number': int(frame_number),
                'success': bool(success),
                'reason': reason,
                'num_matches': int(num_matches),
                'num_inliers': int(num_inliers),
            },
            separators=(',', ':'),
        )
        self.status_pub.publish(status)

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
