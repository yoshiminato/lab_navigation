#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
)


def quat_normalize(q):
    x, y, z, w = q

    n = math.sqrt(
        x * x +
        y * y +
        z * z +
        w * w
    )

    if n == 0.0:
        raise ValueError("Zero-length quaternion")

    return (
        x / n,
        y / n,
        z / n,
        w / n,
    )


def quat_conjugate(q):
    x, y, z, w = q

    return (
        -x,
        -y,
        -z,
        w,
    )


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return (
        w1 * x2
        + x1 * w2
        + y1 * z2
        - z1 * y2,

        w1 * y2
        - x1 * z2
        + y1 * w2
        + z1 * x2,

        w1 * z2
        + x1 * y2
        - y1 * x2
        + z1 * w2,

        w1 * w2
        - x1 * x2
        - y1 * y2
        - z1 * z2,
    )


def rotate_vector(q, v):
    q = quat_normalize(q)

    vq = (
        v[0],
        v[1],
        v[2],
        0.0,
    )

    r = quat_multiply(
        quat_multiply(q, vq),
        quat_conjugate(q),
    )

    return (
        r[0],
        r[1],
        r[2],
    )


def invert_transform(t, q):
    """
    T_a_b を反転して T_b_a を求める。

    Parameters
    ----------
    t : tuple
        並進 (x, y, z)
    q : tuple
        quaternion (x, y, z, w)

    Returns
    -------
    t_inv, q_inv
    """

    q_inv = quat_conjugate(
        quat_normalize(q)
    )

    t_inv = rotate_vector(
        q_inv,
        (
            -t[0],
            -t[1],
            -t[2],
        ),
    )

    return t_inv, q_inv


def compose_transform(
    t1,
    q1,
    t2,
    q2,
):
    """
    T = T1 * T2

    例:
        T_map_base =
            T_map_camera *
            T_camera_base
    """

    rotated_t2 = rotate_vector(
        q1,
        t2,
    )

    t = (
        t1[0] + rotated_t2[0],
        t1[1] + rotated_t2[1],
        t1[2] + rotated_t2[2],
    )

    q = quat_multiply(
        q1,
        q2,
    )

    return (
        t,
        quat_normalize(q),
    )


class HlocTfPublisher(Node):

    def __init__(self):
        super().__init__(
            'hloc_tf_publisher'
        )

        # -----------------------------
        # Parameters
        # -----------------------------

        self.declare_parameter(
            'result_file',
            '',
        )

        self.declare_parameter(
            'query_name',
            'test1.png',
        )

        self.declare_parameter(
            'map_frame',
            'map',
        )

        self.declare_parameter(
            'odom_frame',
            'odom',
        )

        self.declare_parameter(
            'base_frame',
            'base_link',
        )

        self.declare_parameter(
            'camera_frame',
            'camera_optical_frame',
        )

        # -----------------------------
        # Read parameters
        # -----------------------------

        self.result_file = (
            self.get_parameter(
                'result_file'
            ).value
        )

        self.query_name = (
            self.get_parameter(
                'query_name'
            ).value
        )

        self.map_frame = (
            self.get_parameter(
                'map_frame'
            ).value
        )

        self.odom_frame = (
            self.get_parameter(
                'odom_frame'
            ).value
        )

        self.base_frame = (
            self.get_parameter(
                'base_frame'
            ).value
        )

        self.camera_frame = (
            self.get_parameter(
                'camera_frame'
            ).value
        )

        # -----------------------------
        # Validate result file
        # -----------------------------

        if not self.result_file:
            raise RuntimeError(
                'Parameter "result_file" is empty.'
            )

        result_path = Path(
            self.result_file
        )

        if not result_path.exists():
            raise RuntimeError(
                f'Result file not found: '
                f'{result_path}'
            )

        # -----------------------------
        # TF
        # -----------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = (
            TransformListener(
                self.tf_buffer,
                self,
            )
        )

        self.tf_broadcaster = (
            TransformBroadcaster(
                self
            )
        )

        # -----------------------------
        # Read HLoc pose
        # -----------------------------

        pose = self.read_hloc_result(
            result_path,
            self.query_name,
        )

        if pose is None:
            raise RuntimeError(
                f'Query "{self.query_name}" '
                f'not found in '
                f'{result_path}'
            )

        # HLoc/COLMAP pose
        #
        # X_cam =
        # R_cw X_world + t_cw
        #
        # を ROS TF の
        #
        # map -> camera_optical_frame
        #
        # に変換
        self.map_to_camera = (
            self.colmap_pose_to_ros(
                *pose
            )
        )

        t_map_camera, q_map_camera = (
            self.map_to_camera
        )

        self.get_logger().info(
            'Loaded HLoc pose: '
            f'{self.query_name}'
        )

        self.get_logger().info(
            'map -> camera: '
            f'x={t_map_camera[0]:.3f}, '
            f'y={t_map_camera[1]:.3f}, '
            f'z={t_map_camera[2]:.3f}'
        )

        self.get_logger().info(
            'map -> camera quaternion: '
            f'x={q_map_camera[0]:.6f}, '
            f'y={q_map_camera[1]:.6f}, '
            f'z={q_map_camera[2]:.6f}, '
            f'w={q_map_camera[3]:.6f}'
        )

        # 初回成功確認用
        self.ready = False

        # -----------------------------
        # Timer
        # -----------------------------

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.get_logger().info(
            'Waiting for TFs: '
            f'{self.base_frame} -> '
            f'{self.camera_frame}, '
            f'{self.odom_frame} -> '
            f'{self.base_frame}'
        )

    @staticmethod
    def read_hloc_result(
        path,
        query_name,
    ):
        """
        HLoc の localization result を読む。

        想定フォーマット:

        image_name qw qx qy qz tx ty tz
        """

        with path.open(
            'r',
            encoding='utf-8',
        ) as f:

            for line in f:

                line = line.strip()

                if not line:
                    continue

                values = line.split()

                if len(values) != 8:
                    continue

                name = values[0]

                if (
                    name == query_name
                    or
                    Path(name).name
                    == Path(query_name).name
                ):
                    try:
                        return tuple(
                            map(
                                float,
                                values[1:],
                            )
                        )

                    except ValueError:
                        return None

        return None

    @staticmethod
    def colmap_pose_to_ros(
        qw,
        qx,
        qy,
        qz,
        tx,
        ty,
        tz,
    ):
        """
        COLMAP / HLoc pose:

            X_cam =
                R_cw X_world + t_cw

        これは world -> camera の変換。

        ROS TF として欲しいのは:

            map -> camera_optical_frame

        なので逆変換する。

            R_wc = R_cw^T

            t_wc =
                -R_cw^T t_cw
        """

        q_cw = quat_normalize(
            (
                qx,
                qy,
                qz,
                qw,
            )
        )

        # camera -> world
        q_wc = quat_conjugate(
            q_cw
        )

        t_wc = rotate_vector(
            q_wc,
            (
                -tx,
                -ty,
                -tz,
            ),
        )

        return (
            t_wc,
            q_wc,
        )

    def timer_callback(self):

        try:
            # --------------------------------
            # base_link -> camera_optical_frame
            # URDF / static TF
            # --------------------------------

            tf_base_camera = (
                self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    Time(),
                    timeout=Duration(
                        seconds=0.1
                    ),
                )
            )

            # --------------------------------
            # odom -> base_link
            # Wheel odometry
            # --------------------------------

            tf_odom_base = (
                self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(
                        seconds=0.1
                    ),
                )
            )

        except Exception as e:

            if not self.ready:
                self.get_logger().info(
                    'Waiting for required TFs: '
                    f'{e}'
                )

            return

        # ====================================
        # base -> camera
        # ====================================

        t_base_camera = (
            tf_base_camera
            .transform
            .translation
            .x,

            tf_base_camera
            .transform
            .translation
            .y,

            tf_base_camera
            .transform
            .translation
            .z,
        )

        q_base_camera = (
            tf_base_camera
            .transform
            .rotation
            .x,

            tf_base_camera
            .transform
            .rotation
            .y,

            tf_base_camera
            .transform
            .rotation
            .z,

            tf_base_camera
            .transform
            .rotation
            .w,
        )

        # camera -> base

        (
            t_camera_base,
            q_camera_base,
        ) = invert_transform(
            t_base_camera,
            q_base_camera,
        )

        # ====================================
        # map -> camera
        # HLoc から取得済み
        # ====================================

        (
            t_map_camera,
            q_map_camera,
        ) = self.map_to_camera

        # ====================================
        # map -> base
        #
        # T_map_base =
        # T_map_camera *
        # T_camera_base
        # ====================================

        (
            t_map_base,
            q_map_base,
        ) = compose_transform(
            t_map_camera,
            q_map_camera,
            t_camera_base,
            q_camera_base,
        )

        # ====================================
        # odom -> base
        # ====================================

        t_odom_base = (
            tf_odom_base
            .transform
            .translation
            .x,

            tf_odom_base
            .transform
            .translation
            .y,

            tf_odom_base
            .transform
            .translation
            .z,
        )

        q_odom_base = (
            tf_odom_base
            .transform
            .rotation
            .x,

            tf_odom_base
            .transform
            .rotation
            .y,

            tf_odom_base
            .transform
            .rotation
            .z,

            tf_odom_base
            .transform
            .rotation
            .w,
        )

        # base -> odom

        (
            t_base_odom,
            q_base_odom,
        ) = invert_transform(
            t_odom_base,
            q_odom_base,
        )

        # ====================================
        # map -> odom
        #
        # T_map_odom =
        # T_map_base *
        # T_base_odom
        # ====================================

        (
            t_map_odom,
            q_map_odom,
        ) = compose_transform(
            t_map_base,
            q_map_base,
            t_base_odom,
            q_base_odom,
        )

        # ====================================
        # map -> odom publish
        # ====================================

        msg = TransformStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            self.map_frame
        )

        msg.child_frame_id = (
            self.odom_frame
        )

        msg.transform.translation.x = (
            t_map_odom[0]
        )

        msg.transform.translation.y = (
            t_map_odom[1]
        )

        msg.transform.translation.z = (
            t_map_odom[2]
        )

        msg.transform.rotation.x = (
            q_map_odom[0]
        )

        msg.transform.rotation.y = (
            q_map_odom[1]
        )

        msg.transform.rotation.z = (
            q_map_odom[2]
        )

        msg.transform.rotation.w = (
            q_map_odom[3]
        )

        self.tf_broadcaster.sendTransform(
            msg
        )

        # ====================================
        # 初回のみログ表示
        # ====================================

        if not self.ready:

            self.ready = True

            self.get_logger().info(
                'Required TFs obtained.'
            )

            self.get_logger().info(
                'HLoc map -> base_link: '
                f'x={t_map_base[0]:.3f}, '
                f'y={t_map_base[1]:.3f}, '
                f'z={t_map_base[2]:.3f}'
            )

            self.get_logger().info(
                'Publishing map -> odom: '
                f'x={t_map_odom[0]:.3f}, '
                f'y={t_map_odom[1]:.3f}, '
                f'z={t_map_odom[2]:.3f}'
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = HlocTfPublisher()

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
