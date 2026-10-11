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
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/n, y/n, z/n, w/n)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def rotate_vector(q, v):
    q = quat_normalize(q)

    vq = (v[0], v[1], v[2], 0.0)

    r = quat_multiply(
        quat_multiply(q, vq),
        quat_conjugate(q)
    )

    return (r[0], r[1], r[2])


def invert_transform(t, q):
    q_inv = quat_conjugate(quat_normalize(q))

    rt = rotate_vector(
        q_inv,
        (-t[0], -t[1], -t[2])
    )

    return rt, q_inv


def compose_transform(t1, q1, t2, q2):
    """
    T = T1 * T2
    """

    r2 = rotate_vector(q1, t2)

    t = (
        t1[0] + r2[0],
        t1[1] + r2[1],
        t1[2] + r2[2],
    )

    q = quat_multiply(q1, q2)

    return t, quat_normalize(q)


class HlocTfPublisher(Node):

    def __init__(self):
        super().__init__('hloc_tf_publisher')

        self.declare_parameter('result_file', '')
        self.declare_parameter('query_name', 'test1.png')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'camera_frame',
            'camera_optical_frame'
        )

        self.result_file = self.get_parameter(
            'result_file'
        ).value

        self.query_name = self.get_parameter(
            'query_name'
        ).value

        self.map_frame = self.get_parameter(
            'map_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.camera_frame = self.get_parameter(
            'camera_frame'
        ).value

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        pose = self.read_hloc_result(
            Path(self.result_file),
            self.query_name
        )

        if pose is None:
            raise RuntimeError(
                f'{self.query_name} not found'
            )

        self.map_to_camera = (
            self.colmap_pose_to_ros(*pose)
        )

        self.ready = False

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'Waiting for '
            'base_link -> camera_optical_frame TF...'
        )

    @staticmethod
    def read_hloc_result(path, query_name):

        with path.open('r') as f:

            for line in f:

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
                    return tuple(
                        map(float, values[1:])
                    )

        return None

    @staticmethod
    def colmap_pose_to_ros(
        qw, qx, qy, qz,
        tx, ty, tz
    ):
        """
        COLMAP/HLoc:
          X_cam = R_cw X_world + t_cw

        欲しいもの:
          map -> camera_optical_frame
        """

        q_cw = quat_normalize(
            (qx, qy, qz, qw)
        )

        # world <- camera
        q_wc = quat_conjugate(q_cw)

        t_wc = rotate_vector(
            q_wc,
            (-tx, -ty, -tz)
        )

        return t_wc, q_wc


def timer_callback(self):

    try:
        # URDF:
        # base_link -> camera_optical_frame
        tf_base_camera = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            Time(),
            timeout=Duration(seconds=0.1)
        )

        # Wheel odometry:
        # odom -> base_link
        tf_odom_base = self.tf_buffer.lookup_transform(
            self.odom_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.1)
        )

    except Exception as e:
        if not self.ready:
            self.get_logger().info(
                f'Waiting for required TFs: {e}'
            )
        return

    # -----------------------------
    # base -> camera
    # -----------------------------

    t_base_camera = (
        tf_base_camera.transform.translation.x,
        tf_base_camera.transform.translation.y,
        tf_base_camera.transform.translation.z,
    )

    q_base_camera = (
        tf_base_camera.transform.rotation.x,
        tf_base_camera.transform.rotation.y,
        tf_base_camera.transform.rotation.z,
        tf_base_camera.transform.rotation.w,
    )

    # camera -> base
    t_camera_base, q_camera_base = invert_transform(
        t_base_camera,
        q_base_camera
    )

    # -----------------------------
    # map -> camera
    # HLocから取得済み
    # -----------------------------

    t_map_camera, q_map_camera = self.map_to_camera

    # -----------------------------
    # map -> base
    #
    # T_map_base =
    # T_map_camera * T_camera_base
    # -----------------------------

    t_map_base, q_map_base = compose_transform(
        t_map_camera,
        q_map_camera,
        t_camera_base,
        q_camera_base
    )

    # -----------------------------
    # odom -> base
    # -----------------------------

    t_odom_base = (
        tf_odom_base.transform.translation.x,
        tf_odom_base.transform.translation.y,
        tf_odom_base.transform.translation.z,
    )

    q_odom_base = (
        tf_odom_base.transform.rotation.x,
        tf_odom_base.transform.rotation.y,
        tf_odom_base.transform.rotation.z,
        tf_odom_base.transform.rotation.w,
    )

    # base -> odom
    t_base_odom, q_base_odom = invert_transform(
        t_odom_base,
        q_odom_base
    )

    # -----------------------------
    # map -> odom
    #
    # T_map_odom =
    # T_map_base * T_base_odom
    # -----------------------------

    t_map_odom, q_map_odom = compose_transform(
        t_map_base,
        q_map_base,
        t_base_odom,
        q_base_odom
    )

    # -----------------------------
    # map -> odom をpublish
    # -----------------------------

    msg = TransformStamped()

    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = self.map_frame
    msg.child_frame_id = self.odom_frame

    msg.transform.translation.x = t_map_odom[0]
    msg.transform.translation.y = t_map_odom[1]
    msg.transform.translation.z = t_map_odom[2]

    msg.transform.rotation.x = q_map_odom[0]
    msg.transform.rotation.y = q_map_odom[1]
    msg.transform.rotation.z = q_map_odom[2]
    msg.transform.rotation.w = q_map_odom[3]

    self.tf_broadcaster.sendTransform(msg)

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

    rclpy.init(args=args)

    node = HlocTfPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()