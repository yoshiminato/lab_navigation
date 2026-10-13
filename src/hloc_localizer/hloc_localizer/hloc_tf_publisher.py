#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
)

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
)


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


def invert_transform(t, q):

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

    return (
        t_inv,
        q_inv,
    )


def compose_transform(
    t1,
    q1,
    t2,
    q2,
):

    rotated_t2 = rotate_vector(
        q1,
        t2,
    )

    t = (
        t1[0]
        + rotated_t2[0],

        t1[1]
        + rotated_t2[1],

        t1[2]
        + rotated_t2[2],
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

        self.declare_parameter(
            'pose_topic',
            '/hloc/camera_pose',
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

        self.pose_topic = (
            self.get_parameter(
                'pose_topic'
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

        self.pose_sub = (
            self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_callback,
                pose_qos,
            )
        )

        self.pending_pose = None

        self.last_processed_stamp = None

        self.map_to_odom = None

        self.timer = self.create_timer(
            0.05,
            self.timer_callback,
        )

        self.get_logger().info(
            'Waiting for HLoc pose: '
            f'{self.pose_topic}'
        )

    def pose_callback(
        self,
        msg,
    ):

        self.pending_pose = msg

    def process_pose(
        self,
        pose,
    ):

        pose_time = Time.from_msg(
            pose.header.stamp
        )

        try:

            # base -> camera
            tf_base_camera = (
                self.tf_buffer
                .lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    Time(),
                    timeout=Duration(
                        seconds=0.2
                    ),
                )
            )

            # odom -> base at image timestamp
            tf_odom_base = (
                self.tf_buffer
                .lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    pose_time,
                    timeout=Duration(
                        seconds=0.2
                    ),
                )
            )

        except Exception as error:

            self.get_logger().warning(
                'Cannot obtain required TF: '
                f'{error}'
            )

            return False

        # ----------------------------------------------------
        # base -> camera
        # ----------------------------------------------------

        t_base_camera = (
            tf_base_camera
            .transform
            .translation.x,

            tf_base_camera
            .transform
            .translation.y,

            tf_base_camera
            .transform
            .translation.z,
        )

        q_base_camera = (
            tf_base_camera
            .transform
            .rotation.x,

            tf_base_camera
            .transform
            .rotation.y,

            tf_base_camera
            .transform
            .rotation.z,

            tf_base_camera
            .transform
            .rotation.w,
        )

        # camera -> base
        (
            t_camera_base,
            q_camera_base,
        ) = invert_transform(
            t_base_camera,
            q_base_camera,
        )

        # ----------------------------------------------------
        # map -> camera from HLoc
        # ----------------------------------------------------

        t_map_camera = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )

        q_map_camera = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )

        # ----------------------------------------------------
        # map -> base
        # ----------------------------------------------------

        (
            t_map_base,
            q_map_base,
        ) = compose_transform(
            t_map_camera,
            q_map_camera,
            t_camera_base,
            q_camera_base,
        )

        # ----------------------------------------------------
        # odom -> base
        # ----------------------------------------------------

        t_odom_base = (
            tf_odom_base
            .transform
            .translation.x,

            tf_odom_base
            .transform
            .translation.y,

            tf_odom_base
            .transform
            .translation.z,
        )

        q_odom_base = (
            tf_odom_base
            .transform
            .rotation.x,

            tf_odom_base
            .transform
            .rotation.y,

            tf_odom_base
            .transform
            .rotation.z,

            tf_odom_base
            .transform
            .rotation.w,
        )

        # base -> odom
        (
            t_base_odom,
            q_base_odom,
        ) = invert_transform(
            t_odom_base,
            q_odom_base,
        )

        # ----------------------------------------------------
        # map -> odom
        # ----------------------------------------------------

        (
            t_map_odom,
            q_map_odom,
        ) = compose_transform(
            t_map_base,
            q_map_base,
            t_base_odom,
            q_base_odom,
        )

        self.map_to_odom = (
            t_map_odom,
            q_map_odom,
        )

        self.get_logger().info(
            '\n'
            'HLoc TF updated\n'
            f'map -> camera: '
            f'{t_map_camera[0]:.3f}, '
            f'{t_map_camera[1]:.3f}, '
            f'{t_map_camera[2]:.3f}\n'
            f'map -> base:   '
            f'{t_map_base[0]:.3f}, '
            f'{t_map_base[1]:.3f}, '
            f'{t_map_base[2]:.3f}\n'
            f'map -> odom:   '
            f'{t_map_odom[0]:.3f}, '
            f'{t_map_odom[1]:.3f}, '
            f'{t_map_odom[2]:.3f}'
        )

        return True

    def publish_map_to_odom(self):

        if self.map_to_odom is None:
            return

        (
            t,
            q,
        ) = self.map_to_odom

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

        msg.transform.translation.x = t[0]
        msg.transform.translation.y = t[1]
        msg.transform.translation.z = t[2]

        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(
            msg
        )

    def timer_callback(self):

        if self.pending_pose is not None:

            stamp = (
                self.pending_pose
                .header
                .stamp
            )

            stamp_key = (
                stamp.sec,
                stamp.nanosec,
            )

            if (
                stamp_key
                != self.last_processed_stamp
            ):

                if self.process_pose(
                    self.pending_pose
                ):

                    self.last_processed_stamp = (
                        stamp_key
                    )

        self.publish_map_to_odom()


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