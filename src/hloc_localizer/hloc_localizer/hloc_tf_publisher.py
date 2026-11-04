#!/usr/bin/env python3
"""HLOC (Isaac world) -> GLIM map -> odom のTFを発行するROS 2ノード。

座標変換:
    T_map_odom = T_map_world * T_world_camera * T_camera_base * T_base_odom

注意:
- HLOCが発行する /hloc/camera_pose の数値はworld座標として扱う。
  frame_id が従来の 'map' でも、補正適用前の数値がworld座標である想定。
- world->map の補正値は暫定値。基準点の対応から校正すること。
- /odom と odom->base_link がないとTFは発行できない。
- map->odom の配信元は必ずこのノード1つにする。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def quat_normalize(q):
    n = math.sqrt(sum(x * x for x in q))
    if n < 1e-12:
        raise ValueError('Zero-length quaternion')
    return tuple(x / n for x in q)


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
    rotated = quat_multiply(
        quat_multiply(q, (v[0], v[1], v[2], 0.0)),
        quat_conjugate(q),
    )
    return rotated[:3]


def invert_transform(t, q):
    q_inv = quat_conjugate(quat_normalize(q))
    t_inv = rotate_vector(q_inv, tuple(-x for x in t))
    return t_inv, q_inv


def compose_transform(t1, q1, t2, q2):
    t2_rotated = rotate_vector(q1, t2)
    t = tuple(a + b for a, b in zip(t1, t2_rotated))
    q = quat_normalize(quat_multiply(q1, q2))
    return t, q


def read_transform(tf):
    p = tf.transform.translation
    r = tf.transform.rotation
    return (p.x, p.y, p.z), (r.x, r.y, r.z, r.w)


class HlocTfPublisher(Node):
    def __init__(self):
        super().__init__('hloc_tf_publisher')

        self.declare_parameter('pose_topic', '/hloc/camera_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')

        # map (= GLIM) <- world (= Isaac Sim) の固定変換。
        # 初期値は無補正。試験時だけ launch から推定値を渡す。
        self.declare_parameter('world_to_map_x', 0.0)
        self.declare_parameter('world_to_map_y', 0.0)
        self.declare_parameter('world_to_map_z', 0.0)
        self.declare_parameter('world_to_map_yaw_deg', 0.0)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.t_map_world = (
            float(self.get_parameter('world_to_map_x').value),
            float(self.get_parameter('world_to_map_y').value),
            float(self.get_parameter('world_to_map_z').value),
        )
        yaw = math.radians(
            float(self.get_parameter('world_to_map_yaw_deg').value)
        )
        self.q_map_world = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, pose_qos
        )
        self.pending_pose = None
        self.last_processed_stamp = None
        self.map_to_odom = None
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'Waiting for HLOC pose: {self.pose_topic}; '
            f'T_map_world translation={self.t_map_world}, '
            f'yaw_deg={math.degrees(yaw):.3f}'
        )

    def pose_callback(self, msg):
        self.pending_pose = msg

    def process_pose(self, pose):
        pose_time = Time.from_msg(pose.header.stamp)
        try:
            # カメラ取り付け変換（固定）
            tf_base_camera = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, Time(),
                timeout=Duration(seconds=0.2),
            )
            # 撮影時刻の車輪オドメトリ（動的）
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, pose_time,
                timeout=Duration(seconds=0.2),
            )
        except Exception as error:
            self.get_logger().warning(f'Cannot obtain required TF: {error}')
            return False

        t_base_camera, q_base_camera = read_transform(tf_base_camera)
        t_camera_base, q_camera_base = invert_transform(
            t_base_camera, q_base_camera
        )

        # HLOCの出力値は Isaac Sim world 上のカメラ姿勢と解釈する。
        t_world_camera = (
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        )
        r = pose.pose.orientation
        q_world_camera = quat_normalize((r.x, r.y, r.z, r.w))

        # ★追加: world -> GLIM map の剛体変換をここで適用する。
        t_map_camera, q_map_camera = compose_transform(
            self.t_map_world, self.q_map_world,
            t_world_camera, q_world_camera,
        )

        t_map_base, q_map_base = compose_transform(
            t_map_camera, q_map_camera,
            t_camera_base, q_camera_base,
        )

        t_odom_base, q_odom_base = read_transform(tf_odom_base)
        t_base_odom, q_base_odom = invert_transform(t_odom_base, q_odom_base)
        t_map_odom, q_map_odom = compose_transform(
            t_map_base, q_map_base,
            t_base_odom, q_base_odom,
        )

        self.map_to_odom = (t_map_odom, q_map_odom)
        self.get_logger().info(
            'HLOC TF updated\n'
            f'world -> camera: {tuple(round(x, 3) for x in t_world_camera)}\n'
            f'map -> camera:   {tuple(round(x, 3) for x in t_map_camera)}\n'
            f'map -> base:     {tuple(round(x, 3) for x in t_map_base)}\n'
            f'map -> odom:     {tuple(round(x, 3) for x in t_map_odom)}'
        )
        return True

    def publish_map_to_odom(self):
        if self.map_to_odom is None:
            return

        t, q = self.map_to_odom
        msg = TransformStamped()
        # 既存実装と同じく最新のシミュレーション時刻で再送する。
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = t[0]
        msg.transform.translation.y = t[1]
        msg.transform.translation.z = t[2]
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(msg)

    def timer_callback(self):
        if self.pending_pose is not None:
            s = self.pending_pose.header.stamp
            stamp_key = (s.sec, s.nanosec)
            if stamp_key != self.last_processed_stamp:
                if self.process_pose(self.pending_pose):
                    self.last_processed_stamp = stamp_key
        self.publish_map_to_odom()


def main(args=None):
    rclpy.init(args=args)
    node = HlocTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
