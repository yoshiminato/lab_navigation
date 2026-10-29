#!/usr/bin/env python3

import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageFilePublisher(Node):

    def __init__(self):

        super().__init__(
            'image_file_publisher'
        )

        self.declare_parameter(
            'image_path',
            '',
        )

        self.declare_parameter(
            'topic',
            '/camera/image_raw',
        )

        self.declare_parameter(
            'frame_id',
            'camera_optical_frame',
        )

        self.declare_parameter(
            'publish_rate',
            0.0,
        )

        self.image_path = (
            self.get_parameter(
                'image_path'
            ).value
        )

        self.topic = (
            self.get_parameter(
                'topic'
            ).value
        )

        self.frame_id = (
            self.get_parameter(
                'frame_id'
            ).value
        )

        self.publish_rate = float(
            self.get_parameter(
                'publish_rate'
            ).value
        )

        if not self.image_path:

            raise RuntimeError(
                'Parameter "image_path" '
                'is empty.'
            )

        self.image = cv2.imread(
            self.image_path,
            cv2.IMREAD_COLOR,
        )

        if self.image is None:

            raise RuntimeError(
                'Failed to load image: '
                f'{self.image_path}'
            )

        self.bridge = CvBridge()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
            ),
            durability=(
                DurabilityPolicy.VOLATILE
            ),
        )

        self.publisher = (
            self.create_publisher(
                Image,
                self.topic,
                qos,
            )
        )

        self.published_once = False

        if self.publish_rate > 0.0:

            self.timer = (
                self.create_timer(
                    1.0 /
                    self.publish_rate,
                    self.timer_callback,
                )
            )

            self.get_logger().info(
                'Publishing repeatedly at '
                f'{self.publish_rate:.2f} Hz: '
                f'{self.image_path}'
            )

        else:

            self.timer = (
                self.create_timer(
                    0.2,
                    self.publish_once,
                )
            )

            self.get_logger().info(
                'Waiting for subscriber '
                'before one-shot publish...'
            )

    def make_message(self):

        msg = (
            self.bridge
            .cv2_to_imgmsg(
                self.image,
                encoding='bgr8',
            )
        )

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            self.frame_id
        )

        return msg

    def publish_once(self):

        if self.published_once:
            return

        if (
            self.publisher
            .get_subscription_count()
            == 0
        ):
            return

        self.publisher.publish(
            self.make_message()
        )

        self.published_once = True

        self.get_logger().info(
            'Published image once to '
            f'{self.topic}'
        )

    def timer_callback(self):

        if (
            self.publisher
            .get_subscription_count()
            == 0
        ):
            return

        self.publisher.publish(
            self.make_message()
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ImageFilePublisher()

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