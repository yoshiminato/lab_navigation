"""Export ROS Image frames without lossy encoding; verify source chronology."""
import argparse
from pathlib import Path

from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from rosidl_runtime_py.convert import message_to_ordereddict
import rosbag2_py

from .dataset import Dataset, write_json
from .camera_contract import CameraContract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/camera_info')
    parser.add_argument('--save-fps', type=float, default=2.0)
    parser.add_argument('--storage-id', default='sqlite3', help='Humble default sqlite3; MCAP requires plugin')
    options = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(options.bag), storage_id=options.storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    if types.get(options.image_topic) != 'sensor_msgs/msg/Image':
        parser.error('The selected image topic must exist and have type sensor_msgs/msg/Image')
    if (options.camera_info_topic in types and
            types[options.camera_info_topic] != 'sensor_msgs/msg/CameraInfo'):
        parser.error('The selected camera info topic must have type sensor_msgs/msg/CameraInfo')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[options.image_topic, options.camera_info_topic]))
    dataset = Dataset(options.output, options.save_fps,
                      dict(source='rosbag2', bag=str(options.bag.resolve()),
                           image_topic=options.image_topic,
                           received_ns_semantics='rosbag2 recording timestamp'))
    bridge = CvBridge()
    contract = CameraContract()
    info_saved = False
    try:
        while reader.has_next():
            topic, data, recorded_ns = reader.read_next()
            if topic == options.camera_info_topic:
                message = deserialize_message(data, CameraInfo)
                info = message_to_ordereddict(message)
                contract.observe_info(info)
                if not info_saved:
                    write_json(dataset.path / 'camera_info.json', info)
                    dataset.update(camera_calibrated=bool(message.k[0] != 0))
                    info_saved = True
                continue
            message = deserialize_message(data, Image)
            contract.observe_image(message.width, message.height, message.encoding,
                                   message.header.frame_id)
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if dataset.eligible(stamp):
                dataset.save(bridge.imgmsg_to_cv2(message, 'bgr8'), stamp, recorded_ns,
                             message.header.frame_id, message.encoding)
        if not dataset.stats['saved']:
            raise ValueError('No images found')
        dataset.update(camera_info_received=info_saved)
        dataset.close()
        print(f'{dataset.path}: {dataset.stats}')
    except BaseException as error:
        dataset.close('failed', str(error))
        raise
