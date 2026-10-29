"""Explicit auxiliary recording plan and finalized rosbag2 inventory."""
from pathlib import Path
import re

import yaml


DEFAULT_EXTRA_TOPICS = '/odom /imu/data /imu/data_raw /gps/fix'


def validate_topic(topic):
    if not re.fullmatch(r'/(?:[A-Za-z_]\w*/)*[A-Za-z_]\w*', topic, flags=re.ASCII):
        raise ValueError(f'bag topics must be absolute ROS topic names: {topic}')
    return topic


def recording_topics(include_images=False, extra_topics=DEFAULT_EXTRA_TOPICS,
                     image_topic='/camera/image_raw', camera_info_topic='/camera/camera_info',
                     use_sim_time=False):
    validate_topic(image_topic)
    topics = [camera_info_topic, '/tf', '/tf_static']
    if include_images:
        topics.append(image_topic)
    if use_sim_time:
        topics.append('/clock')
    topics.extend(extra_topics.split())
    for topic in topics:
        validate_topic(topic)
    return list(dict.fromkeys(topics))


def inspect_bag(directory, requested_topics=()):
    directory = Path(directory)
    metadata = yaml.safe_load((directory / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    files = metadata['relative_file_paths']
    if not files or not all((directory / name).is_file() for name in files):
        raise ValueError('bag storage file is missing')
    topics = {entry['topic_metadata']['name']: {
        'type': entry['topic_metadata']['type'], 'message_count': entry['message_count']}
        for entry in metadata['topics_with_message_count']}
    if metadata['message_count'] <= 0:
        raise ValueError('bag contains no messages')
    return dict(status='finalized', storage_identifier=metadata['storage_identifier'],
                message_count=metadata['message_count'], files=files, topics=topics,
                missing_or_empty_topics=[name for name in requested_topics
                    if topics.get(name, {}).get('message_count', 0) == 0])
