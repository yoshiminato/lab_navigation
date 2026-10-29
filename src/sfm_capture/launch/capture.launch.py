"""Capture USB, existing ROS, or Gazebo images with the same dataset contract."""
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
from sfm_capture.bag_support import DEFAULT_EXTRA_TOPICS, inspect_bag, recording_topics
from sfm_capture.dataset import write_json


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    source = value('source').lower()
    if source not in ('usb', 'topic', 'simulation'):
        raise ValueError('source must be usb, topic, or simulation')
    sim_option = value('use_sim_time').lower()
    if sim_option not in ('auto', 'true', 'false'):
        raise ValueError('use_sim_time must be auto, true, or false')
    use_sim_time = source == 'simulation' if sim_option == 'auto' else sim_option == 'true'
    if source == 'usb' and use_sim_time:
        raise ValueError('USB camera timestamps require use_sim_time:=false')
    device = None
    if source == 'usb':
        device = Path(value('device')).expanduser().resolve()
        if not device.exists():
            raise RuntimeError(f'Camera does not exist: {device}. Check USB and /dev/v4l/by-id/.')
    output = Path(value('output_dir')).expanduser().resolve()
    config = Path(value('camera_config')).expanduser().resolve()
    calibration = value('calibration')
    if source != 'usb' and calibration:
        raise ValueError('For topic/simulation sources, provide calibration on camera_info_topic')
    for name in ('record_bag', 'bag_images'):
        if value(name).lower() not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
    if value('bag_images').lower() == 'true' and value('record_bag').lower() != 'true':
        raise ValueError('bag_images:=true requires record_bag:=true')
    timeout_option = value('no_frame_timeout_sec')
    timeout = (0.0 if use_sim_time else 15.0) if timeout_option == 'auto' else float(timeout_option)
    for name in ('save_fps', 'warmup_sec', 'duration_sec'):
        number = float(value(name))
        if not math.isfinite(number) or number < 0 or (name == 'save_fps' and number == 0):
            raise ValueError(f'Invalid {name}: {number}')
    if not math.isfinite(timeout) or timeout < 0:
        raise ValueError('no_frame_timeout_sec must be auto or finite and nonnegative')
    queue_size = int(value('writer_queue_size'))
    if queue_size < 1:
        raise ValueError('writer_queue_size must be positive')
    max_bag_size = int(value('bag_max_size'))
    if max_bag_size != 0 and max_bag_size < 86016:
        raise ValueError('bag_max_size must be 0 or at least 86016 bytes')
    topics = recording_topics(value('bag_images').lower() == 'true', value('extra_bag_topics'),
                              value('image_topic'), value('camera_info_topic'), use_sim_time)
    qos = Path(get_package_share_directory('sfm_capture')) / 'config' / 'bag_qos.yaml'
    # Explicit nonexistent URL avoids accidentally loading an old default calibration.
    camera_info_url = 'file:///nonexistent/sfm_capture_uncalibrated.yaml'
    if calibration:
        calibration_path = Path(calibration).expanduser().resolve()
        if not calibration_path.is_file():
            raise RuntimeError(f'Calibration file does not exist: {calibration_path}')
        camera_info_url = calibration_path.as_uri()
    settings = None
    if source == 'usb':
        settings = yaml.safe_load(config.read_text())['/**']['ros__parameters']
        settings.update(video_device=str(device), camera_info_url=camera_info_url)
    output.mkdir(parents=True, exist_ok=False)
    if settings is not None:
        (output / 'camera_parameters.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}))
    if calibration:
        shutil.copy2(calibration_path, output / 'calibration.yaml')
    info = {'created_utc': datetime.now(timezone.utc).isoformat(),
            'source': source, 'use_sim_time': use_sim_time,
            'image_topic': value('image_topic'), 'camera_info_topic': value('camera_info_topic'),
            'device_requested': value('device') if device else None,
            'device_resolved': str(device) if device else None,
            'camera_config_source': str(config) if settings is not None else None,
            'record_bag': value('record_bag'),
            'bag_images': value('bag_images'),
            'bag_topics': topics if value('record_bag').lower() == 'true' else [],
            'command_arguments': {name: value(name) for name in
                ['save_fps', 'warmup_sec', 'duration_sec', 'extra_bag_topics',
                 'writer_queue_size', 'no_frame_timeout_sec', 'bag_max_size']}}
    (output / 'capture_setup.json').write_text(json.dumps(info, ensure_ascii=False, indent=2) + '\n')
    if device is not None and shutil.which('v4l2-ctl'):
        result = subprocess.run(['v4l2-ctl', '-d', str(device), '--all'],
                                capture_output=True, text=True, timeout=5)
        (output / 'camera_controls_before_start.txt').write_text(result.stdout + result.stderr)

    recorder = Node(package='sfm_capture', executable='recorder', output='screen',
                    sigterm_timeout='15', parameters=[{
        'output_dir': str(output), 'device': str(device) if device else '',
        'image_topic': value('image_topic'), 'camera_info_topic': value('camera_info_topic'),
        'use_sim_time': use_sim_time,
        'save_fps': float(value('save_fps')), 'warmup_sec': float(value('warmup_sec')),
        'duration_sec': float(value('duration_sec')),
        'no_frame_timeout_sec': timeout,
        'writer_queue_size': queue_size,
    }])
    actions = []
    processes = [('recorder', recorder)]
    if source == 'usb':
        camera = Node(package='usb_cam', executable='usb_cam_node_exe', namespace='camera',
                      name='usb_cam', parameters=[settings], output='screen',
                      remappings=[('image_raw', value('image_topic')),
                                  ('camera_info', value('camera_info_topic'))])
        processes.append(('camera', camera))
    if value('record_bag').lower() == 'true':
        shutil.copy2(qos, output / 'bag_qos.yaml')
        clock_options = ['--use-sim-time'] if use_sim_time else []
        bag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', str(output / 'bag'),
                                  '--storage', 'sqlite3', '--storage-preset-profile', 'resilient',
                                  '--max-bag-size', str(max_bag_size),
                                  '--qos-profile-overrides-path', str(output / 'bag_qos.yaml'),
                                  *clock_options, *topics], output='screen', sigterm_timeout='30')
        processes.insert(0, ('bag', bag))
    stopping = False

    def exited(event, context, role):
        nonlocal stopping
        if role == 'bag':
            try:
                report = inspect_bag(output / 'bag', topics)
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
                report = dict(status='failed', error=str(error))
            if event.returncode != 0:
                report.update(status='failed', error=f'bag process exited with code {event.returncode}')
            write_json(output / 'bag_report.json', dict(report, returncode=event.returncode))
        if stopping or context.is_shutdown:
            return []
        stopping = True
        if role != 'recorder' or event.returncode != 0:
            (output / 'capture_failure.json').write_text(json.dumps({
                'process': role, 'returncode': event.returncode,
                'error': f'{role} exited unexpectedly (code {event.returncode})',
            }, indent=2) + '\n')
        return [EmitEvent(event=Shutdown(reason=f'{role} exited'))]

    for role, process in processes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=lambda event, context, role=role: exited(event, context, role))))
    # Subscriptions first; warmup excludes camera initialization frames.
    return actions + [process for role, process in processes]


def generate_launch_description():
    default_dir = str(Path.cwd() / 'captures' / datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    config = str(Path(get_package_share_directory('sfm_capture')) / 'config' / 'camera.yaml')
    defaults = dict(source='usb', use_sim_time='auto',
                    image_topic='/camera/image_raw', camera_info_topic='/camera/camera_info',
                    output_dir=default_dir, device='/dev/video0', camera_config=config,
                    save_fps='2.0', warmup_sec='2.0', duration_sec='0.0', calibration='',
                    record_bag='true', bag_images='false', extra_bag_topics=DEFAULT_EXTRA_TOPICS,
                    writer_queue_size='4', no_frame_timeout_sec='auto', bag_max_size='2147483648')
    descriptions = dict(
        source='usb: start usb_cam; topic: existing ROS images; simulation: existing images with /clock',
        use_sim_time='auto selects simulation time only for source:=simulation; or true/false',
        image_topic='Absolute sensor_msgs/msg/Image topic',
        camera_info_topic='Absolute sensor_msgs/msg/CameraInfo topic',
        output_dir='New capture session directory (must not exist)',
        device='V4L2 camera device or /dev/v4l/by-id path',
        camera_config='usb_cam parameter YAML',
        save_fps='Maximum PNG selection rate, using source timestamps',
        warmup_sec='Warmup in seconds of the selected clock before saving images',
        duration_sec='Duration in seconds of the selected clock after warmup; 0 until Ctrl+C',
        calibration='ROS calibration YAML for this resolution/focus, or empty',
        record_bag='Record auxiliary topics alongside PNGs',
        bag_images='Also record all raw ROS images; high storage/bandwidth cost',
        extra_bag_topics='Space-separated absolute sensor topics (replaces defaults)',
        writer_queue_size='Bounded PNG queue; overflow fails the session',
        no_frame_timeout_sec='Wall-time frame timeout; auto: 15s for real sources, disabled in simulation',
        bag_max_size='Split bag at this many bytes; 0 disables splitting')
    return LaunchDescription([DeclareLaunchArgument(name, default_value=val, description=descriptions[name])
                              for name, val in defaults.items()] + [OpaqueFunction(function=start)])
