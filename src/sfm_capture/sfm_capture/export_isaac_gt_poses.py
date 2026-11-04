"""Export Isaac Sim camera ground truth to the existing COLMAP pose-prior CSV."""
import argparse
import json
from pathlib import Path

from .export_poses import export_poses
from .pose_io import write_json


DEFAULT_GT_TOPIC = '/sfm_gt_tf'


def export_isaac_gt_poses(capture, output, camera_frame, world_frame='world',
                          tf_topic=DEFAULT_GT_TOPIC, position_std=(0.05, 0.05, 0.05),
                          max_tf_gap_sec=0.2, allow_missing=False):
    capture = Path(capture).resolve()
    setup = json.loads((capture / 'capture_setup.json').read_text())
    session = json.loads((capture / 'session.json').read_text())
    if setup.get('source') != 'simulation' or setup.get('use_sim_time') is not True:
        raise ValueError('Capture must use source:=simulation and use_sim_time:=true')
    if str(setup.get('record_bag')).lower() != 'true' or tf_topic not in setup.get('bag_topics', []):
        raise ValueError(f'Capture bag must record the dedicated ground-truth topic {tf_topic}')
    if session.get('status') != 'complete' or (capture / 'capture_failure.json').exists():
        raise ValueError('Capture session is not complete')
    if not camera_frame or camera_frame == world_frame or tf_topic == '/tf':
        raise ValueError('Specify a camera frame and a dedicated ground-truth TF topic')
    report = export_poses(
        capture, output, world_frame=world_frame, source_frame=camera_frame,
        source_is_camera=True, position_std=position_std,
        max_tf_gap_sec=max_tf_gap_sec, tf_topic=tf_topic,
        direct_tf_parent=world_frame)
    report['pose_source'] = 'isaac_sim_camera_prim_ground_truth'
    report['ground_truth_topic'] = tf_topic
    report['orientation_semantics'] = (
        'Quaternion uses the Isaac TF child frame axes; the COLMAP prior importer '
        'uses camera-center position only')
    report['valid_for_prior_import'] = bool(
        report['valid'] and (allow_missing or report['missing'] == 0))
    write_json(Path(output) / 'poses_report.json', report)
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('output', type=Path, help='New output directory')
    parser.add_argument('--camera-frame', required=True,
                        help='Child frame of the Isaac Sim camera prim ground-truth TF')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--tf-topic', default=DEFAULT_GT_TOPIC)
    parser.add_argument('--position-std', nargs=3, type=float, default=(0.05, 0.05, 0.05),
                        metavar=('SX', 'SY', 'SZ'),
                        help='World-axis position standard deviations in metres')
    parser.add_argument('--max-tf-gap-sec', type=float, default=0.2)
    parser.add_argument('--allow-missing', action='store_true')
    options = vars(parser.parse_args(args))
    try:
        report = export_isaac_gt_poses(**options)
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as error:
        parser.exit(1, f'FAIL: {error}\n')
    print(f"{report['poses_csv']}: {report['valid']}/{report['images']} valid; "
          f"{report['missing']} missing")
    if not report['valid_for_prior_import']:
        parser.exit(1, 'Pose failures are preserved in CSV/report; inspect them before importing.\n')


if __name__ == '__main__':
    main()
