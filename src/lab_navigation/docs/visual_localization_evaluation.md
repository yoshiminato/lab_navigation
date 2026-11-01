# Visual Localization evaluation with LiDAR reference

This procedure runs Nav2 using LiDAR Localization while HLoc estimates poses
from the camera stream in parallel. HLoc does not publish TF or control the
robot in this experiment. LiDAR Localization remains the reference estimate
and the only localization source used by Nav2.

## Build and launch

Build the updated packages and source the workspace:

```bash
cd /home/user/lab_navigation_ws
colcon build --symlink-install --packages-select lab_navigation hloc_localizer lidar_localization_ros2_custom
source install/setup.bash
```

Then start the evaluation launch:

```bash
ros2 launch lab_navigation visual_localization_evaluation.launch.py \
  bag_output_dir:=/home/user/lab_navigation_ws/captures/hv_building_lab/evaluation/night_run_01
```

Use RViz to send the same ordered Nav2 goals on each run. The launch does not
invent a route or send goals automatically because no route coordinates are
defined in this package. Use the fixed waypoints chosen for the experiment and
record those coordinates and the lighting condition with the run.

The launch records these topics in a SQLite rosbag:

- `/pcl_pose`: LiDAR Localization `PoseWithCovarianceStamped`
- `/hloc/camera_pose`: successful HLoc camera poses
- `/hloc/localization_status`: one JSON status per image processed by HLoc
- `/tf` and `/tf_static`: robot and sensor transforms
- `/odom`, `/cmd_vel`, and `/cmd_vel_slow`: motion context

Camera images are not recorded by default to avoid very large bags. The
timestamped status topic preserves the count of frames HLoc actually
processed, including failures.

## Confirm the map alignment before calculating errors

The HLoc pose is computed in the SfM model's coordinate system. The message's
`frame_id` is configured as `map`, but that label does not itself align the
SfM model to the LiDAR map. The analyzer therefore refuses to calculate errors
until the fixed alignment has been independently confirmed.

Edit `params/visual_localization_evaluation.yaml` after confirming or
estimating the SfM-to-LiDAR map similarity transform from independent control
points. Set `map_alignment.verified` to `true` and fill in the scale,
translation, and quaternion. Do not estimate the alignment from the same
evaluation trajectory, as that would reduce the measured error by construction.

The analyzer composes the static TF chain from `base_link` to
`camera_optical_frame` in `/tf_static`, converts each HLoc camera pose to a
`base_link` pose, then interpolates the LiDAR trajectory to the HLoc image
timestamp. It only pairs poses bracketed by LiDAR samples no more than the
configured `max_lidar_bracket_gap_sec` apart.

## Analyze a run

```bash
ros2 run lab_navigation evaluate_visual_localization_bag \
  /home/user/lab_navigation_ws/captures/hv_building_lab/evaluation/night_run_01 \
  --config /home/user/lab_navigation_ws/src/lab_navigation/params/visual_localization_evaluation.yaml
```

The command writes `pose_errors.csv` and `summary.json` into the bag directory.
The summary includes HLoc's processed-frame success rate, the number of poses
that could be matched to the LiDAR trajectory, and mean, RMSE, median, 95th
percentile, and maximum errors for XY position, 3D position, and yaw.

If status messages show that every processed image failed, the analyzer still
reports the success rate and failure reasons with pose-error metrics set to
`null`. In this case, LiDAR poses, camera TF, and verified map alignment are
not needed. For successful statuses, their image timestamps must match the
timestamps of the recorded HLoc camera poses; a mismatch stops analysis and
indicates inconsistent or incomplete recording. If the bag has no HLoc status
messages, analysis stops because the success rate cannot be determined.

LiDAR Localization is a reference estimate here, not ground truth. Its
reported covariance is not used by this analysis.
