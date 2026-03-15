# ros2_mono_vo

A monocular visual odometry ROS2 node that estimates the 6-DOF pose of a camera from a single image stream. Built on a Frame/Keyframe/Map architecture inspired by ORB-SLAM, using Lucas-Kanade optical flow tracking and PnP+RANSAC pose estimation.

Currently a pure VO front-end — no loop closure or bundle adjustment. The roadmap is to extend it into a full SLAM system.

> For a detailed breakdown of the algorithm, architecture, and design decisions, see the [project page](https://mro47.github.io/projects/ros2-mono-vo).

![mono_vo_viz](./images/mono_vo_viz.png)

## Features

- **6-DOF Pose Estimation** from a single camera stream
- **Real-time Visualization** — publishes trajectory, pose, and 3D map points to RViz2
- **ROS2 Native** — composable node, standard `sensor_msgs/Image` interface
- **Minimal Dependencies** — ROS2 and OpenCV only

## Prerequisites

- ROS2 Jazzy
- OpenCV
- A calibrated camera publishing `sensor_msgs/Image` and `sensor_msgs/CameraInfo`

## Building and Running

```bash
colcon build --packages-up-to mono_vo
source install/setup.bash
ros2 launch mono_vo mono_vo.launch.py
```

## Subscribed Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/camera/image_rect` | `sensor_msgs/Image` | Rectified camera image |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |

## Published Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/odom` | `nav_msgs/Odometry` | Estimated 6-DOF camera pose |
| `/path` | `nav_msgs/Path` | Full estimated trajectory |
| `/pointcloud` | `sensor_msgs/PointCloud2` | Triangulated 3D map points |

## Parameters

Loaded from `config/vo_params.yaml` at launch.

### Initializer

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `initializer.occupancy_grid_div` | `int` | `50` | Grid size (N) for keypoint distribution check |
| `initializer.kp_distribution_thresh` | `double` | `0.5` | Minimum ratio of occupied grid cells |
| `initializer.lowes_distance_ratio` | `double` | `0.7` | Lowe's ratio for feature matching |
| `initializer.min_matches_for_init` | `int` | `100` | Minimum matches to attempt initialization |
| `initializer.ransac_reproj_thresh` | `double` | `1.0` | RANSAC reprojection threshold (pixels) |
| `initializer.f_inlier_thresh` | `double` | `0.5` | Minimum inlier ratio for Fundamental matrix |
| `initializer.model_score_thresh` | `double` | `0.56` | H/F score ratio threshold for model selection |

### Tracker

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `tracker.tracking_error_thresh` | `double` | `30.0` | LK optical flow error threshold (pixels) |
| `tracker.min_observations_before_triangulation` | `int` | `100` | Keyframe trigger: minimum feature observations |
| `tracker.min_tracked_points` | `int` | `10` | Minimum tracked points before declaring lost |
| `tracker.max_tracking_after_keyframe` | `int` | `10` | Keyframe trigger: max frames since last keyframe |
| `tracker.max_rotation_from_keyframe` | `double` | `0.2618` | Keyframe trigger: max rotation (rad, ~15°) |
| `tracker.max_translation_from_keyframe` | `double` | `1.0` | Keyframe trigger: max translation (meters) |
| `tracker.ransac_reproj_thresh` | `double` | `1.0` | RANSAC reprojection threshold (pixels) |
| `tracker.model_score_thresh` | `double` | `0.85` | H/F score threshold for pose estimation |
| `tracker.f_inlier_thresh` | `double` | `0.5` | Minimum inlier ratio for Fundamental matrix |
| `tracker.lowes_distance_ratio` | `double` | `0.7` | Lowe's ratio for keyframe feature matching |
