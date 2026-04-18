# Perception Overview

## What Is Present

The imported perception tree contains a ROS 2 perception chain rather than only
loose reference code:

- `cone_detector`: Python YOLOv8 camera detector. It subscribes to an image
  topic, defaults to `/camera/image_raw`, and the integration launch sets
  `/camera1/image_raw`. It publishes `/yolo/cones` and `/yolo/debug_image`.
- `cone_interfaces`: custom 2D cone message package used by `cone_detector`.
- `test_cone_segmentation`: C++ LiDAR point-cloud cone segmentation package. It
  subscribes to `/lidar_points` and publishes `/cone_detection_custom`.
- `fs_fusion_box`: C++ LiDAR-camera fusion package. It subscribes to the LiDAR
  and camera cone outputs, converts them into internal `vision_msgs` shapes,
  fuses by projection/matching, and publishes `drd25_msgs/Map`.
- `run_perception`: launch package that starts YOLO, LiDAR segmentation, and
  fusion together.

## SLAM Interface

The fusion launch remaps `fusion/cones` to `/perception/fusion/map`. This is the
topic consumed by `slam_node` through `system.lidar_topic`, so the perception
output type and topic are designed to connect to SLAM.

Use the workspace `slam/drd25_msgs` package as the single source of truth for
`drd25_msgs`. The imported `perception/src/drd25_msgs` has a conflicting
`Cone.msg` color enum and is disabled with `COLCON_IGNORE`.

## Runtime Notes

- `vision_msgs` is installed locally from the official ROS apt package under
  `.ros_deps/opt/ros/humble`.
- CSF is installed locally from the official CSF repository under
  `.ros_deps/csf`.
- `gnss_ins_msg` is provided by the workspace under `gnss/gnss_ins_msg`.
- The ML Python runtime lives in `.venv_ros_ml`; use
  `source scripts/activate_ros_ml.sh`.
- The default rosbag dataset is outside the repository at
  `/media/yupeng/新加卷/Datasets/rosbag2_2026_02_05-11_01_07`.
- Model weights are outside the repository under
  `/media/yupeng/新加卷/Models/DRd26_SLAM`.

## Expected Runtime Chain

Sensor inputs:

- `/camera1/image_raw` for YOLO.
- `/lidar_points` for LiDAR segmentation.
- `/gongji_gnss_ins_64` for SLAM localization.

Intermediate outputs:

- `/yolo/cones`
- `/cone_detection_custom`

Integration output:

- `/perception/fusion/map`

SLAM output:

- `/global_map`
- `vehicle_path`
- `vehicle_odom`
