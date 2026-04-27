# Project Overview

## Purpose

`RacingBrain` is a ROS 2 Humble workspace for GNSS/INS-aided real-time cone
mapping with a reliability-aware perception stack beside it. The mapping node
consumes GNSS/INS and fused cone detections, then maintains stable, candidate,
and rejected-observation map layers before exposing a sparse track graph to the
planning interface.

## Packages

- `slam/drd25_msgs`: custom message definitions used by perception, planning, and
  mapping interfaces.
- `slam/slam`: C++ ROS 2 node package that builds `slam_node`.
- `gnss/gnss_ins_msg`: GNSS/INS custom messages used by mapping and rosbag replay.
- `gnss/cpp_pubsub`: optional serial GNSS/INS bridge that publishes live
  `/gongji_gnss_ins_64`, `/imu`, and `/body_velocity` topics from `/dev/ttyUSB0`.
- `perception/src/cone_ws/src/cone_interfaces`: camera cone detection messages.
- `perception/src/cone_ws/src/cone_detector`: YOLOv8 camera cone detector.
- `perception/src/cone_segmentation_test_3d/src/test_cone_segmentation`: LiDAR
  point-cloud cone segmentation.
- `perception/src/fs_fusion_box`: LiDAR-camera fusion node that publishes
  `drd25_msgs/Map` on `/perception/fusion/map`.
- `perception/src/run_perception`: launch package that starts camera, LiDAR, and
  fusion nodes together.
- `RacingBrain`: top-level launch, health monitor, LiDAR backend arbiter, and
  optional sparse track graph builder.

## Main Node

`slam_node` creates the `slam_processor` node. It:

- Converts GNSS latitude/longitude from WGS84 to a configured UTM CRS with PROJ.
- Uses the first valid GNSS position as the local map origin.
- Publishes vehicle TF, odometry, and path from high-rate GNSS/INS.
- Synchronizes GNSS/INS with perceived cones for map updates.
- Maintains global cones with color gating, Mahalanobis association, a Kalman
  style position update, and existence scoring.
- Publishes stable cones as RViz mesh markers and candidate/rejected layers for
  diagnostics.
- Publishes optional planning-facing track graph markers when the planning
  interface is enabled.

## Track Modes

- `acceleration`: straight-line mapping mode; this is the default launch mode.
- `autocross`: closed-track mode with loop-closure detection and map locking
  after the first completed lap.
- `skidpad`: configured as a mode, but the processing function is currently a
  TODO and does not implement mapping behavior.

## Important Limits

- `vision_msgs` and CSF are workspace-local dependencies under `.ros_deps/`.
  Use `source scripts/activate_ros_ml.sh` before building or running so those
  prefixes are visible.
- A dedicated ROS-compatible ML venv is now prepared at `./.venv_ros_ml`; Python
  YOLO runtime packages are no longer a blocker.
- No rosbag or offline end-to-end dataset is included. The imported perception
  code includes training/data scripts, and this workspace is configured to use
  external model/data paths from `config/hardcoded_paths.ini`.
