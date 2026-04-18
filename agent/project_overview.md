# Project Overview

## Purpose

`DRd26_SLAM` is a ROS 2 Humble workspace for cone-map SLAM / mapping, now with
an imported perception stack beside it. SLAM consumes GNSS/INS and fused cone
detections, then maintains and visualizes a stable global cone map.

## Packages

- `slam/drd25_msgs`: custom message definitions used by perception, planning, and
  mapping interfaces.
- `slam/slam`: C++ ROS 2 node package that builds `slam_node`.
- `perception/src/cone_ws/src/cone_interfaces`: camera cone detection messages.
- `perception/src/cone_ws/src/cone_detector`: YOLOv8 camera cone detector.
- `perception/src/cone_segmentation_test_3d/src/test_cone_segmentation`: LiDAR
  point-cloud cone segmentation.
- `perception/src/fs_fusion_box`: LiDAR-camera fusion node that publishes
  `drd25_msgs/Map` on `/perception/fusion/map`.
- `perception/src/run_perception`: launch package that starts camera, LiDAR, and
  fusion nodes together.

## Main Node

`slam_node` creates the `slam_processor` node. It:

- Converts GNSS latitude/longitude from WGS84 to a configured UTM CRS with PROJ.
- Uses the first valid GNSS position as the local map origin.
- Publishes vehicle TF, odometry, and path from high-rate GNSS/INS.
- Synchronizes GNSS/INS with perceived cones for map updates.
- Maintains global cones with color gating, Mahalanobis association, a Kalman
  style position update, and existence scoring.
- Publishes stable cones as RViz mesh markers.

## Track Modes

- `acceleration`: straight-line mapping mode; this is the default launch mode.
- `autocross`: closed-track mode with loop-closure detection and map locking
  after the first completed lap.
- `skidpad`: configured as a mode, but the processing function is currently a
  TODO and does not implement mapping behavior.

## Important Limits

- The repository currently lacks the external `gnss_ins_msg` package needed by
  `slam`.
- Full perception build currently also needs CSF, `vision_msgs`, and Python YOLO
  runtime packages.
- No rosbag or offline end-to-end dataset is included. The imported perception
  code includes training/data scripts and local model artifacts, but large
  datasets and model weights are intentionally treated as local assets.
