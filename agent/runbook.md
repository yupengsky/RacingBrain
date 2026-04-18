# Runbook

## Clean ROS Build

Use the helper script from the workspace root:

```bash
./scripts/build_ros_clean.sh
```

The script strips conda from the build environment, sources ROS 2 Humble, checks
that `/usr/bin/python3` can import `em` and `numpy`, then runs:

```bash
colcon build --symlink-install --cmake-clean-cache
```

By default it builds from the curated package roots under `slam/` and
`perception/`, avoiding the imported duplicate `perception/src/drd25_msgs`.

Pass normal colcon options after the script name, for example:

```bash
./scripts/build_ros_clean.sh --packages-select drd25_msgs --event-handlers console_direct+
```

## Required External Package

The full workspace build requires `gnss_ins_msg`. Place the real package at:

```text
slam/gnss_ins_msg
```

or source an overlay that already provides it before building this workspace.

The full perception build additionally requires:

```bash
sudo apt install ros-humble-vision-msgs
```

The LiDAR segmentation package also requires CSF headers/library (`CSF.h` and
`libCSF.so`). The camera detector runtime requires `ultralytics`, `torch`,
`torchvision`, and OpenCV on the ROS/system Python stack, not conda.

## Hardcoded External Paths

External model and dataset locations are collected in:

```text
config/hardcoded_paths.ini
```

Current defaults:

- Models root: `/media/yupeng/新加卷/Models/DRd26_SLAM`
- Default YOLO model: `/media/yupeng/新加卷/Models/DRd26_SLAM/perception/src/cone_ws/src/cone_detector/runs/cone_yolov8n6/weights/best.pt`
- Default rosbag dataset: `/media/yupeng/新加卷/Datasets/rosbag2_2026_02_05-11_01_07`

## Launch

After a successful build:

```bash
source install/setup.bash
```

Straight-line mode:

```bash
ros2 launch slam slam.launch.py
```

Autocross mode:

```bash
ros2 launch slam slam.launch.py track:=autocross
```

Without RViz:

```bash
ros2 launch slam slam.launch.py rviz:=false
```

## Runtime Requirement

Starting `slam_node` is not enough to produce a map. These input topics must be
published by live nodes, a rosbag, or mock publishers:

- `/gongji_gnss_ins_64`
- `/perception/fusion/map`

Perception launch:

```bash
ros2 launch run_perception system_run.launch.py
```

Camera input defaults to `/camera1/image_raw`, LiDAR input defaults to
`/lidar_points`, and the fused output is remapped to `/perception/fusion/map`.
The YOLO model path is read from `config/hardcoded_paths.ini`.
