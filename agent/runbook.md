# Runbook

## Isolated Python Runtime

Create or refresh the dedicated ROS-compatible ML venv from the workspace root:

```bash
./scripts/setup_ros_ml_venv.sh
```

This creates `./.venv_ros_ml` with `--system-site-packages`, so ROS 2 Humble
Python modules remain visible while ML packages stay out of the global Python.

Verified imports inside the venv:

- `rclpy`
- `cv2`
- `torch==2.3.1+cpu`
- `torchvision==0.18.1+cpu`
- `ultralytics==8.4.38`

Default setup uses CPU PyTorch for a smaller, more reliable bring-up. To switch
to the CUDA 12.1 wheel set later:

```bash
TORCH_VARIANT=cu121 ./scripts/setup_ros_ml_venv.sh
```

Activate the runtime for shell use:

```bash
source scripts/activate_ros_ml.sh
```

## Clean ROS Build

Build through the venv-aware helper:

```bash
./scripts/build_ros_venv.sh
```

or explicitly:

```bash
ROS_VENV_DIR=.venv_ros_ml ./scripts/build_ros_clean.sh
```

The build helper strips conda from the build environment, sources ROS 2 Humble,
binds `Python3_EXECUTABLE` to the venv interpreter, and runs:

```bash
python3 -m colcon build --symlink-install --cmake-clean-cache
```

By default it builds from the curated package roots under `gnss/`, `slam/`, and
`perception/`, avoiding the imported duplicate `perception/src/drd25_msgs`.

Pass normal colcon options after the script name, for example:

```bash
./scripts/build_ros_venv.sh --packages-select drd25_msgs --event-handlers console_direct+
```

## Local External ROS/C++ Dependencies

The isolated Python environment is ready. The previously missing dependencies
have been resolved locally, without installing into the system Python or global
ROS tree:

1. `vision_msgs`

   This is an official ROS 2 package. Because passwordless `sudo apt install`
   was not available, the ROS apt package was downloaded and extracted into the
   workspace-local prefix:

   ```text
   .ros_deps/opt/ros/humble
   ```

   Reinstall/refresh it with:

   ```bash
   ./scripts/install_ros_apt_deps_local.sh ros-humble-vision-msgs
   ```

2. CSF

   The LiDAR segmentation package needs `CSF.h` and `libCSF`. They are installed
   from the official CSF repository into:

   ```text
   .ros_deps/csf
   ```

   Reinstall/refresh it with:

   ```bash
   ./scripts/install_csf_local.sh
   ```

3. `gnss_ins_msg`

   This is project-specific, not an official ROS package. It is now provided by
   this workspace under `gnss/gnss_ins_msg`. The same imported tree also contains
   `gnss/cpp_pubsub`, an optional serial GNSS/INS bridge.

The rosbag at
`/media/yupeng/新加卷/Datasets/rosbag2_2026_02_05-11_01_07` publishes
`/gongji_gnss_ins_64` with type `gnss_ins_msg/msg/Gnssins64`.

Current build-missing file inventory:

- `gnss_ins_msg`: solved by the team/project package now stored at
  `gnss/gnss_ins_msg`.
- `vision_msgs`: solved by the official ROS package extracted locally into
  `.ros_deps/opt/ros/humble`.
- `CSF.h` and `libCSF`: solved by the official CSF source build installed into
  `.ros_deps/csf`.
- No additional source/package files are missing for a clean build of the
  curated workspace.

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
source scripts/activate_ros_ml.sh
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

Dataset-to-SLAM integration check:

```bash
./scripts/run_dataset_slam_chain.sh
```

This launches perception, launches SLAM without RViz, replays only the required
bag topics, and writes logs plus `summary.json` under `log/runtime/`. The
success condition is:

- bag input arrives on `/camera1/image_raw`, `/lidar_points`,
  `/gongji_gnss_ins_64`;
- perception publishes `/yolo/cones` and `/cone_detection_custom`;
- fusion publishes non-empty `/perception/fusion/map`;
- SLAM publishes non-empty `/global_map`.

Current verified status:

- `./scripts/build_ros_venv.sh --event-handlers console_direct+` finishes:
  9 packages built, warnings only.
- `run_perception` builds in the venv.
- `cone_detector` builds in the venv and its installed launcher shebang points
  to the workspace venv interpreter.
- `gnss_ins_msg`, `cpp_pubsub`, and `slam` build after renaming the imported
  GNSS tree to `gnss/`.
- `test_cone_segmentation` builds against workspace-local CSF.
- `fs_fusion_box` builds against workspace-local `vision_msgs`.
- `ros2 launch run_perception system_run.launch.py --show-args` works after
  `source scripts/activate_ros_ml.sh && source install/setup.bash`.
- `./scripts/run_dataset_slam_chain.sh` verifies the runtime path from the
  configured rosbag dataset to non-empty SLAM `/global_map` output.
