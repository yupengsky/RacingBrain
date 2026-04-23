# LocalizationMapping

This folder contains the real-time localization and mapping stack for RacingBrain.

Main contents:

- `RacingBrain/`: top-level launch and function orchestration package.
- `slam/`: legacy ROS package name for the GNSS/INS-aided C++ mapping node.
- `perception/`: camera, LiDAR, cone detection, and fusion packages.
- `gnss/`: GNSS/INS messages and serial bridge.
- `PointPillars/`: TensorRT LiDAR cone detector runtime.
- `config/`: runtime path configuration.
- `doc/` and `agent/`: notes, reports, and integration material.

Mapping entry:

```bash
./scripts/build_ros_clean.sh --packages-select gnss_ins_msg drd25_msgs slam racingbrain
source install/setup.bash
ros2 launch racingbrain localization_mapping.launch.py enable_perception:=false enable_mapping:=true rviz:=false
```
