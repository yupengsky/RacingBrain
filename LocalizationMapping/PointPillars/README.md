# PointPillars Runtime

This directory keeps only the runtime TensorRT FP16 PointPillars detector used to replace the original PCL clustering LiDAR cone detector.

Removed from the imported bundle:

- duplicate FP16/INT8 test workspaces
- OpenPCDet x64 research workspace
- build/install/log artifacts
- PTQ calibration npz files, INT8 cache, ONNX, zip archives, and duplicated engines

Runtime package:

- `trt_cone_detector`: ROS 2 C++ node wrapping TensorRT inference.
- Model: `trt_cone_detector/models/pointpillars_cone_fp16.engine`.
- Output topic: `/cone_detection_custom`, using the existing `test_cone_segmentation/msg/ThreeDConeArray` interface so downstream fusion does not need a new message type.

The package builds `trt_infer_node` only when CUDA and TensorRT are available. On machines without those dependencies, colcon installs the package resources but skips the executable with a CMake warning.
