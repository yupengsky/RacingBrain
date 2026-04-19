# 代码目录与 ROS 包职责

## 1. 根目录结构

仓库主要目录如下：

```text
DRd26_SLAM/
├── agent/
├── config/
├── gnss/
├── perception/
├── scripts/
└── slam/
```

其中：

- `agent/`：工程说明、运行手册、解析文档和本技术报告。
- `config/`：跨模块使用的硬编码外部路径配置。
- `gnss/`：GNSS/INS 消息和串口桥接。
- `perception/`：相机检测、LiDAR 分割、融合和感知 launch。
- `scripts/`：构建、依赖安装、环境激活和端到端验证脚本。
- `slam/`：自定义消息和 SLAM 主节点。

## 2. 有效 ROS 包列表

通过 `colcon list` 可以识别当前有效包：

```text
cone_detector
cone_interfaces
cpp_pubsub
drd25_msgs
fs_fusion_box
gnss_ins_msg
run_perception
slam
test_cone_segmentation
```

注意：

```text
perception/src/drd25_msgs/COLCON_IGNORE
```

说明感知目录下也有一个旧的 `drd25_msgs`，但当前被禁用。有效 `drd25_msgs` 来自：

```text
slam/drd25_msgs
```

## 3. slam/drd25_msgs

路径：

```text
slam/drd25_msgs
```

类型：

```text
ament_cmake 消息包
```

职责：

- 定义下游通用锥桶地图接口。
- 被融合节点和 SLAM 节点共同依赖。

关键消息：

```text
msg/Cone.msg
msg/Map.msg
msg/Waypoint.msg
msg/Path.msg
msg/Point.msg
msg/Pointcloud.msg
```

核心消息是：

```text
drd25_msgs/msg/Map
drd25_msgs/msg/Cone
```

## 4. slam/slam

路径：

```text
slam/slam
```

类型：

```text
ament_cmake C++ 节点包
```

构建产物：

```text
slam_node
```

运行节点名：

```text
slam_processor
```

关键文件：

```text
src/slam_node.cpp
src/loop_closure_detector.cpp
include/slam/loop_closure_detector.hpp
config/params.yaml
config/acceleration.yaml
config/autocross.yaml
config/skidpad.yaml
launch/slam.launch.py
rviz/slam.rviz
meshes/cone.dae
meshes/cone_big.dae
```

职责：

- 加载系统参数和赛道参数。
- 订阅 GNSS/INS 与融合地图。
- 执行全局锥桶地图维护。
- 发布 TF、Odom、Path 和全局锥桶可视化地图。

## 5. gnss/gnss_ins_msg

路径：

```text
gnss/gnss_ins_msg
```

类型：

```text
ament_cmake 消息包
```

关键消息：

```text
msg/Gnssins.msg
msg/Gnssins64.msg
```

当前 SLAM 主要使用：

```text
gnss_ins_msg/msg/Gnssins64
```

原因是经纬度使用 `float64`，避免经纬度精度损失。

## 6. gnss/cpp_pubsub

路径：

```text
gnss/cpp_pubsub
```

类型：

```text
ament_cmake C++ 节点包
```

构建产物：

```text
talker
listener
```

其中 `talker` 是真正有用的串口桥接节点。`listener` 是示例订阅器。

关键文件：

```text
src/publisher_member_function.cpp
src/serial_stream.cpp
include/cpp_pubsub/stream.h
include/cpp_pubsub/macros.h
```

职责：

- 打开 `/dev/ttyUSB0`。
- 按协议头和 CRC 解析惯导数据帧。
- 发布 GNSS/INS、IMU 和车体系速度。

## 7. perception/src/cone_ws/src/cone_interfaces

路径：

```text
perception/src/cone_ws/src/cone_interfaces
```

类型：

```text
ament_cmake 消息包
```

职责：

- 为相机 YOLO 检测输出定义二维锥桶框消息。

消息：

```text
msg/Cone.msg
msg/ConeArray.msg
```

这里的 `Cone` 表示图像坐标下的 2D 框，不是全局地图锥桶。

## 8. perception/src/cone_ws/src/cone_detector

路径：

```text
perception/src/cone_ws/src/cone_detector
```

类型：

```text
ament_python Python 节点包
```

构建入口：

```text
yolo_detector=cone_detector.yolo_detector:main
```

关键文件：

```text
cone_detector/yolo_detector.py
setup.py
package.xml
```

职责：

- 加载 YOLOv8 权重。
- 订阅相机图像。
- 发布二维锥桶检测。
- 发布调试图像。
- 做可选 HSV 颜色一致性检查。

## 9. perception/src/cone_segmentation_test_3d/src/test_cone_segmentation

路径：

```text
perception/src/cone_segmentation_test_3d/src/test_cone_segmentation
```

类型：

```text
ament_cmake C++ 节点包 + 消息包
```

构建产物：

```text
cone_segmentation_node
```

消息：

```text
msg/ThreeDCone.msg
msg/ThreeDConeArray.msg
```

职责：

- 从 LiDAR 点云中分割锥桶。
- 发布三维锥桶中心和尺寸。
- 提供可视化和调试点云。

依赖：

- PCL
- OpenMP
- CSF

## 10. perception/src/fs_fusion_box

路径：

```text
perception/src/fs_fusion_box
```

类型：

```text
ament_cmake C++ 节点包
```

构建产物：

```text
fusion_box_node
libfs_fusion_box_math.a
```

关键文件：

```text
src/fs_fusion_box_node.cpp
src/fs_fusion_box_math.cpp
include/fs_fusion_box/fs_fusion_box_math.hpp
config/calibration.yaml
launch/fusion_box.launch.py
```

职责：

- 对齐相机和 LiDAR 检测。
- 投影匹配。
- 生成融合后的 `drd25_msgs/Map`。

## 11. perception/src/run_perception

路径：

```text
perception/src/run_perception
```

类型：

```text
ament_python launch 包
```

关键文件：

```text
launch/system_run.launch.py
```

职责：

- 读取模型和数据路径配置。
- 启动 YOLO 节点。
- 启动 LiDAR 分割节点。
- 延迟启动融合节点。

默认相机话题：

```text
/camera1/image_raw
```

默认 LiDAR 话题：

```text
/lidar_points
```

## 12. scripts

路径：

```text
scripts
```

职责：

- 构建隔离 ROS + ML 环境。
- 本地安装缺失依赖。
- 一键构建。
- 一键端到端验证。

关键脚本：

```text
activate_ros_ml.sh
setup_ros_ml_venv.sh
build_ros_clean.sh
build_ros_venv.sh
install_csf_local.sh
install_ros_apt_deps_local.sh
run_dataset_slam_chain.sh
```

其中 `run_dataset_slam_chain.sh` 是最重要的验收脚本。

