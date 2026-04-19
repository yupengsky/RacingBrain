# DRd26_SLAM 极度详细技术报告

本文档集面向“接手工程、理解工程、复现工程核心功能、用于技术汇报”四个目标编写。

本报告不是泛泛介绍 ROS 2 或 SLAM，而是严格按当前仓库代码实现展开。阅读顺序从传感器数据进入系统开始，一直讲到 SLAM 节点发布全局稳定锥桶地图为止。一个已经会 Python、C++ 和 ROS 2 基础的人，按这些章节可以复现工程 80% 以上功能，并完整复现核心链路。

## 当前工程一句话定义

`DRd26_SLAM` 是一个 ROS 2 Humble 工作空间，用于无人方程式赛车锥桶赛道的多传感器语义建图。它使用：

- 相机 YOLOv8 检测锥桶二维框和颜色。
- LiDAR 点云分割检测锥桶三维几何位置。
- LiDAR-Camera 融合节点把相机颜色赋给 LiDAR 空间位置。
- GNSS/INS 提供车辆全局位姿。
- SLAM 后端维护全局锥桶地图，并发布稳定锥桶 Marker。

它不是传统纯视觉 SLAM 或纯 LiDAR SLAM。当前实现更准确地说是：

> GNSS/INS 位姿驱动的语义锥桶全局建图系统。

## 建议阅读路线

如果你要快速用于汇报：

1. 读 `01_系统总览/01_系统目标与总体架构.md`
2. 读 `02_接口与数据流/01_ROS话题与消息接口.md`
3. 读 `05_融合层/01_LiDAR_Camera融合节点.md`
4. 读 `06_SLAM核心/03_锥桶观测入图与数据关联.md`
5. 读 `06_SLAM核心/04_地图维护发布与赛道模式.md`
6. 读 `10_工程评估/01_亮点问题与改进路线.md`

如果你要复现核心功能：

1. 读 `03_传感器输入/01_GNSS_INS数据接入.md`
2. 读 `04_感知层/01_相机YOLO锥桶检测.md`
3. 读 `04_感知层/02_LiDAR点云锥桶分割.md`
4. 读 `05_融合层/02_投影匹配与补漏数学.md`
5. 读 `06_SLAM核心/01_SLAM节点生命周期与参数.md`
6. 读 `06_SLAM核心/02_GNSS位姿与坐标转换.md`
7. 读 `06_SLAM核心/03_锥桶观测入图与数据关联.md`
8. 读 `09_复现指南/01_核心功能复现路线图.md`

如果你要讲算法细节：

1. 读 `08_术语专题/01_马氏距离.md`
2. 读 `08_术语专题/02_卡尔曼更新与协方差.md`
3. 读 `08_术语专题/03_各向异性观测噪声.md`
4. 读 `08_术语专题/04_PROJ与UTM坐标.md`
5. 读 `08_术语专题/05_ROS2时间同步与QoS.md`

## 报告目录说明

- `00_阅读入口`：如何看这份报告，如何把它变成 PPT。
- `01_系统总览`：系统目标、包结构、端到端链路。
- `02_接口与数据流`：话题、消息、坐标系、时间同步。
- `03_传感器输入`：GNSS/INS、相机、LiDAR 原始数据如何进入系统。
- `04_感知层`：YOLO 检测和点云锥桶分割。
- `05_融合层`：LiDAR-Camera 融合实现。
- `06_SLAM核心`：核心 SLAM 后端实现。
- `07_运行构建与验证`：环境、构建、一键运行、端到端验证。
- `08_术语专题`：工程中关键算法概念。
- `09_复现指南`：如何从零复现核心链路。
- `10_工程评估`：亮点、风险、改进路线。

## 本报告基于的主要源码

核心源码路径：

- `slam/slam/src/slam_node.cpp`
- `slam/slam/src/loop_closure_detector.cpp`
- `slam/slam/include/slam/loop_closure_detector.hpp`
- `slam/slam/config/*.yaml`
- `slam/drd25_msgs/msg/*.msg`
- `gnss/gnss_ins_msg/msg/*.msg`
- `gnss/cpp_pubsub/src/publisher_member_function.cpp`
- `perception/src/cone_ws/src/cone_detector/cone_detector/yolo_detector.py`
- `perception/src/cone_segmentation_test_3d/src/test_cone_segmentation/src/test_cone_segmentation.cpp`
- `perception/src/fs_fusion_box/src/fs_fusion_box_node.cpp`
- `perception/src/fs_fusion_box/src/fs_fusion_box_math.cpp`
- `perception/src/run_perception/launch/system_run.launch.py`
- `scripts/run_dataset_slam_chain.sh`

## 最短演示命令

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
RVIZ=true KEEP_RUNNING=true ./scripts/run_dataset_slam_chain.sh
```

这条命令会：

1. 激活 ROS + ML 虚拟环境。
2. source 当前工作空间。
3. 启动感知链路。
4. 启动 SLAM。
5. 回放配置文件里的 rosbag。
6. 监听关键话题。
7. 检查 `/perception/fusion/map` 和 `/global_map` 是否非空。
8. 将结果写入 `log/runtime/.../summary.json`。

## 工程复现的最核心闭环

只要复现下面这个闭环，就复现了工程 100% 核心功能：

```text
/camera1/image_raw        -> YOLO              -> /yolo/cones
/lidar_points             -> 点云分割           -> /cone_detection_custom
/yolo/cones + /cone_detection_custom
                           -> 融合              -> /perception/fusion/map
/gongji_gnss_ins_64 + /perception/fusion/map
                           -> SLAM 后端         -> /global_map
```

