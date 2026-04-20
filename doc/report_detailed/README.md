# DRd26_SLAM 技术报告

这份报告只保留主干内容。

目标很直接。你读完这几份文档后，要能回答下面四个问题：

1. 这个工程的数据从哪里来。
2. 这些数据怎么一步一步变成锥桶观测。
3. SLAM 节点怎么把锥桶观测变成全局锥桶地图。
4. 如果换成你自己的 Python 或 C++ 工程，核心功能该怎么重写。

这套系统的主链路很清楚：

`GNSS/INS串口数据 -> GNSS话题`

`相机图像 -> YOLO 2D锥桶框`

`LiDAR点云 -> 3D锥桶候选`

`2D/3D融合 -> 带颜色的局部锥桶观测`

`GNSS位姿 + 局部锥桶观测 -> 全局锥桶地图`

## 阅读顺序

1. `01_工程总览与代码入口.md`
2. `02_从传感器到融合输出.md`
3. `03_SLAM建图与全局锥桶地图.md`
4. `04_复现路线与关键公式.md`

## 你先记住这几个结论

### 1. 地图的基本单元是锥桶

这个工程最后维护的是锥桶对象。

它维护的是 `GlobalCone`。每个锥桶有：

- 全局二维坐标 `pos`
- 协方差 `P`
- 颜色和类型
- 存在分数 `existence_score`
- 是否稳定 `is_stable`

定义在：

- `slam/slam/include/slam/loop_closure_detector.hpp`

更新逻辑在：

- `slam/slam/src/slam_node.cpp`

### 2. GNSS 负责全局位姿

GNSS/INS 给车的绝对位置和姿态。

SLAM 没有做图优化，也没有做激光里程计。它直接用 GNSS/INS 的位姿，把局部锥桶观测投到全局坐标里。

所以这套系统更像：

`GNSS约束下的锥桶地图滤波器`

### 3. 融合节点只做局部观测整理

融合层不输出全局地图。

它做三件事：

1. 用外参把 LiDAR 3D 框投到图像。
2. 用 IoU 把 3D 框和 YOLO 2D 框配上。
3. 产出一帧局部坐标下的带颜色锥桶列表。

输出消息是：

- `/perception/fusion/map`
- 消息类型 `drd25_msgs/msg/Map`

### 4. 全局地图靠“匹配 + 卡尔曼更新 + 生存分数”长出来

核心函数是：

- `SlamProcessor::updateMap`

核心动作是：

1. 把当前锥桶观测变成全局坐标。
2. 用欧氏距离粗筛。
3. 用马氏距离找旧锥桶。
4. 匹配成功就做卡尔曼更新。
5. 没匹配上就新建锥桶。
6. 每帧再做一次 miss 惩罚和稳定性判定。
7. 只发布稳定锥桶。

## 仓库里最重要的文件

| 作用 | 文件 |
| --- | --- |
| GNSS 串口读取与发布 | `gnss/cpp_pubsub/src/publisher_member_function.cpp` |
| GNSS 消息定义 | `gnss/gnss_ins_msg/msg/Gnssins64.msg` |
| YOLO 检测节点 | `perception/src/cone_ws/src/cone_detector/cone_detector/yolo_detector.py` |
| YOLO 2D 检测消息 | `perception/src/cone_ws/src/cone_interfaces/msg/Cone.msg` |
| YOLO 2D 检测数组 | `perception/src/cone_ws/src/cone_interfaces/msg/ConeArray.msg` |
| LiDAR 锥桶分割节点 | `perception/src/cone_segmentation_test_3d/src/test_cone_segmentation/src/test_cone_segmentation.cpp` |
| LiDAR 3D 锥桶消息 | `perception/src/cone_segmentation_test_3d/src/test_cone_segmentation/msg/ThreeDCone.msg` |
| 融合节点 | `perception/src/fs_fusion_box/src/fs_fusion_box_node.cpp` |
| 融合数学函数 | `perception/src/fs_fusion_box/src/fs_fusion_box_math.cpp` |
| 融合标定参数 | `perception/src/fs_fusion_box/config/calibration.yaml` |
| SLAM 主节点 | `slam/slam/src/slam_node.cpp` |
| 回环检测器 | `slam/slam/src/loop_closure_detector.cpp` |
| 锥桶地图消息 | `slam/drd25_msgs/msg/Map.msg` |
| 一键运行脚本 | `scripts/run_dataset_slam_chain.sh` |

## 建议的源码阅读顺序

如果你第一次读这个仓库，直接按下面顺序看：

1. `scripts/run_dataset_slam_chain.sh`
2. `perception/src/run_perception/launch/system_run.launch.py`
3. `perception/src/fs_fusion_box/launch/fusion_box.launch.py`
4. `perception/src/cone_ws/src/cone_detector/cone_detector/yolo_detector.py`
5. `perception/src/cone_segmentation_test_3d/src/test_cone_segmentation/src/test_cone_segmentation.cpp`
6. `perception/src/fs_fusion_box/src/fs_fusion_box_node.cpp`
7. `perception/src/fs_fusion_box/src/fs_fusion_box_math.cpp`
8. `slam/slam/src/slam_node.cpp`
9. `slam/slam/src/loop_closure_detector.cpp`

这个顺序和运行链一致。你会更容易把整个系统接起来。

## 这份报告怎么压缩了

旧版报告把每个话题都拆成了很多小节。信息很全，但阅读成本高。

现在改成 4 份主文档。每份文档都直接贴着代码讲。能引用代码的地方，我就直接给文件和函数。这样更适合消化工程，也更适合做汇报。

## 一句话总结工程

这是一套基于 ROS2 的锥桶感知与地图系统。它用相机和 LiDAR 先做局部锥桶观测，再用 GNSS/INS 把观测投到全局坐标里，最后靠数据关联、卡尔曼更新、存在分数和回环锁图，得到一张稳定的全局锥桶地图。
