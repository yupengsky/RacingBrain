# RacingBrain 感知链路延迟基准测试

本报告记录 RacingBrain 在同一段 rosbag（机器人记录包）上的 3D 点云感知、2D 视觉感知和完整链路吞吐表现。测试重点是 latency（延迟）和 system bottleneck（系统瓶颈），不是检测精度排名：当前数据包没有人工标注真值，因此本文不报告 precision（精确率）和 recall（召回率）。

## 1. 结论

1. 3D 点云侧，PointPillars（点柱网络）显著快于 PCL clustering（点云库聚类）。PointPillars 平均 22.03 ms，PCL 聚类平均 263.22 ms。对当前工程来说，学习式 3D 检测确实解决了一个主要实时瓶颈。

2. 2D 视觉侧，YOLO（You Only Look Once，单阶段目标检测器）不是最快的算法。传统颜色候选器在 CPU（中央处理器）上更快，但会产生大量候选区域。YOLO 的价值在于用约 8 ms 延迟输出更紧凑的语义框，而不是在所有情况下比传统视觉更快。

3. 完整系统里，YOLO 目前不是主要瓶颈。PointPillars 后端下 `/global_map` 约 5.03 Hz，PCL 后端下约 3.17 Hz。瓶颈主要受 LiDAR（Light Detection and Ranging，激光雷达）检测、融合同步和建图节奏影响。

4. 工程上更合理的架构不是“学习模型替代传统算法”，而是：学习模型作为快速 proposal（候选生成器），传统几何和跨模态一致性作为低成本 verifier（验证器），只有通过验证的结果才写入 map / memory（地图/记忆）。

## 2. 测试环境

| 项目 | 配置 |
|---|---|
| GPU（图形处理器） | NVIDIA GeForce RTX 2050, 4 GB |
| Driver（驱动） | 535.274.02 |
| CUDA runtime（CUDA 运行时） | 可用，1 张卡 |
| PyTorch（深度学习框架） | 2.5.1+cu121 |
| ROS（Robot Operating System，机器人操作系统） | ROS 2 Humble |
| 数据包 | `/media/yupeng/新加卷/Datasets/rosbag2_2026_02_05-11_01_07` |
| 数据时长 | 41.29 s |
| 相机帧数 | 1220 |
| LiDAR 点云帧数 | 412 |

3D 点云测试使用完整 ROS replay（回放）链路；2D 视觉测试直接读取同一数据包中的 `/camera1/image_raw`，逐帧离线计时。

## 3. 测试方法

### 3.1 点云锥桶检测

| 方法 | 类型 | 工程入口 |
|---|---|---|
| PointPillars | 学习式 3D 检测，GPU / TensorRT（英伟达推理加速库） | `trt_cone_detector` |
| PCL clustering | 传统 CPU 几何检测 | `test_cone_segmentation` |

PCL 路径包括 ROI（Region of Interest，感兴趣区域）过滤、地面分割、欧氏聚类和锥桶几何筛选。它可解释，但全量运行成本较高。

### 3.2 视觉锥桶检测

| 方法 | 类型 | 工程入口 |
|---|---|---|
| YOLO | 学习式 2D 检测，GPU / PyTorch | `cone_detector.yolo_detector` |
| HSV component | HSV（Hue-Saturation-Value，色相-饱和度-明度）连通域，CPU / OpenCV（开源计算机视觉库） | `scripts/eval/benchmark_camera_detectors.py` |
| Classical cone | 颜色掩膜 + 形态学 + 轮廓 + 几何筛选 + NMS（Non-Maximum Suppression，非极大值抑制），CPU / OpenCV | `scripts/eval/benchmark_camera_detectors.py` |

`Classical cone` 是比粗 HSV 更接近工程传统视觉的基线，但它仍是候选生成器，不等价于经过训练和标定的完整视觉检测系统。

## 4. 点云结果

| 方法 | 帧数 | Mean（均值） | Median（中位数） | P95（95 分位） | Max（最大值） |
|---|---:|---:|---:|---:|---:|
| PointPillars GPU | 231 | 22.03 ms | 20.41 ms | 30.15 ms | 49.43 ms |
| PCL clustering CPU | 159 | 263.22 ms | 271.32 ms | 318.60 ms | 339.03 ms |

PointPillars 在 mean 上约快 12 倍，在 median 上约快 13 倍。这个差距足够改变系统设计：点云主路径使用 PointPillars 可以释放实时预算，但不能把 PCL 全量聚类当作高频 fallback（兜底/回退）。

PCL 路径的主要耗时如下：

| 阶段 | Mean | Median | P95 |
|---|---:|---:|---:|
| 地面分割 | 45.22 ms | 45.91 ms | 53.18 ms |
| 欧氏聚类 | 124.25 ms | 133.32 ms | 170.43 ms |
| 完整 PCL 检测 | 263.22 ms | 271.32 ms | 318.60 ms |

这组数据说明，PCL 更适合作为可解释参照、离线检查或低频兜底；若每次学习模型不确定都完整重跑 PCL，实时周期会被直接拉长到数百毫秒。

## 5. 视觉结果

| 方法 | 帧数 | Mean | Median | P95 | 平均输出 |
|---|---:|---:|---:|---:|---:|
| YOLO GPU | 280 | 7.89 ms | 7.89 ms | 8.27 ms | 1.95 boxes / frame |
| HSV component CPU | 280 | 1.82 ms | 1.80 ms | 2.06 ms | 47.58 components / frame |
| Classical cone CPU | 280 | 3.62 ms | 3.58 ms | 4.01 ms | 23.64 candidates / frame |

视觉侧的结论和点云侧不同。传统颜色方法更快，但输出负载明显更高：`Classical cone` 平均每帧输出 23.64 个候选，YOLO 平均每帧输出 1.95 个框。

在没有标注真值的前提下，不能只根据候选数量判断谁更准。但对完整系统来说，输出负载会影响后续关联、融合和地图写入。YOLO 的工程价值是用较低延迟提供更紧凑的语义输入；传统视觉更适合做局部颜色验证、异常检查或轻量候选补充。

## 6. 完整链路吞吐

### 6.1 PointPillars 后端

| Topic（话题） | Rate（频率） |
|---|---:|
| `/camera1/image_raw` | 20.40 Hz |
| `/yolo/cones` | 8.39 Hz |
| `/lidar_points` | 9.59 Hz |
| `/cone_detection_custom` | 5.74 Hz |
| `/perception/fusion/map` | 5.03 Hz |
| `/global_map` | 5.03 Hz |

runtime budget（实时预算）状态：

| 状态 | 帧数 |
|---|---:|
| nominal（正常） | 42 |
| strained（紧张） | 2 |

PointPillars 后端下，系统大部分时间处于 nominal。此时 YOLO 的 8.39 Hz 输出不是全链路最低频节点，最终地图频率主要由点云检测输出、融合同步和建图触发节奏共同决定。

### 6.2 PCL 后端

| Topic | Rate |
|---|---:|
| `/camera1/image_raw` | 19.67 Hz |
| `/yolo/cones` | 8.38 Hz |
| `/lidar_points` | 9.44 Hz |
| `/cone_detection_custom` | 3.75 Hz |
| `/perception/fusion/map` | 3.17 Hz |
| `/global_map` | 3.17 Hz |

runtime budget 状态：

| 状态 | 帧数 |
|---|---:|
| nominal | 9 |
| strained | 15 |
| degraded（降级） | 17 |
| freeze（冻结） | 1 |

PCL 后端下，点云检测频率下降到 3.75 Hz，地图频率下降到 3.17 Hz，实时预算状态明显变差。这个结果和单节点计时一致：CPU 点云聚类是当前链路中更硬的实时瓶颈。

## 7. 对 RacingBrain 架构的含义

这次测试支持 RacingBrain 采用“快速学习感知 + 可解释治理”的感知组织方式。

点云主路径可以使用 PointPillars 获取实时性。传统 PCL 不适合在每个周期完整重跑，但它仍然有价值：可以拆成局部点数、局部高度、局部宽度、地面关系等低成本几何检查，用来验证 PointPillars 输出是否可信。

视觉主路径可以保留 YOLO 提供语义框。传统颜色与轮廓方法不必替代 YOLO，而是作为轻量 verifier：检查候选框内部是否存在合理颜色区域、边界形状是否异常、连续帧是否稳定。

地图写入不应只看 detector confidence（检测器置信度）。在高速低容错任务中，更关键的是该观测是否通过多源一致性检查，是否会污染下一圈使用的任务记忆。因此感知结果进入地图前应经过明确的 gate（门控）：

```text
learned proposal
    -> local geometric verifier
    -> camera-lidar consistency
    -> runtime health state
    -> commit / downweight / freeze / fallback
    -> task memory
```

这也是 RacingBrain 可以抽象出的研究问题：真实世界高风险重复任务中，智能体如何把快速但不完全可解释的感知结果，转化为可审计、可冻结、可回退的任务记忆。

## 8. 后续实验

当前报告只证明了延迟和系统瓶颈，下一步应补上质量和闭环影响。

### 8.1 精度-延迟曲线

人工标注一小批图像和点云，报告每种方法的 precision、recall、latency 和 output load（输出负载）。目标不是证明某个检测器绝对最好，而是得到 RacingBrain 自己数据上的 Pareto（帕累托）边界。

| 对比 | 指标 |
|---|---|
| YOLO vs Classical cone | precision / recall / latency / boxes per frame |
| PointPillars vs PCL | precision / recall / latency / cones per frame |

### 8.2 低成本验证器

完整 PCL 太慢，下一步应测试局部验证器：

| 验证器 | 作用 |
|---|---|
| 3D box 内局部点数 | 拒绝空框 |
| 局部高度/宽度检查 | 拒绝几何尺寸异常的框 |
| 地面关系检查 | 拒绝漂浮或埋入地面的框 |
| Camera-LiDAR projection IoU（相机-激光投影交并比） | 检查跨模态一致性 |
| 完整 PCL clustering | 低频兜底或离线审计 |

重点指标是 verifier 的触发成本、误拒率、误放率，以及它对 `/global_map` 频率的影响。

### 8.3 瓶颈转移

在同一数据包下比较以下变体：

| Variant（变体） | 目的 |
|---|---|
| `LIDAR_BACKEND=cluster` | 可解释但较慢的基线 |
| `LIDAR_BACKEND=pointpillars` | 当前学习式实时后端 |
| `pointpillars + local verifier` | 推荐的下一步架构 |
| `pointpillars + verifier + fallback policy` | 测试门控和兜底策略 |

同时记录 `/cone_detection_custom`、`/perception/fusion/map`、`/global_map`、runtime budget 状态和地图污染指标，判断加速是否真正传导到任务侧。

### 8.4 故障注入

利用已有 fault injection（故障注入）能力，观察系统是否做出正确 memory policy（记忆策略）：

| 故障 | 关注点 |
|---|---|
| camera blur（相机模糊） | 视觉框稳定性和颜色验证是否下降 |
| camera blank（相机黑屏） | 视觉证据缺失时是否降权 |
| LiDAR timestamp skew（激光时间戳偏移） | 跨模态一致性是否下降 |
| calibration bias（标定偏差） | 投影残差是否升高 |
| GNSS skew（定位时间偏移） | 地图写入是否冻结或降权 |

这部分实验直接对应比赛需求：第一圈形成的记忆不能只追求多，还要避免把错误观测写成下一圈的先验。

## 9. 局限性

1. 视觉和点云检测均未使用人工标注真值，本报告不比较绝对精度。
2. YOLO 当前使用 PyTorch 推理；若部署 TensorRT，视觉延迟可能继续下降。
3. `Classical cone` 是工程基线，不代表充分调参后的传统视觉系统。
4. 点云对比更接近完整系统，因为两个点云后端都接入了 ROS 回放链路并发布到同一下游 topic。
5. 本报告尚未测试闭环圈速、撞桶率和地图污染对规划结果的影响。

## 10. 复现命令

3D 学习后端：

```bash
LOG_DIR=log/eval/speed_pointpillars_$(date +%Y%m%d_%H%M%S) \
LIDAR_BACKEND=pointpillars STARTUP_WAIT=8 EVAL_TIMEOUT=90 IDLE_TIMEOUT=6 \
BAG_RATE=1.0 RVIZ=false ENABLE_PLANNING=false MAPPING_GATE=true \
./scripts/run_dataset_mapping_eval.sh
```

3D 传统后端：

```bash
LOG_DIR=log/eval/speed_cluster_$(date +%Y%m%d_%H%M%S) \
LIDAR_BACKEND=cluster STARTUP_WAIT=8 EVAL_TIMEOUT=90 IDLE_TIMEOUT=6 \
BAG_RATE=1.0 RVIZ=false ENABLE_PLANNING=false MAPPING_GATE=true \
./scripts/run_dataset_mapping_eval.sh
```

2D 视觉 benchmark（基准测试）：

```bash
source scripts/activate_ros_ml.sh
source install/setup.bash
python3 scripts/eval/benchmark_camera_detectors.py \
  --max-frames 300 \
  --warmup 20 \
  --output-dir log/eval/camera_detector_classical_benchmark_$(date +%Y%m%d_%H%M%S)
```

本次实验产物：

- `log/eval/speed_pointpillars_20260424_235434/summary.json`
- `log/eval/speed_cluster_20260424_235543/summary.json`
- `log/eval/camera_detector_classical_benchmark_20260425_000640/summary.json`

## 11. 参考资料

- [MIT Driverless CV Training Infrastructure](https://github.com/cv-core/MIT-Driverless-CV-TrainingInfra)
- [MIT Driverless TensorRT ROS deployment](https://github.com/cv-core/tensorrt_ros)
- [MIT RSS Visual Servoing color segmentation exercise](https://github.com/mit-rss/visual_servoing)
- [FSTD_SLAM roadmap for visual and LiDAR cone detection](https://github.com/aslyansky-m/FSTD_SLAM)
- [AMZ Racing FSD resources](https://github.com/AMZ-Racing/fsd-resources)
- [FSOCO dataset](https://ddavid.github.io/fsoco/)
