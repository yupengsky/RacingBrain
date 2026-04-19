# 相机、LiDAR 与 rosbag 数据入口

## 1. 本工程的三类原始输入

系统从三类原始数据开始：

```text
相机图像       /camera1/image_raw
LiDAR 点云     /lidar_points
GNSS/INS       /gongji_gnss_ins_64
```

这三类数据可以来自实车传感器，也可以来自 rosbag。

当前一键验证脚本使用 rosbag 回放，因此不需要真实相机、真实 LiDAR 或真实惯导硬件即可跑通工程。

## 2. 相机图像入口

话题：

```text
/camera1/image_raw
```

类型：

```text
sensor_msgs/msg/Image
```

消费者：

```text
yolo_detector
```

配置位置：

```text
perception/src/run_perception/launch/system_run.launch.py
```

launch 中设置：

```python
'image_topic': '/camera1/image_raw'
```

进入系统后的流程：

```text
/camera1/image_raw
  -> cv_bridge 转 OpenCV 图像
  -> YOLOv8 推理
  -> /yolo/cones
```

关键要求：

- 图像编码需要能被 `cv_bridge.imgmsg_to_cv2(..., 'bgr8')` 转换。
- 图像 `header.stamp` 应保留采集时间，用于后续融合时间同步。
- 图像尺寸应与融合标定中的 `image_width`、`image_height` 对应。

当前融合标定默认：

```text
640 x 480
```

## 3. LiDAR 点云入口

话题：

```text
/lidar_points
```

类型：

```text
sensor_msgs/msg/PointCloud2
```

消费者：

```text
cone_segmentation_node
```

代码中订阅：

```cpp
pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar_points",
    10,
    std::bind(&ConeSegmentationNode::topic_callback, this, std::placeholders::_1)
);
```

进入系统后的流程：

```text
/lidar_points
  -> PCL 点云
  -> ROI 半径过滤
  -> 体素降采样
  -> 地面分割
  -> 障碍物聚类
  -> 锥桶几何筛选
  -> /cone_detection_custom
```

关键要求：

- 点云字段至少能转换为 `pcl::PointXYZ`。
- 点云 frame_id 在当前数据中是 `hesai_lidar`。
- 点云 header 时间戳要保留采集时间。

## 4. GNSS/INS 入口

话题：

```text
/gongji_gnss_ins_64
```

类型：

```text
gnss_ins_msg/msg/Gnssins64
```

消费者：

```text
slam_processor
```

进入系统后的流程：

```text
/gongji_gnss_ins_64
  -> 经纬度转 UTM
  -> 第一帧设局部地图原点
  -> roll/pitch/yaw 转车辆姿态
  -> TF/Odom/Path 发布
  -> 与 /perception/fusion/map 同步后建图
```

在线时来源：

```text
cpp_pubsub/talker 从 /dev/ttyUSB0 串口解析发布
```

离线时来源：

```text
rosbag play
```

## 5. rosbag 回放入口

一键脚本中播放：

```bash
ros2 bag play "${DATASET_DIR}" \
  --rate "${BAG_RATE}" \
  --topics /camera1/image_raw /lidar_points /gongji_gnss_ins_64
```

这说明端到端演示只依赖三个录制话题。

默认数据集路径来自：

```text
config/hardcoded_paths.ini
```

字段：

```ini
[datasets]
rosbag_2026_02_05 = /media/yupeng/新加卷/Datasets/rosbag2_2026_02_05-11_01_07
```

## 6. rosbag 播放速率

默认：

```text
BAG_RATE=0.25
```

为什么不是 1.0？

- YOLO 推理可能消耗较多 CPU/GPU。
- 点云分割也有计算开销。
- 降低播放速率可以减少队列积压和同步丢失。

如果机器性能足够，可以提高：

```bash
BAG_RATE=1.0 ./scripts/run_dataset_slam_chain.sh
```

## 7. 三类数据的时间戳关系

系统能稳定运行的关键是：

```text
相机图像 header.stamp
LiDAR 点云 header.stamp
GNSS/INS header.stamp
```

需要处在同一个时间基准下。

融合层同步：

```text
/yolo/cones 的 header 继承图像
/cone_detection_custom 的 header 继承点云
```

SLAM 层同步：

```text
/perception/fusion/map 的 header 使用 LiDAR header
/gongji_gnss_ins_64 使用 GNSS/INS header
```

## 8. 最小传感器接入要求

如果你要用自己的硬件替换 rosbag，需要满足：

### 8.1 相机驱动

发布：

```text
/camera1/image_raw sensor_msgs/msg/Image
```

### 8.2 LiDAR 驱动

发布：

```text
/lidar_points sensor_msgs/msg/PointCloud2
```

### 8.3 GNSS/INS 驱动

发布：

```text
/gongji_gnss_ins_64 gnss_ins_msg/msg/Gnssins64
```

并至少填充：

```text
header
latitude
longitude
roll
pitch
yaw
vel_e
vel_n
imu_gyro_z
```

## 9. 数据入口调试命令

检查输入是否存在：

```bash
ros2 topic list | grep -E "camera1|lidar_points|gongji"
```

检查频率：

```bash
ros2 topic hz /camera1/image_raw
ros2 topic hz /lidar_points
ros2 topic hz /gongji_gnss_ins_64
```

检查类型：

```bash
ros2 topic info /camera1/image_raw
ros2 topic info /lidar_points
ros2 topic info /gongji_gnss_ins_64
```

检查一帧 GNSS：

```bash
ros2 topic echo /gongji_gnss_ins_64 --once
```

## 10. 数据入口故障排查

### 10.1 YOLO 没输出

优先检查：

- `/camera1/image_raw` 是否有消息。
- `model_path` 是否存在。
- 图像编码能否转 `bgr8`。
- `max_fps` 是否导致你观察时间太短。

### 10.2 LiDAR 分割没输出

优先检查：

- `/lidar_points` 是否有消息。
- 点云是否为空。
- ROI 半径是否太小。
- 地面分割是否把锥桶都当成地面。
- cluster 参数是否太严格。

### 10.3 SLAM 不建图

优先检查：

- `/gongji_gnss_ins_64` 是否有消息。
- `/perception/fusion/map` 是否有非 Unknown 锥桶。
- 两个话题 header 时间戳是否能同步。
- `is_origin_set_` 是否已经被第一帧 GNSS 设置。

