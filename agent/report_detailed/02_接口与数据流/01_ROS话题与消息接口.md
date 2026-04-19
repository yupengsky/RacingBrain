# ROS 话题与消息接口

本章描述工程中各节点之间的通信契约。复现工程时，接口比实现更重要。只要接口保持一致，内部算法可以逐步替换。

## 1. 总体话题链路

核心话题链路：

```text
/camera1/image_raw
  -> yolo_detector
  -> /yolo/cones

/lidar_points
  -> cone_segmentation_node
  -> /cone_detection_custom

/yolo/cones + /cone_detection_custom
  -> fusion_box_node
  -> /perception/fusion/map

/gongji_gnss_ins_64 + /perception/fusion/map
  -> slam_processor
  -> /global_map
```

## 2. 原始输入话题

### 2.1 相机图像

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

来源：

- 实车相机节点。
- rosbag 回放。

当前 `run_perception` launch 将 YOLO 的 `image_topic` 参数设为 `/camera1/image_raw`。

### 2.2 LiDAR 点云

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

来源：

- 实车 LiDAR 驱动。
- rosbag 回放。

### 2.3 GNSS/INS

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

来源：

- 在线时由 `cpp_pubsub/talker` 串口桥接发布。
- 离线时由 rosbag 回放。

## 3. 感知中间话题

### 3.1 YOLO 锥桶二维框

话题：

```text
/yolo/cones
```

类型：

```text
cone_interfaces/msg/ConeArray
```

生产者：

```text
yolo_detector
```

消费者：

```text
fusion_box_node
```

消息定义：

```text
std_msgs/Header header
Cone[] cones
```

其中 `cone_interfaces/msg/Cone`：

```text
geometry_msgs/Point center
geometry_msgs/Vector3 size
float32 confidence
uint8 color
```

字段语义：

- `center.x`：图像中 bbox 中心 u 坐标，单位像素。
- `center.y`：图像中 bbox 中心 v 坐标，单位像素。
- `size.x`：bbox 宽度，单位像素。
- `size.y`：bbox 高度，单位像素。
- `confidence`：YOLO 置信度。
- `color`：锥桶颜色枚举。

颜色枚举：

```text
BLUE = 0
RED = 1
YELLOW_BIG = 2
YELLOW_SMALL = 3
UNKNOWN = 4
```

### 3.2 YOLO 调试图像

话题：

```text
/yolo/debug_image
```

类型：

```text
sensor_msgs/msg/Image
```

用途：

- RViz 或图像工具中查看检测框。
- 观察模型类别名、置信度和 HSV 一致性检查效果。

### 3.3 LiDAR 锥桶三维框

话题：

```text
/cone_detection_custom
```

类型：

```text
test_cone_segmentation/msg/ThreeDConeArray
```

生产者：

```text
cone_segmentation_node
```

消费者：

```text
fusion_box_node
```

消息定义：

```text
std_msgs/Header header
ThreeDCone[] cones
```

其中 `ThreeDCone`：

```text
geometry_msgs/Point center
geometry_msgs/Vector3 size
```

字段语义：

- `center`：LiDAR 坐标系下的 cluster bbox 中心。
- `size.x`：bbox 长度。
- `size.y`：bbox 宽度。
- `size.z`：bbox 高度。

注意：代码注释中写了 `[px]`，但实际来自 PCL 点云坐标，单位应按点云坐标理解，一般是米。

## 4. 融合输出话题

话题：

```text
/perception/fusion/map
```

类型：

```text
drd25_msgs/msg/Map
```

生产者：

```text
fusion_box_node
```

消费者：

```text
slam_processor
```

消息定义：

```text
std_msgs/Header header
Cone[] track
```

其中 `drd25_msgs/msg/Cone`：

```text
float64 x
float64 y

uint8 BLUE=0
uint8 RED=1
uint8 YELLOW_BIG=2
uint8 YELLOW_SMALL=3
uint8 UNKNOWN=4
uint8 color
```

字段语义：

- `x`：融合后的锥桶局部坐标 x。
- `y`：融合后的锥桶局部坐标 y。
- `color`：融合后的锥桶颜色。

当前 SLAM 把这里的 `x,y` 视作 LiDAR 坐标系下的二维点，然后通过 `T_l2v_` 和车辆位姿转到全局地图。

## 5. SLAM 输出话题

### 5.1 全局锥桶地图

话题：

```text
/global_map
```

类型：

```text
visualization_msgs/msg/MarkerArray
```

生产者：

```text
slam_processor
```

用途：

- RViz 可视化稳定锥桶。
- 当前不是结构化规划接口。

注意：

每次发布时，SLAM 会先发布一个 `DELETEALL` marker 清空上一帧，再重新添加所有稳定锥桶 marker。

### 5.2 车辆轨迹

话题：

```text
/vehicle_path
```

类型：

```text
nav_msgs/msg/Path
```

生产者：

```text
slam_processor
```

用途：

- RViz 显示车辆运动轨迹。

### 5.3 车辆里程计

话题：

```text
/vehicle_odom
```

类型：

```text
nav_msgs/msg/Odometry
```

生产者：

```text
slam_processor
```

用途：

- 显示车辆当前位姿和车体系速度。

### 5.4 TF

变换：

```text
map -> base_link
```

生产者：

```text
slam_processor
```

用途：

- RViz 中统一坐标系。
- 其他节点读取车辆在地图系的位置。

## 6. run_perception 中的话题重映射

`run_perception/launch/system_run.launch.py` 直接启动三个感知相关节点。

YOLO 节点参数：

```text
image_topic = /camera1/image_raw
conf_threshold = 0.5
max_fps = 10.0
```

LiDAR 节点参数：

```text
use_csf = False
```

融合节点通过 `fs_fusion_box/launch/fusion_box.launch.py` 重映射：

```text
perception/lidar/cones_custom  -> /cone_detection_custom
perception/camera/cones_custom -> /yolo/cones
fusion/cones                   -> /perception/fusion/map
```

## 7. slam.launch.py 中的话题配置

SLAM 话题来自 YAML：

```yaml
system:
  gnss_topic: "/gongji_gnss_ins_64"
  lidar_topic: "/perception/fusion/map"
  map_frame: "map"
  base_frame: "base_link"
  utm_zone_epsg: "EPSG:32651"
```

`lidar_topic` 这个名字有一点历史包袱，它实际订阅的是融合后的感知地图，不是原始 LiDAR 点云。

## 8. 复现接口最小集合

如果你不想复现所有调试话题，只要实现以下最小接口即可跑通核心：

输入：

```text
/camera1/image_raw        sensor_msgs/msg/Image
/lidar_points             sensor_msgs/msg/PointCloud2
/gongji_gnss_ins_64       gnss_ins_msg/msg/Gnssins64
```

中间输出：

```text
/yolo/cones               cone_interfaces/msg/ConeArray
/cone_detection_custom    test_cone_segmentation/msg/ThreeDConeArray
/perception/fusion/map    drd25_msgs/msg/Map
```

最终输出：

```text
/global_map               visualization_msgs/msg/MarkerArray
```

