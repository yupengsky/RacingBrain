# ROS 2 时间同步与 QoS

## 1. 为什么需要时间同步

SLAM 地图更新需要同时知道：

- 锥桶在车辆局部坐标下的位置。
- 车辆在地图坐标系下的位置和姿态。

如果感知帧和 GNSS 位姿不在同一时间，会出现：

```text
车辆已经向前走了，但锥桶观测还是旧时刻的
```

高速赛车中，这会造成明显地图错位。

## 2. message_filters ApproximateTime

SLAM 使用：

```cpp
message_filters::sync_policies::ApproximateTime<
    gnss_ins_msg::msg::Gnssins64,
    drd25_msgs::msg::Map
>
```

融合层使用：

```cpp
message_filters::sync_policies::ApproximateTime<
    test_cone_segmentation::msg::ThreeDConeArray,
    cone_interfaces::msg::ConeArray
>
```

ApproximateTime 的含义：

```text
不要求两个消息时间戳完全相等，只要足够接近就配对回调。
```

## 3. SLAM 中的两个 GNSS 订阅

### 3.1 高频订阅

```cpp
high_freq_gnss_sub_ = create_subscription(... SensorDataQoS ...)
```

用途：

- 实时 TF。
- 实时 Odom。
- 实时 Path。

### 3.2 同步订阅

```cpp
gnss_sub_.subscribe(this, sys_.gnss_topic);
perception_sub_.subscribe(this, sys_.lidar_topic);
sync_->registerCallback(syncCallback);
```

用途：

- 建图。

两条链路分开是合理的：

```text
车辆显示越实时越好，建图越对齐越好。
```

## 4. Header 时间戳的重要性

能否同步，取决于消息的：

```text
header.stamp
```

工程中：

- YOLO 输出继承图像 header。
- LiDAR 分割输出继承点云 header。
- 融合输出使用 LiDAR header。
- GNSS 消息 header 来自惯导 GPS 时间或 rosbag。

如果某个节点发布时用 `now()` 替代原始传感器时间，可能导致时间同步偏差。

## 5. QoS

高频 GNSS 订阅：

```cpp
rclcpp::SensorDataQoS()
```

适合传感器流，通常是 Best Effort。

其他 message_filters subscriber 使用默认 QoS。rosbag 回放时如果收不到消息，需要检查 publisher 和 subscriber QoS 是否兼容。

## 6. 常见同步问题

### 6.1 syncCallback 不触发

检查：

```bash
ros2 topic hz /gongji_gnss_ins_64
ros2 topic hz /perception/fusion/map
```

再检查：

```bash
ros2 topic echo /gongji_gnss_ins_64 --once
ros2 topic echo /perception/fusion/map --once
```

看 header 时间戳是否相近。

### 6.2 感知输出有，但 SLAM 不建图

可能原因：

- `is_origin_set_` 还没设置。
- `/perception/fusion/map` 全是 `UNKNOWN`。
- 时间同步没有配对。
- GNSS 和感知的 ROS_DOMAIN_ID 不一致。

### 6.3 地图锥桶沿行驶方向错位

可能原因：

- 感知 header 没继承原始传感器时间。
- 使用最新位姿而不是同步历史位姿。
- rosbag 播放速率过高，节点处理延迟明显。

## 7. 复现建议

最小复现时也应保留同步：

```cpp
message_filters::Subscriber<Gnss> gnss_sub;
message_filters::Subscriber<Map> map_sub;
Synchronizer<ApproximateTime<Gnss, Map>> sync;
sync.registerCallback(syncCallback);
```

不要在建图回调里简单读取一个全局 latest pose，除非车速很低或你能做时间插值。

