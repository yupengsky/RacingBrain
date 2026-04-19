# SLAM 节点生命周期与参数

## 1. 模块位置

SLAM 核心代码位于：

```text
slam/slam/src/slam_node.cpp
slam/slam/src/loop_closure_detector.cpp
slam/slam/include/slam/loop_closure_detector.hpp
```

ROS 包：

```text
slam
```

构建目标：

```text
slam_node
```

节点类：

```cpp
class SlamProcessor : public rclcpp::Node
```

节点名：

```text
slam_processor
```

## 2. main 函数

入口：

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

含义：

1. 初始化 ROS 2。
2. 创建 `SlamProcessor`。
3. 进入 spin，等待回调。
4. 退出时 shutdown。

整个 SLAM 的初始化都发生在 `SlamProcessor` 构造函数中。

## 3. 构造函数执行顺序

构造函数：

```cpp
SlamProcessor() : Node("slam_processor"), is_origin_set_(false)
```

执行步骤：

1. 调用 `loadParameters()`。
2. 创建 PROJ 坐标转换对象。
3. 创建高频 GNSS 订阅。
4. 创建 GNSS + 感知同步订阅。
5. 创建发布器。
6. 打印启动信息。

## 4. 参数文件加载方式

launch 文件：

```text
slam/slam/launch/slam.launch.py
```

核心逻辑：

```python
base_params = os.path.join(pkg_share, "config", "params.yaml")
track_params = os.path.join(pkg_share, "config", track + ".yaml")

slam_node = Node(
    package="slam",
    executable="slam_node",
    name="slam_processor",
    output="screen",
    parameters=[base_params, track_params]
)
```

默认参数：

```python
DeclareLaunchArgument(
    "track",
    default_value="acceleration"
)
```

因此默认加载：

```text
params.yaml
acceleration.yaml
```

如果运行：

```bash
ros2 launch slam slam.launch.py track:=autocross
```

则加载：

```text
params.yaml
autocross.yaml
```

## 5. 赛道类型

枚举：

```cpp
enum class TrackType {
    AUTOCROSS,
    ACCELERATION,
    SKIDPAD
};
```

参数读取：

```cpp
this->declare_parameter("track_type", "acceleration");
track_type_str_ = this->get_parameter("track_type").as_string();

if (track_type_str_ == "autocross") current_track_type_ = TrackType::AUTOCROSS;
else if (track_type_str_ == "skidpad") current_track_type_ = TrackType::SKIDPAD;
else current_track_type_ = TrackType::ACCELERATION;
```

支持：

- `acceleration`
- `autocross`
- `skidpad`

默认：

```text
acceleration
```

## 6. SystemParams

结构：

```cpp
struct SystemParams {
    std::string gnss_topic;
    std::string lidar_topic;
    std::string map_frame;
    std::string base_frame;
    std::string utm_zone_epsg;
} sys_;
```

默认参数：

```cpp
system.gnss_topic = "/gongji_gnss_ins_64"
system.lidar_topic = "/perception/fusion/map"
system.map_frame = "map"
system.base_frame = "base_link"
system.utm_zone_epsg = "EPSG:32651"
```

注意：`lidar_topic` 实际是融合后的感知地图，不是原始 LiDAR 点云。

## 7. MappingParams

结构：

```cpp
struct MappingParams {
    double kf_q_base;
    double kf_p_init;
    double mahalanobis_thresh;
    double max_match_dist;
    double max_lidar_range;
    double lidar_blind_range;
    double fov_angle;
    double sigma_long;
    double sigma_lat;
    double l_hit;
    double l_miss;
    double l_min;
    double l_max;
    double l_stable;
    double distance_factor;
    double speed_factor;
    double gyro_factor;
    double lat_factor;
    double half_tube_width;
    double max_tube_length;
    double max_straight_angular;
} params_;
```

参数分组解释：

### 7.1 卡尔曼滤波参数

```text
kf_q_base
kf_p_init
```

- `kf_q_base`：过程噪声，防止 P 无限变小。
- `kf_p_init`：新锥桶初始不确定度。

### 7.2 数据关联参数

```text
mahalanobis_thresh
max_match_dist
```

- `mahalanobis_thresh`：马氏距离门限。
- `max_match_dist`：马氏距离前的欧氏距离粗筛。

### 7.3 感知范围参数

```text
max_lidar_range
lidar_blind_range
fov_angle
```

用于判断锥桶是否在当前视野内。

### 7.4 观测噪声参数

```text
sigma_long
sigma_lat
distance_factor
speed_factor
gyro_factor
lat_factor
```

用于构造动态协方差。

### 7.5 存在评分参数

```text
l_hit
l_miss
l_min
l_max
l_stable
```

用于 hit/miss 生命值管理。

### 7.6 阿克曼 ROI 参数

```text
half_tube_width
max_tube_length
max_straight_angular
```

用于限制新锥桶生成区域。

## 8. 外参参数

参数名：

```text
extrinsics.t_l2v
```

含义：

```text
T_lidar_to_vehicle
```

代码将 16 个 double 读成 4x4 矩阵：

```cpp
std::vector<double> ext_vec = this->get_parameter("extrinsics.t_l2v").as_double_array();
if(ext_vec.size() == 16) {
    T_l2v_ = Eigen::Matrix4d::Identity();
    T_l2v_ << ext_vec[0],  ext_vec[1],  ext_vec[2],  ext_vec[3],
              ext_vec[4],  ext_vec[5],  ext_vec[6],  ext_vec[7],
              ext_vec[8],  ext_vec[9],  ext_vec[10], ext_vec[11],
              ext_vec[12], ext_vec[13], ext_vec[14], ext_vec[15];
}
```

如果长度不是 16，使用单位矩阵。

## 9. PROJ 坐标转换对象

构造函数中：

```cpp
P_ = proj_create_crs_to_crs(
    PJ_DEFAULT_CTX,
    "EPSG:4326",
    sys_.utm_zone_epsg.c_str(),
    NULL
);
```

含义：

- 输入 CRS：`EPSG:4326`，WGS84 经纬度。
- 输出 CRS：配置项 `system.utm_zone_epsg`。
- 默认 `EPSG:32651`。

析构函数：

```cpp
~SlamProcessor() {
    if (P_) proj_destroy(P_);
}
```

## 10. 订阅器

### 10.1 高频 GNSS 订阅

```cpp
high_freq_gnss_sub_ = this->create_subscription<gnss_ins_msg::msg::Gnssins64>(
    sys_.gnss_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SlamProcessor::fastGnssCallback, this, _1)
);
```

用途：

- 实时发布 TF。
- 实时发布 Odom。
- 实时发布 Path。

不参与地图更新。

### 10.2 同步 GNSS 与感知订阅

```cpp
gnss_sub_.subscribe(this, sys_.gnss_topic);
perception_sub_.subscribe(this, sys_.lidar_topic);
```

同步器：

```cpp
typedef message_filters::sync_policies::ApproximateTime<
    gnss_ins_msg::msg::Gnssins64,
    drd25_msgs::msg::Map
> SyncPolicy;

sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(800), gnss_sub_, perception_sub_
);
```

用途：

- 地图更新。
- 保证感知观测和位姿时间近似对齐。

## 11. 发布器

```cpp
global_map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_map", 10);
path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vehicle_path", 10);
odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("vehicle_odom", 10);
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
```

输出：

- `/global_map`
- `/vehicle_path`
- `/vehicle_odom`
- TF `map -> base_link`

## 12. SLAM 内部地图状态

主要成员：

```cpp
std::vector<GlobalCone> cone_map_;
std::mutex map_mutex_;
long long next_cone_id_ = 0;
std::atomic<bool> map_locked_{false};
int lap_count_ = 0;
```

`cone_map_` 是全局锥桶仓库。

`map_mutex_` 保护地图更新。

`next_cone_id_` 给新锥桶分配 marker ID。

`map_locked_` 表示回环后是否锁图。

## 13. GlobalCone 结构

定义在：

```text
slam/slam/include/slam/loop_closure_detector.hpp
```

结构：

```cpp
struct GlobalCone {
    int id;
    Eigen::Vector2d pos;
    Eigen::Matrix2d P;
    float r, g, b;
    bool is_stable = false;
    double existence_score = 0.0;
    bool matched_this_frame = false;
    int type;
};
```

字段解释：

- `id`：全局锥桶 ID。
- `pos`：全局二维位置。
- `P`：位置协方差。
- `r,g,b`：可视化颜色。
- `is_stable`：是否稳定。
- `existence_score`：存在评分。
- `matched_this_frame`：本帧是否匹配成功。
- `type`：锥桶颜色枚举。

## 14. 回调执行关系

SLAM 运行后有两条回调线：

```text
GNSS 高频消息
  -> fastGnssCallback
  -> 发布 TF/Odom/Path

GNSS + 感知同步消息
  -> syncCallback
  -> 更新 cone_map_
  -> publishGlobalMap
```

这说明：车辆轨迹发布和地图更新是解耦的。

## 15. 节点生命周期最小复现

```cpp
class SlamProcessor : public rclcpp::Node {
public:
    SlamProcessor() : Node("slam_processor") {
        loadParameters();
        initProj();
        initSubscribers();
        initPublishers();
    }

private:
    void fastGnssCallback(Gnssins64 msg);
    void syncCallback(Gnssins64 msg, Map map);
    void updateMap(...);
    void publishGlobalMap(...);
};
```

复现优先级：

1. 参数加载。
2. PROJ 经纬度转局部坐标。
3. GNSS + Map 同步订阅。
4. `cone_map_` 数据结构。
5. `updateMap`。
6. `publishGlobalMap`。

