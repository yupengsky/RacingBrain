# GNSS 位姿与坐标转换

## 1. 本章目标

SLAM 后端要把感知锥桶从车身附近的局部坐标变成全局地图坐标。这个过程依赖车辆当前位姿，而车辆位姿来自 GNSS/INS。

本章解释：

1. 经纬度如何转成米制坐标。
2. 第一帧如何设为地图原点。
3. roll/pitch/yaw 如何转成车辆姿态。
4. 车辆 TF、Odom、Path 如何发布。
5. 局部锥桶如何变换到全局坐标。

## 2. 经纬度转 UTM

代码中使用 PROJ：

```cpp
P_ = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", sys_.utm_zone_epsg.c_str(), NULL);
```

回调中：

```cpp
PJ_COORD input_coord = proj_coord(gnss_msg->latitude, gnss_msg->longitude, 0, 0);
PJ_COORD output_coord = proj_trans(P_, PJ_FWD, input_coord);
double utm_e = output_coord.xy.x;
double utm_n = output_coord.xy.y;
```

注意：常见 PROJ 接口可能有经纬度顺序差异。当前代码按：

```text
proj_coord(latitude, longitude, 0, 0)
```

传入。复现时应与当前代码保持一致，除非专门验证并修正坐标轴顺序。

## 3. 地图原点

成员变量：

```cpp
bool is_origin_set_;
double origin_e_, origin_n_;
```

首次收到有效 GNSS：

```cpp
if (!is_origin_set_) {
    origin_e_ = utm_e;
    origin_n_ = utm_n;
    is_origin_set_ = true;
    return;
}
```

之后局部地图坐标：

```cpp
double tx = utm_e - origin_e_;
double ty = utm_n - origin_n_;
double tz = 0.0;
```

含义：

```text
map 原点 = 第一帧 GNSS 位置
map x = 当前 UTM Easting - 起点 Easting
map y = 当前 UTM Northing - 起点 Northing
```

## 4. 姿态构造

输入：

```text
roll, pitch, yaw，单位 degree
```

转换：

```cpp
Eigen::AngleAxisd roll(gnss_msg->roll * M_PI/180.0, Eigen::Vector3d::UnitX());
Eigen::AngleAxisd pitch(gnss_msg->pitch * M_PI/180.0, Eigen::Vector3d::UnitY());
Eigen::AngleAxisd yaw(gnss_msg->yaw * M_PI/180.0, Eigen::Vector3d::UnitZ());
Eigen::Matrix3d R_imu = (yaw * pitch * roll).toRotationMatrix();
```

这里旋转组合顺序为：

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

也就是常见 ZYX 顺序。

## 5. IMU 到车辆坐标修正

代码定义：

```cpp
Eigen::Matrix3d R_fix;
R_fix << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
Eigen::Matrix3d R_vehicle = R_imu * R_fix;
```

矩阵：

```text
[ 0 -1 0
  1  0 0
  0  0 1 ]
```

等价于绕 z 轴旋转 90 度。

用途：

- 把 IMU 的轴定义转换到 ROS 车辆坐标。
- ROS 车辆坐标默认 `x` 前、`y` 左。

最终车辆旋转是：

```text
R_vehicle = R_imu * R_fix
```

## 6. 车辆位姿矩阵

同步回调中构造：

```cpp
Eigen::Matrix4d historical_T_veh_to_map = Eigen::Matrix4d::Identity();
historical_T_veh_to_map.block<3,3>(0,0) = R_vehicle;
historical_T_veh_to_map.block<3,1>(0,3) = Eigen::Vector3d(tx, ty, tz);
```

矩阵形式：

```text
T_vehicle_to_map =
[ R_vehicle  t
  0 0 0      1 ]
```

其中：

```text
t = [tx, ty, 0]^T
```

## 7. 高频 GNSS 回调

函数：

```cpp
fastGnssCallback
```

职责：

- 经纬度转局部位置。
- 姿态转旋转矩阵。
- 更新 `T_veh_to_map_`。
- 发布 TF、Odometry、Path。

重要说明：

```cpp
// T_veh_to_map_ 是独属于高频发布轨迹回调函数的，不参与建图
```

也就是说：

- 高频 GNSS 用于实时显示车辆轨迹。
- 同步 GNSS 用于建图。

## 8. TF 发布

函数：

```cpp
publishTfAndOdom
```

TF：

```cpp
geometry_msgs::msg::TransformStamped t;
t.header.stamp = header.stamp;
t.header.frame_id = sys_.map_frame;
t.child_frame_id = sys_.base_frame;
t.transform.translation.x = tx;
t.transform.translation.y = ty;
t.transform.translation.z = tz;
t.transform.rotation = q;
tf_broadcaster_->sendTransform(t);
```

默认：

```text
map -> base_link
```

## 9. Odometry 发布

位置和姿态：

```cpp
odom_msg.header = t.header;
odom_msg.child_frame_id = sys_.base_frame;
odom_msg.pose.pose.position.x = tx;
odom_msg.pose.pose.position.y = ty;
odom_msg.pose.pose.position.z = tz;
odom_msg.pose.pose.orientation = t.transform.rotation;
```

速度从地图系转到车体系：

```cpp
Eigen::Vector3d v_map(vel_e, vel_n, 0.0);
Eigen::Vector3d v_vehicle = R_vehicle.transpose() * v_map;
odom_msg.twist.twist.linear.x = v_vehicle.x();
odom_msg.twist.twist.linear.y = v_vehicle.y();
```

为什么用转置？

旋转矩阵正交：

```text
R^-1 = R^T
```

如果：

```text
v_map = R_vehicle * v_vehicle
```

则：

```text
v_vehicle = R_vehicle^T * v_map
```

## 10. Path 发布

代码：

```cpp
geometry_msgs::msg::PoseStamped current_pose;
current_pose.header = t.header;
current_pose.pose.position.x = tx;
current_pose.pose.position.y = ty;
current_pose.pose.position.z = tz;
current_pose.pose.orientation = t.transform.rotation;
path_msg_.poses.push_back(current_pose);
path_pub_->publish(path_msg_);
```

当前代码没有限制 path 长度，长时间运行可能累积较多 pose。

## 11. 当前速度和 yaw rate

同步回调中：

```cpp
current_speed_ = caculateSpeed(gnss_msg);
current_gyro_z_ = caculateYawRate(gnss_msg);
```

速度：

```cpp
return std::sqrt(msg->vel_e * msg->vel_e + msg->vel_n * msg->vel_n);
```

yaw rate：

```cpp
return msg->imu_gyro_z * M_PI / 180.0;
```

用途：

- `current_speed_` 用于锁图后动态观测噪声。
- `current_gyro_z_` 用于横向噪声调整和阿克曼 ROI。

## 12. 局部锥桶转全局

感知地图中的锥桶：

```cpp
cone_obs.x
cone_obs.y
```

构造：

```cpp
Eigen::Vector4d p_lidar(cone_obs.x, cone_obs.y, 0.0, 1.0);
```

变换：

```cpp
Eigen::Vector4d p_global = historical_T_veh_to_map * T_l2v_ * p_lidar;
```

最终传入地图更新：

```cpp
updateMap(p_global.x(), p_global.y(), ...);
```

## 13. 为什么要用同步回调的 historical_T

如果直接使用 `fastGnssCallback` 中的最新 `T_veh_to_map_`：

- 感知帧可能是 100 ms 前采集的。
- 车辆高速运动时已经前进明显距离。
- 锥桶全局位置会被投错。

同步回调用与感知帧时间接近的 GNSS 消息构造 `historical_T_veh_to_map`，减少时间错位。

## 14. 位姿复现最小代码

```cpp
class PoseEstimator {
    PJ* P;
    bool origin_set = false;
    double origin_e, origin_n;

    Pose compute(Gnss msg) {
        utm = proj_trans(P, msg.latitude, msg.longitude);

        if (!origin_set) {
            origin_e = utm.e;
            origin_n = utm.n;
            origin_set = true;
        }

        tx = utm.e - origin_e;
        ty = utm.n - origin_n;

        R_imu = Rz(deg2rad(msg.yaw)) *
                Ry(deg2rad(msg.pitch)) *
                Rx(deg2rad(msg.roll));

        R_fix = [[0,-1,0],
                 [1, 0,0],
                 [0, 0,1]];

        R_vehicle = R_imu * R_fix;

        T = Identity4;
        T.rotation = R_vehicle;
        T.translation = [tx, ty, 0];
        return Pose(T);
    }
};
```

## 15. 常见问题

### 15.1 地图旋转了 90 度

优先检查：

- `R_fix` 是否一致。
- IMU yaw 定义。
- ROS 车辆坐标定义。

### 15.2 地图整体位置漂移

优先检查：

- GNSS 经纬度精度。
- UTM zone 是否正确。
- 是否用 `float32` 经纬度。
- rosbag 时间戳是否对齐。

### 15.3 锥桶前后错位

优先检查：

- GNSS 与融合地图同步。
- `T_l2v_` 平移外参。
- 感知消息 header 是否继承原始传感器时间。

