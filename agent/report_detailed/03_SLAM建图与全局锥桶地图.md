# 03 SLAM 建图与全局锥桶地图

这一份文档讲后半段。

起点是：

- `gongji_gnss_ins_64`
- `/perception/fusion/map`

终点是：

- `/global_map`

核心文件只有两个：

- `slam/slam/src/slam_node.cpp`
- `slam/slam/src/loop_closure_detector.cpp`

如果你只想知道“全局锥桶地图到底怎么做出来”，这一份就是主文档。

## 1. 先给结论

这个工程做全局锥桶地图的办法很直接。

它没有做复杂图优化。

它的主方法是：

1. 用 GNSS/INS 给每一帧锥桶观测一个全局位姿。
2. 把局部锥桶坐标投到全局平面。
3. 对每个全局锥桶做数据关联。
4. 匹配上就卡尔曼更新。
5. 匹配不上就新建锥桶。
6. 每帧再做一次命中加分、漏检减分和稳定性判断。
7. 回环成功后锁图。

全局地图就这样一点点长出来。

## 2. `slam_node` 启动时做了什么

主类是：

- `SlamProcessor`

定义在：

- `slam/slam/src/slam_node.cpp:39-878`

构造函数在：

- `slam_node.cpp:42-87`

启动时做 5 件事。

### 2.1 加载参数

函数：

- `loadParameters`

位置：

- `slam_node.cpp:132-237`

它会加载三组参数：

1. 系统参数
2. 建图参数
3. 回环参数

系统参数包括：

- `system.gnss_topic`
- `system.lidar_topic`
- `system.map_frame`
- `system.base_frame`
- `system.utm_zone_epsg`

建图参数包括：

- `kf_q_base`
- `kf_p_init`
- `mahalanobis_thresh`
- `max_match_dist`
- `max_lidar_range`
- `lidar_blind_range`
- `fov_angle`
- `l_hit`
- `l_miss`
- `l_min`
- `l_max`
- `l_stable`
- `sigma_long`
- `sigma_lat`
- `distance_factor`
- `speed_factor`
- `gyro_factor`
- `lat_factor`
- `half_tube_width`
- `max_tube_length`
- `max_straight_angular`

回环参数包括：

- `distance_threshold`
- `approach_angle_threshold`
- `min_consecutive_detections`
- `max_consecutive_detections`
- `min_time_between_detections`
- `history_buffer_size`
- `min_approach_speed`
- `min_lap_time`

### 2.2 创建 PROJ 坐标变换器

代码：

- `slam_node.cpp:48-56`

用法：

```cpp
P_ = proj_create_crs_to_crs(
    PJ_DEFAULT_CTX,
    "EPSG:4326",
    sys_.utm_zone_epsg.c_str(),
    NULL
);
```

意思很清楚：

- 输入是经纬度 `EPSG:4326`
- 输出是某个 UTM 平面坐标

默认配置在：

- `slam/slam/config/params.yaml:4-17`

默认值：

- `EPSG:32651`

### 2.3 创建两个 GNSS 通道

代码：

- `slam_node.cpp:58-74`

这里有两个 GNSS 入口。

#### A. 高频 GNSS 订阅

代码：

- `high_freq_gnss_sub_`

回调：

- `fastGnssCallback`

作用：

- 实时更新车辆位姿
- 发布 TF
- 发布 Odom
- 发布 Path

#### B. 时间同步 GNSS 订阅

代码：

- `gnss_sub_`
- `perception_sub_`
- `sync_`

回调：

- `syncCallback`

作用：

- 把 GNSS 和融合锥桶观测按时间对齐
- 在同一时刻位姿下更新地图

这个设计很重要。

它把“实时显示”和“按同步时间建图”拆开了。

### 2.4 创建发布器

代码：

- `slam_node.cpp:76-81`

输出有：

- `/global_map`
- `vehicle_path`
- `vehicle_odom`

## 3. GNSS 位姿怎么变成地图位姿

相关代码在：

- `fastGnssCallback`
- `syncCallback`

位置：

- `slam_node.cpp:239-281`
- `slam_node.cpp:283-364`

## 3.1 经纬度先转 UTM

做法在两个回调里一样。

代码：

- `slam_node.cpp:244-247`
- `slam_node.cpp:290-295`

流程：

1. `proj_coord(latitude, longitude, 0, 0)`
2. `proj_trans(P_, PJ_FWD, input_coord)`
3. 取 `output_coord.xy.x` 和 `output_coord.xy.y`

输出：

- `utm_e`
- `utm_n`

## 3.2 第一帧设原点

代码：

- `slam_node.cpp:249-255`

逻辑：

1. 第一帧 GNSS 到来时
2. 记录 `origin_e_` 和 `origin_n_`
3. 后续所有位置都减这个原点

这样地图坐标就从 0 附近起步。

## 3.3 姿态角怎么转成车体旋转矩阵

代码：

- `slam_node.cpp:262-277`
- `slam_node.cpp:311-327`

流程分两步。

### 第一步：把 roll pitch yaw 变成 `R_imu`

```cpp
Eigen::AngleAxisd roll(..., Eigen::Vector3d::UnitX());
Eigen::AngleAxisd pitch(..., Eigen::Vector3d::UnitY());
Eigen::AngleAxisd yaw(..., Eigen::Vector3d::UnitZ());
Eigen::Matrix3d R_imu = (yaw * pitch * roll).toRotationMatrix();
```

### 第二步：乘一个修正矩阵 `R_fix`

代码：

- `slam_node.cpp:267-271`
- `slam_node.cpp:323-327`

```cpp
R_fix << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
R_vehicle = R_imu * R_fix;
```

这一步的作用是把 IMU 坐标定义转成工程里使用的车体系定义。

## 3.4 位姿矩阵怎么组

在两个回调里都会拼 `4x4` 齐次矩阵。

### 高频实时位姿

变量：

- `T_veh_to_map_`

代码：

- `slam_node.cpp:273-280`

### 同步历史位姿

变量：

- `historical_T_veh_to_map`

代码：

- `slam_node.cpp:329-333`

这个变量很关键。

它代表和当前锥桶观测时间对齐的车辆位姿。

地图更新用的是它。

## 3.5 速度和角速度怎么来

代码：

- `caculateSpeed`
- `caculateYawRate`

位置：

- `slam_node.cpp:540-549`

速度：

```cpp
sqrt(vel_e^2 + vel_n^2)
```

角速度：

```cpp
imu_gyro_z * PI / 180.0
```

这两个量后面会用在观测噪声建模和阿克曼 ROI 里。

## 4. 地图里的锥桶长什么样

结构体定义在：

- `slam/slam/include/slam/loop_closure_detector.hpp:13-23`

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

这 8 个字段就是全局锥桶地图的核心状态。

### 4.1 `id`

地图内唯一编号。

### 4.2 `pos`

全局二维平面坐标。

### 4.3 `P`

二维协方差。

它记录位置不确定度。

### 4.4 `r g b`

RViz 上色用。

### 4.5 `type`

颜色类别枚举。

这个字段也参与数据关联。

### 4.6 `matched_this_frame`

本帧有没有被观测命中。

每帧一开始会先清零。

代码：

- `slam_node.cpp:343-346`

### 4.7 `existence_score`

存在分数。

它是这个工程压噪声的核心。

### 4.8 `is_stable`

是否稳定。

只有稳定锥桶才会发布到 `/global_map`。

## 5. 同步回调里地图怎么更新

核心回调：

- `syncCallback`

位置：

- `slam_node.cpp:283-364`

这个回调可以拆成 6 步。

### 第一步：检查基础条件

```cpp
if (!P_ || !is_origin_set_) return;
```

没有坐标变换器，或者原点还没定，就不建图。

### 第二步：算当前同步位姿

也就是前面讲过的：

- 经纬度转 UTM
- 减原点
- 姿态角转旋转矩阵
- 拼 `historical_T_veh_to_map`

### 第三步：更新实时速度和角速度

代码：

- `slam_node.cpp:334-336`

### 第四步：给地图上锁

代码：

- `std::lock_guard<std::mutex> lock(map_mutex_);`

位置：

- `slam_node.cpp:339-341`

### 第五步：把所有锥桶先标记成“本帧未匹配”

代码：

- `slam_node.cpp:343-346`

### 第六步：按赛道模式走不同建图逻辑

代码：

- `slam_node.cpp:348-359`

分三类：

1. `processAccelerationTrack`
2. `processAutocrossTrack`
3. `processSkidpadTrack`

最后统一：

- `publishGlobalMap(stamp);`

## 6. 直线赛和绕桩赛的前端观测处理

`processAccelerationTrack` 和 `processAutocrossTrack` 的前半段几乎一样。

代码：

- `slam_node.cpp:366-417`
- `slam_node.cpp:419-507`

### 6.1 跳过未知颜色锥桶

代码：

- `cone_obs.color == drd25_msgs::msg::Cone::UNKNOWN`

位置：

- `374-375`
- `429-431`

SLAM 不接收未知颜色。

### 6.2 把局部锥桶转全局锥桶

代码：

- `slam_node.cpp:377-380`
- `slam_node.cpp:433-436`

公式：

```cpp
p_global = historical_T_veh_to_map * T_l2v_ * p_lidar;
```

这一步很关键。

它把：

- LiDAR 局部观测
- 车辆同步位姿
- LiDAR 到车体外参

合成一个全局平面点。

### 6.3 把类别转成 RGB

函数：

- `parseConeColor`

位置：

- `slam_node.cpp:519-538`

规则：

- 蓝桶 -> 蓝色
- 红桶 -> 红色
- 黄桶和大黄桶 -> 黄色
- 未知 -> 灰色

### 6.4 算观测距离

代码：

- `slam_node.cpp:388`
- `slam_node.cpp:444`

```cpp
dist = sqrt(cone_obs.x^2 + cone_obs.y^2)
```

这里用的是局部坐标下的传感器距离。

它后面会影响命中奖励和观测协方差。

### 6.5 把观测送进 `updateMap`

代码：

- `slam_node.cpp:392`
- `slam_node.cpp:448`

这个函数就是地图更新核心。

## 7. `updateMap` 才是地图核心

函数：

- `updateMap`

位置：

- `slam_node.cpp:650-776`

这一段代码一定要读透。

你如果自己重写全局锥桶地图，这一段就是模板。

## 7.1 输入是什么

函数签名：

```cpp
void updateMap(
    double gx, double gy,
    double cone_x, double cone_y,
    float r, float g, float b,
    int type,
    double dist_to_sensor,
    const Eigen::Matrix2d& vehicle_R)
```

含义：

- `gx gy`：当前观测的全局位置
- `cone_x cone_y`：当前观测的局部位置
- `r g b`：颜色
- `type`：锥桶类型
- `dist_to_sensor`：观测距离
- `vehicle_R`：当前车体在地图中的二维旋转

## 7.2 第一步：把观测写成二维向量

代码：

- `slam_node.cpp:653-654`

```cpp
Eigen::Vector2d z(gx, gy);
```

这里的 `z` 就是当前测量值。

## 7.3 第二步：按距离算命中奖励

代码：

- `slam_node.cpp:656-662`

逻辑：

1. 离传感器越远，观测越不可信
2. 所以命中加分要变小

实现：

```cpp
confidence = 1.0 - dist/max_range * 0.6
confidence = max(0.4, confidence)
dynamic_l_hit = l_hit * confidence
```

这一步很工程化。

它把“近处观测更可靠”直接写进地图更新。

## 7.4 第三步：构造观测噪声协方差

代码：

- `slam_node.cpp:664-690`

这部分是整套地图里最像“滤波算法”的地方。

### 先在车体系里建噪声

```cpp
sigma_long = params_.sigma_long
sigma_lat  = params_.sigma_lat + params_.lat_factor * abs(current_gyro_z_)
```

意思很清楚：

- 纵向噪声固定
- 横向噪声会随转弯变大

然后组成：

```cpp
R_body = [[sigma_long^2, 0],
          [0, sigma_lat^2]]
```

### 再按距离放大

代码：

- `ratio = dist_to_sensor / distance_factor`
- `overall_penalty = 1 + ratio^2`

距离越远，协方差越大。

### 锁图后再按车速和角速度放大

代码：

- `slam_node.cpp:679-685`

锁图后说明车已经进入更快的圈。

代码会再加一层惩罚。

这里有一个很值得记住的实现细节。

`yaw_rate_penalty` 这一行写的是：

```cpp
double yaw_rate_penalty = params_.speed_factor * std::abs(current_gyro_z_);
```

位置：

- `slam_node.cpp:681`

按变量命名看，这里本来很像想用 `gyro_factor`。

但当前代码实际复用了 `speed_factor`。

如果你要完全按原工程复现，这个行为要原样保留。

如果你要做自己的版本，建议把它改成：

```cpp
params_.gyro_factor * abs(current_gyro_z_)
```

### 最后转到地图系

代码：

- `slam_node.cpp:687-689`

```cpp
R = vehicle_R * R_body * vehicle_R.transpose()
```

这样观测协方差就跟车的朝向一起转到了地图坐标系。

## 7.5 第四步：做数据关联

代码：

- `slam_node.cpp:692-721`

数据关联分三层。

### 第一层：颜色必须一致

```cpp
if (map_cone.type != type) continue;
```

### 第二层：欧氏距离粗筛

```cpp
delta = z - map_cone.pos;
if (delta.norm() > max_match_dist) continue;
```

这一步能挡掉很多离群点。

### 第三层：马氏距离精筛

```cpp
S = map_cone.P + R
m_dist = sqrt(delta^T * S.inverse() * delta)
```

从所有候选里取最小的那个。

阈值是：

- `mahalanobis_thresh`

## 7.6 第五步：匹配成功就做卡尔曼更新

代码：

- `slam_node.cpp:723-744`

流程：

1. 先给旧锥桶协方差加过程噪声 `Q`
2. 算卡尔曼增益 `K`
3. 更新位置
4. 更新协方差
5. 标记 `matched_this_frame = true`
6. 增加存在分数
7. 再判断是否稳定

具体公式：

```cpp
P = P + Q
S = P + R
K = P * S.inverse()
pos = pos + K * (z - pos)
P = (I - K) * P
```

命中奖励：

```cpp
existence_score += dynamic_l_hit
existence_score = min(existence_score, l_max)
```

## 7.7 第六步：没匹配上就新建锥桶

代码：

- `slam_node.cpp:746-775`

这里有两个前置条件。

### 条件 1：锁图后不再加新锥桶

```cpp
if (map_locked_) return;
```

### 条件 2：锥桶必须落在阿克曼 ROI 里

```cpp
if (!isInAckermannTube(cone_x, cone_y)) return;
```

通过后，才会新建：

```cpp
new_cone.id = next_cone_id_++
new_cone.pos = z
new_cone.P = I * kf_p_init
new_cone.type = type
new_cone.is_stable = false
new_cone.matched_this_frame = true
new_cone.existence_score = dynamic_l_hit
```

这里你要注意一件事。

新锥桶刚进地图时通常还不稳定。

它要靠后续多次命中，才能过 `l_stable` 门槛。

## 8. 地图维护怎么做

地图维护逻辑写在：

- `processAccelerationTrack`
- `processAutocrossTrack`

位置：

- `slam_node.cpp:396-416`
- `slam_node.cpp:452-473`

### 8.1 每帧先重置匹配标记

代码：

- `slam_node.cpp:343-346`

### 8.2 对“该看到却没看到”的锥桶减分

判断函数：

- `isInFieldOfView`

代码：

- `slam_node.cpp:604-626`

逻辑：

1. 把锥桶从全局坐标变回当前车体系
2. 计算前向距离和侧向角度
3. 看它是否在 LiDAR 视场里

条件是：

- 在车前方 `x > 0`
- 距离大于盲区半径
- 距离小于最大感知距离
- 夹角小于半视场角

如果在视场里，又没被匹配上，就执行：

```cpp
existence_score += l_miss
```

因为 `l_miss` 是负数，所以这就是减分。

### 8.3 分数低到阈值就删掉

加速度赛：

```cpp
if (existence_score <= l_min) erase
```

绕桩赛：

```cpp
if (existence_score < l_min) erase
```

代码分别在：

- `slam_node.cpp:405-408`
- `slam_node.cpp:461-464`

### 8.4 分数高到阈值就标记稳定

代码：

- `slam_node.cpp:411`
- `slam_node.cpp:467`

```cpp
is_stable = (existence_score > l_stable)
```

这一步决定锥桶能不能进最终发布地图。

## 9. 阿克曼 ROI 在干什么

函数：

- `isInAckermannTube`

位置：

- `slam_node.cpp:628-648`

这一步只在“新锥桶初始化”时用。

目的很简单：

只让更可能属于赛道边界的目标进图。

### 9.1 直线时

如果角速度很小：

```cpp
abs(y) < half_tube_width
```

ROI 就是车前方一个长方形通道。

### 9.2 转弯时

如果角速度不小：

1. 用 `R = v / yaw_rate` 算转弯半径
2. 以转弯圆为中心构一个管道
3. 看锥桶到这条圆轨迹的横向偏差

代码：

- `slam_node.cpp:642-647`

这个思路很适合赛车场景。

它能减少跑偏目标和路边杂物入图。

## 10. 回环检测怎么做

核心文件：

- `slam/slam/src/loop_closure_detector.cpp`

主函数：

- `detectLoopClosure`

位置：

- `loop_closure_detector.cpp:71-131`

## 10.1 回环检测前先设原点

代码：

- `slam_node.cpp:296-300`

第一次进入同步回调时：

```cpp
loop_detector_.setOrigin(Eigen::Vector3d(0.0, 0.0, 0.0));
```

因为整张地图本来就以起点为原点。

## 10.2 回环检测用了哪几条条件

代码：

- `loop_closure_detector.cpp:84-127`

一共 4 条。

### 条件 1：最小圈时

```cpp
elapsed_time >= min_lap_time_
```

### 条件 2：离起点够近

```cpp
distance_to_origin < distance_threshold_
```

### 条件 3：运动方向朝向起点

函数：

- `isMovingTowardsOrigin`

位置：

- `loop_closure_detector.cpp:152-170`

做法：

1. 取最近两帧位置差，估计运动方向
2. 算当前位置指向起点的向量
3. 算两向量夹角
4. 小于阈值才通过

### 条件 4：接近速度够大

函数：

- `hasSufficientApproachSpeed`

位置：

- `loop_closure_detector.cpp:172-183`

做法：

1. 用最近两帧位置差除以时间差估速度
2. 取速度在“朝向起点方向”上的投影
3. 大于阈值才通过

## 10.3 为什么有连续帧计数

代码：

- `loop_closure_detector.cpp:100-127`

这是为了抗抖动。

单帧满足条件不够。

它要求连续多帧都满足，才真触发。

探索圈和冲刺圈要求不同：

- 探索圈用 `max_consecutive_detections_`
- 冲刺圈用 `min_consecutive_detections_`

代码：

- `loop_closure_detector.cpp:105`

## 10.4 触发回环后系统怎么做

逻辑在：

- `processAutocrossTrack`

位置：

- `slam_node.cpp:475-505`

如果 `loop_closed == true`：

1. `lap_count_++`
2. 如果地图还没锁：
   - 说明第一圈完成
   - 删除所有不稳定锥桶
   - `map_locked_ = true`
3. 如果地图已锁：
   - 说明是后续圈
   - 只记圈数
4. 调 `resetLoopStatus()`

### 地图锁住后会发生什么

两件事最关键。

#### 1. 不再新增锥桶

代码：

- `updateMap` 里的 `if (map_locked_) return;`

#### 2. 绕桩模式里停止 miss 删除流程

代码：

- `processAutocrossTrack:452-473`

第一圈结束后，地图进入“只微调，不扩张”的状态。

这很符合赛车场景。

## 11. 全局地图最后怎么发布

函数：

- `publishGlobalMap`

位置：

- `slam_node.cpp:778-843`

流程很清楚。

### 11.1 每次先发 DELETEALL

代码：

- `slam_node.cpp:783-788`

作用：

- 删掉上一帧 RViz 里的旧标记

### 11.2 只发布稳定锥桶

代码：

- `slam_node.cpp:793-796`

```cpp
if (!cone.is_stable) continue;
```

### 11.3 用 mesh 显示锥桶

代码：

- `slam_node.cpp:797-833`

模型文件：

- `slam/slam/meshes/cone.dae`
- `slam/slam/meshes/cone_big.dae`

类型是大黄桶时，用：

- `cone_big.dae`

其他情况用：

- `cone.dae`

### 11.4 位置直接用 `GlobalCone.pos`

代码：

- `slam_node.cpp:809-811`

```cpp
m.pose.position.x = cone.pos.x();
m.pose.position.y = cone.pos.y();
```

这就是全局锥桶地图的最终空间表达。

## 12. 三种赛道模式到底有什么差别

## 12.1 `acceleration`

文件：

- `slam/slam/config/acceleration.yaml`

特点：

- 没有回环逻辑
- 持续建图
- 持续做 miss 惩罚和删点

适合直线加速赛。

## 12.2 `autocross`

文件：

- `slam/slam/config/autocross.yaml`

特点：

- 有回环逻辑
- 第一圈后锁图
- 锁图后不再新增锥桶

这是全局锥桶地图最完整的模式。

## 12.3 `skidpad`

文件：

- `slam/slam/config/skidpad.yaml`

现实情况很直接：

`processSkidpadTrack` 还是 `TODO`。

代码：

- `slam_node.cpp:509-517`

所以按当前代码，八字赛完整建图还没落地。

如果你做汇报，这一点要明说。

## 13. 全局锥桶地图是怎么一步步长出来的

现在把整个过程压成一条链。

### 第 1 步

GNSS/INS 给出当前时刻车辆全局位姿。

### 第 2 步

融合节点给出当前时刻局部锥桶列表。

每个锥桶有：

- 局部 `x y`
- 颜色 `color`

### 第 3 步

SLAM 把局部锥桶通过：

```cpp
T_vehicle_to_map * T_lidar_to_vehicle * p_lidar
```

转成全局平面点。

### 第 4 步

对每个全局点，在 `cone_map_` 里找同色旧锥桶。

### 第 5 步

先用欧氏距离粗筛，再用马氏距离精筛。

### 第 6 步

匹配成功：

- 更新位置
- 更新协方差
- 增加存在分数

### 第 7 步

匹配失败：

- 看是否允许加新锥桶
- 看是否落在阿克曼 ROI
- 满足就新建

### 第 8 步

这一帧结束时，再对视场内没命中的旧锥桶减分。

### 第 9 步

分数超过稳定阈值，就标记稳定。

### 第 10 步

只把稳定锥桶发到 `/global_map`。

这 10 步叠起来，就是“全局锥桶地图”。

## 14. 用伪代码重写一遍

下面这段伪代码已经很接近原工程核心。

```cpp
on_sync_callback(gnss_msg, fused_map_msg):
    T = build_vehicle_pose_from_gnss(gnss_msg)
    speed = calc_speed(gnss_msg)
    yaw_rate = calc_yaw_rate(gnss_msg)

    for cone in cone_map_:
        cone.matched_this_frame = false

    for obs in fused_map_msg.track:
        if obs.color == UNKNOWN:
            continue

        p_lidar = [obs.x, obs.y, 0, 1]
        p_global = T * T_l2v * p_lidar
        R_obs = build_observation_covariance(dist(obs), speed, yaw_rate, T.rotation)

        best = find_best_match_by_color_and_mahalanobis(p_global.xy, obs.color, R_obs)

        if best exists:
            kalman_update(best, p_global.xy, R_obs)
            best.existence_score += dynamic_hit(obs.distance)
            best.matched_this_frame = true
        else:
            if map_locked:
                continue
            if !in_ackermann_tube(obs.x, obs.y):
                continue
            add_new_cone(p_global.xy, obs.color)

    for cone in cone_map_:
        if in_fov(cone, T) and !cone.matched_this_frame:
            cone.existence_score += l_miss
        if cone.existence_score < l_min:
            erase cone
        cone.is_stable = cone.existence_score > l_stable

    if track_type == autocross and detect_loop():
        purge_unstable_cones()
        map_locked = true

    publish_only_stable_cones()
```

## 15. 你该带走的 5 个核心点

### 15.1 这套 SLAM 的本体是“锥桶地图滤波”

它的主对象是 `GlobalCone`。

### 15.2 GNSS/INS 给了全局约束

所以它不需要再做一套里程计前端。

### 15.3 数据关联靠颜色 + 欧氏距离 + 马氏距离

这三层配合起来就够用了。

### 15.4 地图质量靠存在分数稳定下来

这一步对工程效果帮助很大。

### 15.5 回环锁图是赛道模式的关键

第一圈探索，后续圈冲刺。

这就是它能在真实赛道里工作的原因。
