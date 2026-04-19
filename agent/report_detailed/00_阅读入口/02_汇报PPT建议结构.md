# 汇报 PPT 建议结构

本节不是源码解释，而是告诉你如何把本报告转化成一份有技术含量、能讲清楚贡献的 PPT。

## 第 1 页：项目目标

标题可以写：

```text
面向无人方程式锥桶赛道的多传感器语义建图系统
```

核心问题：

- 赛车需要知道左右边界锥桶的位置和颜色。
- 单靠相机，距离和三维位置不稳定。
- 单靠 LiDAR，空间位置准但颜色未知。
- 单靠 GNSS/INS，只有车的位置，没有赛道边界。
- 因此需要把 Camera、LiDAR、GNSS/INS 和 SLAM 后端串起来。

一句话贡献：

```text
系统将 LiDAR 几何、Camera 语义、GNSS/INS 位姿融合，构建稳定全局锥桶地图。
```

## 第 2 页：系统架构

建议画成：

```text
Camera Image  -> YOLO Detector       -> /yolo/cones
LiDAR Points  -> Cone Segmentation   -> /cone_detection_custom
                                      -> Fusion -> /perception/fusion/map
GNSS/INS      -> Localization Pose    -> SLAM   -> /global_map
```

强调四个节点：

- `yolo_detector`
- `cone_segmentation_node`
- `fusion_box_node`
- `slam_processor`

## 第 3 页：消息接口

展示 `drd25_msgs/Map`：

```text
std_msgs/Header header
Cone[] track
```

展示 `drd25_msgs/Cone`：

```text
float64 x
float64 y
uint8 color
```

说明颜色枚举：

- `0`: BLUE
- `1`: RED
- `2`: YELLOW_BIG
- `3`: YELLOW_SMALL
- `4`: UNKNOWN

## 第 4 页：LiDAR-Camera 融合

讲三步：

1. 将 LiDAR 3D 框投影到相机图像。
2. 与 YOLO 2D 框计算 IoU。
3. 匹配成功后用 YOLO 颜色给 LiDAR 坐标上色。

公式：

```text
p_camera = T_lidar_to_camera * p_lidar
u = fx * X / Z + cx
v = fy * Y / Z + cy
```

## 第 5 页：SLAM 后端

讲五步：

1. GNSS 经纬度转 UTM。
2. 第一帧 GNSS 设为局部地图原点。
3. 锥桶局部坐标转全局坐标。
4. 用颜色门控 + 欧氏距离 + 马氏距离做数据关联。
5. 用卡尔曼更新和存在评分维护稳定地图。

## 第 6 页：关键算法亮点

推荐讲这三个：

- 各向异性观测噪声：纵向和横向误差不同，更符合车载传感器。
- 马氏距离关联：考虑地图不确定度和观测不确定度。
- 存在评分：hit 加分、miss 扣分，稳定锥桶才发布。

## 第 7 页：赛道模式

目前支持：

- `acceleration`：直线模式，默认已实现。
- `autocross`：循迹模式，已实现回环检测和锁图。
- `skidpad`：八字模式，当前预留配置，核心建图策略 TODO。

循迹模式可以讲成：

```text
探索圈：允许新增锥桶，逐步建立地图
回环后：删除不稳定锥桶，锁定地图
冲刺圈：不新增锥桶，只更新已有稳定地图
```

## 第 8 页：运行验证

展示一键命令：

```bash
RVIZ=true KEEP_RUNNING=true ./scripts/run_dataset_slam_chain.sh
```

展示验证指标：

- `/camera1/image_raw` 有输入。
- `/lidar_points` 有输入。
- `/gongji_gnss_ins_64` 有输入。
- `/yolo/cones` 有输出。
- `/cone_detection_custom` 有输出。
- `/perception/fusion/map` 非空。
- `/global_map` 非空。

## 第 9 页：问题与改进

客观讲短板会显得你更懂工程：

- 八字模式还未完成。
- `/global_map` 当前是 RViz MarkerArray，规划层若要消费需要结构化地图消息。
- 融合中的吸铁石半径是工程容错，需要后续替换为更严格的数据关联策略。
- 外部模型和数据路径依赖本地配置。
- 部分参数需要基于更多数据集自动调优。

