# LiDAR-Camera 融合节点

## 1. 模块位置

融合包：

```text
perception/src/fs_fusion_box
```

核心文件：

```text
src/fs_fusion_box_node.cpp
src/fs_fusion_box_math.cpp
include/fs_fusion_box/fs_fusion_box_math.hpp
config/calibration.yaml
launch/fusion_box.launch.py
```

节点类：

```cpp
fs_fusion_box::FusionNode
```

节点名：

```text
fusion_box_node
```

## 2. 输入输出

融合节点内部订阅名：

```text
perception/lidar/cones_custom
perception/camera/cones_custom
```

launch 重映射后实际订阅：

```text
/cone_detection_custom
/yolo/cones
```

输出内部名：

```text
fusion/cones
fusion/markers
```

launch 重映射后实际输出：

```text
/perception/fusion/map
/fusion/markers
```

其中 `/perception/fusion/map` 是 SLAM 的输入。

## 3. 为什么需要融合

两个传感器各有优势：

```text
LiDAR：
  优点：空间位置较可靠。
  缺点：没有颜色语义。

Camera + YOLO：
  优点：颜色语义明显。
  缺点：单目深度不可靠。
```

融合目标：

```text
用 LiDAR 的 x,y 作为位置，用 YOLO 的类别作为颜色。
```

最终输出：

```text
drd25_msgs/msg/Map
```

也就是：

```text
header
track: [x, y, color]
```

## 4. 参数

节点声明参数：

```cpp
this->declare_parameter("sync_window", 0.1);
this->declare_parameter("lidar_frame", "hesai_lidar");
this->declare_parameter("force_match_radius", 60.0);
this->declare_parameter("image_width", 640);
this->declare_parameter("image_height", 480);
this->declare_parameter("camera_matrix.fx", 500.0);
this->declare_parameter("camera_matrix.fy", 500.0);
this->declare_parameter("camera_matrix.cx", 320.0);
this->declare_parameter("camera_matrix.cy", 240.0);
this->declare_parameter("dist_coeffs", std::vector<double>({0,0,0,0,0}));
this->declare_parameter("lidar_to_camera_matrix", std::vector<double>());
```

实际配置文件：

```text
perception/src/fs_fusion_box/config/calibration.yaml
```

关键标定：

```yaml
image_width: 640
image_height: 480
camera_matrix:
  fx: 352.18725375126115
  fy: 351.69582704985083
  cx: 325.53089619872935
  cy: 233.98829318962902
dist_coeffs: [...]
lidar_to_camera_matrix: [...]
```

## 5. 同步订阅

融合节点使用 `message_filters` 近似时间同步：

```cpp
typedef message_filters::sync_policies::ApproximateTime<
    test_cone_segmentation::msg::ThreeDConeArray,
    cone_interfaces::msg::ConeArray
> SyncPolicy;

sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), lidar_sub_, camera_sub_
);
```

回调：

```cpp
sync_->registerCallback(
    std::bind(&FusionNode::callback, this, _1, _2)
);
```

注意：代码声明了 `sync_window` 参数，但当前没有把它设置给 Synchronizer 的 slop。因此实际同步窗口由 ApproximateTime 默认策略和队列行为决定。

## 6. 双适配器设计

融合数学函数使用内部标准结构：

```text
vision_msgs/msg/Detection3DArray
vision_msgs/msg/Detection2DArray
```

但工程真实输入是自定义消息：

```text
test_cone_segmentation/msg/ThreeDConeArray
cone_interfaces/msg/ConeArray
```

因此 `FusionNode::callback` 首先做“适配器转换”。

### 6.1 LiDAR 自定义消息转 Detection3DArray

代码逻辑：

```cpp
auto standard_lidar_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();
standard_lidar_msg->header = custom_lidar_msg->header;

for (const auto& custom_cone : custom_lidar_msg->cones) {
    vision_msgs::msg::Detection3D det;
    det.bbox.center.position = custom_cone.center;
    det.bbox.size = custom_cone.size;
    standard_lidar_msg->detections.push_back(det);
}
```

含义：

- `ThreeDCone.center` -> `Detection3D.bbox.center.position`
- `ThreeDCone.size` -> `Detection3D.bbox.size`

### 6.2 Camera 自定义消息转 Detection2DArray

代码逻辑：

```cpp
auto standard_camera_msg = std::make_shared<vision_msgs::msg::Detection2DArray>();
standard_camera_msg->header = custom_camera_msg->header;

for (const auto& box_2d : custom_camera_msg->cones) {
    vision_msgs::msg::Detection2D det;
    det.bbox.center.position.x = box_2d.center.x;
    det.bbox.center.position.y = box_2d.center.y;
    det.bbox.size_x = box_2d.size.x;
    det.bbox.size_y = box_2d.size.y;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.score = box_2d.confidence;
    hyp.hypothesis.class_id = std::to_string(box_2d.color);
    det.results.push_back(hyp);
    standard_camera_msg->detections.push_back(det);
}
```

含义：

- 2D 中心和尺寸直接复制。
- `color` 被转成 `class_id` 字符串。
- `confidence` 被转成 hypothesis score。

这个适配层的好处是：核心融合数学函数不绑定工程自定义消息。

## 7. 标定参数加载

回调中每帧都读取参数：

```cpp
CalibrationParams params;
std::vector<double> matrix_data =
    this->get_parameter("lidar_to_camera_matrix").as_double_array();
if (matrix_data.size() != 16) return;

params.T_l2c = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
    matrix_data.data()
);
```

内参：

```cpp
params.K = cv::Mat::eye(3, 3, CV_64F);
params.K.at<double>(0,0) = fx;
params.K.at<double>(1,1) = fy;
params.K.at<double>(0,2) = cx;
params.K.at<double>(1,2) = cy;
```

畸变：

```cpp
std::vector<double> dist = this->get_parameter("dist_coeffs").as_double_array();
params.D = cv::Mat(dist).clone();
```

图片尺寸：

```cpp
params.img_w = image_width;
params.img_h = image_height;
```

## 8. 融合主流程

核心调用：

```cpp
auto proj_boxes = project_3d_boxes_to_2d(standard_lidar_msg, params);
auto fusion_result = fuse_measurements(proj_boxes, standard_lidar_msg, standard_camera_msg);
auto recovered_cones = recover_missing_cones(
    fusion_result.unmatched_camera_indices,
    standard_camera_msg,
    params
);
```

含义：

1. LiDAR 3D 框投影为 2D 框。
2. 投影框与 YOLO 框做匹配。
3. 对 YOLO 有但 LiDAR 没匹配上的框做单目补漏。

## 9. Unknown 锥桶强制上色容错

标准融合后，LiDAR 框可能因为 IoU 不够而保持 `UNKNOWN`。代码增加了“吸铁石”策略：

```cpp
double magnet_radius = this->get_parameter("force_match_radius").as_double();
```

默认：

```text
60 pixels
```

对所有颜色为 `UNKNOWN` 的融合锥桶：

1. 将锥桶点重新投影到图像。
2. 找所有 YOLO 框中心中距离最近的一个。
3. 如果像素距离小于 `force_match_radius`，把该 YOLO 框颜色赋给锥桶。

伪代码：

```cpp
for cone in fused_cones:
    if cone.color != UNKNOWN: continue

    uv = project(cone.x, cone.y, 0)
    best_box = nearest_camera_box_center(uv)

    if distance(uv, best_box.center) < force_match_radius:
        cone.color = best_box.class_id
```

这个机制用于抵抗外参轻微误差。严格说它不是理论上最干净的融合，但在调试和实车环境中很实用。

## 10. 合并输出

最终锥桶：

```cpp
std::vector<drd25_msgs::msg::Cone> final_cones = fusion_result.fused_cones;
final_cones.insert(final_cones.end(), recovered_cones.begin(), recovered_cones.end());
```

发布 `drd25_msgs/Map`：

```cpp
drd25_msgs::msg::Map map_msg;
map_msg.header = custom_lidar_msg->header;
map_msg.track = final_cones;
map_pub_->publish(map_msg);
```

注意：header 使用 LiDAR 消息 header。

## 11. Marker 可视化

函数：

```cpp
publish_markers(final_cones, custom_lidar_msg->header)
```

发布话题：

```text
fusion/markers
```

frame：

```cpp
std::string viz_frame = this->get_parameter("lidar_frame").as_string();
```

默认：

```text
hesai_lidar
```

每个锥桶可视化为 CYLINDER：

```cpp
m.type = visualization_msgs::msg::Marker::CYLINDER;
m.pose.position.x = cone.x;
m.pose.position.y = cone.y;
m.pose.position.z = 0.0;
m.scale.x = 0.2;
m.scale.y = 0.2;
m.scale.z = 0.45;
```

颜色：

- `0` 蓝色。
- `1` 红色。
- `2` 或 `3` 黄色。
- 其他灰色。

## 12. 复现最小伪代码

```cpp
void callback(lidar_custom, camera_custom) {
    lidar3d = convert_to_detection3d_array(lidar_custom);
    camera2d = convert_to_detection2d_array(camera_custom);

    params = load_calibration();

    projected = project_3d_boxes_to_2d(lidar3d, params);
    result = fuse_measurements(projected, lidar3d, camera2d);
    recovered = recover_missing_cones(result.unmatched_camera_indices, camera2d, params);

    for cone in result.fused_cones:
        if cone.color == UNKNOWN:
            try_force_match_by_center_distance(cone, camera2d, params);

    final = result.fused_cones + recovered;

    map.header = lidar_custom.header;
    map.track = final;
    pub_map.publish(map);
}
```

## 13. 实现注意点

1. `lidar_to_camera_matrix` 必须有 16 个元素，否则回调直接 return。
2. 融合输出的 `x,y` 是局部传感器坐标，不是全局地图坐标。
3. `dist_coeffs` 被读取但核心投影没有真正畸变校正。
4. `force_match_radius` 是工程容错，调太大可能误上色。
5. `recover_missing_cones` 的单目补漏精度较低，应作为召回补充，而不是强位置约束。
6. header 使用 LiDAR header，SLAM 同步时以此时间戳和 GNSS 对齐。

