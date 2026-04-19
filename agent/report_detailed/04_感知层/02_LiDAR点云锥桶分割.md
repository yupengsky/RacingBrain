# LiDAR 点云锥桶分割

## 1. 模块位置

LiDAR 分割节点位于：

```text
perception/src/cone_segmentation_test_3d/src/test_cone_segmentation
```

核心文件：

```text
src/test_cone_segmentation.cpp
include/test_cone_segmentation/test_cone_segmentation.hpp
config/cone_detection_params.yaml
msg/ThreeDCone.msg
msg/ThreeDConeArray.msg
```

节点名：

```text
cone_segmentation_node
```

## 2. 输入输出

输入：

```text
/lidar_points    sensor_msgs/msg/PointCloud2
```

输出：

```text
/cone_detection_custom    test_cone_segmentation/msg/ThreeDConeArray
/cone_bboxes              visualization_msgs/msg/MarkerArray
/cone_markers             visualization_msgs/msg/Marker
/debug/ground             sensor_msgs/msg/PointCloud2
/debug/obstacles          sensor_msgs/msg/PointCloud2
```

其中 `/cone_detection_custom` 是融合层真正使用的接口。

## 3. 参数结构

配置结构体：

```cpp
struct ConeSegmentationCFG
```

关键字段：

```cpp
bool use_csf = true;
double horizontal_distance_threshold = 20.0;
double voxel_size = 0.05;
int min_cluster_size = 3;

double ground_ransac_dist_threshold = 0.05;
int ground_ransac_max_iterations = 100;

bool csf_bSloopSmooth = true;
double csf_cloth_resolution = 0.5;
int csf_rigidness = 3;
double csf_time_step = 0.65;
double csf_class_threshold = 0.05;
int csf_iterations = 20;

double MIN_CONE_HEIGHT = 0.12;
double MAX_CONE_HEIGHT = 0.60;
double MAX_CONE_WIDTH = 0.50;

double cone_ransac_distance_threshold = 0.05;
int cone_ransac_max_iterations = 500;
double cone_min_radius = 0.02;
double cone_max_radius = 0.3;
```

`run_perception` 中设置：

```python
'use_csf': False
```

也就是说集成运行默认用 RANSAC 地面分割，而不是 CSF。

## 4. 节点初始化

构造函数：

```cpp
ConeSegmentationNode::ConeSegmentationNode() : Node("cone_segmentation_node")
```

初始化流程：

1. 调用 `setup_parameters()`。
2. 订阅 `/lidar_points`。
3. 创建调试点云 publisher。
4. 创建 bbox 和中心点 marker publisher。
5. 创建 `/cone_detection_custom` publisher。
6. 打印当前地面分割模式。

## 5. 点云转换

回调函数：

```cpp
void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
```

内部调用：

```cpp
process_pointcloud(pc_msg)
```

第一步是 ROS 点云转 PCL 点云：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg(*pc_msg, *cloud);
```

如果点云为空，直接返回。

## 6. ROI 距离截取

代码使用 KDTree 从原点半径搜索：

```cpp
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
tree->setInputCloud(cloud);
pcl::PointXYZ search_point(0.0, 0.0, 0.0);
tree->radiusSearch(search_point, cfg_.horizontal_distance_threshold, point_indices, point_distances)
```

默认半径：

```text
20 m
```

目的：

- 去掉远处点。
- 降低后续地面分割和聚类计算量。
- 减少远处噪声干扰。

## 7. 体素降采样

使用 PCL VoxelGrid：

```cpp
pcl::VoxelGrid<pcl::PointXYZ> vg;
vg.setInputCloud(cloud_roi);
vg.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
vg.filter(*cloud_filtered);
```

作用：

- 减少点数。
- 提高聚类速度。
- 平滑局部噪声。

集成运行默认参数来自结构体和 YAML，注意代码中 `setup_parameters` 会读取 `voxel_size` 参数。

## 8. 地面分割模式 A：CSF

如果：

```cpp
cfg_.use_csf == true
```

执行 CSF 布料滤波。

流程：

1. 将 PCL 点转换为 `csf::Point`。
2. 创建 `CSF csf`。
3. 设置参数：

```cpp
csf.params.bSloopSmooth = cfg_.csf_bSloopSmooth;
csf.params.cloth_resolution = cfg_.csf_cloth_resolution;
csf.params.rigidness = cfg_.csf_rigidness;
csf.params.time_step = cfg_.csf_time_step;
csf.params.class_threshold = cfg_.csf_class_threshold;
csf.params.interations = cfg_.csf_iterations;
```

4. 执行：

```cpp
csf.setPointCloud(csf_cloud);
csf.do_filtering(groundIndexes, offGroundIndexes);
```

5. 用 `pcl::ExtractIndices` 提取非地面点作为障碍物。

CSF 适合复杂地面，但需要额外依赖 `CSF.h` 和 `libCSF`。

## 9. 地面分割模式 B：RANSAC 平面拟合

如果：

```cpp
cfg_.use_csf == false
```

执行 RANSAC 平面拟合。

核心代码：

```cpp
pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(cfg_.ground_ransac_dist_threshold);
seg.setMaxIterations(cfg_.ground_ransac_max_iterations);
seg.setInputCloud(cloud_filtered);
seg.segment(*inliers, *coefficients);
```

如果没有找到地面：

```cpp
*cloud_obstacles = *cloud_filtered;
```

否则：

- `setNegative(true)` 提取非地面障碍物。
- `setNegative(false)` 可提取地面用于调试。

## 10. 调试点云发布

障碍物点云：

```cpp
debug_csf_obstacle_pub_->publish(msg_debug);
```

代码中条件是：

```cpp
if (debug_csf_obstacle_pub_->get_subscription_count() > 0 || true)
```

因为 `|| true`，障碍物调试点云总会发布。

地面点云只有有订阅者且非空才发布。

## 11. 欧几里得聚类

对障碍物点云进行聚类：

```cpp
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
ec.setClusterTolerance(0.3);
ec.setMinClusterSize(cfg_.min_cluster_size);
ec.setMaxClusterSize(600);
ec.setSearchMethod(tree_obs);
ec.setInputCloud(cloud_obstacles);
ec.extract(cluster_indices);
```

关键参数：

- cluster tolerance：`0.3 m`
- min cluster size：代码中如果大于 3 会强制设成 3
- max cluster size：`600`

聚类结果是多个点索引集合，每个集合是一个候选障碍物。

## 12. 候选锥桶尺寸初筛

对每个 cluster：

```cpp
pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
dx = max_pt.x - min_pt.x;
dy = max_pt.y - min_pt.y;
dz = max_pt.z - min_pt.z;
```

筛选条件：

```cpp
if (dz < cfg_.MIN_CONE_HEIGHT || dz > cfg_.MAX_CONE_HEIGHT) continue;
if (std::max(dx, dy) > cfg_.MAX_CONE_WIDTH) continue;
```

默认：

```text
MIN_CONE_HEIGHT = 0.12
MAX_CONE_HEIGHT = 0.60
MAX_CONE_WIDTH = 0.50
```

这一步去掉太矮、太高、太宽的障碍物。

## 13. RANSAC 圆锥拟合

如果 cluster 点数大于 30：

```cpp
if (cloud_cluster->size() > 30)
```

则尝试用 PCL 的 `SACSegmentationFromNormals` 拟合圆锥：

```cpp
seg_cone.setModelType(pcl::SACMODEL_CONE);
seg_cone.setMethodType(pcl::SAC_RANSAC);
seg_cone.setNormalDistanceWeight(0.1);
seg_cone.setMaxIterations(cfg_.cone_ransac_max_iterations);
seg_cone.setDistanceThreshold(cfg_.cone_ransac_distance_threshold);
seg_cone.setRadiusLimits(cfg_.cone_min_radius, cfg_.cone_max_radius);
seg_cone.setMinMaxOpeningAngle(cfg_.cone_min_opening_angle, cfg_.cone_max_opening_angle);
seg_cone.setAxis(Eigen::Vector3f(0, 0, 1));
seg_cone.setEpsAngle(M_PI / 6);
```

需要先估计法线：

```cpp
ne.setKSearch(5);
ne.compute(*cloud_normals);
```

拟合成功后，取 coefficients 前三个值作为顶点：

```cpp
final_apex = Eigen::Vector3d(coefficients_cone->values[0],
                             coefficients_cone->values[1],
                             coefficients_cone->values[2]);
```

再检查顶点不能偏离几何中心太远：

```cpp
if(dist > 0.5) fit_success = false;
```

## 14. 几何回退

如果拟合失败，调用：

```cpp
detectConeByGeometry(min_pt, max_pt)
```

几何判断：

```cpp
if (dz < MIN_CONE_HEIGHT || dz > MAX_CONE_HEIGHT) return false;
if (dx > MAX_CONE_WIDTH || dy > MAX_CONE_WIDTH) return false;
aspect = max(dx, dy) / min(dx, dy);
if (aspect > 3.0) return false;
return true;
```

如果通过，用 AABB 几何中心和最高点估计锥桶：

```cpp
final_apex = Eigen::Vector3d(
    (min_pt.x + max_pt.x)/2.0,
    (min_pt.y + max_pt.y)/2.0,
    max_pt.z
);
```

## 15. 内部 ConeObject

检测结果存在：

```cpp
struct ConeObject {
    Eigen::Vector3d apex;
    Eigen::Vector3d center;
    Eigen::Vector3d dimensions;
};
```

其中：

- `apex`：锥桶顶点估计。
- `center`：包围盒中心。
- `dimensions`：包围盒长宽高。

实际发布给融合节点的是 `center` 和 `dimensions`，不是 apex。

## 16. 发布 ThreeDConeArray

函数：

```cpp
publish_results(detected_cones, pc_msg->header)
```

构造：

```cpp
test_cone_segmentation::msg::ThreeDConeArray cone_array_msg;
cone_array_msg.header = header;
```

对每个锥桶：

```cpp
cone_msg.center.x = cone.center.x();
cone_msg.center.y = cone.center.y();
cone_msg.center.z = cone.center.z();

cone_msg.size.x = cone.dimensions.x();
cone_msg.size.y = cone.dimensions.y();
cone_msg.size.z = cone.dimensions.z();

cone_array_msg.cones.push_back(cone_msg);
```

最后：

```cpp
custom_cones_pub_->publish(cone_array_msg);
```

## 17. Marker 可视化

节点发布两个可视化：

1. bbox cube：

```text
/cone_bboxes
```

2. apex sphere list：

```text
/cone_markers
```

每帧先 `DELETEALL` 清除上一帧残留。

## 18. 最小复现伪代码

```cpp
void process_pointcloud(PointCloud2 msg) {
    cloud = fromROSMsg(msg);
    cloud_roi = radius_filter(cloud, center=(0,0,0), radius=20.0);
    cloud_filtered = voxel_grid(cloud_roi, leaf=0.05);

    if (use_csf) {
        cloud_obstacles = csf_remove_ground(cloud_filtered);
    } else {
        cloud_obstacles = ransac_plane_remove_ground(cloud_filtered);
    }

    clusters = euclidean_cluster(
        cloud_obstacles,
        tolerance=0.3,
        min_size=3,
        max_size=600
    );

    result = ThreeDConeArray();
    result.header = msg.header;

    for cluster in clusters:
        min_pt, max_pt = get_aabb(cluster);
        dx, dy, dz = size(max_pt - min_pt);

        if dz not in [0.12, 0.60]: continue;
        if max(dx, dy) > 0.50: continue;

        if cluster.size > 30:
            fit_ok, apex = ransac_cone_fit(cluster);
        else:
            fit_ok = false;

        if !fit_ok:
            if !detect_by_geometry(min_pt, max_pt): continue;

        center = (min_pt + max_pt) / 2;
        cone.center = center;
        cone.size = [dx, dy, dz];
        result.cones.push_back(cone);

    pub.publish(result);
}
```

## 19. 实现注意点

1. `ThreeDCone.center` 当前用 bbox 中心，不是 apex。
2. `min_cluster_size` 在代码中被限制最大为 3，和 YAML 注释中的大点数思路不完全一致。
3. 集成运行默认 `use_csf=False`，即使用 RANSAC。
4. `debug/obstacles` 总会发布，因为代码里有 `|| true`。
5. 如果 LiDAR 漏检多，先检查地面分割和聚类容差。
6. 如果误检多，先检查尺寸筛选、长宽比和最大 cluster 大小。

