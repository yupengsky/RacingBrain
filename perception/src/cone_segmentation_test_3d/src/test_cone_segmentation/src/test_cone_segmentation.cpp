#include "test_cone_segmentation/test_cone_segmentation.hpp"

ConeSegmentationNode::ConeSegmentationNode() : Node("cone_segmentation_node")
{
    setup_parameters();
    
    // 订阅雷达话题 (QoS=10 兼容性最好)
    pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar_points", 
        10, 
        std::bind(&ConeSegmentationNode::topic_callback, this, std::placeholders::_1));

    // 初始化各类发布者
    debug_csf_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/ground", 1);
    debug_csf_obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/obstacles", 1);
    bbox_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cone_bboxes", 10);
    cone_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cone_markers", 10);
    
    // [关键] 初始化自定义消息发布者
    // 话题名: /cone_detection_custom
    custom_cones_pub_ = this->create_publisher<test_cone_segmentation::msg::ThreeDConeArray>(
        "/cone_detection_custom", 10);

    // 打印当前模式
    if (cfg_.use_csf) {
        RCLCPP_INFO(this->get_logger(), "当前模式: [CSF 布料滤波]");
    } else {
        RCLCPP_INFO(this->get_logger(), "当前模式: [RANSAC 平面拟合]");
    }
}

void ConeSegmentationNode::setup_parameters()
{
    // 模式切换
    this->declare_parameter<bool>("use_csf", true);
    
    // RANSAC 地面参数
    this->declare_parameter<double>("ground_ransac_dist_threshold", 0.05);
    this->declare_parameter<int>("ground_ransac_max_iterations", 100);

    // CSF 参数
    this->declare_parameter<bool>("csf_bSloopSmooth", cfg_.csf_bSloopSmooth);
    this->declare_parameter<double>("csf_cloth_resolution", 0.5); 
    this->declare_parameter<int>("csf_rigidness", cfg_.csf_rigidness);
    this->declare_parameter<double>("csf_time_step", cfg_.csf_time_step);
    this->declare_parameter<double>("csf_class_threshold", 0.05);  
    this->declare_parameter<int>("csf_iterations", 20); 

    // 通用参数
    this->declare_parameter<double>("voxel_size", cfg_.voxel_size);
    this->declare_parameter<double>("horizontal_distance_threshold", cfg_.horizontal_distance_threshold);
    this->declare_parameter<int>("min_cluster_size", cfg_.min_cluster_size);
    
    // 锥桶拟合参数
    this->declare_parameter<double>("cone_ransac_distance_threshold", cfg_.cone_ransac_distance_threshold);
    this->declare_parameter<int>("cone_ransac_max_iterations", cfg_.cone_ransac_max_iterations);
    
    // 读取参数
    cfg_.use_csf = this->get_parameter("use_csf").as_bool();
    
    cfg_.ground_ransac_dist_threshold = this->get_parameter("ground_ransac_dist_threshold").as_double();
    cfg_.ground_ransac_max_iterations = this->get_parameter("ground_ransac_max_iterations").as_int();

    cfg_.csf_bSloopSmooth = this->get_parameter("csf_bSloopSmooth").as_bool();
    cfg_.csf_cloth_resolution = this->get_parameter("csf_cloth_resolution").as_double();
    cfg_.csf_rigidness = this->get_parameter("csf_rigidness").as_int();
    cfg_.csf_time_step = this->get_parameter("csf_time_step").as_double();
    cfg_.csf_class_threshold = this->get_parameter("csf_class_threshold").as_double();
    cfg_.csf_iterations = this->get_parameter("csf_iterations").as_int();
    
    cfg_.voxel_size = this->get_parameter("voxel_size").as_double();
    cfg_.horizontal_distance_threshold = this->get_parameter("horizontal_distance_threshold").as_double();
    cfg_.min_cluster_size = this->get_parameter("min_cluster_size").as_int(); 
    if(cfg_.min_cluster_size > 3) cfg_.min_cluster_size = 3; 
    
    cfg_.cone_ransac_distance_threshold = this->get_parameter("cone_ransac_distance_threshold").as_double();
    cfg_.cone_ransac_max_iterations = this->get_parameter("cone_ransac_max_iterations").as_int();

    cfg_.MIN_CONE_HEIGHT = 0.12;
    cfg_.MAX_CONE_HEIGHT = 0.60;
}

void ConeSegmentationNode::topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
{
    // 使用 THROTTLE 防止日志刷屏
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Processing... Frame: %s, Mode: %s", 
        pc_msg->header.frame_id.c_str(), 
        cfg_.use_csf ? "CSF" : "RANSAC");
        
    process_pointcloud(pc_msg);
}

void ConeSegmentationNode::process_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
{
    // 0. 转换消息类型 ROS -> PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_msg, *cloud);

    if (cloud->empty()) return;

    // 1. ROI 截取 (简单距离过滤)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::PointXYZ search_point(0.0, 0.0, 0.0); 
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    if (tree->radiusSearch(search_point, cfg_.horizontal_distance_threshold, point_indices, point_distances) > 0)
    {
        cloud_roi->reserve(point_indices.size());
        for (const auto &idx : point_indices)
            cloud_roi->points.push_back(cloud->points[idx]);
        cloud_roi->width = cloud_roi->points.size();
        cloud_roi->height = 1;
        cloud_roi->is_dense = true;
    }
    else { return; }

    // 2. 体素滤波 (降采样)
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_roi);
    vg.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
    vg.filter(*cloud_filtered);

    // 准备容器
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>); 

    // 3. 地面分割 (模式切换逻辑)
    if (cfg_.use_csf) 
    {
        // === 模式 A: CSF ===
        std::vector<csf::Point> csf_cloud;
        for (const auto& pt : cloud_filtered->points) {
            csf::Point p; p.x = pt.x; p.y = pt.y; p.z = pt.z;
            csf_cloud.push_back(p);
        }

        CSF csf;
        csf.params.bSloopSmooth = cfg_.csf_bSloopSmooth;
        csf.params.cloth_resolution = cfg_.csf_cloth_resolution;
        csf.params.rigidness = cfg_.csf_rigidness;
        csf.params.time_step = cfg_.csf_time_step;
        csf.params.class_threshold = cfg_.csf_class_threshold;
        csf.params.interations = cfg_.csf_iterations;

        std::vector<int> groundIndexes, offGroundIndexes;
        csf.setPointCloud(csf_cloud);
        csf.do_filtering(groundIndexes, offGroundIndexes);

        // 提取障碍物
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr off_ground_indices_ptr(new pcl::PointIndices);
        off_ground_indices_ptr->indices = offGroundIndexes;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(off_ground_indices_ptr);
        extract.setNegative(false); 
        extract.filter(*cloud_obstacles);
        
        // 提取地面(调试用)
        if (debug_csf_ground_pub_->get_subscription_count() > 0) {
            pcl::PointIndices::Ptr ground_indices_ptr(new pcl::PointIndices);
            ground_indices_ptr->indices = groundIndexes;
            extract.setIndices(ground_indices_ptr);
            extract.filter(*cloud_ground);
        }
    } 
    else 
    {
        // === 模式 B: RANSAC ===
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE); 
        seg.setMethodType(pcl::SAC_RANSAC);    
        seg.setDistanceThreshold(cfg_.ground_ransac_dist_threshold);
        seg.setMaxIterations(cfg_.ground_ransac_max_iterations);

        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            *cloud_obstacles = *cloud_filtered; // 没找到地面，全算障碍物
        } else {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            
            // 提取障碍物 (Negative=true)
            extract.setNegative(true);
            extract.filter(*cloud_obstacles);
            
            // 提取地面 (Negative=false)
            if (debug_csf_ground_pub_->get_subscription_count() > 0) {
                extract.setNegative(false);
                extract.filter(*cloud_ground);
            }
        }
    }

    // 4. 发布调试点云
    if (debug_csf_obstacle_pub_->get_subscription_count() > 0 || true) {
        sensor_msgs::msg::PointCloud2 msg_debug;
        pcl::toROSMsg(*cloud_obstacles, msg_debug);
        msg_debug.header = pc_msg->header; 
        debug_csf_obstacle_pub_->publish(msg_debug);
    }
    if (debug_csf_ground_pub_->get_subscription_count() > 0 && !cloud_ground->empty()) {
        sensor_msgs::msg::PointCloud2 msg_ground;
        pcl::toROSMsg(*cloud_ground, msg_ground);
        msg_ground.header = pc_msg->header;
        debug_csf_ground_pub_->publish(msg_ground);
    }

    if (cloud_obstacles->empty()) return;

    // 5. 欧几里得聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_obs(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_obs->setInputCloud(cloud_obstacles);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3); // 30cm 距离容差
    ec.setMinClusterSize(cfg_.min_cluster_size);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree_obs);
    ec.setInputCloud(cloud_obstacles);
    ec.extract(cluster_indices);

    // 6. 锥桶筛选与拟合
    // 准备拟合器
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_norm(new pcl::search::KdTree<pcl::PointXYZ>());
    
    seg_cone.setOptimizeCoefficients(true);
    seg_cone.setModelType(pcl::SACMODEL_CONE);
    seg_cone.setMethodType(pcl::SAC_RANSAC);
    seg_cone.setNormalDistanceWeight(0.1);
    seg_cone.setMaxIterations(cfg_.cone_ransac_max_iterations);
    seg_cone.setDistanceThreshold(cfg_.cone_ransac_distance_threshold);
    seg_cone.setRadiusLimits(cfg_.cone_min_radius, cfg_.cone_max_radius);
    seg_cone.setMinMaxOpeningAngle(cfg_.cone_min_opening_angle, cfg_.cone_max_opening_angle);
    seg_cone.setAxis(Eigen::Vector3f(0, 0, 1));
    seg_cone.setEpsAngle(M_PI / 6);

    std::vector<ConeObject> detected_cones;

    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
            cloud_cluster->points.push_back(cloud_obstacles->points[idx]);

        // 计算 AABB 包围盒
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        
        float dx = max_pt.x - min_pt.x;
        float dy = max_pt.y - min_pt.y;
        float dz = max_pt.z - min_pt.z;

        // [规则 A] 尺寸初筛
        if (dz < cfg_.MIN_CONE_HEIGHT || dz > cfg_.MAX_CONE_HEIGHT) continue;
        if (std::max(dx, dy) > cfg_.MAX_CONE_WIDTH) continue;

        bool fit_success = false;
        Eigen::Vector3d final_apex(0,0,0);

        // [规则 B] 点数足够时尝试 RANSAC 锥桶拟合
        if (cloud_cluster->size() > 30) 
        {
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
            ne.setSearchMethod(tree_norm);
            ne.setInputCloud(cloud_cluster);
            ne.setKSearch(5);
            ne.compute(*cloud_normals);

            pcl::ModelCoefficients::Ptr coefficients_cone(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_cone(new pcl::PointIndices);
            
            seg_cone.setInputCloud(cloud_cluster);
            seg_cone.setInputNormals(cloud_normals);
            
            try {
                seg_cone.segment(*inliers_cone, *coefficients_cone);
                if (!inliers_cone->indices.empty()) {
                    fit_success = true;
                    final_apex = Eigen::Vector3d(coefficients_cone->values[0], coefficients_cone->values[1], coefficients_cone->values[2]);
                    
                    // 校验：拟合顶点不能偏离几何中心太远
                    double dist = std::sqrt(std::pow(final_apex.x() - (min_pt.x+max_pt.x)/2, 2) + std::pow(final_apex.y() - (min_pt.y+max_pt.y)/2, 2));
                    if(dist > 0.5) fit_success = false; 
                }
            } catch (...) { fit_success = false; }
        }

        // [回退] 拟合失败，使用几何中心
        if (!fit_success) {
            if (detectConeByGeometry(min_pt, max_pt)) {
                final_apex = Eigen::Vector3d((min_pt.x + max_pt.x)/2.0, (min_pt.y + max_pt.y)/2.0, max_pt.z);
            } else {
                continue; // 既不拟合也不像锥桶，跳过
            }
        }

        // 保存检测结果
        ConeObject cone;
        cone.apex = final_apex;
        cone.center = Eigen::Vector3d((min_pt.x + max_pt.x)/2.0, (min_pt.y + max_pt.y)/2.0, (min_pt.z + max_pt.z)/2.0);
        cone.dimensions = Eigen::Vector3d(dx, dy, dz);
        detected_cones.push_back(cone);
    }

    // 7. 发布最终结果
    if (!detected_cones.empty()) {
        publish_results(detected_cones, pc_msg->header);
    }
}

// 几何辅助判断
bool ConeSegmentationNode::detectConeByGeometry(const pcl::PointXYZ &min_pt, const pcl::PointXYZ &max_pt)
{
    float dx = max_pt.x - min_pt.x;
    float dy = max_pt.y - min_pt.y;
    float dz = max_pt.z - min_pt.z;

    if (dz < cfg_.MIN_CONE_HEIGHT || dz > cfg_.MAX_CONE_HEIGHT) return false;
    if (dx > cfg_.MAX_CONE_WIDTH || dy > cfg_.MAX_CONE_WIDTH) return false;
    
    // 长宽比不能太夸张（比如像一根杆子或一堵墙）
    float aspect = std::max(dx, dy) / std::min(dx, dy);
    if (aspect > 3.0) return false;

    return true;
}

// 结果发布函数
void ConeSegmentationNode::publish_results(const std::vector<ConeObject>& cones, const std_msgs::msg::Header& header)
{
    // ===============================================
    // 1. Rviz 可视化部分
    // ===============================================
    visualization_msgs::msg::MarkerArray bbox_array;
    visualization_msgs::msg::Marker sphere_marker;
    
    // [去延迟技巧] 使用 DELETEALL 清空上一帧残留
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    bbox_array.markers.push_back(delete_marker);

    sphere_marker.header = header; 
    sphere_marker.ns = "cone_centers";
    sphere_marker.id = 0;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.scale.x = 0.2; sphere_marker.scale.y = 0.2; sphere_marker.scale.z = 0.2;
    sphere_marker.color.r = 1.0f; sphere_marker.color.g = 0.5f; sphere_marker.color.b = 0.0f; sphere_marker.color.a = 1.0f;
    sphere_marker.lifetime = rclcpp::Duration::from_seconds(0); 

    // ===============================================
    // 2. 自定义消息发布 (ThreeDConeArray)
    // ===============================================
    test_cone_segmentation::msg::ThreeDConeArray cone_array_msg;
    cone_array_msg.header = header; // 务必继承时间戳

    int id = 0;
    for (const auto& cone : cones) {
        // --- A. 填充 Visualization Marker ---
        visualization_msgs::msg::Marker marker;
        marker.header = header; // 时间戳对齐
        marker.ns = "cone_bboxes";
        marker.id = ++id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = cone.center.x();
        marker.pose.position.y = cone.center.y();
        marker.pose.position.z = cone.center.z();
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = std::max(cone.dimensions.x(), 0.1);
        marker.scale.y = std::max(cone.dimensions.y(), 0.1);
        marker.scale.z = std::max(cone.dimensions.z(), 0.1);
        
        marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.4f;
        marker.lifetime = rclcpp::Duration::from_seconds(0); 
        bbox_array.markers.push_back(marker);

        geometry_msgs::msg::Point p;
        p.x = cone.apex.x(); p.y = cone.apex.y(); p.z = cone.apex.z();
        sphere_marker.points.push_back(p);

        // --- B. 填充自定义消息 ThreeDCone ---
        test_cone_segmentation::msg::ThreeDCone cone_msg;
        
        // 中心点
        cone_msg.center.x = cone.center.x();
        cone_msg.center.y = cone.center.y();
        cone_msg.center.z = cone.center.z();

        // 尺寸 (长宽高)
        cone_msg.size.x = cone.dimensions.x();
        cone_msg.size.y = cone.dimensions.y();
        cone_msg.size.z = cone.dimensions.z();

        cone_array_msg.cones.push_back(cone_msg);
    }

    // 发布所有内容
    cone_marker_pub_->publish(sphere_marker);
    bbox_publisher_->publish(bbox_array);
    custom_cones_pub_->publish(cone_array_msg); // 发布自定义数据
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConeSegmentationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
