#ifndef TEST_CONE_SEGMENTATION_HPP
#define TEST_CONE_SEGMENTATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

// [关键] 引入刚才编译成功的自定义消息头文件
// ROS 2 会自动把 .msg 文件名转成下划线风格的 .hpp
#include "test_cone_segmentation/msg/three_d_cone.hpp"
#include "test_cone_segmentation/msg/three_d_cone_array.hpp"

// PCL 相关
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>

// CSF
#include <CSF.h> 
#include <vector>
#include <Eigen/Dense>

// 内部使用的简单结构体
struct ConeObject {
    Eigen::Vector3d apex;       
    Eigen::Vector3d center;     
    Eigen::Vector3d dimensions; 
};

// 参数配置
struct ConeSegmentationCFG
{
    // [模式开关]
    bool use_csf = true; // true: CSF, false: RANSAC

    // ROI & 滤波
    double horizontal_distance_threshold = 20.0;
    double voxel_size = 0.05;                    
    
    // 聚类
    int min_cluster_size = 3; 
    
    // CSF 参数
    bool csf_bSloopSmooth = true;
    double csf_cloth_resolution = 0.5; 
    int csf_rigidness = 3;
    double csf_time_step = 0.65;
    double csf_class_threshold = 0.05;  
    int csf_iterations = 20;

    // RANSAC 地面分割参数
    double ground_ransac_dist_threshold = 0.05; 
    int ground_ransac_max_iterations = 100;     

    // 几何筛选
    double MIN_CONE_HEIGHT = 0.12; 
    double MAX_CONE_HEIGHT = 0.60;
    double MAX_CONE_WIDTH = 0.50; 
    double MAX_AspectRatio = 3.5;

    // 圆锥拟合参数
    double cone_ransac_distance_threshold = 0.05;
    int cone_ransac_max_iterations = 500;
    double cone_min_inlier_ratio = 0.1;
    double cone_axis_max_angle = 28.0;
    double cone_min_radius = 0.02;
    double cone_max_radius = 0.3;
    double cone_min_opening_angle = 0.0;       
    double cone_max_opening_angle = M_PI / 3.0;
    double MAX_DIST_TO_GROUND = 0.5;
};

class ConeSegmentationNode : public rclcpp::Node
{
public:
    ConeSegmentationNode();

private:
    void setup_parameters();
    void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);
    void process_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);
    void publish_results(const std::vector<ConeObject>& cones, const std_msgs::msg::Header& header);
    bool detectConeByGeometry(const pcl::PointXYZ &min_pt, const pcl::PointXYZ &max_pt);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    
    // 调试与可视化发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_csf_ground_pub_;   
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_csf_obstacle_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_publisher_;  
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cone_marker_pub_;      
    
    // [关键] 自定义消息发布者
    rclcpp::Publisher<test_cone_segmentation::msg::ThreeDConeArray>::SharedPtr custom_cones_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;

    ConeSegmentationCFG cfg_;
    std::string output_topic_ = "/cone_detection_custom";
    std::string metrics_topic_ = "/perception/lidar/evaluation/metrics";
    bool eval_metrics_enabled_ = false;
    bool health_metrics_enabled_ = false;
    bool metrics_enabled_ = false;
};

#endif // TEST_CONE_SEGMENTATION_HPP
