#include <Eigen/Dense> 

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/core/eigen.hpp>

// --- 消息头文件 ---
#include "drd25_msgs/msg/map.hpp" 
#include "drd25_msgs/msg/cone.hpp"
#include "test_cone_segmentation/msg/three_d_cone_array.hpp"
#include "cone_interfaces/msg/cone_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "fs_fusion_box/fs_fusion_box_math.hpp"

using namespace std::chrono_literals;

namespace fs_fusion_box {

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("fusion_box_node") {
        // --- 核心参数 ---
        this->declare_parameter("sync_window", 0.1);
        this->declare_parameter("lidar_frame", "hesai_lidar"); 

        // [强力测试参数] 吸铁石半径 (像素)
        // 只要雷达点投影在相机框中心 60 像素以内，强制上色！
        // 这能无视外参的轻微偏差
        this->declare_parameter("force_match_radius", 60.0); 

        // [内参]
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("camera_matrix.fx", 500.0);
        this->declare_parameter("camera_matrix.fy", 500.0);
        this->declare_parameter("camera_matrix.cx", 320.0);
        this->declare_parameter("camera_matrix.cy", 240.0);
        this->declare_parameter("dist_coeffs", std::vector<double>({0,0,0,0,0}));
        
        // [外参]
        this->declare_parameter("lidar_to_camera_matrix", std::vector<double>());

        // --- 初始化 ---
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fusion/markers", 10);
        map_pub_ = this->create_publisher<drd25_msgs::msg::Map>("fusion/cones", 10);

        lidar_sub_.subscribe(this, "perception/lidar/cones_custom"); 
        camera_sub_.subscribe(this, "perception/camera/cones_custom"); 

        typedef message_filters::sync_policies::ApproximateTime<
            test_cone_segmentation::msg::ThreeDConeArray, 
            cone_interfaces::msg::ConeArray               
        > SyncPolicy;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), lidar_sub_, camera_sub_
        );
        
        sync_->registerCallback(
            std::bind(&FusionNode::callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Fusion Node Ready (Magnet Test Mode).");
    }

private:
    void callback(
        const test_cone_segmentation::msg::ThreeDConeArray::ConstSharedPtr& custom_lidar_msg,
        const cone_interfaces::msg::ConeArray::ConstSharedPtr& custom_camera_msg) 
    {
        // 1. 3D 数据转换
        auto standard_lidar_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();
        standard_lidar_msg->header = custom_lidar_msg->header;
        for (const auto& custom_cone : custom_lidar_msg->cones) {
            vision_msgs::msg::Detection3D det;
            det.bbox.center.position = custom_cone.center;
            det.bbox.size = custom_cone.size;
            standard_lidar_msg->detections.push_back(det);
        }

        // 2. 2D 数据转换
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

        // 3. 参数加载
        CalibrationParams params;
        std::vector<double> matrix_data = this->get_parameter("lidar_to_camera_matrix").as_double_array();
        if (matrix_data.size() != 16) return; 
        params.T_l2c = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(matrix_data.data());

        params.img_w = this->get_parameter("image_width").as_int();
        params.img_h = this->get_parameter("image_height").as_int();
        params.K = cv::Mat::eye(3, 3, CV_64F);
        params.K.at<double>(0,0) = this->get_parameter("camera_matrix.fx").as_double();
        params.K.at<double>(1,1) = this->get_parameter("camera_matrix.fy").as_double();
        params.K.at<double>(0,2) = this->get_parameter("camera_matrix.cx").as_double();
        params.K.at<double>(1,2) = this->get_parameter("camera_matrix.cy").as_double();
        std::vector<double> dist = this->get_parameter("dist_coeffs").as_double_array();
        params.D = cv::Mat(dist).clone();

        // 4. 标准融合 (先试着正常匹配一下)
        auto proj_boxes = project_3d_boxes_to_2d(standard_lidar_msg, params);
        auto fusion_result = fuse_measurements(proj_boxes, standard_lidar_msg, standard_camera_msg);

        // 5. 补全 (检测远距离)
        // 这一步生成纯视觉锥筒，保持远距离检测能力
        auto recovered_cones = recover_missing_cones(fusion_result.unmatched_camera_indices, standard_camera_msg, params);

        // ==========================================
        // [新增] 强力吸铁石：修复近处灰色圆柱
        // ==========================================
        double magnet_radius = this->get_parameter("force_match_radius").as_double();
        std::vector<drd25_msgs::msg::Cone>& current_cones = fusion_result.fused_cones;

        // 遍历所有已经是“灰色(Unknown)”的雷达锥筒
        for (size_t i = 0; i < current_cones.size(); ++i) {
            // 如果颜色未知 (Color == 4 或其他定义为灰色的值)
            // drd25_msgs/Cone 定义: 0=Blue, 1=Red, 2=Yellow, 3=BigOrange, 4=Unknown
            if (current_cones[i].color == 4) {
                
                // 1. 重新投影这个雷达点到图像
                // drd25_msgs::Cone 没有 z 字段，默认 z 为 0.0
                Eigen::Vector4d pt_lidar(current_cones[i].x, current_cones[i].y, 0.0, 1.0);
                Eigen::Vector4d pt_cam = params.T_l2c * pt_lidar;
                
                if (pt_cam(2) > 0.1) { // 必须在相机前方
                    double u = (pt_cam(0) * params.K.at<double>(0,0) / pt_cam(2)) + params.K.at<double>(0,2);
                    double v = (pt_cam(1) * params.K.at<double>(1,1) / pt_cam(2)) + params.K.at<double>(1,2);

                    // 矫正畸变 (简单近似，或调用 opencv undistortPoints，这里假设畸变不大直接用)
                    // 如果需要极高精度，应使用 cv::projectPoints，但这里为了速度和逻辑简单直接手算

                    // 2. 在所有 2D 框中找最近的“颜色源”
                    double min_dist = 1e9;
                    int best_match_idx = -1;

                    for (size_t j = 0; j < standard_camera_msg->detections.size(); ++j) {
                        double cx = standard_camera_msg->detections[j].bbox.center.position.x;
                        double cy = standard_camera_msg->detections[j].bbox.center.position.y;

                        // 计算像素距离
                        double dist = std::sqrt(std::pow(u - cx, 2) + std::pow(v - cy, 2));
                        
                        if (dist < min_dist) {
                            min_dist = dist;
                            best_match_idx = j;
                        }
                    }

                    // 3. 如果最近的框在“吸铁石”范围内，强制上色！
                    if (best_match_idx != -1 && min_dist < magnet_radius) {
                        std::string class_id = standard_camera_msg->detections[best_match_idx].results[0].hypothesis.class_id;
                        try {
                            current_cones[i].color = std::stoi(class_id);
                            // 颜色被吸附了！
                        } catch (...) {}
                    }
                }
            }
        }

        // 7. 合并与发布
        std::vector<drd25_msgs::msg::Cone> final_cones = fusion_result.fused_cones;
        final_cones.insert(final_cones.end(), recovered_cones.begin(), recovered_cones.end());

        drd25_msgs::msg::Map map_msg;
        map_msg.header = custom_lidar_msg->header;
        map_msg.track = final_cones; 
        map_pub_->publish(map_msg);

        publish_markers(final_cones, custom_lidar_msg->header);
    }

    void publish_markers(const std::vector<drd25_msgs::msg::Cone>& cones, const std_msgs::msg::Header& header) {
        std::string viz_frame = this->get_parameter("lidar_frame").as_string();
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_all.header.frame_id = viz_frame; 
        markers.markers.push_back(delete_all);

        int id = 0;
        for(const auto& cone : cones) {
            visualization_msgs::msg::Marker m;
            m.header = header;
            m.header.frame_id = viz_frame;
            m.ns = "fused_cones";
            m.id = ++id;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = cone.x;
            m.pose.position.y = cone.y;
            m.pose.position.z = 0.0;
            m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.45;
            m.color.a = 0.8;
            
            // 灰色的 (Unknown) 现在应该很少了
            if(cone.color == 0) { m.color.b = 1.0; } // Blue
            else if(cone.color == 1) { m.color.r = 1.0; } // Red
            else if(cone.color == 2 || cone.color == 3) { m.color.r = 1.0; m.color.g = 1.0; } // Yellow
            else { m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; } // Unknown (Grey)
            
            m.lifetime = rclcpp::Duration::from_seconds(0.2);
            markers.markers.push_back(m);
        }
        marker_pub_->publish(markers);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<drd25_msgs::msg::Map>::SharedPtr map_pub_;
    message_filters::Subscriber<test_cone_segmentation::msg::ThreeDConeArray> lidar_sub_;
    message_filters::Subscriber<cone_interfaces::msg::ConeArray> camera_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        test_cone_segmentation::msg::ThreeDConeArray, cone_interfaces::msg::ConeArray>>> sync_;
};

} 

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fs_fusion_box::FusionNode>());
    rclcpp::shutdown();
    return 0;
}