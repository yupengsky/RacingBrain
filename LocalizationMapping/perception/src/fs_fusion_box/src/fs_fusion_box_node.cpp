#include <Eigen/Dense> 

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/core/eigen.hpp>
#include <std_msgs/msg/string.hpp>

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

namespace {
using SteadyClock = std::chrono::steady_clock;

double elapsed_ms(SteadyClock::time_point start, SteadyClock::time_point end = SteadyClock::now())
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

double stamp_to_sec(const std_msgs::msg::Header& header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}

double clamp01(double value)
{
    if (!std::isfinite(value)) return 0.0;
    return std::max(0.0, std::min(1.0, value));
}

double ratio_or_nan(double numerator, double denominator)
{
    if (denominator <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return numerator / denominator;
}

double mean_or_nan(const std::vector<double>& values)
{
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

double percentile_or_nan(std::vector<double> values, double fraction)
{
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    if (values.size() == 1) {
        return values.front();
    }
    const double index = (static_cast<double>(values.size()) - 1.0) * fraction;
    const auto lo = static_cast<size_t>(std::floor(index));
    const auto hi = static_cast<size_t>(std::ceil(index));
    if (lo == hi) {
        return values[lo];
    }
    const double weight = index - static_cast<double>(lo);
    return values[lo] * (1.0 - weight) + values[hi] * weight;
}

std::string json_number(double value)
{
    if (!std::isfinite(value)) {
        return "null";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(9) << value;
    return out.str();
}
}  // namespace

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("fusion_box_node") {
        // --- 核心参数 ---
        this->declare_parameter("sync_window", 0.1);
        this->declare_parameter("lidar_frame", "hesai_lidar"); 
        this->declare_parameter("evaluation.enable_debug_metrics", false);
        this->declare_parameter("runtime_health.enable_metrics", false);
        eval_metrics_enabled_ = this->get_parameter("evaluation.enable_debug_metrics").as_bool();
        health_metrics_enabled_ = this->get_parameter("runtime_health.enable_metrics").as_bool();
        metrics_enabled_ = eval_metrics_enabled_ || health_metrics_enabled_;

        // [强力测试参数] 吸铁石半径 (像素)
        // 只要雷达点投影在相机框中心 60 像素以内，强制上色！
        // 这能无视外参的轻微偏差
        this->declare_parameter("force_match_radius", 60.0); 
        this->declare_parameter("iou_match_threshold", 0.3);

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
        if (metrics_enabled_) {
            metrics_pub_ = this->create_publisher<std_msgs::msg::String>("/perception/fusion/evaluation/metrics", 10);
        }

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
        const auto callback_start = SteadyClock::now();
        double convert_3d_ms = 0.0;
        double convert_2d_ms = 0.0;
        double parameter_load_ms = 0.0;
        double project_fuse_ms = 0.0;
        double recover_ms = 0.0;
        double magnet_ms = 0.0;
        double publish_ms = 0.0;
        int lidar_count = static_cast<int>(custom_lidar_msg->cones.size());
        int camera_count = static_cast<int>(custom_camera_msg->cones.size());
        int projected_count = 0;
        int valid_projected_count = 0;
        int fused_count = 0;
        int recovered_count = 0;
        int final_count = 0;
        int unknown_count = 0;
        int force_match_count = 0;
        int iou_match_count = 0;
        int unmatched_camera_count = 0;
        int association_sample_count = 0;
        int low_iou_count = 0;
        double sync_window_ms = this->get_parameter("sync_window").as_double() * 1000.0;
        double iou_match_threshold = this->get_parameter("iou_match_threshold").as_double();
        double magnet_radius = this->get_parameter("force_match_radius").as_double();
        double stamp_delta_ms = (stamp_to_sec(custom_camera_msg->header) - stamp_to_sec(custom_lidar_msg->header)) * 1000.0;
        double abs_stamp_delta_ms = std::abs(stamp_delta_ms);
        double valid_projection_ratio = std::numeric_limits<double>::quiet_NaN();
        double iou_match_ratio = std::numeric_limits<double>::quiet_NaN();
        double unmatched_camera_ratio = std::numeric_limits<double>::quiet_NaN();
        double low_iou_ratio = std::numeric_limits<double>::quiet_NaN();
        double mean_nearest_camera_error_px = std::numeric_limits<double>::quiet_NaN();
        double p95_nearest_camera_error_px = std::numeric_limits<double>::quiet_NaN();
        double mean_best_iou = std::numeric_limits<double>::quiet_NaN();
        double min_best_iou = std::numeric_limits<double>::quiet_NaN();
        double unknown_ratio = std::numeric_limits<double>::quiet_NaN();
        double recovered_ratio = std::numeric_limits<double>::quiet_NaN();
        double force_match_ratio = std::numeric_limits<double>::quiet_NaN();
        double consistency_score = std::numeric_limits<double>::quiet_NaN();
        double calibration_drift_score = std::numeric_limits<double>::quiet_NaN();

        auto publish_metrics = [&](const std::string& event) {
            if (!metrics_enabled_ || !metrics_pub_) return;

            std_msgs::msg::String msg;
            std::ostringstream out;
            out << std::fixed << std::setprecision(9);
            out << "{"
                << "\"component\":\"fusion\""
                << ",\"event\":\"" << event << "\""
                << ",\"stamp\":" << stamp_to_sec(custom_lidar_msg->header)
                << ",\"lidar_frame_id\":\"" << custom_lidar_msg->header.frame_id << "\""
                << ",\"camera_frame_id\":\"" << custom_camera_msg->header.frame_id << "\""
                << ",\"lidar_count\":" << lidar_count
                << ",\"camera_count\":" << camera_count
                << ",\"projected_count\":" << projected_count
                << ",\"valid_projected_count\":" << valid_projected_count
                << ",\"fused_count\":" << fused_count
                << ",\"recovered_count\":" << recovered_count
                << ",\"final_count\":" << final_count
                << ",\"unknown_count\":" << unknown_count
                << ",\"force_match_count\":" << force_match_count
                << ",\"iou_match_count\":" << iou_match_count
                << ",\"unmatched_camera_count\":" << unmatched_camera_count
                << ",\"association_sample_count\":" << association_sample_count
                << ",\"low_iou_count\":" << low_iou_count
                << ",\"sync_window_ms\":" << json_number(sync_window_ms)
                << ",\"camera_lidar_stamp_delta_ms\":" << json_number(stamp_delta_ms)
                << ",\"abs_camera_lidar_stamp_delta_ms\":" << json_number(abs_stamp_delta_ms)
                << ",\"valid_projection_ratio\":" << json_number(valid_projection_ratio)
                << ",\"iou_match_ratio\":" << json_number(iou_match_ratio)
                << ",\"unmatched_camera_ratio\":" << json_number(unmatched_camera_ratio)
                << ",\"low_iou_ratio\":" << json_number(low_iou_ratio)
                << ",\"mean_nearest_camera_error_px\":" << json_number(mean_nearest_camera_error_px)
                << ",\"p95_nearest_camera_error_px\":" << json_number(p95_nearest_camera_error_px)
                << ",\"mean_best_iou\":" << json_number(mean_best_iou)
                << ",\"min_best_iou\":" << json_number(min_best_iou)
                << ",\"unknown_ratio\":" << json_number(unknown_ratio)
                << ",\"recovered_ratio\":" << json_number(recovered_ratio)
                << ",\"force_match_ratio\":" << json_number(force_match_ratio)
                << ",\"consistency_score\":" << json_number(consistency_score)
                << ",\"calibration_drift_score\":" << json_number(calibration_drift_score)
                << ",\"iou_match_threshold\":" << json_number(iou_match_threshold)
                << ",\"magnet_radius_px\":" << json_number(magnet_radius)
                << ",\"convert_3d_ms\":" << convert_3d_ms
                << ",\"convert_2d_ms\":" << convert_2d_ms
                << ",\"parameter_load_ms\":" << parameter_load_ms
                << ",\"project_fuse_ms\":" << project_fuse_ms
                << ",\"recover_ms\":" << recover_ms
                << ",\"magnet_ms\":" << magnet_ms
                << ",\"publish_ms\":" << publish_ms
                << ",\"total_ms\":" << elapsed_ms(callback_start)
                << "}";
            msg.data = out.str();
            metrics_pub_->publish(msg);
        };

        // 1. 3D 数据转换
        const auto convert_3d_start = SteadyClock::now();
        auto standard_lidar_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();
        standard_lidar_msg->header = custom_lidar_msg->header;
        for (const auto& custom_cone : custom_lidar_msg->cones) {
            vision_msgs::msg::Detection3D det;
            det.bbox.center.position = custom_cone.center;
            det.bbox.size = custom_cone.size;
            standard_lidar_msg->detections.push_back(det);
        }
        convert_3d_ms = elapsed_ms(convert_3d_start);

        // 2. 2D 数据转换
        const auto convert_2d_start = SteadyClock::now();
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
        convert_2d_ms = elapsed_ms(convert_2d_start);

        // 3. 参数加载
        const auto parameter_load_start = SteadyClock::now();
        CalibrationParams params;
        std::vector<double> matrix_data = this->get_parameter("lidar_to_camera_matrix").as_double_array();
        if (matrix_data.size() != 16) {
            parameter_load_ms = elapsed_ms(parameter_load_start);
            publish_metrics("missing_calibration");
            return;
        }
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
        parameter_load_ms = elapsed_ms(parameter_load_start);

        // 4. 标准融合 (先试着正常匹配一下)
        const auto project_fuse_start = SteadyClock::now();
        auto proj_boxes = project_3d_boxes_to_2d(standard_lidar_msg, params);
        projected_count = static_cast<int>(proj_boxes.size());
        for (const auto& box : proj_boxes) {
            if (box.valid) valid_projected_count++;
        }
        valid_projection_ratio = ratio_or_nan(valid_projected_count, lidar_count);

        std::vector<double> nearest_camera_errors_px;
        std::vector<double> best_ious;
        for (const auto& pbox : proj_boxes) {
            if (!pbox.valid || standard_camera_msg->detections.empty()) {
                continue;
            }
            const double proj_cx = static_cast<double>(pbox.rect.x) + static_cast<double>(pbox.rect.width) * 0.5;
            const double proj_cy = static_cast<double>(pbox.rect.y) + static_cast<double>(pbox.rect.height) * 0.5;
            double best_dist = std::numeric_limits<double>::infinity();
            double best_iou = 0.0;

            for (const auto& det : standard_camera_msg->detections) {
                const double cam_cx = det.bbox.center.position.x;
                const double cam_cy = det.bbox.center.position.y;
                const double dist = std::hypot(proj_cx - cam_cx, proj_cy - cam_cy);
                best_dist = std::min(best_dist, dist);

                cv::Rect cam_rect(
                    det.bbox.center.position.x - det.bbox.size_x / 2.0,
                    det.bbox.center.position.y - det.bbox.size_y / 2.0,
                    det.bbox.size_x,
                    det.bbox.size_y
                );
                best_iou = std::max(best_iou, calculate_overlap(pbox.rect, cam_rect));
            }

            if (std::isfinite(best_dist)) {
                nearest_camera_errors_px.push_back(best_dist);
            }
            best_ious.push_back(best_iou);
            if (best_iou < iou_match_threshold) {
                low_iou_count++;
            }
        }
        association_sample_count = static_cast<int>(best_ious.size());
        mean_nearest_camera_error_px = mean_or_nan(nearest_camera_errors_px);
        p95_nearest_camera_error_px = percentile_or_nan(nearest_camera_errors_px, 0.95);
        mean_best_iou = mean_or_nan(best_ious);
        min_best_iou = best_ious.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::min_element(best_ious.begin(), best_ious.end());
        low_iou_ratio = ratio_or_nan(low_iou_count, association_sample_count);

        auto fusion_result = fuse_measurements(proj_boxes, standard_lidar_msg, standard_camera_msg, iou_match_threshold);
        fused_count = static_cast<int>(fusion_result.fused_cones.size());
        unmatched_camera_count = static_cast<int>(fusion_result.unmatched_camera_indices.size());
        unmatched_camera_ratio = ratio_or_nan(unmatched_camera_count, camera_count);
        for (const auto& cone : fusion_result.fused_cones) {
            if (cone.color != drd25_msgs::msg::Cone::UNKNOWN) {
                iou_match_count++;
            }
        }
        iou_match_ratio = ratio_or_nan(iou_match_count, lidar_count);
        project_fuse_ms = elapsed_ms(project_fuse_start);

        // 5. 补全 (检测远距离)
        // 这一步生成纯视觉锥筒，保持远距离检测能力
        const auto recover_start = SteadyClock::now();
        auto recovered_cones = recover_missing_cones(fusion_result.unmatched_camera_indices, standard_camera_msg, params);
        recovered_count = static_cast<int>(recovered_cones.size());
        recover_ms = elapsed_ms(recover_start);

        // ==========================================
        // [新增] 强力吸铁石：修复近处灰色圆柱
        // ==========================================
        std::vector<drd25_msgs::msg::Cone>& current_cones = fusion_result.fused_cones;

        // 遍历所有已经是“灰色(Unknown)”的雷达锥筒
        const auto magnet_start = SteadyClock::now();
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
                            force_match_count++;
                            // 颜色被吸附了！
                        } catch (...) {}
                    }
                }
            }
        }
        magnet_ms = elapsed_ms(magnet_start);

        // 7. 合并与发布
        std::vector<drd25_msgs::msg::Cone> final_cones = fusion_result.fused_cones;
        final_cones.insert(final_cones.end(), recovered_cones.begin(), recovered_cones.end());
        final_count = static_cast<int>(final_cones.size());
        for (const auto& cone : final_cones) {
            if (cone.color == drd25_msgs::msg::Cone::UNKNOWN) unknown_count++;
        }
        unknown_ratio = ratio_or_nan(unknown_count, final_count);
        recovered_ratio = ratio_or_nan(recovered_count, final_count);
        force_match_ratio = ratio_or_nan(force_match_count, final_count);

        const double time_quality = clamp01(1.0 - abs_stamp_delta_ms / std::max(1.0, sync_window_ms));
        const double projection_quality = clamp01(std::isfinite(valid_projection_ratio) ? valid_projection_ratio : 0.0);
        const double pixel_quality = std::isfinite(mean_nearest_camera_error_px)
            ? clamp01(1.0 - mean_nearest_camera_error_px / std::max(1.0, 2.0 * magnet_radius))
            : 0.0;
        const double output_quality = clamp01(1.0 - (std::isfinite(unknown_ratio) ? unknown_ratio : 1.0));
        const double force_quality = clamp01(1.0 - (std::isfinite(force_match_ratio) ? force_match_ratio : 0.0));
        consistency_score = clamp01(
            0.25 * time_quality +
            0.20 * projection_quality +
            0.25 * pixel_quality +
            0.15 * output_quality +
            0.15 * force_quality
        );

        const double residual_risk = std::isfinite(mean_nearest_camera_error_px)
            ? clamp01(mean_nearest_camera_error_px / std::max(1.0, magnet_radius))
            : 0.0;
        const double low_iou_risk = std::isfinite(low_iou_ratio) ? clamp01(low_iou_ratio) : 0.0;
        const double force_risk = std::isfinite(force_match_ratio) ? clamp01(force_match_ratio) : 0.0;
        const double time_risk = clamp01(abs_stamp_delta_ms / std::max(1.0, sync_window_ms));
        calibration_drift_score = clamp01(
            0.45 * residual_risk +
            0.30 * low_iou_risk +
            0.15 * force_risk +
            0.10 * time_risk
        );

        const auto publish_start = SteadyClock::now();
        drd25_msgs::msg::Map map_msg;
        map_msg.header = custom_lidar_msg->header;
        map_msg.track = final_cones; 
        map_pub_->publish(map_msg);

        publish_markers(final_cones, custom_lidar_msg->header);
        publish_ms = elapsed_ms(publish_start);
        publish_metrics("processed");
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
    message_filters::Subscriber<test_cone_segmentation::msg::ThreeDConeArray> lidar_sub_;
    message_filters::Subscriber<cone_interfaces::msg::ConeArray> camera_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        test_cone_segmentation::msg::ThreeDConeArray, cone_interfaces::msg::ConeArray>>> sync_;
    bool eval_metrics_enabled_ = false;
    bool health_metrics_enabled_ = false;
    bool metrics_enabled_ = false;
};

} 

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fs_fusion_box::FusionNode>());
    rclcpp::shutdown();
    return 0;
}
