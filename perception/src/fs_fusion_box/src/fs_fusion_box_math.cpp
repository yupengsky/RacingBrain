#include "fs_fusion_box/fs_fusion_box_math.hpp"
#include <opencv2/core/eigen.hpp>
#include <algorithm>
#include <cmath>

namespace fs_fusion_box {

    // --- 1. 实现3D框投影 ---
    std::vector<ProjectedBox> project_3d_boxes_to_2d(
        const vision_msgs::msg::Detection3DArray::ConstSharedPtr& lidar_msg,
        const CalibrationParams& params) 
    {
        std::vector<ProjectedBox> results;

        for (size_t i = 0; i < lidar_msg->detections.size(); ++i) {
            const auto& detection = lidar_msg->detections[i];
            
            // 1. 获取3D框的中心和尺寸
            double cx = detection.bbox.center.position.x;
            double cy = detection.bbox.center.position.y;
            double cz = detection.bbox.center.position.z;
            double dx = detection.bbox.size.x / 2.0;
            double dy = detection.bbox.size.y / 2.0;
            double dz = detection.bbox.size.z / 2.0;

            // 2. 生成8个角点 (雷达坐标系)
            // 假设bbox旋转是标准的，如果有四元数旋转需要先转换，这里简化处理
            std::vector<Eigen::Vector4d> corners_lidar;
            int signs[8][3] = {
                {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
                {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
            };

            for(int k=0; k<8; ++k) {
                corners_lidar.push_back(Eigen::Vector4d(
                    cx + signs[k][0]*dx, 
                    cy + signs[k][1]*dy, 
                    cz + signs[k][2]*dz, 
                    1.0
                ));
            }

            // 3. 变换到相机坐标系并投影
            std::vector<cv::Point2f> img_pts;
            double center_depth = 0.0;

            for(const auto& pt_l : corners_lidar) {
                Eigen::Vector3d pt_c = (params.T_l2c * pt_l).head<3>();
                
                // 简单的深度检查
                if(pt_c.z() <= 0.1) continue; 

                // 手动投影或使用 projectPoints (为了性能这里简化手写)
                double u = (pt_c.x() * params.K.at<double>(0,0) / pt_c.z()) + params.K.at<double>(0,2);
                double v = (pt_c.y() * params.K.at<double>(1,1) / pt_c.z()) + params.K.at<double>(1,2);
                
                img_pts.push_back(cv::Point2f(u, v));
            }

            // 计算中心点的深度用于后续记录
            Eigen::Vector4d center_l(cx, cy, cz, 1.0);
            center_depth = (params.T_l2c * center_l).z();

            // 4. 生成包络矩形
            ProjectedBox proj_box;
            proj_box.original_index = i;
            proj_box.depth = center_depth;
            proj_box.valid = false;

            if (!img_pts.empty()) {
                cv::Rect bounding_rect = cv::boundingRect(img_pts);
                
                // 边界保护：截断在图像范围内
                bounding_rect = bounding_rect & cv::Rect(0, 0, params.img_w, params.img_h);

                if (bounding_rect.area() > 0) {
                    proj_box.rect = bounding_rect;
                    proj_box.valid = true;
                }
            }
            results.push_back(proj_box);
        }
        return results;
    }

    // --- 2. 计算 IoU (交并比) ---
    double calculate_overlap(const cv::Rect& box1, const cv::Rect& box2) {
        cv::Rect intersection = box1 & box2;
        if (intersection.area() <= 0) return 0.0;

        double area_inter = intersection.area();
        double area_union = box1.area() + box2.area() - area_inter;
        
        return area_inter / area_union;
    }

    // --- 3. 核心融合逻辑 ---
    FusionResult fuse_measurements(
        const std::vector<ProjectedBox>& proj_boxes,
        const vision_msgs::msg::Detection3DArray::ConstSharedPtr& lidar_msg,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& camera_msg,
        double overlap_threshold)
    {
        FusionResult result;
        std::vector<bool> is_camera_matched(camera_msg->detections.size(), false);

        // 遍历所有雷达投影框
        for (const auto& pbox : proj_boxes) {
            drd25_msgs::msg::Cone cone;
            // 获取原始3D坐标
            const auto& raw_det = lidar_msg->detections[pbox.original_index];
            cone.x = raw_det.bbox.center.position.x;
            cone.y = raw_det.bbox.center.position.y;
            
            // 默认颜色：Unknown (如果在图像外，或者没匹配上)
            cone.color = drd25_msgs::msg::Cone::UNKNOWN;

            if (pbox.valid) {
                // 在 YOLO 结果中寻找最佳匹配
                double max_iou = 0.0;
                int best_match_idx = -1;

                for (size_t i = 0; i < camera_msg->detections.size(); ++i) {
                    // 如果你要严格的一对一匹配，可以加 if(is_camera_matched[i]) continue;
                    // 但这里允许容错，先找IOU最大的

                    // 转换 vision_msgs bbox 到 cv::Rect
                    auto& bbox = camera_msg->detections[i].bbox;
                    cv::Rect cam_rect(
                        bbox.center.position.x - bbox.size_x/2,
                        bbox.center.position.y - bbox.size_y/2,
                        bbox.size_x, bbox.size_y
                    );

                    double iou = calculate_overlap(pbox.rect, cam_rect);
                    if (iou > max_iou) {
                        max_iou = iou;
                        best_match_idx = i;
                    }
                }

                // 判定是否匹配成功
                if (best_match_idx != -1 && max_iou > overlap_threshold) {
                    // 置信度高：把 YOLO 的颜色赋给 3D 框
                    if (!camera_msg->detections[best_match_idx].results.empty()) {
                        int class_id = std::stoi(camera_msg->detections[best_match_idx].results[0].hypothesis.class_id);
                        
                        // 简单的ID映射，防止越界
                        if (class_id >= 0 && class_id <= 4) {
                             cone.color = static_cast<uint8_t>(class_id);
                        }
                    }
                    is_camera_matched[best_match_idx] = true; // 标记该YOLO框已被使用
                }
                // else: 置信度低，保持 Unknown
            }
            
            result.fused_cones.push_back(cone);
        }

        // 记录哪些YOLO框是多出来的（雷达没看到的）
        for (size_t i = 0; i < camera_msg->detections.size(); ++i) {
            if (!is_camera_matched[i]) {
                result.unmatched_camera_indices.push_back(i);
            }
        }

        return result;
    }

    // --- 4. 视觉反推 (补漏) ---
    std::vector<drd25_msgs::msg::Cone> recover_missing_cones(
        const std::vector<int>& unmatched_indices,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& camera_msg,
        const CalibrationParams& params)
    {
        std::vector<drd25_msgs::msg::Cone> recovered_cones;
        
        // 相机坐标系 -> 雷达坐标系 (T_c2l = T_l2c的逆)
        Eigen::Matrix4d T_c2l = params.T_l2c.inverse();

        for (int idx : unmatched_indices) {
            const auto& det = camera_msg->detections[idx];
            
            // 简单的单目测距模型：基于“假设地面平坦”或“相似三角形”
            // 这里使用简化的相似三角形原理： Z = (f * real_height) / pixel_height
            
            double fx = params.K.at<double>(0,0);
            double fy = params.K.at<double>(1,1);
            double cx = params.K.at<double>(0,2);
            
            // 假设锥筒真实高度 ~30cm (小锥筒)
            double real_height = 0.30; 
            if (det.bbox.size_y < 5.0) continue; // 太小了忽略

            double z_c = (fy * real_height) / det.bbox.size_y;
            double x_c = (det.bbox.center.position.x - cx) * z_c / fx;
            double y_c = 0.2; // 假设光心离地高度的相对值，或者设为0让T矩阵去处理

            Eigen::Vector4d pt_cam(x_c, y_c, z_c, 1.0);
            Eigen::Vector4d pt_lidar = T_c2l * pt_cam;

            drd25_msgs::msg::Cone cone;
            cone.x = pt_lidar.x();
            cone.y = pt_lidar.y();
            
            // 颜色直接取YOLO的
            if (!det.results.empty()) {
                cone.color = static_cast<uint8_t>(std::stoi(det.results[0].hypothesis.class_id));
            } else {
                cone.color = drd25_msgs::msg::Cone::UNKNOWN;
            }

            recovered_cones.push_back(cone);
        }

        return recovered_cones;
    }

} // namespace fs_fusion_box