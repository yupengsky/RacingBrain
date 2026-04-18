#pragma once

#include <vector>
#include <deque>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
// 消息依赖
#include "drd25_msgs/msg/cone.hpp"

// 全局锥桶结构体定义
struct GlobalCone {
    int id;                   // 锥桶id
    Eigen::Vector2d pos;      // 状态量：[x, y]
    Eigen::Matrix2d P;        // 协方差矩阵
    float r, g, b;            // 颜色
    bool is_stable = false;   // 是否已收敛
    double existence_score = 0.0;
    bool matched_this_frame = false;    // 是否被匹配
    int type;                 // 存储于 msg 中的 color
};

// 回环检测器类声明
class EnhancedLoopClosureDetector {
public:
    EnhancedLoopClosureDetector(
        double distance_threshold = 0.2,
        double approach_angle_threshold = 30.0,
        int min_consecutive_detections = 2,
        int max_consecutive_detections = 5,
        double min_time_between_detections = 2.0,
        int history_buffer_size = 5,
        double min_approach_speed = 0.1,
        double min_lap_time = 20.0
    );

    // 接收来自.yaml文件的参数并覆盖默认值
    void configure(
        double distance_threshold,
        double approach_angle_threshold,
        int max_consecutive_detections,
        int min_consecutive_detections,
        double min_time_between_detections,
        int history_buffer_size,
        double min_approach_speed,
        double min_lap_time
    );

    // 设置原点位置
    void setOrigin(const Eigen::Vector3d& position);

    // 检测是否发生回环
    bool detectLoopClosure(
        double current_time,
        const Eigen::Vector3d& current_position,
        bool is_exploration_lap
    );

    // 重置回环状态
    void resetLoopStatus();

private:
    // 内部辅助函数
    double calculateDistanceToOrigin(const Eigen::Vector3d& position) const;
    void updateHistory(double timestamp, const Eigen::Vector3d& position);
    bool isMovingTowardsOrigin(const Eigen::Vector3d& current_position) const;
    bool hasSufficientApproachSpeed() const;

private:
    // 内部成员变量
    Eigen::Vector3d origin_position_;
    bool is_loop_origin_set_;
    
    // 时间戳
    double start_time_;     // 起跑时间
    bool is_start_time_set_;

    // 参数
    double distance_threshold_;     // 几何距离阈值
    double approach_angle_threshold_;   // 运动朝向阈值
    double min_approach_speed_;     // 最低接近速度
    int min_consecutive_detections_;    // 连续满足回环条件最小值
    int max_consecutive_detections_;    // 连续满足回环条件最小值
    double min_time_between_detections_;    // 触发回环冷却时间
    int history_buffer_size_;   // 历史轨迹缓存
    double min_lap_time_;  //触发回环最小圏时
    
    // 状态
    int consecutive_detections_;
    double last_detection_time_;
    bool loop_closed_;
    
    // 历史缓存
    std::deque<Eigen::Vector3d> position_history_;
    std::deque<double> timestamp_history_;
};