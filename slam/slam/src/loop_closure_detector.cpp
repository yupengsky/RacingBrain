#include "slam/loop_closure_detector.hpp"

// 这是在EnhancedLoopClosureDetector类下创建的同名构造函数
// 括号内为构造函数的输入参数
EnhancedLoopClosureDetector::EnhancedLoopClosureDetector(
    double distance_threshold,
    double approach_angle_threshold,
    int min_consecutive_detections,
    int max_consecutive_detections,
    double min_time_between_detections,
    int history_buffer_size,
    double min_approach_speed,
    double min_lap_time
) : 
    // 构造函数成员初始化列表
    // origin_position_ = (0, 0, 0)
    origin_position_(Eigen::Vector3d::Zero()),
    start_time_(0.0),
    is_start_time_set_(false),
    is_loop_origin_set_(false),
    consecutive_detections_(0),
    last_detection_time_(0.0),
    loop_closed_(false)
{   
    configure(      // configure()为ELCD类中的成员函数
        distance_threshold,
        approach_angle_threshold,
        min_consecutive_detections,
        max_consecutive_detections,
        min_time_between_detections,
        history_buffer_size,
        min_approach_speed,
        min_lap_time
    );
}

// 配置参数
void EnhancedLoopClosureDetector::configure(
    double distance_threshold,
    double approach_angle_threshold,
    int min_consecutive_detections,
    int max_consecutive_detections,
    double min_time_between_detections,
    int history_buffer_size,
    double min_approach_speed,
    double min_lap_time
) {
    // 更新参数
    distance_threshold_ = distance_threshold;
    approach_angle_threshold_ = approach_angle_threshold * M_PI / 180.0;    //使用弧度制
    min_consecutive_detections_ = min_consecutive_detections;
    max_consecutive_detections_ = max_consecutive_detections;
    min_time_between_detections_ = min_time_between_detections;
    history_buffer_size_ = history_buffer_size;
    min_approach_speed_ = min_approach_speed;
    min_lap_time_ = min_lap_time;
}

void EnhancedLoopClosureDetector::setOrigin(const Eigen::Vector3d& position) {
    origin_position_ = position;    // 记录原点坐标
    is_loop_origin_set_ = true;
    is_start_time_set_ = false;
    consecutive_detections_ = 0;    // 清零连续检测计数器
    loop_closed_ = false;           // 重置闭环状态
    position_history_.clear();      // 清空位置历史记录
    timestamp_history_.clear();     // 清空时间戳历史记录
}

// ************************ 判断是否触发回环的核心逻辑 ************************
// 在感知与GNSS的同步回调中调用，每传入一帧都会进行回环判断
bool EnhancedLoopClosureDetector::detectLoopClosure(
    double current_time,
    const Eigen::Vector3d& current_position,
    bool is_exploration_lap
) {
    if (!is_loop_origin_set_) return false;

    if (!is_start_time_set_){
        start_time_ = current_time;
        is_start_time_set_ = true;
        RCLCPP_INFO(rclcpp::get_logger("loop_closure"), "Start time recorded. Waiting for %.1f seconds to enable LoopClosure.", min_lap_time_);
    }
    
    // 第一步检查：最小圈时检查
    // 第一次触发回环的时间 - 第一帧地图的时间 应大于 一定时间阈值
    double elapsed_time = current_time - start_time_;
    if (elapsed_time < min_lap_time_){
        return false;
    }
    // 建立一个缓冲区，里面保存一定数量的历史数据（包含时间戳和车辆位置）
    updateHistory(current_time, current_position);
    
    // 如果触发了回环，会有一定冷却时间，冷却时间内loop_closed_ = true
    // 如果过了回环冷却时间，车辆重新进入检测回环状态
    if (loop_closed_ && (current_time - last_detection_time_ > min_time_between_detections_)) {
        loop_closed_ = false;
        consecutive_detections_ = 0;
    }
    
    // 第二步检查：几何条件检查
    // 分别检查距离条件，几何条件，速度条件，三个条件需同时满足，检测到回环次数+1
    // 如果有一帧不满足，则次数归零（意味着需要有多帧连续满足该几何约束）
    // 如果是探索圈，速度较慢，我们连续匹配帧数的要求可以给到更高一点的数值
    // 如果是冲刺圈，在检测范围内很可能不足以有时间连续满足多帧匹配，所以我们把连续匹配帧数的要求降低
    int detection_threshold = is_exploration_lap ? max_consecutive_detections_ : min_consecutive_detections_;
    double distance_to_origin = calculateDistanceToOrigin(current_position);
    bool distance_condition = (distance_to_origin < distance_threshold_);
    bool direction_condition = isMovingTowardsOrigin(current_position);
    bool speed_condition = hasSufficientApproachSpeed();
    
    if (distance_condition && direction_condition && speed_condition) {
        consecutive_detections_++;
        RCLCPP_INFO(rclcpp::get_logger("loop_closure"), 
            "Loop condition met: dist=%.3f, count: %d/total: %d", distance_to_origin, consecutive_detections_, detection_threshold);
    } else {
        consecutive_detections_ = 0; 
    }
    // 触发判定
    bool loop_detected = false;
    if (consecutive_detections_ >= detection_threshold && 
        !loop_closed_ && 
        (current_time - last_detection_time_ > min_time_between_detections_)) {

        loop_detected = true;
        loop_closed_ = true;
        last_detection_time_ = current_time;
        RCLCPP_INFO(rclcpp::get_logger("loop_closure"), ">>> LOOP CLOSURE DETECTED <<<");
    }
    
    return loop_detected;
}

void EnhancedLoopClosureDetector::resetLoopStatus() {
    loop_closed_ = false;
    consecutive_detections_ = 0;
}

double EnhancedLoopClosureDetector::calculateDistanceToOrigin(const Eigen::Vector3d& position) const {
    return (position - origin_position_).norm();
}

// 在一定数量的缓冲区内添加历史轨迹
void EnhancedLoopClosureDetector::updateHistory(double timestamp, const Eigen::Vector3d& position) {
    position_history_.push_back(position);
    timestamp_history_.push_back(timestamp);
    if (position_history_.size() > static_cast<size_t>(history_buffer_size_)) {
        position_history_.pop_front();
        timestamp_history_.pop_front();
    }
}

bool EnhancedLoopClosureDetector::isMovingTowardsOrigin(const Eigen::Vector3d& current_position) const {
    if (position_history_.size() < 2) return false;
    
    // 新一帧坐标减去上一帧坐标，得到代表当前赛车行驶方向的向量
    Eigen::Vector3d current_to_prev = position_history_.back() - position_history_[position_history_.size()-2];
    if (current_to_prev.norm() < 1e-3) return false; 
    // 起点坐标减去新一帧坐标，得到赛车从当前位置指向起始位置的向量
    Eigen::Vector3d current_to_origin = origin_position_ - current_position;
    if (current_to_origin.norm() < 1e-3) return true; 
    
    // 向量归一化与夹角运算
    // 两个向量的夹角需要与一个阈值，才能说明车是朝着起点驶去
    Eigen::Vector3d motion_direction = current_to_prev.normalized();
    Eigen::Vector3d to_origin_direction = current_to_origin.normalized();
    
    double cos_angle = motion_direction.dot(to_origin_direction);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle)); 
    return (std::acos(cos_angle) < approach_angle_threshold_);
}

bool EnhancedLoopClosureDetector::hasSufficientApproachSpeed() const {
    if (position_history_.size() < 2) return false;
    
    Eigen::Vector3d diff = position_history_.back() - position_history_[position_history_.size()-2];
    double dt = timestamp_history_.back() - timestamp_history_[timestamp_history_.size()-2];
    if (dt < 1e-3) return false;
    
    Eigen::Vector3d vel = diff / dt;
    Eigen::Vector3d to_origin = (origin_position_ - position_history_.back()).normalized();
    
    return (std::abs(vel.dot(to_origin)) >= min_approach_speed_);
}
