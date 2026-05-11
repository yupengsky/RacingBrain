#include <cpp_controller/vehicle_pid_controller.hpp>


// 横向PID控制器
PIDLateralController::PIDLateralController(double K_PP, double C_PP, double I_PP, double D_PP): 
            _K_PP(K_PP) , _C_PP(C_PP) , _I_PP(I_PP) , _D_PP(D_PP) {}

// 计算两向量的夹角
double PIDLateralController::calculate_angle(const std::array<double, 2>& v1, const std::array<double, 2>& v2) {
    double dot_product = v1[0] * v2[0] + v1[1] * v2[1];
    double magnitude_v1 = std::hypot(v1[0], v1[1]);
    double magnitude_v2 = std::hypot(v2[0], v2[1]);
    double cos_theta = dot_product / (magnitude_v1 * magnitude_v2);
    return std::acos(cos_theta);
}

// 计算两点之间的距离
double PIDLateralController::calculate_distance(const std::array<double, 2>& v1, const std::array<double, 2>& v2) {
    return std::hypot(v1[0] - v2[0], v1[1] - v2[1]);
}

// run_step 方法实现
double PIDLateralController::run_step(
    const std::array<double, 3>& current_pose, 
    const std::deque<std::array<double, 2>> _waypoints_queue,
    const double current_speed) 
{
    double yaw = current_pose[2];
    std::array<double, 2> v_begin = {current_pose[0], current_pose[1]};
    std::array<double, 2> v_end = {v_begin[0] + std::cos(yaw), v_begin[1] + std::sin(yaw)};
    std::array<double, 2> v_vec = {v_end[0] - v_begin[0], v_end[1] - v_begin[1]};  // 当前航向向量

    lookahead_distance = _K_PP * current_speed + _C_PP;

    // 先找出前视点，再计算目标航向向量
    
    // 第一版：直接找第一个大于等于前视距离的点（默认路径队列从近到远排列）
    for(auto& waypoints:_waypoints_queue){
        waypoint = waypoints;
        if (calculate_distance(v_begin, waypoints) >= lookahead_distance) {
            break;
        }
    }
    

    // 第二版：找出距离最近的点
    /*
    double min_distance = 100000.0;
    bool found = false;
    for(auto& waypoints:_waypoints_queue){
        double temp_distance = calculate_distance(v_begin, waypoints);
        if (temp_distance >= lookahead_distance && temp_distance < min_distance) {
            min_distance = temp_distance;
            waypoint = waypoints;
            found = true;
        }
    }
    if (!found) {
        waypoint = _waypoints_queue.back();
    }
    */

    std::array<double, 2> w_vec = {waypoint[0] - v_begin[0], waypoint[1] - v_begin[1]};  // 目标航向向量

    // 计算两向量的夹角
    double _dot = calculate_angle(v_vec, w_vec);

    // 计算叉乘，判断转向方向
    double _cross = v_vec[0] * w_vec[1] - v_vec[1] * w_vec[0];
    if (_cross < 0) {
        _dot *= -1.0;
    
    }

    

    double previous_error = error;
    error = lookahead_distance * std::sin(_dot);

    // 限制积分项，避免积分饱和
    error_integral = std::clamp(error_integral + error, -40.0, 40.0);
    error_derivative = error - previous_error;

    // 计算PID输出
    double output = atan(2 * wheel_base * error / pow(lookahead_distance,2))+ _I_PP * error_integral + _D_PP * error_derivative;
    double steer = std::clamp(output/0.380635, -1.0, 1.0);  // 限制输出范围
    //double steer = std::clamp(output, -1.0, 1.0);  // 限制输出范围

    // 如果变化过快，保持上一次的转向值
    if (std::abs(steer - last_steer) > 1.0) {
        return last_steer;
    } 
    else {       
        last_steer = steer;
        return steer;
    }
}

// 纵向PID控制器
PIDLongitudinalController::PIDLongitudinalController(double K_P, double K_D, double K_I)
    : _K_P(K_P), _K_D(K_D), _K_I(K_I), error(0.0), error_integral(0.0), error_derivative(0.0) {}

// run_step 方法实现
double PIDLongitudinalController::run_step(double target_speed, double current_speed) {
    double previous_error = error;
    error = target_speed - current_speed;

    // 限制积分项，避免积分饱和
    error_integral = std::clamp(error_integral + error, -40.0, 40.0);
    error_derivative = error - previous_error;

    // 计算PID输出
    double output = _K_P * error + _K_I * error_integral + _K_D * error_derivative;

    // 限制输出范围
    return std::clamp(output, -1.0, 1.0);
}

// 车辆PID控制器
VehiclePIDController::VehiclePIDController(const std::map<std::string, double>& args_lateral,
                                           const std::map<std::string, double>& args_longitudinal)
    : _lon_controller(args_longitudinal.at("K_P"), args_longitudinal.at("K_D"), args_longitudinal.at("K_I")),
      _lat_controller(args_lateral.at("K_PP"), args_lateral.at("C_PP"), args_lateral.at("I_PP"), args_lateral.at("D_PP")) {}

// run_step 方法实现
std::pair<double, double> VehiclePIDController::run_step(
    double target_speed, 
    double current_speed, 
    const std::array<double, 3>& current_pose, 
    const std::deque<std::array<double, 2>> _waypoints_queue) 
{
    // 调用纵向控制器计算油门值
    double throttle = _lon_controller.run_step(target_speed, current_speed);

    // 调用横向控制器计算转向值
    double steering = _lat_controller.run_step(current_pose, _waypoints_queue, current_speed);

    // 返回转向和油门值
    return {steering, throttle};
}
