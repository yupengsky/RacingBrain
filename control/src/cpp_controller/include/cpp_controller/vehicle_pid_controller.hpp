#ifndef VEHICLE_PID_CONTROLLER_HPP
#define VEHICLE_PID_CONTROLLER_HPP

#include <deque>       // 用于双端队列
#include <map>         // 用于存储PID参数
#include <cmath>       // 用于数学计算
#include <algorithm>   // 用于std::clamp
#include <array>       // 用于存储向量s
#include <string>      // 用于存储字符串

// 定义Pose和Waypoint类型
using Pose = std::array<double, 3>;       // [x, y, yaw]
using Waypoint = std::array<double, 2>;   // [x, y]

// 横向PID控制器
class PIDLateralController {
public:
    // 构造函数，默认参数
    PIDLateralController(double K_PP = 1.0, double C_PP = 2.0, double I_PP = 0.0, double D_PP = 0.0);

    // 执行一步控制，返回转向角度
    double run_step(
        const std::array<double, 3>& current_pose, 
        const std::deque<std::array<double, 2>> _waypoints_queue, 
        double curent_speed
    );

private:
    double wheel_base = 1.53;  // 车辆轴距
    double lookahead_distance;  // 前视距离
    double last_steer;  // 上一次转向角度
    double _K_PP;  // 比例系数
    double _C_PP;  // 前视距离系数
    double _I_PP;  // 积分系数
    double _D_PP;  // 微分系数
    double error;  // 当前误差
    double error_integral;  // 积分项误差
    double error_derivative;  // 微分项误差

    std::array<double, 2> waypoint;  // 目标点

    double calculate_angle(const std::array<double, 2>& v1, const std::array<double, 2>& v2);
    double calculate_distance(const std::array<double, 2>& v1, const std::array<double, 2>& v2);
};

// 纵向PID控制器
class PIDLongitudinalController {
public:
    // 构造函数，默认参数
    PIDLongitudinalController(double K_P = 1.0, double K_D = 0.0, double K_I = 0.0);

    // 执行一步控制，返回油门值
    double run_step(double target_speed, double current_speed);

private:
    double _K_P;  // 比例系数
    double _K_D;  // 微分系数
    double _K_I;  // 积分系数
    double error;  // 当前误差
    double error_integral;  // 积分项误差
    double error_derivative;  // 微分项误差
};

// 车辆PID控制器
class VehiclePIDController {
public:
    // 构造函数，默认参数
    VehiclePIDController(const std::map<std::string, double>& args_lateral = {{"K_PP", 1.0}, {"C_PP", 2.0}, {"I_PP", 0.0}, {"D_PP", 0.0}},
                         const std::map<std::string, double>& args_longitudinal = {{"K_P", 1.0}, {"K_D", 0.0}, {"K_I", 0.0}});

    // 执行一步控制，返回转向和油门值
    std::pair<double, double> run_step(
        double target_speed, 
        double current_speed, 
        const std::array<double, 3>& current_pose, 
        const std::deque<std::array<double, 2>> _waypoints_queue
    );

private:
    PIDLongitudinalController _lon_controller;  // 纵向PID控制器
    PIDLateralController _lat_controller;       // 横向PID控制器
};

#endif // VEHICLE_PID_CONTROLLER_HPP

