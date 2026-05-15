#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/parameter.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <fs_msgs/msg/control_command.hpp>
#include <drd25_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rclcpp/subscription_options.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cpp_controller/vehicle_pid_controller.hpp>
#include <cpp_controller/longitudinal_vp.hpp>

#include <tf2/utils.h> // 用于四元数转欧拉角

#include <map>
#include <deque>
#include <cmath>
#include <vector>
#include <memory>
#include <iostream>

double euclidean_distance(const std::vector<double>& v1, const std::vector<double>& v2) {
    double sum = 0;
    for (unsigned int i = 0; i < v1.size(); i++) {
        sum += std::pow(v1[i] - v2[i], 2);
    }
    return std::sqrt(sum);
}

std::vector<double> inertial_to_body_frame(const std::vector<double>& ego_location, double xi, double yi, double psi) {
    double cos_psi = std::cos(psi);
    double sin_psi = std::sin(psi);
    double dx = xi - ego_location[0];
    double dy = yi - ego_location[1];
    double xx = cos_psi * dx + sin_psi * dy;
    double yy = -sin_psi * dx + cos_psi * dy;
    return {xx, yy};
}

class Controller : public rclcpp::Node {
public:
    Controller(const std::string& name) : Node(name) {

        RCLCPP_INFO(this->get_logger(), "Starting PID controller node");

        this->declare_parameter("control_time_step", 0.01);
        control_time_step = this->get_parameter("control_time_step").as_double(); // 控制时间步长

        this->declare_parameter("lateral_K_PP", 0.2);
        this->declare_parameter("lateral_C_PP", 0.25);
        this->declare_parameter("lateral_I_PP", 0.0);
        this->declare_parameter("lateral_D_PP", 0.0);
        args_lateral_dict["K_PP"] = this->get_parameter("lateral_K_PP").as_double();
        args_lateral_dict["C_PP"] = this->get_parameter("lateral_C_PP").as_double();
        args_lateral_dict["I_PP"] = this->get_parameter("lateral_I_PP").as_double();
        args_lateral_dict["D_PP"] = this->get_parameter("lateral_D_PP").as_double(); // 横向PID参数
        
        this->declare_parameter("longitudinal_KP", 0.75);
        this->declare_parameter("longitudinal_KI", 0.0);
        this->declare_parameter("longitudinal_KD", 0.0);
        args_longitudinal_dict["K_P"] = this->get_parameter("longitudinal_KP").as_double();
        args_longitudinal_dict["K_I"] = this->get_parameter("longitudinal_KI").as_double();
        args_longitudinal_dict["K_D"] = this->get_parameter("longitudinal_KD").as_double(); // 纵向PID参数
        
        // 声明并获取纵向VP参数
        this->declare_parameter("a_y_max", 2.0);
        this->declare_parameter("a_accel_max", 6.0);
        this->declare_parameter("a_decel_max", 6.0);
        this->declare_parameter("delta_s", 0.25);
        a_y_max = this->get_parameter("a_y_max").as_double();
        a_accel_max = this->get_parameter("a_accel_max").as_double();
        a_decel_max = this->get_parameter("a_decel_max").as_double();
        delta_s = this->get_parameter("delta_s").as_double();

        // 变化率限制参数
        steer_limit = 0.1;
        throttle_limit = 0.8;
        brake_limit = 0.8;

        _current_pose = {0.0, 0.0, 0.0};  // 当前位置
        _current_speed = 0.0; // 当前速度

        this->declare_parameter("target_speed", 3.0);
        _target_speed = this->get_parameter("target_speed").as_double(); // 目标速度
        RCLCPP_INFO(this->get_logger(), "target_speed: %f", _target_speed);

        this->declare_parameter("odom_topic", "/testing_only/odom");
        this->declare_parameter("path_topic", "/drd25/path");
        this->declare_parameter("state_indicator_topic", "/drd25/state_indicator");
        this->declare_parameter("brake_topic", "/drd25/brake_command");
        this->declare_parameter("off_track_topic", "/drd25/off_track");
        this->declare_parameter("fitted_path_topic", "/drd25/fitted_path");
        this->declare_parameter("control_topic", "/control_command");
        odom_topic = this->get_parameter("odom_topic").as_string();
        path_topic = this->get_parameter("path_topic").as_string();
        state_indicator_topic = this->get_parameter("state_indicator_topic").as_string();
        brake_topic = this->get_parameter("brake_topic").as_string();
        off_track_topic = this->get_parameter("off_track_topic").as_string();
        fitted_path_topic = this->get_parameter("fitted_path_topic").as_string();
        control_topic = this->get_parameter("control_topic").as_string();

        // 新增恢复模式参数
        this->declare_parameter("recovery_max_steer", 0.35);
        this->declare_parameter("recovery_speed_ratio", 0.5);
        recovery_max_steer = this->get_parameter("recovery_max_steer").as_double();
        recovery_speed_ratio = this->get_parameter("recovery_speed_ratio").as_double();



        parameters_callback = this->add_on_set_parameters_callback(
            std::bind(&Controller::parameters_cb, this, std::placeholders::_1));

        _waypoints_queue = std::deque<std::array<double, 2>>(30); // 路径缓冲队列

        // 创建互斥回调组
        callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options;

        // subscribers
        options.callback_group = callback_group;
        _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 1, std::bind(&Controller::odometry_cb, this, std::placeholders::_1), options);
        _path_subscriber = this->create_subscription<drd25_msgs::msg::Path>(
            path_topic, 1, std::bind(&Controller::path_cb, this, std::placeholders::_1), options);
        
        _state_indicator_subscriber = this->create_subscription<std_msgs::msg::Bool>(
            state_indicator_topic, 1,std::bind(&Controller::state_indicator_cb, this, std::placeholders::_1), options);
        
        _brake_subscriber = this->create_subscription<std_msgs::msg::Bool>(
            brake_topic, 1,std::bind(&Controller::brake_cb, this, std::placeholders::_1), options);
        
        _offtrack_subscriber = this->create_subscription<std_msgs::msg::Bool>(
            off_track_topic, 1,std::bind(&Controller::offtrack_cb, this, std::placeholders::_1), options);
        
        
        // publisher
        _control_cmd_publisher = this->create_publisher<fs_msgs::msg::ControlCommand>(control_topic, 1);

        //发布拟合后的路径
        _fitted_path_publisher = this->create_publisher<drd25_msgs::msg::Path>(fitted_path_topic, 1);

        // 初始化PID控制器
        _vehicle_controller = VehiclePIDController(args_lateral_dict, args_longitudinal_dict);
        _longitudinal_vp = LongitudinalVP(a_y_max, a_accel_max, a_decel_max, delta_s);
    }

private:
    void odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        _current_pose[0] = msg->pose.pose.position.x;
        _current_pose[1] = msg->pose.pose.position.y;
        _current_speed = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2));
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
        _current_pose[2] = yaw;

        //RCLCPP_INFO(this->get_logger(), "Successfully subscribed to odometry message");
    }

    void state_indicator_cb(const std_msgs::msg::Bool::SharedPtr msg){
        vp_flag = msg->data;
    }

    void brake_cb(const std_msgs::msg::Bool::SharedPtr msg){
        brake_flag = msg->data;
    }

    void offtrack_cb(const std_msgs::msg::Bool::SharedPtr msg){
        recover_mode = msg->data;
        if(recover_mode){
            _base_speed = _target_speed;
            _target_speed = std::min(3.0, _base_speed * recovery_speed_ratio);
        }
    }

    void path_cb(drd25_msgs::msg::Path::SharedPtr msg) 
    {
        if (msg->waypoints.size() >= 3) 
        {
            std::vector<double> ego_location = {_current_pose[0], _current_pose[1]};
            std::vector<double> path_final_point = {msg->waypoints[msg->waypoints.size() - 1].x, msg->waypoints[msg->waypoints.size() - 1].y};
            auto front = inertial_to_body_frame(ego_location, path_final_point[0], path_final_point[1], _current_pose[2]);
            if (front[0] >= 0) 
            {
                _waypoints_queue.clear();
                for (auto& waypoint : msg->waypoints) {
                    if (!_waypoints_queue.empty()) {
                        double dx = waypoint.x - _waypoints_queue.back()[0];
                        double dy = waypoint.y - _waypoints_queue.back()[1];
                        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
                            // 跳过重复点
                            continue;
                        }
                    }
                    _waypoints_queue.push_back({waypoint.x, waypoint.y});
                }
            }

        // 取前一半的点
        /*
        if(vp_flag){
            _waypoints_queue.erase(_waypoints_queue.begin() + _waypoints_queue.size() / 2, _waypoints_queue.end());
        }
        else{
            ;
        }
        */

        // 使用LongitudinalVP类进行曲线拟合
        _waypoints_queue = _longitudinal_vp.curve_fitting(_waypoints_queue);
        
        /* 输出拟合后的路径点
        for (auto& waypoint : _waypoints_queue) 
        {
            RCLCPP_INFO(this->get_logger(), "waypoint: %f, %f", waypoint[0], waypoint[1]);
        }
        RCLCPP_INFO(this->get_logger(), "\n");
        */
            
        // 发布拟合后的路径，与代码功能无关
        auto fitted_path = std::make_shared<drd25_msgs::msg::Path>();
        for(auto& waypoint : _waypoints_queue){
            drd25_msgs::msg::Waypoint wp;
            wp.x = waypoint[0];
            wp.y = waypoint[1];
            fitted_path->waypoints.push_back(wp);
        }
        _fitted_path_publisher->publish(*fitted_path);
       

        if (!timer_) {
            timer_ = this->create_wall_timer(std::chrono::duration<double>(control_time_step), std::bind(&Controller::control_timer_cb, this));
        }
        }
    }

    //删除了找最近点的代码，改为在横向控制中直接找点

    void run_step() {

        // 删除了target_pose的定义和使用

        if(vp_flag){

            //_target_speed = 6.0;
            _target_speed = _longitudinal_vp.run_step(_waypoints_queue, _current_speed)[1];
        }
        else{
            ;
        };
        //RCLCPP_INFO(this->get_logger(), "target_speed: %f ??? _current_speed: %f", _target_speed, _current_speed);

        //RCLCPP_INFO(this->get_logger(), "target_speed: %f", _target_speed);

        auto cmd = std::make_shared<fs_msgs::msg::ControlCommand>();

        if(brake_flag){
            cmd->header.stamp = this->now(); // 时间戳
            cmd->throttle = 0.0;
            cmd->brake = 1.0;
            cmd->steering = 0.0;
            _control_cmd_publisher->publish(*cmd);
            return;
        }

        _base_speed = _target_speed;
        double max_steer;
        if(recover_mode){
            _target_speed = std::min(2.5, _base_speed * recovery_speed_ratio);
            max_steer = recovery_max_steer;
        }
        else{
            _target_speed = _base_speed;
            max_steer = 1.0;
        }

        auto control_command = _vehicle_controller.run_step(
            _target_speed * 3.6, _current_speed * 3.6, _current_pose, _waypoints_queue);

        if(recover_mode){
            control_command.first = std::clamp(control_command.first, -max_steer, max_steer);
        }

        // 如果控制命令有效，则发布控制消息
        if (control_command.first && control_command.second) {
            //auto cmd = std::make_shared<fs_msgs::msg::ControlCommand>();
            cmd->header.stamp = this->now(); // 时间戳

            // 油门控制
            cmd->throttle = (control_command.second > 0) ? control_command.second : 0.0;
            double throttle_change = cmd->throttle - last_throttle;
            // 限制油门变化
            if (std::abs(throttle_change) > throttle_limit) {
                cmd->throttle = last_throttle + std::copysign(throttle_limit, throttle_change);
            }
            last_throttle = cmd->throttle;

            // 刹车控制
            cmd->brake = (control_command.second < 0) ? -control_command.second : 0.0;
            double brake_change = cmd->brake - last_brake;
            // 限制刹车变化
            if (std::abs(brake_change) > brake_limit) {
                cmd->brake = last_brake + std::copysign(brake_limit, brake_change);
            }
            last_brake = cmd->brake;

            // 低通滤波
            /*
            double alpha = 1.0;
            if (!std::isfinite(cmd->steering)) { // 或者检查是否已初始化
                cmd->steering = -control_command.first; // 初始值
            }
            cmd->steering = alpha * -control_command.first + (1.0 - alpha) * cmd->steering;
            */

            cmd->steering = -control_command.first;
            double steer_change = cmd->steering - last_steering;
            // 限制转向变化
            if (std::abs(steer_change) > steer_limit) {
                cmd->steering = last_steering + std::copysign(steer_limit, steer_change);
            }
            last_steering = cmd->steering;

            _control_cmd_publisher->publish(*cmd); // 发布控制消息
        }
    }

    rcl_interfaces::msg::SetParametersResult parameters_cb(const std::vector<rclcpp::Parameter> &params) 
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params) {
            if (param.get_name() == "lateral_K_PP") {
                args_lateral_dict["K_PP"] = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated lateral_K_PP to: %f", args_lateral_dict["K_PP"]);
            } else if (param.get_name() == "lateral_C_PP") {
                args_lateral_dict["C_PP"] = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated lateral_C_PP to: %f", args_lateral_dict["C_PP"]);
            } else if (param.get_name() == "lateral_I_PP") {
                args_lateral_dict["I_PP"] = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated lateral_I_PP to: %f", args_lateral_dict["I_PP"]);
            } else if (param.get_name() == "lateral_D_PP") {
                args_lateral_dict["D_PP"] = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated lateral_D_PP to: %f", args_lateral_dict["D_PP"]);
            } else if (param.get_name() == "longitudinal_KP") {
                args_longitudinal_dict["K_P"] = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated longitudinal_KP to: %f", args_longitudinal_dict["K_P"]);
            } else if (param.get_name() == "longitudinal_KI") {
                args_longitudinal_dict["K_I"] = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated longitudinal_KI to: %f", args_longitudinal_dict["K_I"]);
            } else if (param.get_name() == "longitudinal_KD") {
                args_longitudinal_dict["K_D"] = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated longitudinal_KD to: %f", args_longitudinal_dict["K_D"]);
            } else if (param.get_name() == "control_time_step") {
                control_time_step = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated control_time_step to: %f", control_time_step);
            } else if (param.get_name() == "target_speed") {
                _target_speed = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated target_speed to: %f", _target_speed);
            } else if (param.get_name() == "recovery_speed_ratio") {
                recovery_speed_ratio = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated recovery_speed_ratio to: %f", recovery_speed_ratio);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown parameter: %s", param.get_name().c_str());
                result.successful = false;
            }
        }
        return result;
    }

    void control_timer_cb() {
        run_step();
    }

    // 成员变量
    double control_time_step;
    std::map<std::string, double> args_lateral_dict;
    std::map<std::string, double> args_longitudinal_dict;
    std::array<double, 3> _current_pose;
    double _current_speed;
    double _target_speed;
    double _base_speed; // 基础速度

    double recovery_speed_ratio; // 恢复模式速度比例
    double recovery_max_steer; // 恢复模式最大转向角度

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameters_callback;
    std::deque<std::array<double, 2>> _waypoints_queue;

    double a_y_max;
    double a_accel_max;
    double a_decel_max;
    double delta_s;

    bool vp_flag = false;
    bool brake_flag = false;
    bool recover_mode = false;

    // ROS2 相关成员
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;
    rclcpp::Subscription<drd25_msgs::msg::Path>::SharedPtr _path_subscriber;

    //新增2个订阅者，用于接收刹车信号和恢复模式状态
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _brake_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _offtrack_subscriber;

    // 新增一个订阅者，用于判断圈数
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _state_indicator_subscriber;

    rclcpp::Publisher<drd25_msgs::msg::Path>::SharedPtr _fitted_path_publisher;

    rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr _control_cmd_publisher;
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

    double last_steering = 0.0;
    double last_throttle = 0.0;
    double last_brake = 0.0;
    double steer_limit;
    double throttle_limit;
    double brake_limit;

    std::string odom_topic;
    std::string path_topic;
    std::string state_indicator_topic;
    std::string brake_topic;
    std::string off_track_topic;
    std::string fitted_path_topic;
    std::string control_topic;



    VehiclePIDController _vehicle_controller;
    LongitudinalVP _longitudinal_vp;
    

    int w_size = 10; // 假设 w_size 的值为 10
    int closest_wp_index = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>("PID_controller_node_cpp");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
