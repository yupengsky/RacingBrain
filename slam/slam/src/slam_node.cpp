#include <deque> 
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <proj.h>
#include "rclcpp/rclcpp.hpp"   
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
// 消息过滤器（时间同步）
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// 接收感知信息
#include "drd25_msgs/msg/map.hpp"            
#include "drd25_msgs/msg/cone.hpp"
// 接收Ins消息
#include "gnss_ins_msg/msg/gnssins64.hpp"  
// 发布
#include "visualization_msgs/msg/marker_array.hpp" 
#include <nav_msgs/msg/path.hpp>     
#include <nav_msgs/msg/odometry.hpp>    
// 回环检测器
#include "slam/loop_closure_detector.hpp"


// 赛道类型枚举
enum class TrackType {
    AUTOCROSS,
    ACCELERATION,
    SKIDPAD
};

// 一个继承自rclcpp::Node基类，名为SlamProcessor的类
class SlamProcessor : public rclcpp::Node {
public:
    // 创建名为slam_processor的ROS2节点
    SlamProcessor() : 
    Node("slam_processor"), is_origin_set_(false)
    {  
        // 从ROS2参数服务器加载所有参数
        loadParameters();

        // ****************** PROJ坐标转换 ******************
        // EPSG:4326 是经纬度 (WGS84)
        // EPSG:32651 是 UTM Zone 51N (上海地区，在合肥比赛要改为32650)
        P_ = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", sys_.utm_zone_epsg.c_str(), NULL);
        
        if (P_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create PROJ transformation!");
        }
        // ****************** PROJ坐标转换 ******************

        // ****************** 订阅与发布 ******************
        // 高频 GNSS 订阅（只用于实时更新位姿，不参与与感知节点的同步建图）
        // 使用 Best Effort QoS 确保数据实时性
        high_freq_gnss_sub_ = this->create_subscription<gnss_ins_msg::msg::Gnssins64>(
            sys_.gnss_topic, rclcpp::SensorDataQoS(),
            std::bind(&SlamProcessor::fastGnssCallback, this, std::placeholders::_1));

        // 感知和 GNSS 数据的时间同步
        // 使用 message_filters 进行同步订阅
        gnss_sub_.subscribe(this, sys_.gnss_topic);
        perception_sub_.subscribe(this, sys_.lidar_topic);
        // 使用 ApproximateTime 进行两个节点消息的时间同步
        typedef message_filters::sync_policies::ApproximateTime<gnss_ins_msg::msg::Gnssins64, drd25_msgs::msg::Map> SyncPolicy;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(800), gnss_sub_, perception_sub_);
        // 注册同步回调函数
        sync_->registerCallback(std::bind(&SlamProcessor::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Time Synchronization started.");

        // 其他相关话题的发布
        global_map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_map", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);       //广播坐标变换
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vehicle_path", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("vehicle_odom", 10); 
        path_msg_.header.frame_id = sys_.map_frame;
        // ****************** 订阅与发布 ******************

        RCLCPP_INFO(this->get_logger(), "Slam Started. Mode: [%s], EPSG: [%s]", 
            track_type_str_.c_str(), sys_.utm_zone_epsg.c_str());
        
    }

    ~SlamProcessor() {
        if (P_) proj_destroy(P_);
        //析构函数，如果c掉程序或者程序意外退出，会自动执行这段代码，销毁掉P_，以释放其占有的内存
    }

private:
    struct VehiclePoseEstimate {
        double tx = 0.0;
        double ty = 0.0;
        double tz = 0.0;
        double vel_e = 0.0;
        double vel_n = 0.0;
        Eigen::Matrix3d R_vehicle = Eigen::Matrix3d::Identity();
        Eigen::Matrix4d T_veh_to_map = Eigen::Matrix4d::Identity();
    };

    // 定义系统参数（基本不会变的参数）
    struct SystemParams {
        std::string gnss_topic;
        std::string lidar_topic;
        std::string map_frame;
        std::string base_frame;
        std::string utm_zone_epsg;
    }sys_;

    // 定义核心算法参数
    struct MappingParams {
        double kf_q_base;   // 卡尔曼过程噪声，即Q值，用于动态更新锥桶坐标
        double kf_p_init;   // 初始地图不确定度，用于新锥桶初始化
        double mahalanobis_thresh;  // 马氏距离阈值，用于锥桶匹配
        double max_match_dist;  // 欧式距离阈值，用于触发回环后将当前地图与保存好的第一帧地图匹配
        double max_lidar_range;   // 最大感知距离，用于基于距离计算置信度
        double lidar_blind_range; // 由于雷达线束高度打不到地面，引起扇形的雷达视野死角的半径
        double fov_angle;   // 雷达视场角，用于计算锥桶是否在视野范围内
        double sigma_long;  // 纵向噪声
        double sigma_lat;   // 横向噪声
        double l_hit;       // 匹配成功一次的基础增益
        double l_miss;      // 丢失视野一次的基础惩罚
        double l_min;       // 斩杀线
        double l_max;       // 生命上限
        double l_stable;    // 锥桶被判定为稳定的得分门槛
        double distance_factor;  // 协方差矩阵随距离而衰减的分母，用于噪声建模
        double speed_factor;   // 探索圈结束后由于速度加快带来的观测不确定度惩罚
        double gyro_factor;    // 探索圈结束后由于弯道转向带来的观测不确定度惩罚
        double lat_factor;     // 过弯时的横向误差惩罚
        double half_tube_width; // ROI管道的一半宽度
        double max_tube_length; // ROI管道长度的最大值
        double max_straight_angular;   // 直线行驶时可能的最大角速度
    }params_;

    TrackType current_track_type_;
    std::string track_type_str_;

    void loadParameters() {
        // 加载赛道类型，默认为acceleration
        // 如果launch文件里有覆盖，会自动替换
        this->declare_parameter("track_type", "acceleration");
        track_type_str_ = this->get_parameter("track_type").as_string();
        
        if (track_type_str_ == "autocross") current_track_type_ = TrackType::AUTOCROSS;
        else if (track_type_str_ == "skidpad") current_track_type_ = TrackType::SKIDPAD;
        else current_track_type_ = TrackType::ACCELERATION;

        // 系统参数加载
        this->declare_parameter("system.gnss_topic", "/gongji_gnss_ins_64");
        this->declare_parameter("system.lidar_topic", "/perception/fusion/map");
        this->declare_parameter("system.map_frame", "map");
        this->declare_parameter("system.base_frame", "base_link");
        this->declare_parameter("system.utm_zone_epsg", "EPSG:32651");

        sys_.gnss_topic = this->get_parameter("system.gnss_topic").as_string();
        sys_.lidar_topic = this->get_parameter("system.lidar_topic").as_string();
        sys_.map_frame = this->get_parameter("system.map_frame").as_string();
        sys_.base_frame = this->get_parameter("system.base_frame").as_string();
        sys_.utm_zone_epsg = this->get_parameter("system.utm_zone_epsg").as_string();

        // 外参矩阵加载
        std::vector<double> identity_matrix(16, 0.0);
        identity_matrix[0]=1; identity_matrix[5]=1; identity_matrix[10]=1; identity_matrix[15]=1;
        this->declare_parameter("extrinsics.t_l2v", identity_matrix);
        std::vector<double> ext_vec = this->get_parameter("extrinsics.t_l2v").as_double_array();
        
        if(ext_vec.size() == 16) {
            T_l2v_ = Eigen::Matrix4d::Identity();
            T_l2v_ << ext_vec[0],  ext_vec[1],  ext_vec[2],  ext_vec[3],
                      ext_vec[4],  ext_vec[5],  ext_vec[6],  ext_vec[7],
                      ext_vec[8],  ext_vec[9],  ext_vec[10], ext_vec[11],
                      ext_vec[12], ext_vec[13], ext_vec[14], ext_vec[15];
        } else {
            RCLCPP_ERROR(this->get_logger(), "Extrinsics size error! Use default.");
            T_l2v_ = Eigen::Matrix4d::Identity();
        }

        // 算法参数加载
        this->declare_parameter("mapping.kf_q_base", 0.05);
        this->declare_parameter("mapping.kf_p_init", 0.5);
        this->declare_parameter("mapping.mahalanobis_thresh", 1.5);
        this->declare_parameter("mapping.max_match_dist", 2.0);
        this->declare_parameter("mapping.max_lidar_range", 30.0);
        this->declare_parameter("mapping.lidar_blind_range", 1.0);
        this->declare_parameter("mapping.fov_angle", 120.0);
        this->declare_parameter("mapping.l_hit", 0.85);
        this->declare_parameter("mapping.l_miss", -0.4);
        this->declare_parameter("mapping.l_min", -2.0);
        this->declare_parameter("mapping.l_max", 3.5);
        this->declare_parameter("mapping.l_stable", 1.1);
        this->declare_parameter("mapping.sigma_long", 0.4);
        this->declare_parameter("mapping.sigma_lat", 0.2);
        this->declare_parameter("mapping.distance_factor", 15.0);
        this->declare_parameter("mapping.speed_factor", 0.3);
        this->declare_parameter("mapping.gyro_factor", 2.0);
        this->declare_parameter("mapping.lat_factor", 0.5);
        this->declare_parameter("mapping.half_tube_width", 2.0);
        this->declare_parameter("mapping.max_tube_length", 10.0);
        this->declare_parameter("mapping.max_straight_angular", 0.1);

        params_.kf_q_base = this->get_parameter("mapping.kf_q_base").as_double();
        params_.kf_p_init = this->get_parameter("mapping.kf_p_init").as_double();
        params_.mahalanobis_thresh = this->get_parameter("mapping.mahalanobis_thresh").as_double();
        params_.max_match_dist = this->get_parameter("mapping.max_match_dist").as_double();
        params_.max_lidar_range = this->get_parameter("mapping.max_lidar_range").as_double();
        params_.lidar_blind_range = this->get_parameter("mapping.lidar_blind_range").as_double();
        params_.fov_angle = this->get_parameter("mapping.fov_angle").as_double();
        params_.distance_factor = this->get_parameter("mapping.distance_factor").as_double();
        params_.l_hit = this->get_parameter("mapping.l_hit").as_double();
        params_.l_miss = this->get_parameter("mapping.l_miss").as_double();
        params_.l_min = this->get_parameter("mapping.l_min").as_double();
        params_.l_max = this->get_parameter("mapping.l_max").as_double();
        params_.sigma_long = this->get_parameter("mapping.sigma_long").as_double();
        params_.sigma_lat = this->get_parameter("mapping.sigma_lat").as_double();
        params_.l_stable = this->get_parameter("mapping.l_stable").as_double();
        params_.speed_factor = this->get_parameter("mapping.speed_factor").as_double();
        params_.gyro_factor = this->get_parameter("mapping.gyro_factor").as_double();
        params_.lat_factor = this->get_parameter("mapping.lat_factor").as_double();
        params_.half_tube_width = this->get_parameter("mapping.half_tube_width").as_double();
        params_.max_tube_length = this->get_parameter("mapping.max_tube_length").as_double();
        params_.max_straight_angular = this->get_parameter("mapping.max_straight_angular").as_double();

        // 回环检测器参数加载
        this->declare_parameter("loop_closure.distance_threshold", 1.5);
        this->declare_parameter("loop_closure.approach_angle_threshold", 35.0);
        this->declare_parameter("loop_closure.min_consecutive_detections", 2);
        this->declare_parameter("loop_closure.max_consecutive_detections", 5);
        this->declare_parameter("loop_closure.min_time_between_detections", 3.0);
        this->declare_parameter("loop_closure.history_buffer_size", 10);
        this->declare_parameter("loop_closure.min_approach_speed", 0.8);
        this->declare_parameter("loop_closure.min_lap_time", 20.0);

        loop_detector_.configure(
            this->get_parameter("loop_closure.distance_threshold").as_double(),
            this->get_parameter("loop_closure.approach_angle_threshold").as_double(),
            this->get_parameter("loop_closure.min_consecutive_detections").as_int(),
            this->get_parameter("loop_closure.max_consecutive_detections").as_int(),
            this->get_parameter("loop_closure.min_time_between_detections").as_double(),
            this->get_parameter("loop_closure.history_buffer_size").as_int(),
            this->get_parameter("loop_closure.min_approach_speed").as_double(),
            this->get_parameter("loop_closure.min_lap_time").as_double()
        );
    }

    // 负责高频轨迹 TF, Odom, Path 发布 (100Hz)
    void fastGnssCallback(const gnss_ins_msg::msg::Gnssins64::SharedPtr gnss_msg) {
        if (!P_) return;

        double utm_e = 0.0;
        double utm_n = 0.0;
        projectToUtm(*gnss_msg, utm_e, utm_n);
        if (!ensureOriginInitialized(utm_e, utm_n)) return;

        VehiclePoseEstimate pose = buildVehiclePoseEstimate(*gnss_msg, utm_e, utm_n);

        // T_veh_to_map_ 实时更新的最新的车辆位姿
        // T_veh_to_map_ 是独属于高频发布轨迹回调函数的，不参与建图
        T_veh_to_map_ = pose.T_veh_to_map;

        // 立即发布 TF 和 Odom
        publishTfAndOdom(gnss_msg->header, pose.tx, pose.ty, pose.tz, pose.R_vehicle, pose.vel_e, pose.vel_n);
    }

    // 同步回调函数，确保感知与位姿信息处于同一时间戳下，负责地图更新
    void syncCallback(const gnss_ins_msg::msg::Gnssins64::ConstSharedPtr& gnss_msg,const drd25_msgs::msg::Map::ConstSharedPtr& map_msg) 
    {
        if (!P_ || !is_origin_set_) return;

        double utm_e = 0.0;
        double utm_n = 0.0;
        projectToUtm(*gnss_msg, utm_e, utm_n);
        maybeInitializeLoopDetector();
        VehiclePoseEstimate pose = buildVehiclePoseEstimate(*gnss_msg, utm_e, utm_n);

        // 获取实时对地速度和z轴角速度（并未和ROS坐标系对齐）
        current_speed_ = caculateSpeed(gnss_msg);
        current_gyro_z_ = caculateYawRate(gnss_msg);
        //  *********************** 位姿提取和计算 ***********************

        //  *********************** 建图 ***********************
        // 保证在当前代码块内独占访问 cone_map_，防止多线程数据竞争和崩溃
        std::lock_guard<std::mutex> lock(map_mutex_);

        // 每一帧建图开始前，先把所有锥桶标记为未匹配
        resetFrameMatches();

        // 根据不同赛道，调用不同的建图策略
        switch (current_track_type_) {
            case TrackType::ACCELERATION:
                processAccelerationTrack(*map_msg, pose.T_veh_to_map);
                break;
            case TrackType::AUTOCROSS:
                processAutocrossTrack(*map_msg, pose.T_veh_to_map, gnss_msg->header.stamp, pose.tx, pose.ty, pose.tz);
                break;
            case TrackType::SKIDPAD:
                processSkidpadTrack(*map_msg, pose.T_veh_to_map, gnss_msg->header.stamp, pose.tx, pose.ty, pose.tz);
                break;
        }
    // *********************** 建图 **********************
    
        // 发布地图
        publishGlobalMap(gnss_msg->header.stamp);
    }

    // 直线赛道建图策略
    void processAccelerationTrack(const drd25_msgs::msg::Map& map_msg, 
                                  const Eigen::Matrix4d& historical_T_veh_to_map) 
    {
        processTrackObservations(map_msg, historical_T_veh_to_map);
        maintainConeMap(historical_T_veh_to_map, true);
    }

    // 循迹赛道建图策略
    void processAutocrossTrack(const drd25_msgs::msg::Map& map_msg, 
                               const Eigen::Matrix4d& historical_T_veh_to_map, 
                               const builtin_interfaces::msg::Time& stamp, 
                               double tx, double ty, double tz)
    {
        processTrackObservations(map_msg, historical_T_veh_to_map);

        // 只有在未锁图的情况下才进行锥桶删减
        if (!map_locked_){
            maintainConeMap(historical_T_veh_to_map, false);
        }

        // ************************* 回环检测 ************************
        handleAutocrossLoopClosure(stamp, tx, ty, tz);
    }
    
    // 八字赛道建图策略
    void processSkidpadTrack(const drd25_msgs::msg::Map& /*map_msg*/, 
                               const Eigen::Matrix4d& /*historical_T_veh_to_map*/, 
                               const builtin_interfaces::msg::Time& /*stamp*/, 
                               double /*tx*/, double /*ty*/, double /*tz*/)
    {
        // TODO:待完善八字的建图策略
    }
    
    // ************************************ 工具函数 ************************************
    // 解析锥桶颜色，输出锥桶对应的r,g,b属性
    void parseConeColor(int color_type, float& r, float& g, float& b) const {
        switch(color_type) {
            case drd25_msgs::msg::Cone::BLUE: // 0
                r = 0.0f; g = 0.0f; b = 1.0f; 
                break;
            case drd25_msgs::msg::Cone::RED: // 1
                r = 1.0f; g = 0.0f; b = 0.0f; 
                break;
            case drd25_msgs::msg::Cone::YELLOW_BIG: // 2 
            case drd25_msgs::msg::Cone::YELLOW_SMALL: // 3
                r = 1.0f; g = 1.0f; b = 0.0f; 
                break;
            case drd25_msgs::msg::Cone::UNKNOWN: // 4
            default:
                r = 0.5f; g = 0.5f; b = 0.5f; // 灰色代表未知
                break;
        }
    }

    // 获取车辆相对地面的绝对速度
    double caculateSpeed(const gnss_ins_msg::msg::Gnssins64::ConstSharedPtr& msg){
        return std::sqrt(msg->vel_e * msg->vel_e + msg->vel_n * msg->vel_n);
    }

    // 获取车辆绕z轴的角速度(单位为rad/s且有正负之分）
    // 大于0为逆时针，向左转；小于0为顺时针，向右转
    double caculateYawRate(const gnss_ins_msg::msg::Gnssins64::ConstSharedPtr& msg){
        return msg->imu_gyro_z * M_PI / 180.0;
    }

    void projectToUtm(const gnss_ins_msg::msg::Gnssins64& msg, double& utm_e, double& utm_n) const {
        PJ_COORD input_coord = proj_coord(msg.latitude, msg.longitude, 0, 0);
        PJ_COORD output_coord = proj_trans(P_, PJ_FWD, input_coord);
        utm_e = output_coord.xy.x;
        utm_n = output_coord.xy.y;
    }

    bool ensureOriginInitialized(double utm_e, double utm_n) {
        if (is_origin_set_) return true;

        origin_e_ = utm_e;
        origin_n_ = utm_n;
        is_origin_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Origin set at E:%.2f, N:%.2f", origin_e_, origin_n_);
        return false;
    }

    void maybeInitializeLoopDetector() {
        if (is_lcd_initialized_) return;

        loop_detector_.setOrigin(Eigen::Vector3d(0.0, 0.0, 0.0));
        is_lcd_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Loop Closure Detector Initialized");
    }

    VehiclePoseEstimate buildVehiclePoseEstimate(const gnss_ins_msg::msg::Gnssins64& msg, double utm_e, double utm_n) const {
        VehiclePoseEstimate pose;
        pose.tx = utm_e - origin_e_;
        pose.ty = utm_n - origin_n_;
        pose.tz = 0.0;
        pose.vel_e = msg.vel_e;
        pose.vel_n = msg.vel_n;

        // 旋转部分 (ZYX 顺序)
        // imu坐标系是x-右， y-前；以北向(y轴)正方向为航向角0度，逆时针为正，x轴指向东向
        // Eigen/ROS是x-前， y-左；以东向(x轴)正方向为航向角0度，逆时针为正
        // 二者虽然在坐标系的定义和航向角的定义都不同，但是两个不同却成就了相同的坐标系(x都指东,y都指北)
        Eigen::AngleAxisd roll(msg.roll * M_PI/180.0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(msg.pitch * M_PI/180.0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(msg.yaw * M_PI/180.0, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d R_imu = (yaw * pitch * roll).toRotationMatrix();

        // 现在得到的旋转矩阵R_imu是imu坐标系的，我们最后还要让其变成x-前， y-左的ROS坐标系
        // imu——>vehicle的修正矩阵,x向右转换成x向前
        Eigen::Matrix3d R_fix;
        R_fix << 0, -1, 0,
                 1, 0, 0,
                 0, 0, 1;
        pose.R_vehicle = R_imu * R_fix;

        // historical_T_veh_to_map 是与感知消息时间戳对齐的历史位姿
        pose.T_veh_to_map.block<3,3>(0,0) = pose.R_vehicle;
        pose.T_veh_to_map.block<3,1>(0,3) = Eigen::Vector3d(pose.tx, pose.ty, pose.tz);
        return pose;
    }

    void resetFrameMatches() {
        for (auto &cone : cone_map_) {
            cone.matched_this_frame = false;
        }
    }

    void processTrackObservations(const drd25_msgs::msg::Map& map_msg, const Eigen::Matrix4d& historical_T_veh_to_map) {
        const Eigen::Matrix4d T_lidar_to_map = historical_T_veh_to_map * T_l2v_;
        const Eigen::Matrix2d R_current = historical_T_veh_to_map.block<2, 2>(0, 0);

        for (const auto &cone_obs : map_msg.track) {
            if (cone_obs.color == drd25_msgs::msg::Cone::UNKNOWN) {
                continue;
            }

            Eigen::Vector4d p_lidar(cone_obs.x, cone_obs.y, 0.0, 1.0);
            Eigen::Vector4d p_global = T_lidar_to_map * p_lidar;

            float r = 0.0f;
            float g = 0.0f;
            float b = 0.0f;
            parseConeColor(cone_obs.color, r, g, b);

            double dist = std::sqrt(cone_obs.x * cone_obs.x + cone_obs.y * cone_obs.y);
            updateMap(p_global.x(), p_global.y(), cone_obs.x, cone_obs.y, r, g, b, cone_obs.color, dist, R_current);
        }
    }

    void maintainConeMap(const Eigen::Matrix4d& historical_T_veh_to_map, bool remove_on_equal_min) {
        for (auto it = cone_map_.begin(); it != cone_map_.end(); ) {
            bool in_view = isInFieldOfView(it->pos, historical_T_veh_to_map);
            if (in_view && !it->matched_this_frame) {
                it->existence_score += params_.l_miss;
            }

            bool should_remove = remove_on_equal_min
                ? (it->existence_score <= params_.l_min)
                : (it->existence_score < params_.l_min);

            if (should_remove) {
                it = cone_map_.erase(it);
            } else {
                it->is_stable = (it->existence_score > params_.l_stable);
                ++it;
            }
        }
    }

    void handleAutocrossLoopClosure(const builtin_interfaces::msg::Time& stamp, double tx, double ty, double tz) {
        double current_time = stamp.sec + stamp.nanosec * 1e-9;
        bool loop_closed = loop_detector_.detectLoopClosure(current_time, Eigen::Vector3d(tx, ty, tz), !map_locked_);
        if (!loop_closed) return;

        lap_count_ ++;

        if (!map_locked_) {
            RCLCPP_INFO(this->get_logger(), "=== LAP 1 COMPLETED! Map Locked! ===");

            int all_cones = cone_map_.size();
            cone_map_.erase(
                std::remove_if(cone_map_.begin(), cone_map_.end(),
                            [](const GlobalCone& c) {return !c.is_stable;}),
                cone_map_.end()
            );
            int left_cones = cone_map_.size();
            RCLCPP_INFO(this->get_logger(), "Purged %d unstable cones. Remaning %ld cones.",
                        all_cones - left_cones, cone_map_.size());
            map_locked_ = true;
        } else {
            RCLCPP_INFO(this->get_logger(), "LAP %d COMPLETE. Let's fuuuuuuuucking push!", lap_count_ );
        }

        loop_detector_.resetLoopStatus();
    }

    // 发布Tf, Odom, Path的辅助函数
    void publishTfAndOdom(const std_msgs::msg::Header& header, double tx, double ty, double tz, 
                          const Eigen::Matrix3d& R_vehicle, double vel_e, double vel_n) {
        Eigen::Quaterniond q(R_vehicle);

        // TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = header.stamp; // 使用原始消息的时间戳
        t.header.frame_id = sys_.map_frame;
        t.child_frame_id = sys_.base_frame;
        t.transform.translation.x = tx; 
        t.transform.translation.y = ty; 
        t.transform.translation.z = tz;
        t.transform.rotation.x = q.x(); 
        t.transform.rotation.y = q.y(); 
        t.transform.rotation.z = q.z(); 
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // Odom
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = t.header;
        odom_msg.child_frame_id = sys_.base_frame;
        odom_msg.pose.pose.position.x = tx; 
        odom_msg.pose.pose.position.y = ty; 
        odom_msg.pose.pose.position.z = tz;
        odom_msg.pose.pose.orientation  = t.transform.rotation;

        // 将地图系速度(东北) 转换为 车身系速度(前左)
        // 公式: V_vehicle = R_vehicle_transpose * V_map
        // V_map = R_map2vehicle * V_vehicle
        // V_vehicle = (R_map2vehicle).inverse * V_map
        // transpose这里是转置的意思，由于旋转矩阵的正交性，其逆矩阵等于转置矩阵
        double v_east  = vel_e;
        double v_north = vel_n;
        Eigen::Vector3d v_map(v_east, v_north, 0.0);
        Eigen::Vector3d v_vehicle = R_vehicle.transpose() * v_map;
        odom_msg.twist.twist.linear.x = v_vehicle.x(); // 车身前向速度
        odom_msg.twist.twist.linear.y = v_vehicle.y(); // 车身侧向速度
        odom_msg.twist.twist.linear.z = 0.0;
        odom_pub_->publish(odom_msg);

        // Path
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header = t.header;
        current_pose.pose.position.x = tx; current_pose.pose.position.y = ty; current_pose.pose.position.z = tz;
        current_pose.pose.orientation = t.transform.rotation;
        path_msg_.poses.push_back(current_pose);
        // 如果发的是高频位姿，那么path的轨迹会很多很多，这里先不设上限
        // if (path_msg_.poses.size() > 1000) path_msg_.poses.erase(path_msg_.poses.begin());
        path_pub_->publish(path_msg_);
    }

    // 判断锥桶是否出现在合理的视野范围内
    bool isInFieldOfView(const Eigen::Vector2d& global_pos, const Eigen::Matrix4d& current_pose) 
    {
        // p_global 锥桶在世界坐标系下的位置
        // current_pose 车辆在世界坐标系下的位姿
        // p_local 锥桶相对于车辆的位置
        Eigen::Vector4d p_global(global_pos.x(), global_pos.y(), 0.0, 1.0);
        Eigen::Vector4d p_local = current_pose.inverse() * p_global; // 使用传入的同步位姿

        double x = p_local.x();     // 车头方向 (ROS坐标系)
        double y = p_local.y();     // 车左方向
       
        double blind_range = params_.lidar_blind_range;       // 雷达视野死区
        double max_range = params_.max_lidar_range;          // 最大感知距离 (米)
        double half_fov = (params_.fov_angle / 2.0 )* M_PI / 180.0;     // 单侧视场角

        double dist = std::sqrt(x*x + y*y);
        double angle = std::atan2(std::abs(y), x);       // 计算与车头轴线的夹角

        // 由于雷达的安装有一定高度，所以当线束打在地面时，很可能比前几排的锥桶还高，导致雷达并没有前几排锥桶的视野，形成盲区
        // 雷达真正的视野范围应该为以雷达为中心，最大视野为半径，FOV为弧度的大扇形减去视野盲区距离为半径，FOV为弧度的小扇形
        return (x > 0 && dist > blind_range && dist < max_range && angle < half_fov);
    }

    // 基于阿克曼的管道ROI
    bool isInAckermannTube(double x, double y){
        double half_tube_width_ = params_.half_tube_width;
        double max_tube_length_ = params_.max_tube_length;
        // 如果锥桶位于ROI的最大长度外，或者在车的身后，直接剔除
        // 注意：这里采用ROS定义坐标系，即x前，y左
        if (x > max_tube_length_ || x < 0.0){
            return false;
        }
        // 如果角速度小于一定值，接近直线行驶，退化为矩形ROI
        // 分布在ROI宽度之外的锥桶都会被过滤掉
        if (std::abs(current_gyro_z_) < params_.max_straight_angular){
            return std::abs(y) < half_tube_width_;
        }
        double v = current_speed_;
        // R大于0为向左转；R小于0为向右转
        double R = v / current_gyro_z_;
        double dist_to_center_of_circle = std::sqrt(x * x + (y-R) * (y-R));
        double letaral_deviation = std::abs(dist_to_center_of_circle - std::abs(R));
        return letaral_deviation < half_tube_width_;
    }

    // 锥桶关联和位置更新的核心逻辑
    void updateMap(double gx, double gy, double cone_x, double cone_y, float r, float g, float b, int type, double dist_to_sensor, const Eigen::Matrix2d& vehicle_R) {

        // 锥桶二维全局坐标
        Eigen::Vector2d z(gx, gy); 

        // ************* 基于距离的置信度判定 ************ 
        // 一次匹配成功的增益（dynamic_l_hit） = 置信度 * l_hit
        double max_sensing_range = params_.max_lidar_range;
        float confidence = 1.0f - static_cast<float>(dist_to_sensor / max_sensing_range) * 0.6f;
        confidence = std::max(0.4f, confidence); // 置信度保底 0.4
        double dynamic_l_hit = params_.l_hit * confidence; 
        // ************* 基于距离的置信度判定 ************ 

        // *************** 各向异性噪声建模 ***************
        // 定义车身坐标系下的噪声标准差
        double sigma_long = params_.sigma_long; // 纵向（车头方向）噪声
        // 横向（左右方向）噪声基于角速度进行修正，因为过弯时横向漂移较大，可适当放宽观测噪声
        double sigma_lat  = params_.sigma_lat + params_.lat_factor * std::abs(current_gyro_z_); 
        // 构建车身系协方差矩阵 R_body (对角矩阵)
        // R_body​=[σ_long2  ​0
        //         ​0   σ_lat2​​]
        Eigen::Matrix2d R_body;
        R_body << sigma_long * sigma_long, 0.0, 0.0, sigma_lat * sigma_lat;
        // 如果是探索圈，速度较慢，观测误差只会随观测距离的增大而膨胀
        double ratio = dist_to_sensor / params_.distance_factor;
        double overall_penalty = 1.0 + (ratio * ratio);
        // 如果已锁图，车辆进入冲刺圈，与探索圈相比，观测的准确度就会大大降低
        // 给冲刺圈的观测都加上基于速度和角速度的噪声，降低其在滤波中的权重
        if (map_locked_){
            double speed_penalty = params_.speed_factor * current_speed_;
            double yaw_rate_penalty = params_.speed_factor * std::abs(current_gyro_z_);
            // 动态惩罚系数 = 1 + 速度惩罚系数 * 速度 + 角速度惩罚系数 * 角速度
            double dynamic_factor = 1.0 + speed_penalty + yaw_rate_penalty;
            overall_penalty *= dynamic_factor;
        }
        R_body *= overall_penalty;
        // 利用旋转矩阵将协方差变换到地图坐标系
        // 公式：Cov_map = R * Cov_body * R^T
        Eigen::Matrix2d R = vehicle_R * R_body * vehicle_R.transpose();
        // *************** 各向异性噪声建模 ***************

        // *************** 马氏距离锥桶匹配 ***************
        // 通常马氏距离取3.0时就可以覆盖约98.9%的真实测量值
        double min_mahalanobis = params_.mahalanobis_thresh; 
        GlobalCone* best_match = nullptr;

        //寻找最佳匹配
        for (auto &map_cone : cone_map_) {
            // 如果颜色不匹配，直接不进行配对
            // 因为前端传不进来未知类型
            if (map_cone.type != type)
                continue;

            // 计算马氏距离，P为地图存在的点的不确定度；R为这一帧观测的不确定度
            // 计算协方差矩阵 S = P (地图不确定度) + R (观测不确定度)，即S为两者不确定度的叠加
            // 距离公式：sqrt( (z-x)^T * (P+R)^-1 * (z-x) )
            // 如果不确定度(S)较大，可以容许delta较大；如果不确定度很小，delta很大的话，那么算出来的马氏距离会很大
            // 因为马氏距离的阈值放的很高，为了避免一些离群点加入匹配导致锥桶位置不断抽动的现象，如果新入的点和原锥桶欧式距离相差过大，就直接取消数据关联
            Eigen::Vector2d delta = z - map_cone.pos;
            if (delta.norm() > params_.max_match_dist)
                continue;

            Eigen::Matrix2d S = map_cone.P + R; 
            double m_dist = std::sqrt(delta.transpose() * S.inverse() * delta);

            if (m_dist < min_mahalanobis) {
                min_mahalanobis = m_dist;
                best_match = &map_cone;
            }
        }
        // *************** 马氏距离锥桶匹配 ***************

        // *************** 执行卡尔曼更新 ***************
        // 随着观测次数无限增加，根据卡尔曼公式，P 会趋近于 0
        // 加入过程噪声Q，防止滤波器因为不确定度过小而拒绝加入新观测
        if (best_match) {
            Eigen::Matrix2d Q_noise = Eigen::Matrix2d::Identity() * params_.kf_q_base; 
            best_match->P += Q_noise;

            // 计算卡尔曼增益 K
            Eigen::Matrix2d S = best_match->P + R;
            Eigen::Matrix2d K = best_match->P * S.inverse(); 
            // 坐标更新
            best_match->pos = best_match->pos + K * (z - best_match->pos);
            best_match->P = (Eigen::Matrix2d::Identity() - K) * best_match->P;
            // 标记看到这一帧物体
            best_match->matched_this_frame = true;
            // 成功匹配的奖励机制
            best_match->existence_score += dynamic_l_hit;
            best_match->existence_score = std::min(best_match->existence_score, params_.l_max);
            // 如果存在概率大于稳定阈值，is_stable = True
            best_match->is_stable = (best_match->existence_score > params_.l_stable);
        }
        // *************** 执行卡尔曼更新 ***************

        // **************** 新锥桶初始化 ****************
        else {
            // 如果已经锁图，不添加新锥桶
            if (map_locked_) return;

            // 如果在阿克曼ROI之外，不添加
            if (!isInAckermannTube(cone_x, cone_y)){
                return;
            }

            // 如果新传入的锥桶没有找到最佳匹配，则将其视为新锥桶
            // 新锥桶的属性：id(编号) pos(二维坐标) P(不确定度) rgb(颜色) type(类型) is_stable(是否为稳定锥桶)
            GlobalCone new_cone;
            new_cone.id = next_cone_id_++;      
            new_cone.pos = z;
            new_cone.P = Eigen::Matrix2d::Identity() * params_.kf_p_init;   //初始不确定度
            new_cone.r = r; new_cone.g = g; new_cone.b = b;
            new_cone.type = type;
            new_cone.is_stable = false;
            new_cone.matched_this_frame = true; // 标记为已处理
            // 初始的存在概率同样基于置信度
            new_cone.existence_score = dynamic_l_hit;

            // 添加新的锥桶
            cone_map_.push_back(new_cone);
            //记录新创建的锥桶候选点
            RCLCPP_INFO(this->get_logger(), "New Cone [ID:%d] at (%.2f, %.2f)", 
                new_cone.id, z.x(), z.y());
        }
        // **************** 新锥桶初始化 ****************
    }

    // 锥桶地图的发布
    void publishGlobalMap(const builtin_interfaces::msg::Time& stamp) {
        visualization_msgs::msg::MarkerArray map_msg;

        //删除上一帧的图像，完成更新
        visualization_msgs::msg::Marker delete_all;
        delete_all.header.frame_id = sys_.map_frame;
        delete_all.header.stamp = stamp; // 使用传入的时间戳
        // 在每一帧更新锥桶地图的时候，先将原来的全部清理，再重新添入
        delete_all.action = 3; // 3 代表 DELETEALL
        map_msg.markers.push_back(delete_all);

        //统计变量，用于日志输出
        int published_count = 0;

        for (const auto &cone : cone_map_) {
            // 只发布稳定锥桶
            if (!cone.is_stable)
                continue;
            // ************** 视觉渲染 **************
            // 锥桶模型导入了爱丁堡大学的开源仿真器里的.dae文件
            visualization_msgs::msg::Marker m;
            m.header.frame_id = sys_.map_frame;
            m.header.stamp = stamp;
            m.id = cone.id; // 仓库里的唯一ID
            m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            m.action = visualization_msgs::msg::Marker::ADD;

            // 关闭模型自带材质，允许使用真实的rgb上色
            m.mesh_use_embedded_materials = false;

            m.pose.position.x = cone.pos.x();
            m.pose.position.y = cone.pos.y();
            m.pose.position.z = 0.0;
            // 不对模型进行旋转
            m.pose.orientation.x = 0.0;
            m.pose.orientation.y = 0.0;
            m.pose.orientation.z = 0.0;
            m.pose.orientation.w = 1.0;

            // 根据类型设置尺寸
            // 直接使用模型的尺寸，不需要进行缩放
            if (cone.type == drd25_msgs::msg::Cone::YELLOW_BIG) {
                m.mesh_resource = "package://slam/meshes/cone_big.dae";
                m.scale.x = 1.0; 
                m.scale.y = 1.0; 
                m.scale.z = 1.0; 
            } 
            else {
                m.mesh_resource = "package://slam/meshes/cone.dae";
                m.scale.x = 1.0; 
                m.scale.y = 1.0; 
                m.scale.z = 1.0;
            }
            m.color.r = cone.r; m.color.g = cone.g; m.color.b = cone.b; m.color.a = 1.0;
            map_msg.markers.push_back(m);
            published_count++;
        }
        // ************** 视觉渲染 **************
    
        global_map_pub_->publish(map_msg);
    
        // 打印当前跟踪总量和已确认稳定的数量
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Map Status: Total Tracked: %ld | Confirmed Stable: %d", cone_map_.size(), published_count);
    }
    // ************************************ 工具函数 ************************************

    // 成员变量声明
    PJ *P_;
    bool is_origin_set_;
    bool is_lcd_initialized_ = false;
    double origin_e_, origin_n_;
    double current_speed_ = 0.0;   // 车辆实时对地速度（合成）
    double current_gyro_z_ = 0.0;    // 车辆实时绕x轴角速度
    long long next_cone_id_ = 0; // 全局 ID 计数器
    int lap_count_ = 0;     // 记录车辆行驶了多少圈
    std::atomic<bool> map_locked_{false};       //地图锁

    Eigen::Matrix4d T_veh_to_map_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_l2v_ = Eigen::Matrix4d::Identity();           //雷达外参
    std::vector<GlobalCone> cone_map_; // 锥桶仓库
    std::mutex map_mutex_; // 保护cone_map_的互斥锁
    EnhancedLoopClosureDetector loop_detector_;

    // ROS接口
    // 订阅
    rclcpp::Subscription<gnss_ins_msg::msg::Gnssins64>::SharedPtr high_freq_gnss_sub_;
    message_filters::Subscriber<gnss_ins_msg::msg::Gnssins64> gnss_sub_;
    message_filters::Subscriber<drd25_msgs::msg::Map> perception_sub_;

    typedef message_filters::sync_policies::ApproximateTime<gnss_ins_msg::msg::Gnssins64, drd25_msgs::msg::Map> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 发布
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_map_pub_;
    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<SlamProcessor>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
