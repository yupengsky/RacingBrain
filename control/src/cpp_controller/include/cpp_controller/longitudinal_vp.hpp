#ifndef LONGITUDINAL_VP_HPP
#define LONGITUDINAL_VP_HPP

#include <cmath>
#include <deque>
#include <array>
#include <vector>
#include <algorithm>
#include <limits>

struct SplineSegment {
        double a;       // 常数项 (y_i)
        double b;       // 一阶项系数
        double c;       // 二阶项系数 (M_i/2)
        double d;       // 三阶项系数
        double s_start; // 区间起始弧长
        double s_end;   // 区间结束弧长
    };

class LongitudinalVP {
public:
    LongitudinalVP(double a_y_max = 5.0, double a_accel_max = 5.0, double a_decel_max = -10.0, double delta_s=1.0);
    
    std::deque<double> run_step(const std::deque<std::array<double, 2>>& fitted_points, double current_speed);
    std::deque<std::array<double, 2>> curve_fitting(const std::deque<std::array<double, 2>>& waypoints_queue);



private:
    
    std::deque<double> calculate_curvature(const std::deque<std::array<double, 2>>& fitted_points);
    std::deque<double> calculate_cornering_speed(const std::deque<double>& curvature);
    std::deque<double> calculate_forward_profile(const std::deque<double>& cornering_speed, double current_speed);
    std::deque<double> calculate_backward_profile(const std::deque<double>& cornering_speed);
    std::deque<double> get_final_profile(
        const std::deque<double>& cornering_speed,
        const std::deque<double>& forward_profile, 
        const std::deque<double>& backward_profile
    );

    std::vector<double> calculate_second_derivatives(
        const std::vector<double>& s, 
        const std::vector<double>& f, 
        const std::vector<double>& h);
    std::vector<SplineSegment> build_spline_segments(
        const std::vector<double>& s,
        const std::vector<double>& values,
        const std::vector<double>& m
    );
    std::deque<std::array<double, 2>> generate_smooth_points(
        const std::vector<SplineSegment>& x_segments,
        const std::vector<SplineSegment>& y_segments,
        double total_s      // 总弧长
    );

    




    // 车辆参数
    double a_y_max;
    double a_accel_max;
    double a_decel_max;
    double delta_s; // 采样间隔

    double k_delta_s = 2; //

    std::vector<SplineSegment> x_segments;
    std::vector<SplineSegment> y_segments;
};

#endif // LONGITUDINAL_VP_HPP