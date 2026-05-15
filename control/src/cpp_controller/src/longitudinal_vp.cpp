#include <cpp_controller/longitudinal_vp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>

LongitudinalVP::LongitudinalVP(
    double a_y_max,
    double a_accel_max,
    double a_decel_max,
    double delta_s
) : 
    a_y_max(a_y_max),
    a_accel_max(a_accel_max),
    a_decel_max(a_decel_max),
    delta_s(delta_s)  // 采样间隔
    {}

// run_step 函数
std::deque<double> LongitudinalVP::run_step(const std::deque<std::array<double, 2>>& fitted_points, double current_speed)
{
    // 如果现在的速度小于2m/s，则把速度设为2m/s
    /*
    if (current_speed < 2.0) {
        current_speed = 2.0;
    }
    */
    // 1.曲率计算
    std::deque<double> curvature = calculate_curvature(fitted_points);
    // 2.弯道最大速度计算
    std::deque<double> cornering_speed = calculate_cornering_speed(curvature);
    // 3.前向加速剖面计算
    std::deque<double> forward_profile = calculate_forward_profile(cornering_speed, current_speed);
    // 4.后向减速剖面计算
    std::deque<double> backward_profile = calculate_backward_profile(cornering_speed);
    // 5.最终速度剖面计算
    std::deque<double> final_profile = get_final_profile(cornering_speed, forward_profile, backward_profile);
    return final_profile;

}

// 根据路径点进行曲线拟合,使用三次样条插值法
std::deque<std::array<double, 2>> LongitudinalVP::curve_fitting(const std::deque<std::array<double, 2>>& waypoints_queue)
{   
    int n = waypoints_queue.size();
    // 如果路径点数量小于3，直接返回
    if (n < 3) {
        return waypoints_queue;
    }

    std::deque<std::array<double, 2>> fitted_points;
    // 1.参数化路径
    // 计算得到弦长序列，s[i]表示从起点到第i个点的弦长
    std::vector<double> s(n);
    s[0] = 0.0;
    for (int i = 1; i < n; ++i) 
    {
        double dx = waypoints_queue[i][0] - waypoints_queue[i-1][0];
        double dy = waypoints_queue[i][1] - waypoints_queue[i-1][1];
        s[i] = s[i-1] + std::hypot(dx, dy);
    }

    // 2. 提取x和y坐标
    std::vector<double> x, y;
    for (const auto& p : waypoints_queue) 
    {
        x.push_back(p[0]);
        y.push_back(p[1]);
    }

    // 3.计算每个区间的间隔
    std::vector<double> h(n-1);
    for(int i = 0; i < n - 1; ++i) 
    {
        h[i] = s[i + 1] - s[i];
    }

    // 4.求解三对角线性方程组，得到二阶导数
    std::vector<double> m_x = calculate_second_derivatives(s, x, h);
    std::vector<double> m_y = calculate_second_derivatives(s, y, h);

    // 5.构建样条段
    x_segments = build_spline_segments(s, x, m_x);
    y_segments = build_spline_segments(s, y, m_y);

    // 6.生成平滑曲线上的点 
    double total_s = s.back();

    //std::cout << "total_s: " << total_s << std::endl;

    fitted_points = generate_smooth_points(x_segments, y_segments, total_s);
    return fitted_points;
}

// 根据拟合的曲线计算曲率
std::deque<double> LongitudinalVP::calculate_curvature(const std::deque<std::array<double, 2>>& fitted_points)
{   
    std::deque<double> curvature;

     for (size_t i = 0; i < fitted_points.size(); ++i) {
        double s = i * delta_s; // 假设按采样间隔递增

        // 查找对应的样条段
        const SplineSegment* x_seg = nullptr;
        const SplineSegment* y_seg = nullptr;
        for (const auto& seg : x_segments) {
            if (s >= seg.s_start && s <= seg.s_end) {
                x_seg = &seg;
                break;
            }
        }
        for (const auto& seg : y_segments) {
            if (s >= seg.s_start && s <= seg.s_end) {
                y_seg = &seg;
                break;
            }
        }

        if (!x_seg || !y_seg) {
            curvature.push_back(0.0);
            continue;
        }

        // 计算导数
        double ds_x = s - x_seg->s_start;
        double x_prime = x_seg->b + 2 * x_seg->c * ds_x + 3 * x_seg->d * ds_x * ds_x;
        double x_prime_prime = 2 * x_seg->c + 6 * x_seg->d * ds_x;

        double ds_y = s - y_seg->s_start;
        double y_prime = y_seg->b + 2 * y_seg->c * ds_y + 3 * y_seg->d * ds_y * ds_y;
        double y_prime_prime = 2 * y_seg->c + 6 * y_seg->d * ds_y;

        // 计算曲率
        double numerator = std::abs(x_prime * y_prime_prime - y_prime * x_prime_prime);
        double denominator = std::pow(x_prime * x_prime + y_prime * y_prime, 1.5);
        double k = (denominator > 1e-9) ? numerator / denominator : 0.0;
        curvature.push_back(k);
    }

    return curvature;
}

// 计算弯道最大速度
std::deque<double> LongitudinalVP::calculate_cornering_speed(const std::deque<double>& curvature) {
    std::deque<double> corner_speed;
    for (auto k : curvature) {
        if (std::abs(k) < 1e-3) {
            corner_speed.push_back(INFINITY); // 直道无限制
        } else {
            corner_speed.push_back(std::sqrt(a_y_max / std::abs(k)));
        }
    }
    return corner_speed;
}


// 前向加速剖面（纵向加速约束 + 实时曲率限制）
std::deque<double> LongitudinalVP::calculate_forward_profile(const std::deque<double>& cornering_speed, double current_speed) {
    std::deque<double> forward_profile;
    forward_profile.push_back(current_speed); // 初始速度
    for (size_t i = 1; i < cornering_speed.size(); ++i) {
        // 计算理论加速速度
        double v_acc = std::sqrt(forward_profile[i-1] * forward_profile[i-1] + 2 * a_accel_max * delta_s);

        forward_profile.push_back(v_acc);
    }
    return forward_profile;
}

// 后向减速剖面（纵向刹车约束 + 实时曲率限制）
std::deque<double> LongitudinalVP::calculate_backward_profile(const std::deque<double>& cornering_speed) {
    std::deque<double> backward_profile(cornering_speed.size(), 0.0);
    if (cornering_speed.empty()) return backward_profile;

    // 找到转角速度最小的点的索引
    auto min_it = std::min_element(cornering_speed.begin(), cornering_speed.end());
    int start_index = std::distance(cornering_speed.begin(), min_it);

    // 初始化起点的速度为转角速度最小的点的速度
    backward_profile[start_index] = *min_it;

    // 从转角速度最小的点开始反向传播
    for (int i = start_index - 1; i >= 0; --i) {
        // 计算理论刹车速度
        double v_dec = std::sqrt(backward_profile[i+1] * backward_profile[i+1] + 2 * a_decel_max * delta_s);
        backward_profile[i] = v_dec;
    }

    // 如果转角速度最小的点不是第一个点，则还需要向前计算
    for (int i = start_index + 1; i < static_cast<int>(cornering_speed.size()); ++i) {
        // 计算理论刹车速度
        double v_dec = std::sqrt(backward_profile[i-1] * backward_profile[i-1] + 2 * a_accel_max * delta_s);
        backward_profile[i] = v_dec;
    }

    return backward_profile;
}


// 最终速度剖面（取加速和减速的较小值）
std::deque<double> LongitudinalVP::get_final_profile(
    const std::deque<double>& cornering_speed,
    const std::deque<double>& forward_profile,
    const std::deque<double>& backward_profile)
{
    std::deque<double> final_profile;
    for (size_t i = 0; i < forward_profile.size(); ++i) {
        
        
        if(i==1)
        {   /*
            std::cout << "  cornering_speed: " << cornering_speed[i] ;
            std::cout << "  forward_profile: " << forward_profile[i] ;
            std::cout << "  backward_profile: " << backward_profile[i] << "\n" << std::endl;
            */
            //存入.txt文件
            /*
            std::ofstream outfile;
            outfile.open("/home/she/DRd25_ws/src/cpp_controller/test/print_files/profile_collections.txt", std::ios::out); // 使用 std::ios::out 而不是 std::ios::app
            outfile << "cornering_speed: " << cornering_speed[i] 
                    << "  forward_profile: " << forward_profile[i] 
                    << "  backward_profile: " << backward_profile[i] << "\n";
            outfile.close();
            */
        }
        
       
        final_profile.push_back(std::min(cornering_speed[i], std::min(forward_profile[i], backward_profile[i])));
    }
    return final_profile;
} 

// 计算三次样条插值的二阶导数
std::vector<double> LongitudinalVP::calculate_second_derivatives(const std::vector<double>& s, const std::vector<double>& f, const std::vector<double>& h)
{
    int n = s.size();
    /*-----------------------------------------------------------
        Step 1: 构造三对角线性方程的系数
        对于内部节点 i = 1, 2, ..., n-2（注意：端点 i=0 和 i=n-1 已知，根据自然边界条件 m[0]=m[n-1]=0）
        每个内部节点对应以下方程：
            A_i * m[i-1] + B_i * m[i] + C_i * m[i+1] = D_i
        其中:
            A_i = h[i-1]
            B_i = 2 * (h[i-1] + h[i])
            C_i = h[i]
            D_i = 6 * [ (f[i+1] - f[i]) / h[i] - (f[i] - f[i-1]) / h[i-1] ]  注：f[i]即为x[i]或者y[i]
        注意: 为了方便数组下标，从 1 到 n-2 的节点，我们用数组下标 i-1 来存储系数
    -----------------------------------------------------------*/
    int numInternal = n - 2; // 内部未知数的数量
    std::vector<double> A(numInternal, 0.0); // 下对角线系数
    std::vector<double> B(numInternal, 0.0); // 主对角线系数
    std::vector<double> C(numInternal, 0.0); // 上对角线系数
    std::vector<double> D(numInternal, 0.0); // 方程右侧常数项

    // 填充系数数组
    for (int i = 1; i <= n - 2; ++i) {
        A[i - 1] = h[i - 1];                     
        B[i - 1] = 2.0 * (h[i - 1] + h[i]);      
        C[i - 1] = h[i];                          
        D[i - 1] = 6.0 * ((f[i + 1] - f[i]) / h[i] - (f[i] - f[i - 1]) / h[i - 1]);
    }

    /*---------------------------------------------------
        Step 2: 利用追赶法（Thomas 算法）求解三对角方程组
        追赶法分为两步：前向消去和回代求解
    ---------------------------------------------------*/
    // 2.1 前向消去
    // 创建两个临时数组，分别用于存储修改后的上对角系数和右侧常数
    std::vector<double> c_prime(numInternal, 0.0);
    std::vector<double> d_prime(numInternal, 0.0);

    // 对第一个方程（i = 0，对应原始节点 i = 1）：
    c_prime[0] = C[0] / B[0];
    d_prime[0] = D[0] / B[0];

    // 对剩余方程依次消去下对角项
    for (int i = 1; i < numInternal; ++i) {
        // 计算当前方程的分母: B[i] - A[i] * c_prime[i - 1]
        double denom = B[i] - A[i] * c_prime[i - 1];
        // 修改后的上对角系数
        c_prime[i] = C[i] / denom;
        // 修改后的右侧常数
        d_prime[i] = (D[i] - A[i] * d_prime[i - 1]) / denom;
    }

    // 2.2 回代求解
    // 先创建一个数组存储内部未知数 m_internal（对应原始节点 i = 1 到 n-2）
    std::vector<double> m_internal(numInternal, 0.0);
    // 最后一个内部节点的 m 值直接由 d_prime 给出
    m_internal[numInternal - 1] = d_prime[numInternal - 1];
    // 从倒数第二个开始，依次回代求解 m_internal
    for (int i = numInternal - 2; i >= 0; --i) {
        m_internal[i] = d_prime[i] - c_prime[i] * m_internal[i + 1];
    }

    /*---------------------------------------------------
        Step 3: 整合结果
        根据自然边界条件，首尾的二阶导数 m[0] 和 m[n-1] 为 0，
        内部节点的 m 值从 m_internal 中得到，注意下标转换
    ---------------------------------------------------*/
    std::vector<double> m(n, 0.0);
    m[0] = 0.0;  // 自然边界条件
    for (int i = 1; i <= n - 2; ++i) {
        m[i] = m_internal[i - 1];
    }
    m[n - 1] = 0.0;  // 自然边界条件

    // 返回所有节点处的二阶导数
    return m;
}

// 构建样条段的函数
std::vector<SplineSegment> LongitudinalVP::build_spline_segments(
    const std::vector<double>& s,
    const std::vector<double>& values,
    const std::vector<double>& m    
) {
    std::vector<SplineSegment> segments;
    for (size_t i = 0; i < s.size() - 1; ++i) {
        double h = s[i+1] - s[i];
        SplineSegment seg;
        seg.a = values[i];
        seg.b = (values[i+1] - values[i]) / h - h * (2 * m[i] + m[i+1]) / 6.0;
        seg.c = m[i] / 2.0;
        seg.d = (m[i+1] - m[i]) / (6.0 * h);
        seg.s_start = s[i];
        seg.s_end = s[i+1];
        segments.push_back(seg);
    }
    return segments;
}

// 均匀采样生成平滑曲线点的函数
std::deque<std::array<double, 2>> LongitudinalVP::generate_smooth_points(
    const std::vector<SplineSegment>& x_segments,
    const std::vector<SplineSegment>& y_segments,
    double total_s      // 总弧长
) {
    std::deque<std::array<double, 2>> smooth_points;
    
    // 对总弧长从 0 到 total_s 以 delta_s 为间隔进行采样
    for (double s = 0; s <= total_s; s += delta_s) {
        // 查找当前 s 所在的样条段（这里采用简单线性查找，也可以用二分查找优化）
        const SplineSegment* x_seg = nullptr;
        const SplineSegment* y_seg = nullptr;
        for (const auto& seg : x_segments) {
            if (s >= seg.s_start && s <= seg.s_end) {
                x_seg = &seg;
                break;
            }
        }
        for (const auto& seg : y_segments) {
            if (s >= seg.s_start && s <= seg.s_end) {
                y_seg = &seg;
                break;
            }
        }
        // 如果找不到，则取边界值（可根据实际需求处理）
        if (!x_seg) x_seg = &x_segments.front();
        if (!y_seg) y_seg = &y_segments.front();
        
        // 计算当前 s 在所在段的偏移量 ds
        double ds = s - x_seg->s_start;
        // 使用样条段多项式计算 x 和 y 坐标
        double interp_x = x_seg->a + x_seg->b * ds + x_seg->c * ds * ds + x_seg->d * ds * ds * ds;
        double interp_y = y_seg->a + y_seg->b * ds + y_seg->c * ds * ds + y_seg->d * ds * ds * ds;
        
        smooth_points.push_back({interp_x, interp_y});
    }
    
    return smooth_points;
}
