#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <std_msgs/msg/string.hpp>
#include "test_cone_segmentation/msg/three_d_cone.hpp"
#include "test_cone_segmentation/msg/three_d_cone_array.hpp"
#include "trt_cone_detector/trt_detector.hpp"
#include "trt_cone_detector/voxel_generator.hpp"
#include "trt_cone_detector/post_processor.hpp"

// =========================
// ROS 2 推理节点
// =========================
// 功能划分：
// 1. 初始化阶段：读取参数、加载 TensorRT engine、分配 CPU/GPU 内存。
// 2. 输入阶段：订阅 PointCloud2，解析 xyz/intensity 并做物理范围过滤。
// 3. GPU 阶段：H2D 拷贝、CUDA 体素化、TensorRT 推理、D2H 拷贝。
// 4. 后处理阶段：解码网络输出、执行 NMS、发布 Marker 和自定义 bbox。
class LidarDetectorNode : public rclcpp::Node {
public:
    LidarDetectorNode() : Node("lidar_cone_detector") {
        // ---------- 1. ROS 参数区 ----------
        // score/nms 控制后处理；max_* 控制候选数量和内存容量；print_latency 用于性能观测。
        this->declare_parameter("score_thresh", 0.25); // 适当调高置信度
        this->declare_parameter("big_cone_score_thresh", 0.25);
        this->declare_parameter("nms_thresh", 0.1);
        this->declare_parameter("print_latency", true);
        this->declare_parameter("max_raw_points", 300000);
        this->declare_parameter("max_pre_nms", 1024);
        this->declare_parameter("max_output_boxes", 200);
        this->declare_parameter("intensity_scale", -1.0); // <0: 自动识别 0~1 或 0~255
        this->declare_parameter("input_topic", "/lidar_points");
        this->declare_parameter("output_topic", "/cone_detection_custom");
        this->declare_parameter("marker_topic", "/detected_cones_markers");
        this->declare_parameter("engine_path", "");
        this->declare_parameter("evaluation.enable_debug_metrics", false);

        score_thresh_ = this->get_parameter("score_thresh").as_double();
        float big_cone_score_thresh = this->get_parameter("big_cone_score_thresh").as_double();
        float nms_thresh = this->get_parameter("nms_thresh").as_double();
        print_latency_ = this->get_parameter("print_latency").as_bool();
        max_raw_points_ = this->get_parameter("max_raw_points").as_int();
        intensity_scale_ = this->get_parameter("intensity_scale").as_double();
        int max_pre_nms = this->get_parameter("max_pre_nms").as_int();
        int max_output_boxes = this->get_parameter("max_output_boxes").as_int();
        const std::string input_topic = this->get_parameter("input_topic").as_string();
        const std::string output_topic = this->get_parameter("output_topic").as_string();
        const std::string marker_topic = this->get_parameter("marker_topic").as_string();
        eval_metrics_enabled_ = this->get_parameter("evaluation.enable_debug_metrics").as_bool();

        // ---------- 2. 模型路径区 ----------
        // engine 安装在 share/<package>/models 下，launch 后无需手动写绝对路径。
        std::string pkg_share = ament_index_cpp::get_package_share_directory("trt_cone_detector");
        std::string engine_path = this->get_parameter("engine_path").as_string();
        if (engine_path.empty()) {
            engine_path = pkg_share + "/models/pointpillars_cone_fp16.engine";
        }

        try {
            // ---------- 3. 核心模块初始化 ----------
            // TrtConeDetector 管理 TensorRT engine；VoxelGenerator 生成 PointPillars 输入；
            // PostProcessor 把 dense 输出转换为最终 3D 框。
            detector_ = std::make_unique<trt_cone_detector::TrtConeDetector>(engine_path);
            detector_->allocateMemory(40000); 
            voxel_generator_ = std::make_unique<trt_cone_detector::VoxelGenerator>();
            post_processor_ = std::make_unique<trt_cone_detector::PostProcessor>(
                score_thresh_, nms_thresh, max_pre_nms, max_output_boxes,
                big_cone_score_thresh);
            
            if (max_raw_points_ <= 0) {
                throw std::runtime_error("max_raw_points must be positive");
            }

            // ---------- 4. 原始点云暂存区 ----------
            // h_raw_points_ 使用 pinned host memory，提高异步 H2D 拷贝效率。
            check_cuda(cudaMalloc((void**)&d_raw_points_, max_raw_points_ * 4 * sizeof(float)),
                       "cudaMalloc d_raw_points");
            check_cuda(cudaHostAlloc((void**)&h_raw_points_, max_raw_points_ * 4 * sizeof(float),
                                     cudaHostAllocDefault),
                       "cudaHostAlloc h_raw_points");

            // ---------- 5. 输出张量 host 缓冲区 ----------
            // 根据 engine 的实际 dtype/shape 分配，兼容 FP16 和 FP32 输出。
            cls_dtype_ = detector_->getTensorDataType("cls_preds");
            box_dtype_ = detector_->getTensorDataType("box_preds");
            allocate_host_output("cls_preds", post_processor_->expectedClsElementCount(),
                                 cls_dtype_, &h_cls_preds_, &h_cls_bytes_);
            allocate_host_output("box_preds", post_processor_->expectedBoxElementCount(),
                                 box_dtype_, &h_box_preds_, &h_box_bytes_);

            if (detector_->hasTensor("dir_cls_preds")) {
                dir_dtype_ = detector_->getTensorDataType("dir_cls_preds");
                allocate_host_output("dir_cls_preds", post_processor_->expectedDirElementCount(),
                                     dir_dtype_, &h_dir_cls_preds_, &h_dir_bytes_);
            }

            // ---------- 6. CUDA 事件 ----------
            // 事件用于拆分统计 H2D、voxel、TRT、D2H 各阶段 GPU 耗时。
            create_cuda_event(&ev_h2d_start_, "cudaEventCreate h2d_start");
            create_cuda_event(&ev_h2d_done_, "cudaEventCreate h2d_done");
            create_cuda_event(&ev_voxel_done_, "cudaEventCreate voxel_done");
            create_cuda_event(&ev_infer_start_, "cudaEventCreate infer_start");
            create_cuda_event(&ev_infer_done_, "cudaEventCreate infer_done");
            create_cuda_event(&ev_d2h_done_, "cudaEventCreate d2h_done");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "初始化失败: %s", e.what());
            return;
        }

        // ---------- 7. ROS 通信接口 ----------
        // 输入：/lidar_points；输出：RViz Marker 和业务可用的 ThreeDConeArray。
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&LidarDetectorNode::cloud_callback, this, std::placeholders::_1));
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
        bbox_pub_ = this->create_publisher<test_cone_segmentation::msg::ThreeDConeArray>(output_topic, 10);
        if (eval_metrics_enabled_) {
            metrics_pub_ = this->create_publisher<std_msgs::msg::String>("/perception/lidar/evaluation/metrics", 10);
        }

        RCLCPP_INFO(this->get_logger(),
                    "PointPillars detector ready. input=%s output=%s engine=%s",
                    input_topic.c_str(), output_topic.c_str(), engine_path.c_str());
    }

    ~LidarDetectorNode() {
        if (d_raw_points_) cudaFree(d_raw_points_);
        if (h_raw_points_) cudaFreeHost(h_raw_points_);
        if (h_cls_preds_) cudaFreeHost(h_cls_preds_);
        if (h_box_preds_) cudaFreeHost(h_box_preds_);
        if (h_dir_cls_preds_) cudaFreeHost(h_dir_cls_preds_);
        destroy_cuda_event(ev_h2d_start_);
        destroy_cuda_event(ev_h2d_done_);
        destroy_cuda_event(ev_voxel_done_);
        destroy_cuda_event(ev_infer_start_);
        destroy_cuda_event(ev_infer_done_);
        destroy_cuda_event(ev_d2h_done_);
    }

private:
    struct FrameTiming {
        double input_age_ms = -1.0;
        double prep_ms = 0.0;
        double voxel_wall_ms = 0.0;
        double gpu_wait_ms = 0.0;
        double post_ms = 0.0;
        double publish_ms = 0.0;
        double total_ms = 0.0;
        double h2d_ms = 0.0;
        double voxel_gpu_ms = 0.0;
        double infer_ms = 0.0;
        double d2h_ms = 0.0;
        double gpu_work_ms = 0.0;
    };

    // =========================
    // 通用 CUDA/时间工具
    // =========================
    // 这些工具函数只负责错误检查、事件管理和输入消息时间戳统计。
    void check_cuda(cudaError_t status, const char* what) {
        if (status != cudaSuccess) {
            throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(status));
        }
    }

    void create_cuda_event(cudaEvent_t* event, const char* what) {
        check_cuda(cudaEventCreateWithFlags(event, cudaEventDefault), what);
    }

    void destroy_cuda_event(cudaEvent_t& event) {
        if (event != nullptr) {
            cudaEventDestroy(event);
            event = nullptr;
        }
    }

    float elapsed_cuda_ms(cudaEvent_t start, cudaEvent_t end) {
        float ms = 0.0f;
        cudaError_t status = cudaEventElapsedTime(&ms, start, end);
        if (status != cudaSuccess) {
            RCLCPP_WARN(this->get_logger(), "cudaEventElapsedTime failed: %s",
                        cudaGetErrorString(status));
            return 0.0f;
        }
        return ms;
    }

    double message_age_ms(const std_msgs::msg::Header& header) const {
        if (header.stamp.sec == 0 && header.stamp.nanosec == 0) {
            return -1.0;
        }
        return (this->now() - rclcpp::Time(header.stamp)).seconds() * 1000.0;
    }

    static double stamp_to_sec(const std_msgs::msg::Header& header) {
        return static_cast<double>(header.stamp.sec) +
               static_cast<double>(header.stamp.nanosec) * 1e-9;
    }

    static double elapsed_ms(std::chrono::high_resolution_clock::time_point start,
                             std::chrono::high_resolution_clock::time_point end) {
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    void publish_metrics(const std_msgs::msg::Header& header,
                         const std::string& event,
                         int input_points,
                         int valid_points,
                         int voxel_count,
                         int cone_count,
                         const FrameTiming& timing) {
        if (!eval_metrics_enabled_ || !metrics_pub_) return;

        std_msgs::msg::String msg;
        std::ostringstream out;
        out << std::fixed << std::setprecision(9);
        out << "{"
            << "\"component\":\"pointpillars\""
            << ",\"event\":\"" << event << "\""
            << ",\"stamp\":" << stamp_to_sec(header)
            << ",\"frame_id\":\"" << header.frame_id << "\""
            << ",\"input_points\":" << input_points
            << ",\"valid_points\":" << valid_points
            << ",\"voxel_count\":" << voxel_count
            << ",\"cone_count\":" << cone_count
            << ",\"input_age_ms\":" << timing.input_age_ms
            << ",\"prep_ms\":" << timing.prep_ms
            << ",\"voxel_wall_ms\":" << timing.voxel_wall_ms
            << ",\"gpu_wait_ms\":" << timing.gpu_wait_ms
            << ",\"post_ms\":" << timing.post_ms
            << ",\"publish_ms\":" << timing.publish_ms
            << ",\"h2d_ms\":" << timing.h2d_ms
            << ",\"voxel_gpu_ms\":" << timing.voxel_gpu_ms
            << ",\"infer_ms\":" << timing.infer_ms
            << ",\"d2h_ms\":" << timing.d2h_ms
            << ",\"gpu_work_ms\":" << timing.gpu_work_ms
            << ",\"total_ms\":" << timing.total_ms
            << "}";
        msg.data = out.str();
        metrics_pub_->publish(msg);
    }

    // =========================
    // TensorRT 输出 host 内存分配
    // =========================
    // 读取 engine 中的真实输出字节数；如果 shape 是动态导致字节数未知，
    // 则使用后处理模块给出的期望元素数量兜底。
    void allocate_host_output(const std::string& tensor_name, std::size_t expected_elements,
                              trt_cone_detector::TensorDataType dtype,
                              void** host_ptr, std::size_t* byte_size) {
        const std::size_t dtype_size = trt_cone_detector::tensorDataTypeSize(dtype);
        if (dtype_size == 0) {
            throw std::runtime_error("Unsupported output dtype for " + tensor_name);
        }

        std::size_t bytes = detector_->getTensorByteSize(tensor_name);
        const std::size_t min_bytes = expected_elements * dtype_size;
        if (bytes == 0) {
            bytes = min_bytes;
        }
        if (bytes < min_bytes) {
            throw std::runtime_error("Output tensor " + tensor_name + " is smaller than expected");
        }

        check_cuda(cudaHostAlloc(host_ptr, bytes, cudaHostAllocDefault),
                   ("cudaHostAlloc " + tensor_name).c_str());
        *byte_size = bytes;

        RCLCPP_INFO(this->get_logger(), "Output %s: %zu bytes, dtype=%s",
                    tensor_name.c_str(), bytes, trt_cone_detector::tensorDataTypeName(dtype));
    }

    // =========================
    // 点云强度归一化
    // =========================
    // 网络训练通常使用 0~1 intensity；这里兼容雷达驱动输出 0~1 或 0~255。
    float normalize_intensity(float raw_intensity) const {
        if (!std::isfinite(raw_intensity)) {
            return 0.0f;
        }

        float value = 0.0f;
        if (intensity_scale_ > 0.0) {
            value = raw_intensity * static_cast<float>(intensity_scale_);
        } else {
            value = raw_intensity > 1.5f ? raw_intensity / 255.0f : raw_intensity;
        }
        return std::max(0.0f, std::min(1.0f, value));
    }

    // =========================
    // 单帧点云推理主流程
    // =========================
    // 数据流：PointCloud2 -> host raw points -> device raw points -> voxels
    //        -> TensorRT 输出 -> CPU 后处理 -> ROS topic。
    void cloud_callback([[maybe_unused]] const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto t0 = std::chrono::high_resolution_clock::now();
        FrameTiming timing;
        timing.input_age_ms = message_age_ms(msg->header);

        // ---------- 1. 解析 PointCloud2 字段 ----------
        // 默认按常见 xyz float32 偏移读取；如果字段描述不同，则使用 msg->fields 修正偏移。
        int num_points = msg->width * msg->height;
        const uint8_t* data_ptr = msg->data.data();
        int step = msg->point_step;

        int x_offset = 0, y_offset = 4, z_offset = 8, i_offset = -1;
        for (const auto& field : msg->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "intensity") i_offset = field.offset;
        }

        int valid_points = 0; // 记录真实写入显存的有效点数量

        // ---------- 2. CPU 预过滤与格式转换 ----------
        // 过滤 NaN、范围外和明显无关高度点，并整理成连续 [x, y, z, intensity]。
        for (int i = 0; i < num_points; ++i) {
            const uint8_t* pt_ptr = data_ptr + i * step;
            
            float x = *reinterpret_cast<const float*>(pt_ptr + x_offset);
            float y = *reinterpret_cast<const float*>(pt_ptr + y_offset);
            float z = *reinterpret_cast<const float*>(pt_ptr + z_offset);

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            // 拦截 1：剔除超出 YAML 定义的 XY 物理边界的点 (X: 0~20, Y: -20~20)
            if (x < 0.0f || x >= 20.0f || y < -20.0f || y >= 20.0f) continue;

            // 拦截 2：Z 轴黄金高度过滤 (干掉海量地面点和高空噪点)
            // 假设雷达离地 1m，地面在 -1m，锥桶最高 0.7m。保留 -1.5m 到 0.5m
            if (z < -2.0f || z > 1.0f) continue; 

            if (valid_points >= max_raw_points_) break;

            // 只有通过了双重拦截的高质量点，才会被存入宿主内存准备发送给 GPU
            h_raw_points_[valid_points * 4 + 0] = x; 
            h_raw_points_[valid_points * 4 + 1] = y; 
            h_raw_points_[valid_points * 4 + 2] = z; 
        
            if (i_offset != -1) {
                float raw_intensity = *reinterpret_cast<const float*>(pt_ptr + i_offset); 
                h_raw_points_[valid_points * 4 + 3] = normalize_intensity(raw_intensity);
            } else {
                h_raw_points_[valid_points * 4 + 3] = 0.0f; // 如果没有强度字段，安全补零
            }
            
            valid_points++;
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        timing.prep_ms = elapsed_ms(t0, t1);

        // 没有有效点时仍发布空结果，用于清空 RViz 上一帧 marker。
        if (valid_points == 0) {
            publish_results({}, msg->header);
            auto t_after_publish = std::chrono::high_resolution_clock::now();
            timing.publish_ms = elapsed_ms(t1, t_after_publish);
            timing.total_ms = elapsed_ms(t0, t_after_publish);
            publish_metrics(msg->header, "no_valid_points", num_points, valid_points, 0, 0, timing);
            return;
        }

        // ---------- 3. Host -> Device ----------
        // 原始点云进入同一个 CUDA stream，后续体素化和推理按顺序执行。
        cudaStream_t stream = detector_->getStream();
        cudaEventRecord(ev_h2d_start_, stream);
        cudaError_t copy_status = cudaMemcpyAsync(
            d_raw_points_, h_raw_points_, valid_points * 4 * sizeof(float),
            cudaMemcpyHostToDevice, stream);
        if (copy_status != cudaSuccess) {
            RCLCPP_ERROR(this->get_logger(), "cudaMemcpyAsync raw points failed: %s",
                         cudaGetErrorString(copy_status));
            return;
        }
        cudaEventRecord(ev_h2d_done_, stream);

        // ---------- 4. GPU 体素化 ----------
        // 生成 PointPillars engine 需要的三个输入张量：
        // voxels、voxel_coords、voxel_num_points。
        int current_voxel_num = voxel_generator_->generate(
            d_raw_points_, valid_points,
            detector_->getDevicePtr("voxels"),
            detector_->getDevicePtr("voxel_coords"),
            detector_->getDevicePtr("voxel_num_points"),
            detector_->tensorIsFloat16("voxels"),
            stream,
            ev_voxel_done_
        );
        auto t_after_voxel_sync = std::chrono::high_resolution_clock::now();
        timing.voxel_wall_ms = elapsed_ms(t1, t_after_voxel_sync);

        // 所有点都被体素化过滤掉时，发布空结果并结束本帧。
        if (current_voxel_num == 0) {
            publish_results({}, msg->header);
            auto t_after_publish = std::chrono::high_resolution_clock::now();
            timing.publish_ms = elapsed_ms(t_after_voxel_sync, t_after_publish);
            timing.total_ms = elapsed_ms(t0, t_after_publish);
            publish_metrics(msg->header, "no_voxels", num_points, valid_points, current_voxel_num, 0, timing);
            return;
        }

        // ---------- 5. TensorRT 推理 ----------
        // doInference 只 enqueue，不立刻同步；同步放到 D2H 拷贝之后统一完成。
        cudaEventRecord(ev_infer_start_, stream);
        if (!detector_->doInference(current_voxel_num, false)) {
            RCLCPP_ERROR(this->get_logger(), "TensorRT inference enqueue failed");
            return;
        }
        cudaEventRecord(ev_infer_done_, stream);

        // ---------- 6. Device -> Host ----------
        // 网络输出拷贝回 pinned host memory，随后在 CPU 上执行 anchor 解码和 NMS。
        cudaMemcpyAsync(h_cls_preds_, detector_->getDevicePtr("cls_preds"), h_cls_bytes_,
                        cudaMemcpyDeviceToHost, stream);
        cudaMemcpyAsync(h_box_preds_, detector_->getDevicePtr("box_preds"), h_box_bytes_,
                        cudaMemcpyDeviceToHost, stream);
        
        void* d_dir_ptr = detector_->getDevicePtr("dir_cls_preds");
        if (d_dir_ptr) {
            cudaMemcpyAsync(h_dir_cls_preds_, d_dir_ptr, h_dir_bytes_,
                            cudaMemcpyDeviceToHost, stream);
        }
        cudaEventRecord(ev_d2h_done_, stream);

        cudaError_t sync_status = cudaStreamSynchronize(stream);
        if (sync_status != cudaSuccess) {
            RCLCPP_ERROR(this->get_logger(), "cudaStreamSynchronize after inference failed: %s",
                         cudaGetErrorString(sync_status));
            return;
        }
        auto t_after_gpu_sync = std::chrono::high_resolution_clock::now();
        timing.gpu_wait_ms = elapsed_ms(t_after_voxel_sync, t_after_gpu_sync);

        // ---------- 7. CPU 后处理 ----------
        // cls/box/dir 三类输出合成最终 3D bbox；dir 输出是可选张量。
        auto final_boxes = post_processor_->process(
            h_cls_preds_, cls_dtype_,
            h_box_preds_, box_dtype_,
            d_dir_ptr ? h_dir_cls_preds_ : nullptr,
            d_dir_ptr ? dir_dtype_ : trt_cone_detector::TensorDataType::kUnknown
        );
        auto t_after_post = std::chrono::high_resolution_clock::now();
        timing.post_ms = elapsed_ms(t_after_gpu_sync, t_after_post);

        publish_results(final_boxes, msg->header);
        auto t_after_publish = std::chrono::high_resolution_clock::now();
        timing.publish_ms = elapsed_ms(t_after_post, t_after_publish);
        timing.total_ms = elapsed_ms(t0, t_after_publish);
        timing.h2d_ms = elapsed_cuda_ms(ev_h2d_start_, ev_h2d_done_);
        timing.voxel_gpu_ms = elapsed_cuda_ms(ev_h2d_done_, ev_voxel_done_);
        timing.infer_ms = elapsed_cuda_ms(ev_infer_start_, ev_infer_done_);
        timing.d2h_ms = elapsed_cuda_ms(ev_infer_done_, ev_d2h_done_);
        timing.gpu_work_ms = timing.h2d_ms + timing.voxel_gpu_ms + timing.infer_ms + timing.d2h_ms;
        publish_metrics(msg->header, "processed", num_points, valid_points, current_voxel_num,
                        static_cast<int>(final_boxes.size()), timing);

        // ---------- 8. 性能日志 ----------
        // wall time 反映端到端等待；CUDA event time 反映 GPU stream 内实际工作耗时。
        if (print_latency_) {
            int small_cone_count = 0;
            int big_cone_count = 0;
            for (const auto& box : final_boxes) {
                if (box.label == 1) {
                    ++big_cone_count;
                } else {
                    ++small_cone_count;
                }
            }

            RCLCPP_INFO(this->get_logger(), 
                "Age: %.1fms | Pts: %d | Voxels: %d | Cones: %zu(S/B=%d/%d) | Total: %.2fms | CPU(prep/post/pub): %.2f/%.2f/%.2fms | GPU(H2D/voxel/TRT/D2H/work): %.2f/%.2f/%.2f/%.2f/%.2fms | Wait(voxel/gpu): %.2f/%.2fms", 
                timing.input_age_ms,
                valid_points, current_voxel_num, final_boxes.size(), small_cone_count, big_cone_count,
                timing.total_ms,
                timing.prep_ms, timing.post_ms, timing.publish_ms,
                timing.h2d_ms, timing.voxel_gpu_ms, timing.infer_ms, timing.d2h_ms, timing.gpu_work_ms,
                timing.voxel_wall_ms, timing.gpu_wait_ms);
        }
    }

    // =========================
    // 结果发布
    // =========================
    // MarkerArray 用于 RViz 可视化；ThreeDConeArray 用于下游规划/控制模块读取。
    void publish_results(const std::vector<trt_cone_detector::Bndbox>& boxes, const std_msgs::msg::Header& header) {
        visualization_msgs::msg::MarkerArray marker_array;
        test_cone_segmentation::msg::ThreeDConeArray cone_array_msg;
        
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        cone_array_msg.header = header;

        for (size_t i = 0; i < boxes.size(); ++i) {
            const auto& box = boxes[i];

            visualization_msgs::msg::Marker m;
            m.header = header;
            m.ns = "cone";
            m.id = i;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.pose.position.x = box.x;
            m.pose.position.y = box.y;
            m.pose.position.z = box.z;
            
            m.pose.orientation.x = 0.0;
            m.pose.orientation.y = 0.0;
            m.pose.orientation.z = std::sin(box.yaw / 2.0);
            m.pose.orientation.w = std::cos(box.yaw / 2.0);
            
            m.scale.x = box.l;
            m.scale.y = box.w;
            m.scale.z = box.h;
            
            if (box.label == 1) { 
                m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.9;
            } else { 
                m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0; m.color.a = 0.9;
            }
                
            m.lifetime = rclcpp::Duration::from_seconds(0.1);
            marker_array.markers.push_back(m);
            
            test_cone_segmentation::msg::ThreeDCone cone;
            cone.center.x = box.x;
            cone.center.y = box.y;
            cone.center.z = box.z;
            cone.size.x = box.l;
            cone.size.y = box.w;
            cone.size.z = box.h;
            
            cone_array_msg.cones.push_back(cone);
        }
            
        marker_pub_->publish(marker_array);
        bbox_pub_->publish(cone_array_msg);
    }

    // =========================
    // 节点参数与内存资源
    // =========================
    double score_thresh_;
    double intensity_scale_ = -1.0;
    int max_raw_points_ = 300000;
    float* d_raw_points_ = nullptr; 
    float* h_raw_points_ = nullptr;
    bool print_latency_;

    void* h_cls_preds_ = nullptr;
    void* h_box_preds_ = nullptr;
    void* h_dir_cls_preds_ = nullptr;
    std::size_t h_cls_bytes_ = 0;
    std::size_t h_box_bytes_ = 0;
    std::size_t h_dir_bytes_ = 0;
    cudaEvent_t ev_h2d_start_ = nullptr;
    cudaEvent_t ev_h2d_done_ = nullptr;
    cudaEvent_t ev_voxel_done_ = nullptr;
    cudaEvent_t ev_infer_start_ = nullptr;
    cudaEvent_t ev_infer_done_ = nullptr;
    cudaEvent_t ev_d2h_done_ = nullptr;
    trt_cone_detector::TensorDataType cls_dtype_ = trt_cone_detector::TensorDataType::kUnknown;
    trt_cone_detector::TensorDataType box_dtype_ = trt_cone_detector::TensorDataType::kUnknown;
    trt_cone_detector::TensorDataType dir_dtype_ = trt_cone_detector::TensorDataType::kUnknown;

    // =========================
    // 功能模块对象
    // =========================
    std::unique_ptr<trt_cone_detector::TrtConeDetector> detector_;
    std::unique_ptr<trt_cone_detector::VoxelGenerator> voxel_generator_;
    std::unique_ptr<trt_cone_detector::PostProcessor> post_processor_;

    // =========================
    // ROS 订阅与发布接口
    // =========================
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<test_cone_segmentation::msg::ThreeDConeArray>::SharedPtr bbox_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
    bool eval_metrics_enabled_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
