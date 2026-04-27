#ifndef TRT_CONE_DETECTOR_TRT_DETECTOR_HPP_
#define TRT_CONE_DETECTOR_TRT_DETECTOR_HPP_

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>

#include "trt_cone_detector/tensor_types.hpp"

namespace trt_cone_detector {

// =========================
// 1. TensorRT 日志适配器
// =========================
// 只输出 warning 及以上级别，避免正常推理时刷屏。
class TrtLogger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TRT] " << msg << std::endl;
        }
    }
};

// =========================
// 2. TensorRT Engine 管理与推理封装
// =========================
// 作用：加载 .engine、管理输入/输出显存、设置动态 shape、提交 enqueueV3 推理。
class TrtConeDetector {
public:
    TrtConeDetector(const std::string& engine_path);
    ~TrtConeDetector();

    // 分配 TensorRT 输入/输出显存。max_voxels 必须和体素生成上限保持一致。
    void allocateMemory(int max_voxels = 20000);
    // 按当前帧实际 voxel 数量执行推理；synchronize=false 时由调用方统一同步 stream。
    bool doInference(int current_voxel_num, bool synchronize = true);
    // 通过 TensorRT 张量名取显存指针，供 voxel 生成和 D2H 拷贝使用。
    void* getDevicePtr(const std::string& tensor_name);
    cudaStream_t getStream() const { return m_stream; }
    // 张量元信息查询：节点用它判断可选输出、dtype 和需要拷贝的字节数。
    bool hasTensor(const std::string& tensor_name) const;
    bool tensorIsFloat16(const std::string& tensor_name) const;
    TensorDataType getTensorDataType(const std::string& tensor_name) const;
    std::size_t getTensorElementCount(const std::string& tensor_name) const;
    std::size_t getTensorByteSize(const std::string& tensor_name) const;

private:
    // 记录 engine 中每个输入/输出张量的静态元信息。
    struct TensorInfo {
        TensorDataType dtype = TensorDataType::kUnknown;
        nvinfer1::Dims shape{};
        bool is_input = false;
        std::size_t element_count = 0;
        std::size_t byte_size = 0;
    };

    TrtLogger m_logger;
    nvinfer1::IRuntime* m_runtime = nullptr;
    nvinfer1::ICudaEngine* m_engine = nullptr;
    nvinfer1::IExecutionContext* m_context = nullptr;
    cudaStream_t m_stream = nullptr;
    int max_voxels_ = 0;

    std::unordered_map<std::string, TensorInfo> tensor_info_;

    // 动态 shape 支持：把导出 engine 中的 -1 维度替换为当前帧实际 voxel 数。
    bool tensorHasDynamicShape(const std::string& tensor_name) const;
    nvinfer1::Dims makeRuntimeInputShape(const std::string& tensor_name, int current_voxel_num) const;
    bool setInputShapeIfDynamic(const std::string& tensor_name, int current_voxel_num);

    // TensorRT 输入/输出显存，生命周期由 TrtConeDetector 管理。
    void* d_voxels = nullptr;
    void* d_voxel_coords = nullptr;
    void* d_voxel_num_points = nullptr;
    void* d_cls_preds = nullptr;
    void* d_box_preds = nullptr;
    void* d_dir_cls_preds = nullptr;
};

} // namespace trt_cone_detector

#endif  // TRT_CONE_DETECTOR_TRT_DETECTOR_HPP_
