#include "trt_cone_detector/trt_detector.hpp"

#include <cassert>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace trt_cone_detector {

namespace {

// =========================
// 文件内辅助函数
// =========================
// 这些函数只在本实现文件中使用：CUDA 错误检查、TensorRT dtype 转换、
// shape/字节数计算和日志格式化。
void checkCuda(cudaError_t status, const char* what) {
    if (status != cudaSuccess) {
        std::ostringstream oss;
        oss << what << ": " << cudaGetErrorString(status);
        throw std::runtime_error(oss.str());
    }
}

void checkCudaRuntimeAvailable() {
    int device_count = 0;
    cudaError_t status = cudaGetDeviceCount(&device_count);
    if (status != cudaSuccess) {
        std::ostringstream oss;
        oss << "CUDA runtime unavailable: cudaGetDeviceCount failed: "
            << cudaGetErrorString(status);
        throw std::runtime_error(oss.str());
    }
    if (device_count <= 0) {
        throw std::runtime_error("CUDA runtime unavailable: no CUDA devices found");
    }
    checkCuda(cudaSetDevice(0), "cudaSetDevice 0");
}

TensorDataType fromTrtDataType(nvinfer1::DataType dtype) {
    switch (dtype) {
        case nvinfer1::DataType::kFLOAT:
            return TensorDataType::kFloat32;
        case nvinfer1::DataType::kHALF:
            return TensorDataType::kFloat16;
        case nvinfer1::DataType::kINT32:
            return TensorDataType::kInt32;
        case nvinfer1::DataType::kINT8:
            return TensorDataType::kInt8;
        case nvinfer1::DataType::kBOOL:
            return TensorDataType::kBool;
        default:
            return TensorDataType::kUnknown;
    }
}

std::size_t staticElementCount(const nvinfer1::Dims& dims) {
    if (dims.nbDims <= 0) {
        return 0;
    }

    std::size_t count = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] <= 0) {
            return 0;
        }
        count *= static_cast<std::size_t>(dims.d[i]);
    }
    return count;
}

std::string dimsToString(const nvinfer1::Dims& dims) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < dims.nbDims; ++i) {
        if (i > 0) {
            oss << "x";
        }
        oss << dims.d[i];
    }
    oss << "]";
    return oss.str();
}

std::size_t inputBytes(int max_voxels, int dims1, int dims2, TensorDataType dtype, TensorDataType fallback) {
    TensorDataType actual_dtype = dtype == TensorDataType::kUnknown ? fallback : dtype;
    return static_cast<std::size_t>(max_voxels) * dims1 * dims2 * tensorDataTypeSize(actual_dtype);
}

}  // namespace

// =========================
// 1. Engine 加载与张量元信息解析
// =========================
// 从磁盘读取序列化 TensorRT engine，创建 runtime/engine/context，
// 并缓存所有输入输出张量的名称、shape、dtype 和字节数。
TrtConeDetector::TrtConeDetector(const std::string& engine_path) {
    checkCudaRuntimeAvailable();

    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        throw std::runtime_error("Cannot open engine file: " + engine_path);
    }
    file.seekg(0, std::ifstream::end);
    size_t size = file.tellg();
    file.seekg(0, std::ifstream::beg);
    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);
    file.close();

    m_runtime = nvinfer1::createInferRuntime(m_logger);
    if (m_runtime == nullptr) {
        throw std::runtime_error("Failed to create TensorRT runtime");
    }
    m_engine = m_runtime->deserializeCudaEngine(engine_data.data(), size);
    if (m_engine == nullptr) {
        throw std::runtime_error("Failed to deserialize TensorRT engine: " + engine_path);
    }

    m_context = m_engine->createExecutionContext();
    if (m_context == nullptr) {
        throw std::runtime_error("Failed to create TensorRT execution context");
    }

    // 遍历 engine 的 I/O tensor，后续分配显存和拷贝输出都依赖这些元信息。
    for (int i = 0; i < m_engine->getNbIOTensors(); ++i) {
        const char* name = m_engine->getIOTensorName(i);
        if (name == nullptr) {
            continue;
        }

        TensorInfo info;
        info.dtype = fromTrtDataType(m_engine->getTensorDataType(name));
        info.shape = m_engine->getTensorShape(name);
        info.is_input = m_engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
        info.element_count = staticElementCount(info.shape);
        info.byte_size = info.element_count * tensorDataTypeSize(info.dtype);
        tensor_info_[name] = info;

        std::cout << "=> Tensor " << (info.is_input ? "IN  " : "OUT ") << name
                  << " dtype=" << tensorDataTypeName(info.dtype)
                  << " shape=" << dimsToString(info.shape) << std::endl;
    }

    // 本节点假设 engine 使用 PointPillars 常见张量名；名称不匹配说明模型导出配置不同。
    if (!hasTensor("voxels") || !hasTensor("voxel_coords") || !hasTensor("voxel_num_points") ||
        !hasTensor("cls_preds") || !hasTensor("box_preds")) {
        throw std::runtime_error("Engine tensor names do not match expected PointPillars names");
    }

    checkCuda(cudaStreamCreateWithFlags(&m_stream, cudaStreamNonBlocking), "cudaStreamCreateWithFlags");
    std::cout << "=> TensorRT Engine loaded successfully from: " << engine_path << std::endl;
}

// =========================
// 2. TensorRT 输入/输出显存分配
// =========================
// 输入张量由 voxel_generator.cu 写入；输出张量由 TensorRT 写入，再由节点拷回 CPU。
void TrtConeDetector::allocateMemory(int max_voxels) {
    max_voxels_ = max_voxels;

    const TensorDataType voxel_dtype = getTensorDataType("voxels");
    const TensorDataType coord_dtype = getTensorDataType("voxel_coords");
    const TensorDataType num_points_dtype = getTensorDataType("voxel_num_points");

    checkCuda(cudaMalloc(&d_voxels, inputBytes(max_voxels, 32, 4, voxel_dtype, TensorDataType::kFloat32)),
              "cudaMalloc voxels");
    checkCuda(cudaMalloc(&d_voxel_coords, inputBytes(max_voxels, 4, 1, coord_dtype, TensorDataType::kInt32)),
              "cudaMalloc voxel_coords");
    checkCuda(cudaMalloc(&d_voxel_num_points, inputBytes(max_voxels, 1, 1, num_points_dtype, TensorDataType::kInt32)),
              "cudaMalloc voxel_num_points");

    // Engine shape 为静态时直接使用 byte_size；动态 shape 时使用已知网格输出尺寸兜底。
    std::size_t cls_bytes = getTensorByteSize("cls_preds");
    std::size_t box_bytes = getTensorByteSize("box_preds");
    if (cls_bytes == 0) {
        cls_bytes = static_cast<std::size_t>(200 * 100 * 8) *
                    tensorDataTypeSize(getTensorDataType("cls_preds"));
    }
    if (box_bytes == 0) {
        box_bytes = static_cast<std::size_t>(200 * 100 * 28) *
                    tensorDataTypeSize(getTensorDataType("box_preds"));
    }

    checkCuda(cudaMalloc(&d_cls_preds, cls_bytes), "cudaMalloc cls_preds");
    checkCuda(cudaMalloc(&d_box_preds, box_bytes), "cudaMalloc box_preds");

    if (hasTensor("dir_cls_preds")) {
        std::size_t dir_bytes = getTensorByteSize("dir_cls_preds");
        if (dir_bytes == 0) {
            dir_bytes = static_cast<std::size_t>(200 * 100 * 8) *
                        tensorDataTypeSize(getTensorDataType("dir_cls_preds"));
        }
        checkCuda(cudaMalloc(&d_dir_cls_preds, dir_bytes), "cudaMalloc dir_cls_preds");
    }

    std::cout << "=> GPU memory allocated for max " << max_voxels
              << " voxels. voxels dtype=" << tensorDataTypeName(voxel_dtype) << std::endl;
}

// =========================
// 3. 单帧推理提交
// =========================
// current_voxel_num 是本帧真实 voxel 数。动态 engine 需要先设置输入 shape，
// 然后绑定张量地址，最后通过 enqueueV3 投递到同一个 CUDA stream。
bool TrtConeDetector::doInference(int current_voxel_num, bool synchronize) {
    if (current_voxel_num <= 0) {
        return false;
    }

    if (max_voxels_ > 0 && current_voxel_num > max_voxels_) {
        current_voxel_num = max_voxels_;
    }

    // 动态 voxel 维度必须在每帧推理前更新，否则 TensorRT 不知道本帧有效输入长度。
    bool ok = true;
    ok &= setInputShapeIfDynamic("voxels", current_voxel_num);
    ok &= setInputShapeIfDynamic("voxel_coords", current_voxel_num);
    ok &= setInputShapeIfDynamic("voxel_num_points", current_voxel_num);
    if (!ok) {
        std::cout << "[TRT] Failed to set dynamic input shapes for " << current_voxel_num
                  << " voxels" << std::endl;
        return false;
    }

    // TensorRT 10 的 enqueueV3 使用按名称绑定 tensor address。
    m_context->setTensorAddress("voxels", d_voxels);
    m_context->setTensorAddress("voxel_coords", d_voxel_coords);
    m_context->setTensorAddress("voxel_num_points", d_voxel_num_points);
    m_context->setTensorAddress("cls_preds", d_cls_preds);
    m_context->setTensorAddress("box_preds", d_box_preds);
    
    if (hasTensor("dir_cls_preds") && d_dir_cls_preds != nullptr) {
        m_context->setTensorAddress("dir_cls_preds", d_dir_cls_preds);
    }

    if (!m_context->enqueueV3(m_stream)) {
        std::cout << "[TRT] enqueueV3 failed" << std::endl;
        return false;
    }

    if (synchronize) {
        checkCuda(cudaStreamSynchronize(m_stream), "cudaStreamSynchronize inference");
    }

    return true;
}

// =========================
// 4. 张量指针与元信息查询接口
// =========================
// 这些接口供 ROS 节点连接 voxel 生成、TensorRT 推理和输出拷贝三个阶段。
void* TrtConeDetector::getDevicePtr(const std::string& tensor_name) {
    if (tensor_name == "voxels") return d_voxels;
    if (tensor_name == "voxel_coords") return d_voxel_coords;
    if (tensor_name == "voxel_num_points") return d_voxel_num_points;
    if (tensor_name == "cls_preds") return d_cls_preds;
    if (tensor_name == "box_preds") return d_box_preds;
    if (tensor_name == "dir_cls_preds") return d_dir_cls_preds;
    return nullptr;
}

bool TrtConeDetector::hasTensor(const std::string& tensor_name) const {
    return tensor_info_.find(tensor_name) != tensor_info_.end();
}

bool TrtConeDetector::tensorIsFloat16(const std::string& tensor_name) const {
    return getTensorDataType(tensor_name) == TensorDataType::kFloat16;
}

// =========================
// 5. 动态 shape 处理
// =========================
// 如果导出的 engine 中某些输入维度为 -1，这里把它们替换为运行时实际值。
bool TrtConeDetector::tensorHasDynamicShape(const std::string& tensor_name) const {
    auto it = tensor_info_.find(tensor_name);
    if (it == tensor_info_.end()) {
        return false;
    }

    const nvinfer1::Dims& dims = it->second.shape;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] < 0) {
            return true;
        }
    }
    return false;
}

nvinfer1::Dims TrtConeDetector::makeRuntimeInputShape(
    const std::string& tensor_name, int current_voxel_num) const {
    auto it = tensor_info_.find(tensor_name);
    if (it == tensor_info_.end()) {
        return nvinfer1::Dims{};
    }

    nvinfer1::Dims dims = it->second.shape;

    // 多个动态维度通常表示 [B, N, ...]：本节点固定 batch=1，N 使用 current_voxel_num。
    int dynamic_dim_count = 0;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] < 0) {
            ++dynamic_dim_count;
        }
    }

    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] >= 0) {
            continue;
        }

        // Some exported engines include a dynamic batch axis, e.g. [B, N, 32, 4].
        // This node always runs batch 1; the voxel axis gets current_voxel_num.
        if (i == 0 && dims.nbDims > 1 && dynamic_dim_count > 1) {
            dims.d[i] = 1;
        } else {
            dims.d[i] = current_voxel_num;
        }
    }

    return dims;
}

bool TrtConeDetector::setInputShapeIfDynamic(
    const std::string& tensor_name, int current_voxel_num) {
    if (!tensorHasDynamicShape(tensor_name)) {
        return true;
    }

    const nvinfer1::Dims dims = makeRuntimeInputShape(tensor_name, current_voxel_num);
    if (!m_context->setInputShape(tensor_name.c_str(), dims)) {
        std::cout << "[TRT] Failed to set input shape for " << tensor_name
                  << " to " << dimsToString(dims) << std::endl;
        return false;
    }
    return true;
}

// =========================
// 6. Tensor 元信息读取
// =========================
TensorDataType TrtConeDetector::getTensorDataType(const std::string& tensor_name) const {
    auto it = tensor_info_.find(tensor_name);
    if (it == tensor_info_.end()) {
        return TensorDataType::kUnknown;
    }
    return it->second.dtype;
}

std::size_t TrtConeDetector::getTensorElementCount(const std::string& tensor_name) const {
    auto it = tensor_info_.find(tensor_name);
    if (it == tensor_info_.end()) {
        return 0;
    }
    return it->second.element_count;
}

std::size_t TrtConeDetector::getTensorByteSize(const std::string& tensor_name) const {
    auto it = tensor_info_.find(tensor_name);
    if (it == tensor_info_.end()) {
        return 0;
    }
    return it->second.byte_size;
}

// =========================
// 7. 资源释放
// =========================
// 析构时释放 CUDA stream、TensorRT 对象和所有 device buffer。
TrtConeDetector::~TrtConeDetector() {
    if (m_stream) cudaStreamDestroy(m_stream);
    if (m_context) delete m_context;
    if (m_engine) delete m_engine;
    if (m_runtime) delete m_runtime;

    cudaFree(d_voxels);
    cudaFree(d_voxel_coords);
    cudaFree(d_voxel_num_points);
    cudaFree(d_cls_preds);
    cudaFree(d_box_preds);
    cudaFree(d_dir_cls_preds);
}

} // namespace trt_cone_detector
