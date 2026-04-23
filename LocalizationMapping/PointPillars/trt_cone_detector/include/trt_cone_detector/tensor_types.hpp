#ifndef TRT_CONE_DETECTOR_TENSOR_TYPES_HPP_
#define TRT_CONE_DETECTOR_TENSOR_TYPES_HPP_

#include <cstddef>

namespace trt_cone_detector {

// TensorRT 输出/输入张量的数据类型抽象。
// 作用：把 nvinfer1::DataType 转成项目内部统一类型，便于分配内存和读取结果。
enum class TensorDataType {
    kFloat32,
    kFloat16,
    kInt32,
    kInt8,
    kBool,
    kUnknown
};

// 根据张量类型返回单个元素占用字节数，用于 cudaMalloc/cudaHostAlloc。
inline std::size_t tensorDataTypeSize(TensorDataType dtype) {
    switch (dtype) {
        case TensorDataType::kFloat32:
        case TensorDataType::kInt32:
            return 4;
        case TensorDataType::kFloat16:
            return 2;
        case TensorDataType::kInt8:
        case TensorDataType::kBool:
            return 1;
        case TensorDataType::kUnknown:
        default:
            return 0;
    }
}

// 返回可读类型名，用于初始化日志和错误信息。
inline const char* tensorDataTypeName(TensorDataType dtype) {
    switch (dtype) {
        case TensorDataType::kFloat32:
            return "float32";
        case TensorDataType::kFloat16:
            return "float16";
        case TensorDataType::kInt32:
            return "int32";
        case TensorDataType::kInt8:
            return "int8";
        case TensorDataType::kBool:
            return "bool";
        case TensorDataType::kUnknown:
        default:
            return "unknown";
    }
}

}  // namespace trt_cone_detector

#endif  // TRT_CONE_DETECTOR_TENSOR_TYPES_HPP_
