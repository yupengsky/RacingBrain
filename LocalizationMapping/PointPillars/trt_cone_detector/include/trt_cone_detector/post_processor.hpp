#ifndef TRT_CONE_DETECTOR_POST_PROCESSOR_HPP_
#define TRT_CONE_DETECTOR_POST_PROCESSOR_HPP_

#include <cstddef>
#include <vector>

#include "trt_cone_detector/tensor_types.hpp"

namespace trt_cone_detector {

// =========================
// 1. 检测框数据结构
// =========================
// PostProcessor 的最终输出：中心点、尺寸、朝向、置信度和类别。
struct Bndbox {
    float x, y, z, l, w, h, yaw;
    float score;
    int label; // 0: Cone (小锥桶), 1: BigCone (大锥桶)
};

// =========================
// 2. CPU 后处理模块
// =========================
// 作用：把 TensorRT 的 dense 预测张量解码成 3D 框，并执行阈值过滤和 NMS。
class PostProcessor {
public:
    PostProcessor(float score_thresh, float nms_thresh,
                  int max_pre_nms = 1024, int max_output_boxes = 200,
                  float big_cone_score_thresh = -1.0f);
    ~PostProcessor() = default;

    // 核心处理函数：传入从 GPU 拷贝回来的 CPU 指针，返回最终检测框。
    std::vector<Bndbox> process(const void* cls_preds, TensorDataType cls_dtype,
                                const void* box_preds, TensorDataType box_dtype,
                                const void* dir_cls_preds, TensorDataType dir_dtype);

    // 输出张量期望元素数量：节点据此校验/分配 pinned host memory。
    std::size_t expectedClsElementCount() const;
    std::size_t expectedBoxElementCount() const;
    std::size_t expectedDirElementCount() const;

private:
    float score_thresh_;
    float score_logit_thresh_;
    float big_cone_score_thresh_;
    float big_cone_score_logit_thresh_;
    float nms_thresh_;
    int max_pre_nms_;
    int max_output_boxes_;

    // 特征图维度 (20.0 - 0) / 0.1 / stride(2) = 100
    const int grid_x_ = 100;
    const int grid_y_ = 200;

    // 锚框参数 (由 yaml 提取)
    const float min_x_ = 0.0f;
    const float min_y_ = -20.0f;
    const float voxel_x_ = 0.1f;
    const float voxel_y_ = 0.1f;
    const int stride_ = 2;

    // 计算两个 2D 框(鸟瞰图)的 IoU，用于 NMS
    float computeIoU(const Bndbox& box1, const Bndbox& box2);
    // Sigmoid 激活函数
    float sigmoid(float x);
    // 按实际 dtype 读取 TensorRT 输出，兼容 FP32/FP16 engine。
    float readTensorValue(const void* data, TensorDataType dtype, std::size_t index) const;
};

} // namespace trt_cone_detector

#endif // TRT_CONE_DETECTOR_POST_PROCESSOR_HPP_
