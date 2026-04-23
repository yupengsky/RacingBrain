#include "trt_cone_detector/post_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cuda_fp16.h>
#include <limits>

namespace trt_cone_detector {

namespace {

// =========================
// 文件内数学工具
// =========================
// clamp 用于限制异常回归值；normalizeYaw 把角度收回 [-pi, pi]。
constexpr float kPi = 3.14159265358979323846f;

float clampValue(float value, float lo, float hi) {
    return std::max(lo, std::min(value, hi));
}

float normalizeYaw(float yaw) {
    while (yaw > kPi) yaw -= 2.0f * kPi;
    while (yaw < -kPi) yaw += 2.0f * kPi;
    return yaw;
}

}  // namespace

// =========================
// 1. 后处理参数初始化
// =========================
// score_thresh 会转成 logit 阈值，这样遍历 dense 输出时可以先用 logit 快速过滤，
// 避免对大量低分候选框调用 sigmoid。
PostProcessor::PostProcessor(float score_thresh, float nms_thresh,
                             int max_pre_nms, int max_output_boxes,
                             float big_cone_score_thresh)
    : score_thresh_(clampValue(score_thresh, 1e-4f, 0.9999f)),
      score_logit_thresh_(std::log(score_thresh_ / (1.0f - score_thresh_))),
      big_cone_score_thresh_(clampValue(
          big_cone_score_thresh > 0.0f ? big_cone_score_thresh : score_thresh_, 1e-4f, 0.9999f)),
      big_cone_score_logit_thresh_(
          std::log(big_cone_score_thresh_ / (1.0f - big_cone_score_thresh_))),
      nms_thresh_(nms_thresh),
      max_pre_nms_(max_pre_nms),
      max_output_boxes_(max_output_boxes) {}

// =========================
// 2. 基础张量读取工具
// =========================
// TensorRT engine 可能输出 FP16 或 FP32，这里统一读成 float 方便后续解码。
float PostProcessor::sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

float PostProcessor::readTensorValue(const void* data, TensorDataType dtype, std::size_t index) const {
    if (data == nullptr) {
        return 0.0f;
    }

    switch (dtype) {
        case TensorDataType::kFloat32:
            return static_cast<const float*>(data)[index];
        case TensorDataType::kFloat16:
            return __half2float(static_cast<const __half*>(data)[index]);
        default:
            return 0.0f;
    }
}

// =========================
// 3. NMS 所需的鸟瞰图 IoU
// =========================
// 简化的鸟瞰图 2D IoU 计算。交通锥尺寸小、朝向影响弱，轴对齐 IoU 足够快且稳定。
float PostProcessor::computeIoU(const Bndbox& box1, const Bndbox& box2) {
    float x1_min = box1.x - box1.l / 2.0f;
    float x1_max = box1.x + box1.l / 2.0f;
    float y1_min = box1.y - box1.w / 2.0f;
    float y1_max = box1.y + box1.w / 2.0f;

    float x2_min = box2.x - box2.l / 2.0f;
    float x2_max = box2.x + box2.l / 2.0f;
    float y2_min = box2.y - box2.w / 2.0f;
    float y2_max = box2.y + box2.w / 2.0f;

    float inter_x_min = std::max(x1_min, x2_min);
    float inter_x_max = std::min(x1_max, x2_max);
    float inter_y_min = std::max(y1_min, y2_min);
    float inter_y_max = std::min(y1_max, y2_max);

    float inter_w = std::max(0.0f, inter_x_max - inter_x_min);
    float inter_h = std::max(0.0f, inter_y_max - inter_y_min);
    float inter_area = inter_w * inter_h;

    float area1 = box1.l * box1.w;
    float area2 = box2.l * box2.w;

    const float denom = area1 + area2 - inter_area;
    if (denom <= 1e-6f) {
        return 0.0f;
    }
    return inter_area / denom;
}

// =========================
// 4. 输出张量尺寸约定
// =========================
// grid_x * grid_y * anchors * channels 对应 PointPillars head 的 dense 输出布局。
std::size_t PostProcessor::expectedClsElementCount() const {
    return static_cast<std::size_t>(grid_x_) * grid_y_ * 4 * 2;
}

std::size_t PostProcessor::expectedBoxElementCount() const {
    return static_cast<std::size_t>(grid_x_) * grid_y_ * 4 * 7;
}

std::size_t PostProcessor::expectedDirElementCount() const {
    return static_cast<std::size_t>(grid_x_) * grid_y_ * 4 * 2;
}

// =========================
// 5. 网络输出解码主流程
// =========================
// 输入：TensorRT 输出的 cls_preds、box_preds、可选 dir_cls_preds。
// 输出：经过置信度过滤、尺寸合法性过滤和 NMS 后的 Bndbox 列表。
std::vector<Bndbox> PostProcessor::process(const void* cls_preds, TensorDataType cls_dtype,
                                           const void* box_preds, TensorDataType box_dtype,
                                           const void* dir_cls_preds, TensorDataType dir_dtype) {
    std::vector<Bndbox> valid_boxes;
    valid_boxes.reserve(512);

    // Anchor 定义: [l, w, h, bottom_z, rot]
    const float anchor_sizes[4][5] = {
        {0.20f, 0.20f, 0.30f, -1.0f, 0.0f},
        {0.20f, 0.20f, 0.30f, -1.0f, kPi / 2.0f},
        {0.35f, 0.35f, 0.70f, -1.0f, 0.0f},
        {0.35f, 0.35f, 0.70f, -1.0f, kPi / 2.0f}
    };

    const float anchor_diag[4] = {
        std::sqrt(anchor_sizes[0][0] * anchor_sizes[0][0] + anchor_sizes[0][1] * anchor_sizes[0][1]),
        std::sqrt(anchor_sizes[1][0] * anchor_sizes[1][0] + anchor_sizes[1][1] * anchor_sizes[1][1]),
        std::sqrt(anchor_sizes[2][0] * anchor_sizes[2][0] + anchor_sizes[2][1] * anchor_sizes[2][1]),
        std::sqrt(anchor_sizes[3][0] * anchor_sizes[3][0] + anchor_sizes[3][1] * anchor_sizes[3][1])
    };

    const int num_anchors = 4;
    const int num_classes = 2; 

    // ---------- 5.1 遍历特征图和 anchor ----------
    // 每个 grid cell 有 4 个 anchor，每个 anchor 预测 2 类分数和 7 个 bbox 回归量。
    for (int y = 0; y < grid_y_; ++y) {
        float anchor_y = min_y_ + voxel_y_ / 2.0f + y * voxel_y_ * stride_;
        for (int x = 0; x < grid_x_; ++x) {
            float anchor_x = min_x_ + voxel_x_ / 2.0f + x * voxel_x_ * stride_;
            int grid_offset = (y * grid_x_ + x);

            for (int a = 0; a < num_anchors; ++a) {
                float a_l = anchor_sizes[a][0];
                float a_w = anchor_sizes[a][1];
                float a_h = anchor_sizes[a][2];
                float a_bottom_z = anchor_sizes[a][3];
                float a_rot = anchor_sizes[a][4];
                float d_a = anchor_diag[a];

                // Anchor 的 z 在配置里通常是 bottom_z，解码时要转换为中心 z。
                float a_z = a_bottom_z + a_h / 2.0f;

                // ---------- 5.2 分类分支 ----------
                // 先用 logit 阈值过滤低分候选，再计算 sigmoid 得到最终置信度。
                int cls_offset = grid_offset * (num_anchors * num_classes) + a * num_classes;
                const float class_logits[2] = {
                    readTensorValue(cls_preds, cls_dtype, cls_offset + 0),
                    readTensorValue(cls_preds, cls_dtype, cls_offset + 1)
                };

                // ---------- 5.3 框回归分支 ----------
                // dx/dy/dz/dl/dw/dh/dyaw 是相对 anchor 的编码值。
                int box_offset = grid_offset * (num_anchors * 7) + a * 7;
                float dx = readTensorValue(box_preds, box_dtype, box_offset + 0);
                float dy = readTensorValue(box_preds, box_dtype, box_offset + 1);
                float dz = readTensorValue(box_preds, box_dtype, box_offset + 2);
                float dl = readTensorValue(box_preds, box_dtype, box_offset + 3);
                float dw = readTensorValue(box_preds, box_dtype, box_offset + 4);
                float dh = readTensorValue(box_preds, box_dtype, box_offset + 5);
                float dyaw = readTensorValue(box_preds, box_dtype, box_offset + 6);

                if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
                    !std::isfinite(dl) || !std::isfinite(dw) || !std::isfinite(dh) ||
                    !std::isfinite(dyaw)) {
                    continue;
                }

                Bndbox box;
                box.x = dx * d_a + anchor_x;
                box.y = dy * d_a + anchor_y;
                box.z = dz * a_h + a_z;
                box.l = std::exp(clampValue(dl, -4.0f, 4.0f)) * a_l;
                box.w = std::exp(clampValue(dw, -4.0f, 4.0f)) * a_w;
                box.h = std::exp(clampValue(dh, -4.0f, 4.0f)) * a_h;
                box.yaw = normalizeYaw(dyaw + a_rot);

                // ---------- 5.4 方向分类分支 ----------
                // dir_cls_preds 用于修正 yaw 的 pi 周期歧义；有些 engine 可能没有这个输出。
                if (dir_cls_preds != nullptr) {
                    int dir_offset = grid_offset * (num_anchors * 2) + a * 2;
                    float dir_score0 = readTensorValue(dir_cls_preds, dir_dtype, dir_offset + 0);
                    float dir_score1 = readTensorValue(dir_cls_preds, dir_dtype, dir_offset + 1);
                    if (dir_score1 > dir_score0) {
                        box.yaw = normalizeYaw(box.yaw + kPi);
                    }
                }

                // ---------- 5.5 几何合法性过滤 ----------
                // 防止异常回归值生成过小/过大的框影响 NMS 和下游模块。
                if (box.l < 0.03f || box.l > 2.0f ||
                    box.w < 0.03f || box.w > 2.0f ||
                    box.h < 0.05f || box.h > 2.5f) {
                    continue;
                }

                // ---------- 5.6 类别独立候选生成 ----------
                // OpenPCDet 的分类头使用 sigmoid，每个类别分数应独立判断。
                // 如果只取 max 类别，BigCone 分数略低于 Cone 时会被直接丢掉。
                for (int label = 0; label < num_classes; ++label) {
                    const float logit_thresh =
                        label == 1 ? big_cone_score_logit_thresh_ : score_logit_thresh_;
                    if (class_logits[label] < logit_thresh) {
                        continue;
                    }

                    Bndbox class_box = box;
                    class_box.score = sigmoid(class_logits[label]);
                    class_box.label = label;
                    valid_boxes.push_back(class_box);
                }
            }
        }
    }

    // ---------- 5.7 NMS 前排序与候选截断 ----------
    // 按类别分别保留高分候选，避免小锥桶候选过多时把 BigCone 挤出 topK。
    const auto score_desc = [](const Bndbox& a, const Bndbox& b) { return a.score > b.score; };
    std::vector<Bndbox> pre_nms_boxes;
    pre_nms_boxes.reserve(valid_boxes.size());
    for (int label = 0; label < num_classes; ++label) {
        std::vector<Bndbox> class_boxes;
        class_boxes.reserve(valid_boxes.size());
        for (const auto& box : valid_boxes) {
            if (box.label == label) {
                class_boxes.push_back(box);
            }
        }

        if (max_pre_nms_ > 0 && class_boxes.size() > static_cast<size_t>(max_pre_nms_)) {
            auto cut = class_boxes.begin() + max_pre_nms_;
            std::partial_sort(class_boxes.begin(), cut, class_boxes.end(), score_desc);
            class_boxes.erase(cut, class_boxes.end());
        } else {
            std::sort(class_boxes.begin(), class_boxes.end(), score_desc);
        }

        pre_nms_boxes.insert(pre_nms_boxes.end(), class_boxes.begin(), class_boxes.end());
    }
    valid_boxes.swap(pre_nms_boxes);
    std::sort(valid_boxes.begin(), valid_boxes.end(), score_desc);

    std::vector<Bndbox> final_boxes;
    std::size_t reserve_count = valid_boxes.size();
    if (max_output_boxes_ > 0) {
        reserve_count = std::min<std::size_t>(reserve_count, static_cast<std::size_t>(max_output_boxes_));
    }
    final_boxes.reserve(reserve_count);
    std::vector<bool> suppressed(valid_boxes.size(), false);

    // ---------- 5.8 同类别 NMS ----------
    // 同类且 IoU 大于阈值时抑制低分框；不同类别互不抑制。
    for (size_t i = 0; i < valid_boxes.size(); ++i) {
        if (suppressed[i]) continue;
        final_boxes.push_back(valid_boxes[i]);
        if (max_output_boxes_ > 0 && final_boxes.size() >= static_cast<size_t>(max_output_boxes_)) {
            break;
        }

        for (size_t j = i + 1; j < valid_boxes.size(); ++j) {
            if (suppressed[j]) continue;
            // 如果类别不同，或者 IoU 小于阈值，则保留
            if (valid_boxes[i].label == valid_boxes[j].label && computeIoU(valid_boxes[i], valid_boxes[j]) > nms_thresh_) {
                suppressed[j] = true;
            }
        }
    }

    return final_boxes;
}

} // namespace trt_cone_detector
