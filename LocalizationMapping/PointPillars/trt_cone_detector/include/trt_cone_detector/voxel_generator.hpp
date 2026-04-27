#ifndef TRT_CONE_DETECTOR_VOXEL_GENERATOR_HPP_
#define TRT_CONE_DETECTOR_VOXEL_GENERATOR_HPP_

#include <cuda_runtime_api.h>
#include <iostream>

namespace trt_cone_detector {

// =========================
// GPU 体素化模块
// =========================
// 作用：把预过滤后的点云 [N, 4] 切成 PointPillars 输入需要的
// voxels / voxel_coords / voxel_num_points 三个张量。
class VoxelGenerator {
public:
    VoxelGenerator();
    ~VoxelGenerator();

    // 接收一维的原始点云数组，并执行 CUDA 核函数进行切分。
    // points: 原始点云显存指针 [N, 4]
    // num_points: 原始点云的数量
    // d_voxels, d_voxel_coords, d_voxel_num_points: TensorRT 的输入显存指针
    // voxels_are_fp16: 根据 engine 输入 dtype 决定写 float 还是 half
    // stream/voxel_done_event: 复用推理 stream，并可记录 GPU 计时事件
    // 返回值: 实际生成的有效 Voxel 数量
    int generate(const float* points, int num_points, 
                 void* d_voxels, void* d_voxel_coords, void* d_voxel_num_points,
                 bool voxels_are_fp16 = false, cudaStream_t stream = nullptr,
                 cudaEvent_t voxel_done_event = nullptr);

private:
    // 点云物理范围：和训练/导出配置保持一致。
    const float min_x_ = 0.0f;
    const float max_x_ = 20.0f;
    const float min_y_ = -20.0f;
    const float max_y_ = 20.0f;
    const float min_z_ = -3.0f;
    const float max_z_ = 1.0f;

    // 单个 voxel 的物理尺寸。
    const float voxel_x_ = 0.1f;
    const float voxel_y_ = 0.1f;
    const float voxel_z_ = 4.0f;

    // 网格尺寸：(max - min) / voxel_size。
    const int grid_x_ = 200;
    const int grid_y_ = 400;
    const int grid_z_ = 1;

    // TensorRT 输入容量上限。
    const int max_voxels_ = 40000;
    const int max_points_per_voxel_ = 32;

    // CUDA 内部工作空间指针
    int* d_voxel_idx_grid_ = nullptr; // 用于记录网格中是否已经存在体素 [80000]
    int* d_voxel_count_ = nullptr;    // 用于统计当前生成的总 voxel 数量 [1]
};

} // namespace trt_cone_detector

#endif // TRT_CONE_DETECTOR_VOXEL_GENERATOR_HPP_
