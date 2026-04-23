#include "trt_cone_detector/voxel_generator.hpp"
#include <cuda_fp16.h>
#include <math.h>

namespace trt_cone_detector {

// =========================
// 1. CUDA kernel：点云并行体素化
// =========================
// 每个线程处理一个点：
// 1) 过滤越界点；
// 2) 计算点所在 voxel 网格坐标；
// 3) 用原子操作为新 voxel 分配编号；
// 4) 将点写入 voxels，并更新 voxel_num_points。
__global__ void build_voxels_kernel(
    const float* points, int num_points,
    float min_x, float max_x, float min_y, float max_y, float min_z, float max_z,
    float voxel_x, float voxel_y, float voxel_z,
    int grid_x, int grid_y, int grid_z,
    int max_voxels, int max_points_per_voxel,
    int* voxel_idx_grid,
    void* voxels,
    int* voxel_coords,
    int* voxel_num_points,
    int* voxel_count,
    bool voxels_are_fp16
) {
    int pt_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (pt_idx >= num_points) return;

    float x = points[pt_idx * 4 + 0];
    float y = points[pt_idx * 4 + 1];
    float z = points[pt_idx * 4 + 2];
    float i = points[pt_idx * 4 + 3];

    // ---------- 1.1 过滤 NaN 和训练范围外的点 ----------
    if (isnan(x) || isnan(y) || isnan(z)) return;
    if (x < min_x || x >= max_x || y < min_y || y >= max_y || z < min_z || z >= max_z) return;

    // ---------- 1.2 计算 voxel 网格坐标 ----------
    // grid_idx 是把 3D 网格压平成一维后的索引，用于查询/创建 voxel。
    int voxel_idx_x = floor((x - min_x) / voxel_x);
    int voxel_idx_y = floor((y - min_y) / voxel_y);
    int voxel_idx_z = floor((z - min_z) / voxel_z);

    int grid_idx = voxel_idx_z * (grid_y * grid_x) + voxel_idx_y * grid_x + voxel_idx_x;

    // ---------- 1.3 为新 voxel 分配编号 ----------
    // voxel_idx_grid 的状态：
    // -1 表示该网格还没有 voxel；-2 表示其他线程正在初始化；>=0 表示 voxel id。
    // volatile 配合 CAS 自旋锁，避免多个线程同时给同一个网格创建不同 voxel。
    volatile int* grid_ptr = &voxel_idx_grid[grid_idx];
    int v_idx = *grid_ptr;
    
    if (v_idx == -1) {
        // 尝试抢占这块网格：如果状态是 -1，就改成 -2 并获得初始化权。
        int old_idx = atomicCAS((int*)grid_ptr, -1, -2);
        if (old_idx == -1) {
            // 抢占成功后申请新的 voxel id，并写入 batch/z/y/x 坐标。
            int new_idx = atomicAdd(voxel_count, 1);
            if (new_idx < max_voxels) {
                voxel_coords[new_idx * 4 + 0] = 0;
                voxel_coords[new_idx * 4 + 1] = voxel_idx_z;
                voxel_coords[new_idx * 4 + 2] = voxel_idx_y;
                voxel_coords[new_idx * 4 + 3] = voxel_idx_x;
            }
            // 解锁：写入真实 voxel id，让等待线程继续使用同一个 voxel。
            atomicExch((int*)grid_ptr, new_idx);
            v_idx = new_idx;
        } else {
            // 其他线程正在初始化，等待状态变成真实 voxel id。
            while ((v_idx = *grid_ptr) < 0) {}
        }
    } else if (v_idx == -2) {
        // 刚进入时就发现该网格被锁，也等待初始化完成。
        while ((v_idx = *grid_ptr) < 0) {}
    }

    // ---------- 1.4 容量保护 ----------
    // 超过 max_voxels 的 voxel 不写入 TensorRT 输入，避免越界。
    if (v_idx >= max_voxels) return;

    // ---------- 1.5 写入 voxel 点数据 ----------
    // OpenPCDet 训练时 num_points 会被截断到 max_points_per_voxel；
    // 如果这里继续累加，VFE 会用过大的点数做归一化，因此超出点数要回退计数。
    int pt_count = atomicAdd(&voxel_num_points[v_idx], 1);
    if (pt_count < max_points_per_voxel) {
        int base_offset = v_idx * max_points_per_voxel * 4 + pt_count * 4;
        if (voxels_are_fp16) {
            __half* half_voxels = reinterpret_cast<__half*>(voxels);
            half_voxels[base_offset + 0] = __float2half_rn(x);
            half_voxels[base_offset + 1] = __float2half_rn(y);
            half_voxels[base_offset + 2] = __float2half_rn(z);
            half_voxels[base_offset + 3] = __float2half_rn(i);
        } else {
            float* float_voxels = reinterpret_cast<float*>(voxels);
            float_voxels[base_offset + 0] = x;
            float_voxels[base_offset + 1] = y;
            float_voxels[base_offset + 2] = z;
            float_voxels[base_offset + 3] = i;
        }
    } else {
        atomicSub(&voxel_num_points[v_idx], 1);
    }
}

// =========================
// 2. VoxelGenerator 生命周期
// =========================
// d_voxel_idx_grid_ 是稀疏网格到 voxel id 的映射；d_voxel_count_ 是全局 voxel 计数器。
VoxelGenerator::VoxelGenerator() {
    cudaMalloc(&d_voxel_idx_grid_, grid_x_ * grid_y_ * grid_z_ * sizeof(int));
    cudaMalloc(&d_voxel_count_, sizeof(int));
}

VoxelGenerator::~VoxelGenerator() {
    cudaFree(d_voxel_idx_grid_);
    cudaFree(d_voxel_count_);
}

// =========================
// 3. 单帧体素生成入口
// =========================
// 先清空工作区和 TensorRT 输入，再启动 kernel，最后把实际 voxel 数拷回 CPU。
int VoxelGenerator::generate(const float* points, int num_points, 
                             void* d_voxels, void* d_voxel_coords, void* d_voxel_num_points,
                             bool voxels_are_fp16, cudaStream_t stream,
                             cudaEvent_t voxel_done_event) {
    cudaStream_t work_stream = stream;
    // ---------- 3.1 清空网格映射、计数器和输出张量 ----------
    cudaMemsetAsync(d_voxel_idx_grid_, -1, grid_x_ * grid_y_ * grid_z_ * sizeof(int), work_stream);
    cudaMemsetAsync(d_voxel_count_, 0, sizeof(int), work_stream);
    cudaMemsetAsync(d_voxel_coords, 0, max_voxels_ * 4 * sizeof(int), work_stream);
    cudaMemsetAsync(d_voxel_num_points, 0, max_voxels_ * sizeof(int), work_stream);
    const size_t voxel_element_size = voxels_are_fp16 ? sizeof(__half) : sizeof(float);
    cudaMemsetAsync(d_voxels, 0, max_voxels_ * max_points_per_voxel_ * 4 * voxel_element_size, work_stream);

    int threads_per_block = 256;
    int blocks_per_grid = (num_points + threads_per_block - 1) / threads_per_block;

    // ---------- 3.2 启动 CUDA kernel ----------
    build_voxels_kernel<<<blocks_per_grid, threads_per_block, 0, work_stream>>>(
        points, num_points,
        min_x_, max_x_, min_y_, max_y_, min_z_, max_z_,
        voxel_x_, voxel_y_, voxel_z_,
        grid_x_, grid_y_, grid_z_,
        max_voxels_, max_points_per_voxel_,
        d_voxel_idx_grid_,
        (float*)d_voxels, (int*)d_voxel_coords, (int*)d_voxel_num_points,
        d_voxel_count_,
        voxels_are_fp16
    );

    cudaError_t kernel_status = cudaGetLastError();
    if (kernel_status != cudaSuccess) {
        std::cerr << "build_voxels_kernel launch failed: " << cudaGetErrorString(kernel_status) << std::endl;
        return 0;
    }

    // ---------- 3.3 回读 voxel 数并同步 ----------
    // TensorRT 推理需要知道本帧真实 voxel 数；这里同步保证后续推理看到完整输入。
    int actual_voxel_count = 0;
    cudaMemcpyAsync(&actual_voxel_count, d_voxel_count_, sizeof(int), cudaMemcpyDeviceToHost, work_stream);
    if (voxel_done_event != nullptr) {
        cudaEventRecord(voxel_done_event, work_stream);
    }
    cudaStreamSynchronize(work_stream);

    return actual_voxel_count > max_voxels_ ? max_voxels_ : actual_voxel_count;
}

} // namespace trt_cone_detector
