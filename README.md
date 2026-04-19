# DRd26_SLAM Skeleton

这个分支被重置成“从零复刻”骨架，只保留环境壳、支持文件、`agent/`
说明文档，以及一套你可以自己逐步补完的注释式目录结构。

## 目标链路

从数据集开始，完整走一遍感知 + SLAM，到点云导出为止：

1. 数据集准备与路径配置
2. 相机锥桶数据集整理 / 训练数据导出
3. 相机锥桶检测
4. LiDAR 点云分割 / 锥桶提取
5. 相机 + LiDAR 融合
6. GNSS / INS 轨迹与姿态接入
7. SLAM 前端关联、后端地图维护、回环
8. 全局地图转点云并导出 `.pcd` / `.ply`

## 保留内容

- `agent/`: 原工程的说明、接口、运行笔记和长文档
- `config/`: 外部模型、数据集、导出路径模板
- `scripts/`: 环境准备、构建、联调脚本的占位壳
- `gnss/`, `perception/`, `slam/`: 与原工程一致的包结构，但实现已清空

## 建议重建顺序

1. 先填 `config/hardcoded_paths.ini`
2. 再补 `gnss/gnss_ins_msg` 和 `slam/drd25_msgs` 的消息定义
3. 实现 `perception/src/cone_ws` 的数据集与 YOLO 检测
4. 实现 `perception/src/cone_segmentation_test_3d` 的点云锥桶分割
5. 实现 `perception/src/fs_fusion_box` 的多传感器融合
6. 实现 `slam/slam` 的定位、关联、建图、回环、点云导出
7. 最后把 `scripts/run_dataset_slam_chain.sh` 串起来

## 备注

当前仓库不保证可直接构建运行，因为所有核心实现都已被替换成注释骨架。
这个分支的作用是给你一条清晰、干净、可逐段回填的复刻路线。
