# RacingBrain Benchmark Runbook

本 runbook 目标是让工程结果能被复现、汇总和解释。它不替代真实车测试；它用于 replay、消融和论文表格生成。

## 1. 环境准备

```bash
./scripts/build_ros_clean.sh
source scripts/activate_ros_ml.sh
source install/setup.bash
```

默认数据与模型路径来自：

```text
LocalizationMapping/config/hardcoded_paths.ini
```

如果路径不在本机，设置：

```bash
export DRD26_PATH_CONFIG=/absolute/path/to/hardcoded_paths.ini
```

## 2. 单次链路冒烟

传统聚类后端：

```bash
LIDAR_BACKEND=cluster \
BAG_RATE=0.5 \
EVAL_TIMEOUT=90 \
./scripts/run_dataset_mapping_eval.sh
```

PointPillars 后端：

```bash
LIDAR_BACKEND=pointpillars \
LIDAR_VERIFIER=true \
BAG_RATE=1.0 \
EVAL_TIMEOUT=90 \
./scripts/run_dataset_mapping_eval.sh
```

自动仲裁：

```bash
LIDAR_BACKEND=auto \
LIDAR_VERIFIER=true \
MAPPING_GATE=true \
./scripts/run_dataset_mapping_eval.sh
```

输出目录形如：

```text
log/eval/dataset_mapping_eval_<timestamp>/
```

## 3. 速度对比

PointPillars：

```bash
LOG_DIR=log/eval/speed_pointpillars_$(date +%Y%m%d_%H%M%S) \
LIDAR_BACKEND=pointpillars \
LIDAR_VERIFIER=true \
BAG_RATE=1.0 \
RVIZ=false \
ENABLE_PLANNING=false \
MAPPING_GATE=true \
./scripts/run_dataset_mapping_eval.sh
```

PCL clustering：

```bash
LOG_DIR=log/eval/speed_cluster_$(date +%Y%m%d_%H%M%S) \
LIDAR_BACKEND=cluster \
BAG_RATE=1.0 \
RVIZ=false \
ENABLE_PLANNING=false \
MAPPING_GATE=true \
./scripts/run_dataset_mapping_eval.sh
```

关注指标：

- `pointpillars_total_mean_ms`
- `cluster_total_mean_ms`
- `/cone_detection_custom` rate
- `/perception/fusion/map` rate
- `/global_map` rate
- `runtime_budget_states`

## 4. Camera Benchmark

```bash
source scripts/activate_ros_ml.sh
source install/setup.bash

python3 scripts/eval/benchmark_camera_detectors.py \
  --max-frames 300 \
  --warmup 20 \
  --output-dir log/eval/camera_detector_classical_benchmark_$(date +%Y%m%d_%H%M%S)
```

关注指标：

- `yolo_gpu_ms`
- `classical_cone_cpu_ms`
- `yolo_boxes_per_frame`
- `classical_cones_per_frame`

解释方式：YOLO 不一定是最少 CPU 时间，但它显著降低后端候选负载；传统视觉更适合作 verifier 或候选补充。

## 5. Fault Injection

单场景：

```bash
LIDAR_BACKEND=cluster \
FAULT_PROFILE=camera_blank \
./scripts/run_dataset_mapping_eval.sh
```

可选场景：

```text
none
camera_blank
camera_blur
camera_dropout
lidar_stamp_skew
gnss_stamp_skew
fusion_calibration_bias
```

示例：

```bash
FAULT_PROFILE=lidar_stamp_skew \
FAULT_LIDAR_STAMP_OFFSET_SEC=0.12 \
./scripts/run_dataset_mapping_eval.sh

FAULT_PROFILE=fusion_calibration_bias \
CALIB_BIAS_YAW_DEG=4.0 \
./scripts/run_dataset_mapping_eval.sh
```

## 6. Mapping Gate Ablation

```bash
SCENARIOS="none camera_blank camera_blur fusion_calibration_bias" \
GATE_VARIANTS="true false" \
LIDAR_BACKEND=cluster \
./scripts/run_dataset_fault_benchmark.sh
```

输出目录形如：

```text
log/benchmark/dataset_fault_benchmark_<timestamp>/
```

关注指标：

- `candidate_residue_total`
- `final_duplicate_pairs`
- `created_cones_total`
- `risk_gate_downweighted_observations`
- `risk_gate_rejected_new_cones`
- `map_stability_score`

解释方式：gate-on 不一定追求更多稳定锥桶；它追求低污染、低候选残留、低重复和更可控的写入策略。

## 7. Planning Interface Smoke Test

```bash
ENABLE_PLANNING=true \
LIDAR_BACKEND=auto \
LIDAR_VERIFIER=true \
MAPPING_GATE=true \
./scripts/run_dataset_mapping_eval.sh
```

关注指标：

- `/planning/track_graph` count/rate
- `planning.frames`
- `planning.ready_counts`
- `planning_ready_ratio`
- `left_boundary_count`
- `right_boundary_count`
- `paired_boundary_count`
- `centerline_count`

当前 planning 是保守接口，不是完整高速控制器。论文表述应写成 planning-facing representation。

## 8. 汇总生成 Paper Tables

```bash
python3 scripts/eval/summarize_benchmarks.py \
  --input-root log/eval \
  --benchmark-root log/benchmark \
  --output-dir log/benchmark/benchmark_summary_latest
```

生成：

```text
log/benchmark/benchmark_summary_latest/
  runs_summary.csv
  aggregate_summary.csv
  gate_comparisons.csv
  paper_tables.md
  manifest.json
```

论文和汇报优先读：

```text
log/benchmark/benchmark_summary_latest/paper_tables.md
```

## 9. 推荐实验矩阵

| Axis | Values |
|---|---|
| LiDAR backend | `cluster`, `pointpillars`, `auto` |
| Local verifier | `true`, `false` |
| Mapping gate | `true`, `false` |
| Fault profile | `none`, `camera_blank`, `camera_blur`, `lidar_stamp_skew`, `gnss_stamp_skew`, `fusion_calibration_bias` |
| Planning interface | `false`, `true` |

最小强证据组合：

```text
backend: cluster / pointpillars / auto
gate: true / false
fault: none / camera_blank / camera_blur / fusion_calibration_bias
planning: true for final replay only
```

## 10. 汇报口径

可以主张：

- 系统能在同一 replay 上比较学习式与传统感知的实时性。
- 系统能把跨模态不一致、标定漂移、时延异常转成任务风险。
- 系统能通过 risk-aware mapping gate 影响地图写入策略。
- 系统能生成稳定地图、候选层、拒绝层和规划接口状态。

暂时不要主张：

- 绝对定位精度已经达到某个数值。
- 地图误差已经优于某个公开 SOTA。
- 高速闭环控制已经完成。

这些需要 reference map、真实车闭环和控制实验补齐。

