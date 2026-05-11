# RacingBrain Metrics Schema

本文档定义 replay 与 benchmark 中最重要的指标，目的是把日志字段转化为科研表达。

## 1. Topic Health

### Topic count/rate

来源：

```text
summary.json.topics
```

常用 topic：

- `/camera1/image_raw`
- `/yolo/cones`
- `/lidar_points`
- `/cone_detection_custom`
- `/perception/fusion/map`
- `/global_map`
- `/mapping/candidate_cones`
- `/mapping/rejected_observations`
- `/racingbrain/health/system`
- `/planning/track_graph`

解释：

- 证明链路是否真实跑通。
- 比较学习后端与传统后端是否把检测速度传递到了融合和建图。
- 不等价于检测精度。

## 2. Runtime Metrics

### `pointpillars_total_ms`

来源：

```text
processing_time_ms.pointpillars.total_ms
```

含义：单帧 PointPillars ROS 节点总耗时，包含预处理、H2D、GPU voxel、TensorRT、D2H、后处理和发布。

用途：证明学习式 3D 检测是否满足实时预算。

### `lidar_cluster_total_ms`

来源：

```text
processing_time_ms.lidar_cluster.total_ms
```

含义：传统 PCL clustering 单帧耗时。

用途：作为可解释但较慢的参照和 fallback 成本估计。

### `yolo.inference_ms`

来源：

```text
processing_time_ms.yolo.inference_ms
```

含义：在线 YOLO 推理耗时。

注意：第一次推理常有 warmup 峰值，论文表格应同时看 median / p95。

### `runtime_budget_state`

来源：

```text
system_health.csv
summary.json.runtime_budget.state_counts
```

状态：

- `nominal`: 延迟处于预算内。
- `strained`: 接近预算边缘。
- `degraded`: 超过 warning budget。
- `freeze`: 超过 severe budget 或关键组件 stale/missing。

科研含义：实时性不是旁观指标，而是感知结果是否可写入任务记忆的条件。

## 3. Fusion Consistency

### `abs_camera_lidar_stamp_delta_ms`

来源：

```text
fusion_consistency.abs_camera_lidar_stamp_delta_ms
```

含义：相机检测与 LiDAR 检测的时间戳差。

风险：高速场景下时间偏差会把空间对齐误差放大。

### `mean_nearest_camera_error_px`

来源：

```text
fusion_consistency.mean_nearest_camera_error_px
```

含义：LiDAR 投影到图像后，与最近相机框中心的像素残差。

用途：监控标定漂移、同步偏差和融合质量下降。

### `consistency_score`

来源：

```text
fusion_consistency.consistency_score
```

含义：融合节点综合时间质量、投影质量、像素残差、UNKNOWN 比例和强制匹配比例得到的一致性分数。

方向：越高越好。

### `calibration_drift_score`

来源：

```text
fusion_consistency.calibration_drift_score
```

含义：由投影残差、低 IoU、强制匹配和时间偏差构成的漂移嫌疑分数。

方向：越低越好。

注意：这是 drift suspicion，不是外参误差的物理真值。

## 4. Map Quality and Pollution

### `final_stable_cones`

来源：

```text
summary.json.map.final_stable_cones
```

含义：最终稳定锥桶数。

注意：更多不一定更好；错误稳定锥桶会污染下一圈规划。

### `candidate_residue_total`

来源：

```text
summary.json.map_pollution.candidate_residue_total
```

含义：回放结束后仍停留在候选层的锥桶数量。

科研含义：衡量观测是否产生了长期悬而未决的地图残留。

### `final_duplicate_density`

来源：

```text
summary.json.map_pollution.final_duplicate_density
```

含义：最终稳定地图中的重复锥桶密度。

用途：衡量数据关联和风险门控是否避免了重复 landmark。

### `unknown_observation_ratio`

来源：

```text
summary.json.map_pollution.unknown_observation_ratio
```

含义：融合输出中 UNKNOWN 观测占比。

用途：衡量相机颜色/语义证据是否足够可靠。

### `map_stability_score`

来源：

```text
summary.json.map_pollution.map_stability_score
```

含义：综合候选残留、重复、UNKNOWN 观测、移除 churn 等得到的自一致性分数。

方向：越高越好。

注意：这是 replay self-consistency，不是绝对地图精度。

## 5. Risk Gate

### `risk_gate_downweighted_observations`

来源：

```text
summary.json.map_pollution.risk_gate_downweighted_observations
```

含义：风险门控降低写入权重的观测数量。

解释：表示系统没有盲目信任输入，而是在 degraded 状态下保守更新已有地图。

### `risk_gate_rejected_new_cones`

来源：

```text
summary.json.map_pollution.risk_gate_rejected_new_cones
```

含义：风险门控拒绝创建的新锥桶数量。

解释：这是防止地图污染的直接证据。

### `world_model_write_policy`

来源：

```text
system_health.csv
summary.json.task_risk.world_model_write_policy_counts
```

策略：

- `open`
- `monitor_only`
- `downweight_observations`
- `freeze_new_landmarks`

科研含义：把感知和实时性异常转化为任务记忆写入策略。

## 6. Task Risk

### `task_risk_score`

来源：

```text
summary.json.task_risk.task_risk_score
```

含义：系统对当前任务风险的总体估计。

### `map_contamination_risk`

来源：

```text
summary.json.task_risk.map_contamination_risk
```

含义：当前观测污染长期地图的风险。

### `planning_readiness_risk`

来源：

```text
summary.json.task_risk.planning_readiness_risk
```

含义：当前世界模型是否足够支持规划。

解释：这两个风险应分开报告，因为某些状态可能不会污染地图，但会让规划输入不足。

## 7. Planning Interface

### `planning.ready_counts`

来源：

```text
summary.json.planning.ready_counts
```

含义：track graph builder 认为规划输入是否足够。

### `planning_ready_ratio`

由汇总脚本计算：

```text
ready_true_frames / planning_frames
```

含义：回放中规划接口处于可消费状态的比例。

注意：这不是控制成功率；它只是 planning-facing representation readiness。

## 8. Camera Detector Benchmark

### `yolo_gpu_ms`

来源：

```text
camera benchmark summary.json
```

含义：离线 YOLO GPU 推理耗时。

### `classical_cone_cpu_ms`

含义：传统颜色/形态学/轮廓候选器 CPU 耗时。

### `boxes/candidates_per_frame`

含义：输出负载。候选数量会影响后续关联、融合和地图写入。

解释口径：YOLO 的价值不只是速度，而是更紧凑的语义输入；传统视觉适合作为 verifier 或 candidate supplement。

## 9. 论文表格推荐

### Runtime Table

字段：

- backend
- LiDAR mean/p95 ms
- `/cone_detection_custom` Hz
- `/perception/fusion/map` Hz
- `/global_map` Hz
- runtime budget state counts

### Map Pollution Table

字段：

- scenario
- gate on/off
- stable cones
- candidate residue
- duplicate pairs
- rejected new cones
- downweighted observations
- map stability score

### Robust Fusion Table

字段：

- scenario
- timestamp delta
- projection residual
- consistency score
- drift score
- task risk state
- write policy

### Planning Interface Table

字段：

- stable cones
- paired boundaries
- centerline points
- ready frames
- planning ready ratio

