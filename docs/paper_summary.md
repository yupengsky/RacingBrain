# RacingBrain 博士代表作包装摘要

## 1. 核心定位

RacingBrain 不应被包装成一个泛泛的无人车工程，而应被包装成：

> 面向高速无人赛车的风险校准感知、任务记忆与规划接口系统。

近年文献坐标见 `docs/literature_landscape.md`。本文档聚焦工程贡献和实验包装。

它的研究问题是：

> 当高速车辆依赖学习感知时，系统如何判断哪些观测不该写入长期地图，并把这种风险以可审计的形式传递给规划控制层？

这比“YOLO + PointPillars + SLAM”更强，因为贡献点落在真实高速任务最脆弱的位置：学习感知会失效，失效会污染地图，污染后的地图会误导下一圈规划。

## 2. 当前可主张的贡献

### 2.1 快速学习感知与可解释验证共存

工程中已经存在两条 LiDAR 后端：

- PointPillars TensorRT 主路径，用于降低 3D 检测延迟。
- PCL clustering 传统路径，用于保守 fallback 和离线参照。

PointPillars 输出还经过局部点云几何、相机投影和时间连续性验证。这个结构可以被讲成：

> Learned proposal + geometric verifier + temporal verifier.

核心入口：

- `LocalizationMapping/PointPillars/trt_cone_detector`
- `LocalizationMapping/RacingBrain/racingbrain/perception/pointpillars_local_verifier.py`
- `LocalizationMapping/RacingBrain/racingbrain/perception/lidar_backend_arbiter.py`

### 2.2 跨模态一致性被显式量化

融合节点不仅输出锥桶，还记录以下诊断信号：

- camera-LiDAR timestamp offset
- projection residual
- low-IoU ratio
- unknown ratio
- force-match ratio
- consistency score
- calibration drift score

这让系统能够从“融合结果看起来还行”升级为“融合质量可被在线审计”。

核心入口：

- `LocalizationMapping/perception/src/fs_fusion_box/src/fs_fusion_box_node.cpp`

### 2.3 地图写入受任务风险控制

系统健康监控会把感知、融合、建图、实时预算综合成任务风险：

- `task_risk_state`
- `task_risk_score`
- `map_contamination_risk`
- `planning_readiness_risk`
- `world_model_write_policy`

建图节点消费这些信号，并对地图写入执行：

- open
- monitor
- downweight observations
- freeze new landmarks

这就是当前最有科研味的部分：不是只评估 detector，而是评估 detector 失败会不会污染任务记忆。

核心入口：

- `LocalizationMapping/RacingBrain/racingbrain/health/monitor.py`
- `LocalizationMapping/slam/slam/src/slam_node.cpp`

### 2.4 规划接口已经有保守输入

当前规划层还不是完整高速控制器，但已有从稳定锥桶图生成稀疏赛道图的接口：

- `/planning/track_graph`
- `/racingbrain/planning/input_state`

这可以被表述为：

> The planner consumes stable task memory rather than raw perception.

核心入口：

- `LocalizationMapping/RacingBrain/racingbrain/planning/track_graph_node.py`

## 3. 当前证据链

仓库已经具备以下证据来源：

- `log/eval/*/summary.json`
- `log/eval/*/processing_times.csv`
- `log/eval/*/system_health.csv`
- `log/eval/*/perception_failure_state.csv`
- `log/eval/*/planning_state.csv`
- `log/benchmark/*/mapping_gate_comparison.csv`

推荐用新增脚本生成 paper-ready 表格：

```bash
python3 scripts/eval/summarize_benchmarks.py \
  --input-root log/eval \
  --benchmark-root log/benchmark \
  --output-dir log/benchmark/benchmark_summary_latest
```

输出：

- `runs_summary.csv`
- `aggregate_summary.csv`
- `gate_comparisons.csv`
- `paper_tables.md`
- `manifest.json`

## 4. 短论文式结构

### Title

Risk-Calibrated Task Memory for High-Speed Autonomous Racing

### Abstract Sketch

High-speed autonomous racing requires learned perception, but learned perception is not safe to treat as always-truthful world state. RacingBrain combines fast LiDAR and camera cone detection with cross-modal consistency monitoring, runtime health estimation, and a risk-aware mapping gate. The system protects GNSS/INS-aided cone maps from transient perception failures and exposes stable sparse track graphs for downstream planning.

### Method Sections

1. Real-time perception stack
2. Local geometric and temporal verification
3. Camera-LiDAR consistency scoring
4. Runtime task-risk estimation
5. Risk-aware map update and confidence layers
6. Planning-facing sparse track graph

### Experiment Sections

1. Runtime benchmark: PointPillars vs PCL clustering
2. Camera detector latency and output-load comparison
3. Fault injection: camera blank, blur, timestamp skew, calibration bias
4. Mapping gate ablation: gate on vs gate off
5. Planning-interface readiness from stable map

## 5. 需要补强后才能写成强论文的点

### Ground Truth

当前很多指标是自一致性和任务风险指标，不能直接声称绝对精度。下一步应补：

- surveyed cone map
- annotated cone positions
- RTK reference trajectory
- manually verified failure-case labels

### Planning Control

当前规划接口是中心线预览，不是高速闭环控制。下一步应补：

- constrained boundary pairing
- racing line generation
- curvature-speed profile
- MPCC or MPC controller
- actuator latency handling

### Calibrated Risk

当前风险分数主要是启发式规则。博士代表作应升级为：

- calibrated uncertainty
- cost-sensitive failure recognition
- planner-facing confidence bounds
- conformal or empirical risk calibration

## 6. 代表作路线

### 0-2 周

- 固化 benchmark 汇总表。
- 补充指标定义。
- 清理 runbook。
- 生成一套能给导师看的 paper tables。

### 2-6 周

- 标注一个小型 reference cone map。
- 加绝对 map error、duplicate rate、false candidate lifetime。
- 做 verifier、fallback、mapping gate 的完整消融矩阵。

### 6-12 周

- 把 track graph 接到局部规划。
- 用地图风险调整速度上限或 planning readiness。
- 把 planner failure 和 map contamination 关联起来。

### 长线博士贡献

从“可靠感知建图”升级到：

> Risk-calibrated world-model writing and planning for high-speed embodied autonomy.
