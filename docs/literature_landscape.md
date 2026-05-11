# RacingBrain 近年文献定位

检索日期：2026-05-11

本文档不是完整综述，而是用于论文选题、开题汇报和代表作包装的“文献坐标系”。核心问题是：RacingBrain 应该站在哪些近年研究线上，哪些地方可以主张差异化贡献，哪些地方还不能过度声称。

## 1. 建议论文定位

推荐定位：

> Risk-calibrated task-memory writing for high-speed autonomous racing.

中文表达：

> 面向高速无人赛车的风险校准任务记忆写入与规划接口。

这个定位比“多传感器融合建图系统”更强，因为它把贡献放在高速任务链路中最容易造成级联失效的位置：感知错误进入地图，地图污染影响下一圈规划，规划控制又依赖地图置信度。

## 2. 高速自动驾驶与无人赛车系统

近年无人赛车论文的共同特点是强系统工程：感知、定位、规划、控制、状态管理必须在高动态约束下协同工作。

- TUM Autonomous Motorsport 总结了 IAC 系统栈，强调无人赛车是自动驾驶技术极限验证平台，覆盖 object detection、localization、planning、prediction、control，并给出了 270 km/h 量级的比赛经验。对 RacingBrain 的启发：代表作需要以系统闭环指标讲故事，而不是只报 detector 指标。参考：<https://arxiv.org/abs/2205.15979>
- KAIST IAC 系统论文强调 multi-modal perception、高速 overtaking planner、resilient control stack 和 system status manager，并报告约 220 km/h 头对头比赛表现。对 RacingBrain 的启发：`health/monitor.py` 和 mapping gate 应被提升为系统贡献，而不是辅助模块。参考：<https://arxiv.org/abs/2303.09463>
- 2024 IAC racecar control-informed design 把 by-wire actuation、perception、localization、计算和软件管线放入控制设计约束中，强调高速瞬态动力学带来的系统设计问题。对 RacingBrain 的启发：实时预算与 planning readiness 应作为写入策略约束。参考：<https://arxiv.org/abs/2407.17737>
- 2025 ARS 高速赛车栈论文报告三代系统、不同赛道验证、260 km/h 速度和多传感器数据集释放。对 RacingBrain 的启发：后续如果能释放 replay/benchmark 子集，会显著增强科研可见度。参考：<https://arxiv.org/abs/2512.06892>

RacingBrain 当前的可主张差异不是“速度比 IAC 更高”，而是：在 Formula Student/锥桶赛道任务中，把学习感知风险、地图写入策略和规划输入显式连接。

## 3. Planning-Oriented 感知规划一体化

2023 以来自动驾驶主线明显从单独感知指标转向 planning-oriented 表征。

- UniAD 提出以最终规划目标组织 perception、prediction 和 planning，多个任务通过统一 query 接口协作。对 RacingBrain 的启发：不要把 `/global_map` 只说成建图输出，应说成 planning-facing world state。参考：<https://openaccess.thecvf.com/content/CVPR2023/html/Hu_Planning-Oriented_Autonomous_Driving_CVPR_2023_paper.html>
- VAD 用向量化场景表征替代密集 raster 表征，强调显式 instance-level 规划约束和效率。对 RacingBrain 的启发：`/planning/track_graph` 的稀疏边界、中心线和稳定锥桶图，可以对齐 vectorized planning representation。参考：<https://arxiv.org/abs/2303.12077>
- MapTRv2 将在线 HD map 构建表述为端到端向量化地图元素学习，并强调在线地图服务于规划。对 RacingBrain 的启发：锥桶地图虽不是城市 HD map，但“在线地图元素 + downstream planning”是同一研究语境。参考：<https://arxiv.org/abs/2308.05736>

RacingBrain 的短期包装口径：不是挑战端到端 SOTA，而是提出一个可审计、可门控、可复现实验的 planning-facing task memory。

## 4. 在线地图不确定性与下游任务

近年一个重要趋势是：地图不只要生成，还要产生置信度或不确定性，并让下游预测/规划使用。

- CVPR 2024 的 online map uncertainty 工作指出，在线地图估计如果缺少 uncertainty/confidence，会难以和下游任务紧密集成；引入地图不确定性可改善轨迹预测训练和性能。对 RacingBrain 的启发：`map_stability_score`、candidate residue、duplicate density、write policy 可以被包装为“任务记忆置信度”的工程原型。参考：<https://arxiv.org/abs/2403.16439>

当前要谨慎：RacingBrain 现有指标更多是 replay self-consistency 和 rule-based risk，不是严格概率校准。论文中可以说 risk-aware / uncertainty-proxy，暂时不要说 formal calibrated uncertainty。

## 5. 风险感知规划与 Conformal Safety

安全规划研究正在强调感知/预测不确定性对控制约束的影响。

- Safe Planning in Dynamic Environments using Conformal Prediction 将 trajectory prediction regions 接入 MPC，以获得概率安全保证。对 RacingBrain 的启发：后续可以把感知失效、地图污染和 planning readiness 转成 conformal 或 empirical risk bound。参考：<https://arxiv.org/abs/2210.10254>
- Safe Adaptive Cruise Control under Perception Uncertainty 将 deep ensemble、conformal prediction 和 tube MPC 结合，使 RGB 感知不确定性影响下游控制。对 RacingBrain 的启发：当前 `task_risk_score` 和 `world_model_write_policy` 可作为“感知不确定性影响控制/规划”的模块化前身。参考：<https://arxiv.org/abs/2412.03792>
- 2024 不确定性感知自治栈研究强调 upstream uncertainty quantification 可改善模块化系统的鲁棒性，并让下游模块获得更可校准的输出。对 RacingBrain 的启发：博士线可从 heuristic risk gate 升级到 calibrated uncertainty passing。参考：<https://www2.eecs.berkeley.edu/Pubs/TechRpts/2024/EECS-2024-204.html>

短期论文表述：

> RacingBrain implements an auditable risk gate for world-model writing and exposes risk-aware planning inputs; formal statistical calibration is future work.

## 6. 与 RacingBrain 模块的对应关系

| 文献主线 | RacingBrain 对应模块 | 当前可证明 | 后续补强 |
| --- | --- | --- | --- |
| 高速赛车系统栈 | `scripts/run_dataset_mapping_eval.sh`, health monitor, perception/fusion/mapping/planning topics | 多模块 replay 跑通、状态可观测 | 真车闭环或高保真仿真闭环 |
| 多模态感知 | YOLO, PointPillars, fusion node | 推理延迟、topic rate、融合一致性 | 绝对检测精度、颜色分类精度 |
| Planning-oriented representation | `/global_map`, `/planning/track_graph`, `/racingbrain/planning/input_state` | stable cone map 到 track graph 的接口 | 局部规划、速度曲线、MPC/MPCC |
| 在线地图不确定性 | `map_stability_score`, candidate residue, duplicate density | 地图污染 proxy 和门控消融 | surveyed map / annotated cone GT |
| 风险感知规划 | `task_risk_score`, `world_model_write_policy`, mapping gate | 风险状态影响地图写入 | conformal calibration / risk-bound planner |

## 7. 论文贡献写法

建议三条贡献：

1. 一个面向高速锥桶赛道的实时多模态感知与任务记忆系统，结合学习式 LiDAR 检测、视觉语义和传统 fallback。
2. 一个可审计的跨模态一致性与系统健康监控机制，用于检测同步、标定、实时预算和语义不确定性风险。
3. 一个风险感知地图写入门控，将感知/系统风险转化为 world-model write policy，并输出 planning-facing track graph。

建议避免的过度表述：

- 不要说“达到高速自动驾驶 SOTA”，除非有同条件竞品和真车速度数据。
- 不要说“证明安全”，除非引入形式化或统计保证。
- 不要说“地图精度优于 X”，除非有 surveyed cone map 或人工标注真值。

## 8. 最值得补的实验

短期最有性价比的实验矩阵：

| 实验 | 目的 | 关键输出 |
| --- | --- | --- |
| PointPillars vs clustering | 证明实时感知收益和 fallback 成本 | detection latency, topic rate, runtime state |
| mapping gate on/off | 证明风险门控抑制地图污染 | residue, duplicate, stability, write policy |
| camera blank / blur / timestamp skew | 证明失效被诊断而非静默写入 | consistency, drift, task risk, map stability |
| planning interface smoke | 证明稳定任务记忆可供规划消费 | track graph rate, ready ratio, centerline count |

这些实验已经能由 `scripts/eval/summarize_benchmarks.py` 汇总成 `paper_tables.md`，是当前最适合快速打磨成代表作证据链的部分。
