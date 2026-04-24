# Evaluation

This workspace includes a sidecar evaluation path for the dataset perception-to-mapping chain.
It is designed for offline baseline comparison and is not required during normal on-car or
demo execution.

## Run

```bash
./scripts/run_dataset_mapping_eval.sh
```

On the `imp-pointpillars` branch, the default LiDAR backend is PointPillars.
Use `LIDAR_BACKEND=cluster ./scripts/run_dataset_mapping_eval.sh` to run the old
PCL clustering backend for an A/B check on the same branch.
Use `LIDAR_BACKEND=auto ./scripts/run_dataset_mapping_eval.sh` to enable the
runtime arbiter. In auto mode the selected output still appears on
`/cone_detection_custom`, while `/racingbrain/perception/failure_state` records
the active backend and learning-failure reasons.

The script launches the full `racingbrain localization_mapping` stack with `eval_debug:=true`
and `enable_health:=true`, plays the configured rosbag, and runs a separate monitor process.
Normal runs through `./scripts/run_dataset_mapping_chain.sh` still keep sidecar evaluation
debug output disabled, but the lightweight system health topic remains available online.

Useful overrides:

```bash
BAG_RATE=0.25 EVAL_TIMEOUT=220 ./scripts/run_dataset_mapping_eval.sh
TRACK=autocross ./scripts/run_dataset_mapping_eval.sh
DUPLICATE_THRESHOLD=0.5 ./scripts/run_dataset_mapping_eval.sh
```

## Fault Profiles

The same evaluation entrypoint can replay degraded sensor conditions without
changing the online stack code. Set `FAULT_PROFILE` to one of:

- `none`: baseline replay.
- `camera_blank`: replace camera frames with black images.
- `camera_blur`: apply Gaussian blur to the camera stream.
- `camera_dropout`: drop every Nth camera frame.
- `lidar_stamp_skew`: shift LiDAR header timestamps forward.
- `gnss_stamp_skew`: shift GNSS/INS header timestamps forward.
- `fusion_calibration_bias`: replay with a perturbed camera-LiDAR calibration file.

Examples:

```bash
LIDAR_BACKEND=cluster FAULT_PROFILE=camera_blank ./scripts/run_dataset_mapping_eval.sh
FAULT_PROFILE=lidar_stamp_skew FAULT_LIDAR_STAMP_OFFSET_SEC=0.12 \
  ./scripts/run_dataset_mapping_eval.sh
FAULT_PROFILE=fusion_calibration_bias CALIB_BIAS_YAW_DEG=4.0 \
  ./scripts/run_dataset_mapping_eval.sh
```

For repeated comparisons across scenarios, use:

```bash
SCENARIOS="none camera_blank camera_blur fusion_calibration_bias" \
  ./scripts/run_dataset_fault_benchmark.sh
```

This wrapper creates one evaluation directory per scenario plus a top-level
`benchmark_summary.csv` and `benchmark_report.md`.

To compare the risk-aware mapping gate against the ungated mapper, run:

```bash
SCENARIOS="none camera_blank" GATE_VARIANTS="true false" \
  ./scripts/run_dataset_fault_benchmark.sh
```

The wrapper then creates per-scenario `mapping_gate_true` and
`mapping_gate_false` folders and writes `mapping_gate_comparison.csv` with
on-minus-off deltas for stable cones, duplicate pairs, candidate residue, and
map-stability score.

## Outputs

Each run writes to:

```text
log/eval/dataset_mapping_eval_<timestamp>/
```

Important artifacts:

- `summary.json`: machine-readable top-level metrics.
- `report.md`: human-readable report.
- `topic_rates.csv`: topic counts and observed rates.
- `latency.csv`: header-stamp deltas between adjacent stages.
- `processing_times.csv`: per-frame processing-time JSON from YOLO, LiDAR clustering,
  fusion, and mapping when evaluation debug is enabled. Fusion rows also include
  camera-LiDAR consistency fields such as stamp offset, projection residual,
  best-IoU quality, consistency score, and calibration-drift score.
- `fusion_frames.csv`: per-frame fused cone counts, colors, unknown ratio, and duplicate risk.
- `map_frames.csv`: per-frame stable global map counts and nearest-neighbor statistics.
- `/mapping/candidate_cones` and `/mapping/rejected_observations` are monitored
  as confidence layers: the former shows tracked cones that have not yet become
  stable, while the latter shows per-frame observations rejected by map lock,
  ROI, or the risk-aware gate.
- `odom.csv`: local trajectory samples and cumulative odometry length.
- `mapping_debug_frames.csv`: optional mapping-node counters from `/slam/evaluation/metrics`.
  This includes risk-gate state, rejected new cones, and downweighted observations
  when risk-aware mapping is enabled.
- `summary.json.map_pollution`: candidate-residue, stable-creation, removal-churn,
  UNKNOWN-observation, duplicate-density, and gate-action metrics for judging
  whether degraded perception is polluting the global cone map.
- `system_health.csv`: unified online health snapshots from `/racingbrain/health/system`.
  It now also records task-level risk fields such as `task_risk_state`,
  `task_risk_score`, `map_contamination_risk`, `planning_readiness_risk`, and
  `world_model_write_policy`.
- `perception_failure_state.csv`: online perception failure decisions and active
  LiDAR backend, including arbiter-side task-risk hints that explain why the
  system fell back or stayed in monitor/degraded mode.
- `planning_state.csv`: optional sparse-track-graph readiness snapshots when
  `ENABLE_PLANNING=true` is used during replay.
- `scenario.json`: replay fault configuration for the current run.
- `fault_injector_stats.json`: shadow-topic replay counts for injected scenarios.
- `plots/`: quick-look charts when matplotlib is available.

## Metric Scope

Current metrics are self-consistency and runtime metrics. They quantify chain health,
timing, fusion quality, map stability, duplicate risk, trajectory closure indicators,
camera-LiDAR alignment quality, and the aggregated status of the online health bus.
The health bus now carries a task-facing view of risk, so replay outputs can
separate "module warnings" from "risk to world-model integrity" and "risk to
planning readiness".
They are not absolute accuracy metrics unless annotated cone positions or a reference
trajectory are added later.

`summary.json.task_risk` aggregates the online task-risk stream across the
entire replay and reports state counts, score distributions, and the final
world-model write policy seen by the mapper.

Risk-aware mapping can be toggled during replay with:

```bash
MAPPING_GATE=false ./scripts/run_dataset_mapping_eval.sh
```

The default is `MAPPING_GATE=true`.
