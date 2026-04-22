# Evaluation

This workspace includes a sidecar evaluation path for the dataset perception-to-SLAM chain.
It is designed for offline baseline comparison and is not required during normal on-car or
demo execution.

## Run

```bash
./scripts/run_dataset_slam_eval.sh
```

The script launches the existing perception stack, launches `slam_node` with
`eval_debug:=true`, plays the configured rosbag, and runs a separate monitor process.
Normal runs through `./scripts/run_dataset_slam_chain.sh` keep evaluation debug output
disabled.

Useful overrides:

```bash
BAG_RATE=0.25 EVAL_TIMEOUT=220 ./scripts/run_dataset_slam_eval.sh
TRACK=autocross ./scripts/run_dataset_slam_eval.sh
DUPLICATE_THRESHOLD=0.5 ./scripts/run_dataset_slam_eval.sh
```

## Outputs

Each run writes to:

```text
log/eval/dataset_slam_eval_<timestamp>/
```

Important artifacts:

- `summary.json`: machine-readable top-level metrics.
- `report.md`: human-readable report.
- `topic_rates.csv`: topic counts and observed rates.
- `latency.csv`: header-stamp deltas between adjacent stages.
- `fusion_frames.csv`: per-frame fused cone counts, colors, unknown ratio, and duplicate risk.
- `map_frames.csv`: per-frame stable global map counts and nearest-neighbor statistics.
- `odom.csv`: local trajectory samples and cumulative odometry length.
- `slam_debug_frames.csv`: optional SLAM internal counters from `/slam/evaluation/metrics`.
- `plots/`: quick-look charts when matplotlib is available.

## Metric Scope

Current metrics are self-consistency and runtime metrics. They quantify chain health,
timing, fusion quality, map stability, duplicate risk, and trajectory closure indicators.
They are not absolute accuracy metrics unless annotated cone positions or a reference
trajectory are added later.
