## 纯 GNSS/INS

```bash
DATASET_DIR=/media/yupeng/Ventoy/rosbag2_2026_05_10-15_02_50 \
OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/slam_mapping_2026_05_10-15_02_50.html \
RUN_DURATION_SEC=full BAG_RATE=1.0 ./scripts/run_dataset_slam_html.sh
```

```bash
xdg-open /home/yupeng/GitHub/RacingBrain/results/slam_mapping_2026_05_10-15_02_50.html
```

---

## LIO GNSS/INS 评估

```bash
DATASET_DIR=/media/yupeng/Ventoy/rosbag2_2026_05_10-15_02_50 \
OUTPUT_DIR=/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50 \
OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50.html \
./scripts/run_dataset_simple_lio_eval.sh
```

```bash
xdg-open /home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50.html
```

结果文件：

```bash
/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50/summary.json
/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50/lio_gnss_synced_trace.csv
/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50/lio_gnss_synced_trace.json
```

---

## FUSE

默认生成 6 组 GNSS/INS 丢失档位：

```bash
DATASET_DIR=/media/yupeng/Ventoy/rosbag2_2026_05_10-15_02_50 \
OUTPUT_DIR=/home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50 \
OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50.html \
./scripts/run_dataset_fuse_eval.sh
```

只生成一组档位：

```bash
DATASET_DIR=/media/yupeng/Ventoy/rosbag2_2026_05_10-15_02_50 \
GAP_COUNT=5 GAP_DURATION_SEC=10 \
OUTPUT_DIR=/home/yupeng/GitHub/RacingBrain/results/fuse_eval_5_gaps_10s \
OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/fuse_eval_5_gaps_10s.html \
./scripts/run_dataset_fuse_eval.sh
```

手动指定 GNSS/INS 丢失区间：

```bash
DATASET_DIR=/media/yupeng/Ventoy/rosbag2_2026_05_10-15_02_50 \
SCENARIO_MODE=single OUTAGES=45:8,185:10,360:8 \
OUTPUT_DIR=/home/yupeng/GitHub/RacingBrain/results/fuse_eval_custom \
OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/fuse_eval_custom.html \
./scripts/run_dataset_fuse_eval.sh
```

```bash
xdg-open /home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50.html
```

结果文件：

```bash
/home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50/summary.json
/home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50/fused_gnss_lio_trace.csv
/home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50/fused_gnss_lio_trace.json
```
