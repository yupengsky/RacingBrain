## pure GNSS/INS

DATASET_DIR=/media/yupeng/S11/rosbag2_2026_05_10-15_02_50 
OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/slam_mapping_2026_05_10-15_02_50.html 
RUN_DURATION_SEC=full BAG_RATE=1.0 ./scripts/run_dataset_slam_html.sh

---

## LIO GNSS/INS evaluation

Dataset:

```bash
/media/yupeng/S11/rosbag2_2026_05_10-15_02_50
```

Run the offline simple LIO evaluation:

```bash
OUTPUT_DIR=/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50 OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50.html ./scripts/run_dataset_simple_lio_eval.sh
```

Open the dynamic HTML result:

```bash
xdg-open /home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50.html
```

Latest full-dataset result:

```text
samples: 5176
duration: 518.12 s
mean position error: 6.38 m
median position error: 3.82 m
RMSE: 8.82 m
p95 position error: 18.48 m
max position error: 20.45 m
```

Detailed outputs:

```bash
/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50/summary.json
/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50/lio_gnss_synced_trace.csv
/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50/lio_gnss_synced_trace.json
```
