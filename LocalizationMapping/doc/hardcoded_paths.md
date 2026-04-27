# Hardcoded Paths

The project-level hardcoded runtime paths are collected in:

```text
config/hardcoded_paths.ini
```

Current external roots:

- Models: `/media/yupeng/新加卷/Models/RacingBrain`
- Datasets: `/media/yupeng/新加卷/Datasets`
- Default rosbag: `/media/yupeng/新加卷/Datasets/rosbag2_2026_02_05-11_01_07`
- Default YOLO runtime model:
  `/media/yupeng/新加卷/Models/RacingBrain/perception/src/cone_ws/src/cone_detector/runs/cone_yolov8n6/weights/best.pt`

Keep code pointed at this file rather than adding new absolute paths inline.
