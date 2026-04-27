# 最直接运行指令

## 一键录制四联屏实时建图演示视频，1.0x 倍速

```bash
cd /home/yupeng/GitHub/RacingBrain
./scripts/run_mapping_demo_recording.sh
```

默认输出到 `~/Videos/racingbrain_mapping_demo_时间戳.mp4`。

## 一键演示，带 RViz 动画

```bash
cd /home/yupeng/GitHub/RacingBrain
RVIZ=true KEEP_RUNNING=true ./scripts/run_dataset_mapping_chain.sh
```

按 `Ctrl-C` 停止。

## 一键验证链路，不开 RViz

```bash
cd /home/yupeng/GitHub/RacingBrain
./scripts/run_dataset_mapping_chain.sh
```

当前 `imp-pointpillars` 分支默认使用 PointPillars LiDAR 后端；需要临时回退传统聚类时：

```bash
cd /home/yupeng/GitHub/RacingBrain
LIDAR_BACKEND=cluster ./scripts/run_dataset_mapping_chain.sh
```

## 一键量化评估链路

```bash
cd /home/yupeng/GitHub/RacingBrain
./scripts/run_dataset_mapping_eval.sh
```

默认输出到 `log/eval/dataset_mapping_eval_时间戳/`，包含 `summary.json`、
`report.md`、`processing_times.csv`、其他 CSV 明细和 `plots/` 图表。正常演示链路不会开启评估诊断。

## 重新构建

```bash
cd /home/yupeng/GitHub/RacingBrain
./scripts/build_ros_venv.sh --event-handlers console_direct+
```

## 手动启动感知

```bash
cd /home/yupeng/GitHub/RacingBrain
source scripts/activate_ros_ml.sh
source install/setup.bash
ros2 launch run_perception system_run.launch.py
```

## 手动启动实时建图

```bash
cd /home/yupeng/GitHub/RacingBrain
source scripts/activate_ros_ml.sh
source install/setup.bash
ros2 launch slam slam.launch.py rviz:=true
```

## 手动播放数据集

```bash
cd /home/yupeng/GitHub/RacingBrain
source scripts/activate_ros_ml.sh
source install/setup.bash
ros2 bag play /media/yupeng/新加卷/Datasets/rosbag2_2026_02_05-11_01_07 --rate 1.0 --topics /camera1/image_raw /lidar_points /gongji_gnss_ins_64
```

## 查看最近一次运行结果

```bash
cd /home/yupeng/GitHub/RacingBrain
cat "$(ls -td log/runtime/dataset_mapping_chain_* | head -n 1)/summary.json"
```
