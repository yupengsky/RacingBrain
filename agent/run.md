# 最直接运行指令

## 一键录制四联屏完整演示视频，1.0x 倍速

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
./scripts/run_full_demo_recording.sh
```

默认输出到 `~/Videos/drd26_full_demo_时间戳.mp4`。

## 一键演示，带 RViz 动画

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
RVIZ=true KEEP_RUNNING=true ./scripts/run_dataset_slam_chain.sh
```

按 `Ctrl-C` 停止。

## 一键验证链路，不开 RViz

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
./scripts/run_dataset_slam_chain.sh
```

## 重新构建

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
./scripts/build_ros_venv.sh --event-handlers console_direct+
```

## 手动启动感知

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
source scripts/activate_ros_ml.sh
source install/setup.bash
ros2 launch run_perception system_run.launch.py
```

## 手动启动 SLAM

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
source scripts/activate_ros_ml.sh
source install/setup.bash
ros2 launch slam slam.launch.py rviz:=true
```

## 手动播放数据集

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
source scripts/activate_ros_ml.sh
source install/setup.bash
ros2 bag play /media/yupeng/新加卷/Datasets/rosbag2_2026_02_05-11_01_07 --rate 1.0 --topics /camera1/image_raw /lidar_points /gongji_gnss_ins_64
```

## 查看最近一次运行结果

```bash
cd /home/yupeng/GitHub/DRd26_SLAM
cat "$(ls -td log/runtime/dataset_slam_chain_* | head -n 1)/summary.json"
```
