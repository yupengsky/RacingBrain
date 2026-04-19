#!/usr/bin/env bash
set -euo pipefail

# TODO: 在这里实现一键联调脚本，把整条链路串起来：
# 1. source scripts/activate_ros_ml.sh
# 2. source install/setup.bash
# 3. 启动相机检测
# 4. 启动 LiDAR 分割
# 5. 启动融合
# 6. 启动 GNSS / INS 输入
# 7. 启动 SLAM / 点云导出
# 8. 回放 rosbag，并检查关键 topic 是否非空

echo "[TODO] run_dataset_slam_chain.sh"
echo "在这里实现从数据集回放到点云导出的端到端联调脚本。"
