#!/usr/bin/env bash
set -euo pipefail

# TODO: 在这里补 colcon clean build 逻辑。
# 推荐职责：
# 1. 清理 build/ install/ log/
# 2. source scripts/activate_ros_ml.sh
# 3. colcon build --symlink-install --cmake-clean-cache
# 4. 支持 --packages-select / --event-handlers 等透传参数

echo "[TODO] build_ros_clean.sh"
echo "在这里实现一次干净的 ROS 2 工作区构建。"
