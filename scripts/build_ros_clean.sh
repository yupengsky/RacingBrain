#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

# Keep ROS builds on the Ubuntu/ROS Python stack, even if the shell was opened
# from a conda-enabled terminal.
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset CONDA_PROMPT_MODIFIER
unset CONDA_EXE
unset CONDA_PYTHON_EXE
unset _CONDA_EXE
unset _CONDA_ROOT
unset PYTHONHOME
unset PYTHONPATH

FILTERED_PATH="$(
  printf '%s' "${PATH:-}" \
    | tr ':' '\n' \
    | awk 'NF && $0 !~ /(^|\/)(miniconda|anaconda|mambaforge|miniforge|conda)(\/|$)/ && !seen[$0]++' \
    | paste -sd ':' -
)"
export PATH="/opt/ros/humble/bin:/usr/sbin:/usr/bin:/sbin:/bin${FILTERED_PATH:+:${FILTERED_PATH}}"
hash -r

set +u
source "${ROS_SETUP}"
set -u

echo "Workspace: ${WORKSPACE_DIR}"
echo "ROS_DISTRO: ${ROS_DISTRO:-unset}"
echo "ros2: $(command -v ros2)"
echo "colcon: $(command -v colcon)"
echo "python3: $(command -v python3)"
python3 - <<'PY'
import sys
print("python_version:", sys.version.replace("\n", " "))
print("python_executable:", sys.executable)
import em
import numpy
print("python_ros_deps: em, numpy OK")
PY

cd "${WORKSPACE_DIR}"

DEFAULT_BASE_PATHS=(
  "slam"
  "perception/src/cone_ws/src/cone_interfaces"
  "perception/src/cone_ws/src/cone_detector"
  "perception/src/cone_segmentation_test_3d/src/test_cone_segmentation"
  "perception/src/fs_fusion_box"
  "perception/src/run_perception"
)

COLCON_ARGS=("$@")
HAS_BASE_PATHS=0
for arg in "${COLCON_ARGS[@]}"; do
  if [[ "${arg}" == "--base-paths" || "${arg}" == --base-paths=* ]]; then
    HAS_BASE_PATHS=1
    break
  fi
done

if [[ "${HAS_BASE_PATHS}" -eq 0 ]]; then
  EXISTING_BASE_PATHS=()
  for path in "${DEFAULT_BASE_PATHS[@]}"; do
    if [[ -d "${path}" ]]; then
      EXISTING_BASE_PATHS+=("${path}")
    fi
  done

  if [[ "${#EXISTING_BASE_PATHS[@]}" -gt 0 ]]; then
    COLCON_ARGS=(--base-paths "${EXISTING_BASE_PATHS[@]}" "${COLCON_ARGS[@]}")
  fi
fi

colcon build \
  --symlink-install \
  --cmake-clean-cache \
  "${COLCON_ARGS[@]}" \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3
