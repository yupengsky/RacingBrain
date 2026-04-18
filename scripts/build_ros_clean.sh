#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_VENV_DIR="${ROS_VENV_DIR:-}"
LOCAL_ROS_DEPS_PREFIX="${LOCAL_ROS_DEPS_PREFIX:-${WORKSPACE_DIR}/.ros_deps/opt/ros/humble}"
CSF_ROOT="${CSF_ROOT:-${WORKSPACE_DIR}/.ros_deps/csf}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ -n "${ROS_VENV_DIR}" && "${ROS_VENV_DIR}" != /* ]]; then
  ROS_VENV_DIR="${WORKSPACE_DIR}/${ROS_VENV_DIR}"
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
export PYTHONNOUSERSITE=1

FILTERED_PATH="$(
  printf '%s' "${PATH:-}" \
    | tr ':' '\n' \
    | awk 'NF && $0 !~ /(^|\/)(miniconda|anaconda|mambaforge|miniforge|conda)(\/|$)/ && !seen[$0]++' \
    | paste -sd ':' -
)"

VENV_BIN=""
if [[ -n "${ROS_VENV_DIR}" ]]; then
  VENV_BIN="${ROS_VENV_DIR}/bin"
  if [[ ! -x "${VENV_BIN}/python3" ]]; then
    echo "ROS venv python not found: ${VENV_BIN}/python3" >&2
    exit 1
  fi
fi

export PATH="${VENV_BIN:+${VENV_BIN}:}/opt/ros/humble/bin:/usr/sbin:/usr/bin:/sbin:/bin${FILTERED_PATH:+:${FILTERED_PATH}}"
hash -r

DEFAULT_PYTHON_EXECUTABLE="/usr/bin/python3"
if [[ -n "${VENV_BIN}" ]]; then
  DEFAULT_PYTHON_EXECUTABLE="${VENV_BIN}/python3"
fi
ROS_PYTHON_EXECUTABLE="${ROS_PYTHON_EXECUTABLE:-${DEFAULT_PYTHON_EXECUTABLE}}"

if [[ ! -x "${ROS_PYTHON_EXECUTABLE}" ]]; then
  echo "Python executable not found: ${ROS_PYTHON_EXECUTABLE}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

if [[ -d "${LOCAL_ROS_DEPS_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${LOCAL_ROS_DEPS_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export CMAKE_PREFIX_PATH="${LOCAL_ROS_DEPS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LOCAL_ROS_DEPS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PYTHONPATH="${LOCAL_ROS_DEPS_PREFIX}/local/lib/python3.10/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
fi

if [[ -d "${CSF_ROOT}" ]]; then
  export CSF_ROOT
  export LD_LIBRARY_PATH="${CSF_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

echo "Workspace: ${WORKSPACE_DIR}"
echo "ROS_DISTRO: ${ROS_DISTRO:-unset}"
echo "ros2: $(command -v ros2)"
echo "python3: $(command -v python3)"
echo "ROS_PYTHON_EXECUTABLE: ${ROS_PYTHON_EXECUTABLE}"
"${ROS_PYTHON_EXECUTABLE}" - <<'PY'
import sys
print("python_version:", sys.version.replace("\n", " "))
print("python_executable:", sys.executable)
import em
import numpy
print("python_ros_deps: em, numpy OK")
import colcon_core
print("python_colcon: OK")
PY

cd "${WORKSPACE_DIR}"

DEFAULT_BASE_PATHS=(
  "gnss/gnss_ins_msg"
  "gnss/cpp_pubsub"
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

"${ROS_PYTHON_EXECUTABLE}" -m colcon build \
  --symlink-install \
  --cmake-clean-cache \
  "${COLCON_ARGS[@]}" \
  --cmake-args \
    "-DPython3_EXECUTABLE=${ROS_PYTHON_EXECUTABLE}"
