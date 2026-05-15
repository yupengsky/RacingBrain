#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Please source this script: source scripts/activate_ros_ml.sh" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
VENV_DIR="${VENV_DIR:-${WORKSPACE_DIR}/.venv_ros_ml}"
LOCAL_ROS_DEPS_PREFIX="${LOCAL_ROS_DEPS_PREFIX:-${WORKSPACE_DIR}/.ros_deps/opt/ros/humble}"
CSF_ROOT="${CSF_ROOT:-${WORKSPACE_DIR}/.ros_deps/csf}"

if [[ "${VENV_DIR}" != /* ]]; then
  VENV_DIR="${WORKSPACE_DIR}/${VENV_DIR}"
fi

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

RESTORE_NOUNSET=0
case $- in
  *u*) RESTORE_NOUNSET=1 ;;
esac

set +u
source "${ROS_SETUP}"
source "${VENV_DIR}/bin/activate"
if [[ "${RESTORE_NOUNSET}" -eq 1 ]]; then
  set -u
fi

VENV_PYTHONPATH="$(
  "${VENV_DIR}/bin/python3" - <<'PY'
import sysconfig

paths = [
    sysconfig.get_path("purelib"),
    sysconfig.get_path("platlib"),
]
print(":".join(dict.fromkeys(path for path in paths if path)))
PY
)"
if [[ -n "${VENV_PYTHONPATH}" ]]; then
  export PYTHONPATH="${VENV_PYTHONPATH}${PYTHONPATH:+:${PYTHONPATH}}"
fi

if [[ -d "${LOCAL_ROS_DEPS_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${LOCAL_ROS_DEPS_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export CMAKE_PREFIX_PATH="${LOCAL_ROS_DEPS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LOCAL_ROS_DEPS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  if [[ -d "${LOCAL_ROS_DEPS_PREFIX}/lib/x86_64-linux-gnu" ]]; then
    export LD_LIBRARY_PATH="${LOCAL_ROS_DEPS_PREFIX}/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  export PYTHONPATH="${LOCAL_ROS_DEPS_PREFIX}/local/lib/python3.10/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
fi

if [[ -d "${CSF_ROOT}" ]]; then
  export CSF_ROOT
  export LD_LIBRARY_PATH="${CSF_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

export ROS_VENV_DIR="${VENV_DIR}"
export ROS_PYTHON_EXECUTABLE="${VENV_DIR}/bin/python3"

cd "${WORKSPACE_DIR}"

echo "ROS env ready in ${WORKSPACE_DIR}"
echo "python3: $(command -v python3)"
echo "ROS_PYTHON_EXECUTABLE: ${ROS_PYTHON_EXECUTABLE}"
