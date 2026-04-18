#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
VENV_DIR="${VENV_DIR:-${WORKSPACE_DIR}/.venv_ros_ml}"
BOOTSTRAP_DIR="${BOOTSTRAP_DIR:-${WORKSPACE_DIR}/.bootstrap_tools}"
TORCH_VARIANT="${TORCH_VARIANT:-cpu}"
TORCH_INDEX_URL="${TORCH_INDEX_URL:-https://download.pytorch.org/whl/${TORCH_VARIANT}}"

if [[ "${VENV_DIR}" != /* ]]; then
  VENV_DIR="${WORKSPACE_DIR}/${VENV_DIR}"
fi

if [[ "${BOOTSTRAP_DIR}" != /* ]]; then
  BOOTSTRAP_DIR="${WORKSPACE_DIR}/${BOOTSTRAP_DIR}"
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

export PYTHONNOUSERSITE=1

if [[ ! -d "${VENV_DIR}" ]]; then
  echo "Creating ROS ML venv at ${VENV_DIR}"
  if ! /usr/bin/python3 -m venv --system-site-packages "${VENV_DIR}"; then
    echo "Falling back to user-scoped virtualenv bootstrap"
    rm -rf "${VENV_DIR}"
    mkdir -p "${BOOTSTRAP_DIR}"
    if ! PYTHONPATH="${BOOTSTRAP_DIR}" /usr/bin/python3 - <<'PY' >/dev/null 2>&1
import importlib
importlib.import_module("virtualenv")
PY
    then
      /usr/bin/python3 -m pip install --target "${BOOTSTRAP_DIR}" virtualenv
    fi
    PYTHONPATH="${BOOTSTRAP_DIR}" /usr/bin/python3 -m virtualenv --system-site-packages "${VENV_DIR}"
  fi
else
  echo "Reusing existing ROS ML venv at ${VENV_DIR}"
fi

VENV_PYTHON="${VENV_DIR}/bin/python3"
if [[ ! -x "${VENV_PYTHON}" ]]; then
  echo "Venv python not found: ${VENV_PYTHON}" >&2
  exit 1
fi

"${VENV_PYTHON}" -m pip install --upgrade pip wheel
"${VENV_PYTHON}" -m pip install --upgrade "setuptools<80"
"${VENV_PYTHON}" -m pip install \
  filelock \
  typing-extensions
"${VENV_PYTHON}" -m pip install \
  --index-url "${TORCH_INDEX_URL}" \
  torch==2.3.1 \
  torchvision==0.18.1
"${VENV_PYTHON}" -m pip install \
  "numpy<2" \
  "opencv-python-headless==4.10.0.84" \
  pandas \
  polars \
  psutil \
  py-cpuinfo \
  seaborn \
  tqdm \
  ultralytics-thop
"${VENV_PYTHON}" -m pip install --no-deps ultralytics

"${VENV_PYTHON}" - <<'PY'
import importlib

modules = ["rclpy", "cv2", "numpy", "torch", "torchvision", "ultralytics"]
for name in modules:
    mod = importlib.import_module(name)
    print(f"{name}: OK {getattr(mod, '__version__', '')}")

import torch
print("torch.cuda.is_available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("torch.cuda.device_name:", torch.cuda.get_device_name(0))
PY

echo
echo "ROS ML venv ready: ${VENV_DIR}"
echo "Torch variant: ${TORCH_VARIANT}"
echo "Build with: ROS_VENV_DIR=${VENV_DIR} ./scripts/build_ros_clean.sh"
