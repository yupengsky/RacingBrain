#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${VENV_DIR:-${WORKSPACE_DIR}/.venv_ros_ml}"

if [[ "${VENV_DIR}" != /* ]]; then
  VENV_DIR="${WORKSPACE_DIR}/${VENV_DIR}"
fi

export ROS_VENV_DIR="${VENV_DIR}"
export ROS_PYTHON_EXECUTABLE="${VENV_DIR}/bin/python3"

exec "${WORKSPACE_DIR}/scripts/build_ros_clean.sh" "$@"
