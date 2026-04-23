#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export RACINGBRAIN_WORKSPACE="${WORKSPACE_DIR}"
export DRD26_PATH_CONFIG="${DRD26_PATH_CONFIG:-${WORKSPACE_DIR}/LocalizationMapping/config/hardcoded_paths.ini}"

source "${WORKSPACE_DIR}/scripts/activate_ros_ml.sh" >/tmp/racingbrain_activate_ros_ml.log

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "install/setup.bash not found. Run ./scripts/build_ros_clean.sh first." >&2
  exit 1
fi

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec ros2 launch racingbrain localization_mapping.launch.py "$@"
