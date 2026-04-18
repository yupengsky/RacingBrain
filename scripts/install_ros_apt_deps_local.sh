#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_DEPS_DIR="${LOCAL_DEPS_DIR:-${WORKSPACE_DIR}/.ros_deps}"
APT_CACHE_DIR="${LOCAL_DEPS_DIR}/apt"
PACKAGE="${1:-ros-humble-vision-msgs}"

mkdir -p "${APT_CACHE_DIR}"

pushd "${APT_CACHE_DIR}" >/dev/null
apt-get download "${PACKAGE}"
DEB_FILE="$(ls -t "${PACKAGE}"_*.deb | head -n 1)"
dpkg-deb -x "${DEB_FILE}" "${LOCAL_DEPS_DIR}"
popd >/dev/null

LOCAL_ROS_PREFIX="${LOCAL_DEPS_DIR}/opt/ros/humble"
cat > "${LOCAL_ROS_PREFIX}/local_setup.bash" <<EOF
# Workspace-local ROS dependency prefix created by install_ros_apt_deps_local.sh.
_local_ros_prefix="${LOCAL_ROS_PREFIX}"
export AMENT_PREFIX_PATH="\${_local_ros_prefix}\${AMENT_PREFIX_PATH:+:\${AMENT_PREFIX_PATH}}"
export CMAKE_PREFIX_PATH="\${_local_ros_prefix}\${CMAKE_PREFIX_PATH:+:\${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="\${_local_ros_prefix}/lib\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
export PYTHONPATH="\${_local_ros_prefix}/local/lib/python3.10/dist-packages\${PYTHONPATH:+:\${PYTHONPATH}}"
unset _local_ros_prefix
EOF

cat > "${LOCAL_ROS_PREFIX}/local_setup.sh" <<EOF
# Workspace-local ROS dependency prefix created by install_ros_apt_deps_local.sh.
_local_ros_prefix="${LOCAL_ROS_PREFIX}"
AMENT_PREFIX_PATH="\${_local_ros_prefix}\${AMENT_PREFIX_PATH:+:\${AMENT_PREFIX_PATH}}"
CMAKE_PREFIX_PATH="\${_local_ros_prefix}\${CMAKE_PREFIX_PATH:+:\${CMAKE_PREFIX_PATH}}"
LD_LIBRARY_PATH="\${_local_ros_prefix}/lib\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
PYTHONPATH="\${_local_ros_prefix}/local/lib/python3.10/dist-packages\${PYTHONPATH:+:\${PYTHONPATH}}"
export AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
unset _local_ros_prefix
EOF

cat > "${LOCAL_ROS_PREFIX}/local_setup.zsh" <<EOF
# Workspace-local ROS dependency prefix created by install_ros_apt_deps_local.sh.
_local_ros_prefix="${LOCAL_ROS_PREFIX}"
export AMENT_PREFIX_PATH="\${_local_ros_prefix}\${AMENT_PREFIX_PATH:+:\${AMENT_PREFIX_PATH}}"
export CMAKE_PREFIX_PATH="\${_local_ros_prefix}\${CMAKE_PREFIX_PATH:+:\${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="\${_local_ros_prefix}/lib\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
export PYTHONPATH="\${_local_ros_prefix}/local/lib/python3.10/dist-packages\${PYTHONPATH:+:\${PYTHONPATH}}"
unset _local_ros_prefix
EOF

echo "Installed ${PACKAGE} into ${LOCAL_DEPS_DIR}"
echo "Local ROS prefix: ${LOCAL_ROS_PREFIX}"
