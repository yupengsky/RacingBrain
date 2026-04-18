#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_DEPS_DIR="${LOCAL_DEPS_DIR:-${WORKSPACE_DIR}/.ros_deps}"
CSF_REPO="${CSF_REPO:-https://github.com/jianboqi/CSF.git}"
CSF_SRC="${LOCAL_DEPS_DIR}/src/CSF"
CSF_BUILD="${LOCAL_DEPS_DIR}/build/csf"
CSF_PREFIX="${CSF_PREFIX:-${LOCAL_DEPS_DIR}/csf}"

mkdir -p "${LOCAL_DEPS_DIR}/src" "${LOCAL_DEPS_DIR}/build"

if [[ ! -d "${CSF_SRC}/.git" ]]; then
  git clone --depth 1 "${CSF_REPO}" "${CSF_SRC}"
fi

cmake -S "${CSF_SRC}" -B "${CSF_BUILD}" \
  -DCMAKE_INSTALL_PREFIX="${CSF_PREFIX}" \
  -DBUILD_DEMO=OFF
cmake --build "${CSF_BUILD}" --target install -j"$(nproc)"

echo "Installed CSF into ${CSF_PREFIX}"
echo "Use with: export CSF_ROOT=${CSF_PREFIX}"
