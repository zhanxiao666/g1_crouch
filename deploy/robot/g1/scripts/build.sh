#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROBOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
BUILD_DIR="${ROBOT_DIR}/build"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
ENABLE_ARM_IK_LCM="${ENABLE_ARM_IK_LCM:-ON}"
JOBS="${JOBS:-$(nproc)}"

cmake -S "${ROBOT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
  -DENABLE_ARM_IK_LCM="${ENABLE_ARM_IK_LCM}"
cmake --build "${BUILD_DIR}" -j"${JOBS}"

echo "Build finished: ${BUILD_DIR}/g1_ctrl"
