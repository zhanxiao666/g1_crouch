#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROBOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
BUILD_DIR="${ROBOT_DIR}/build"
CTRL_BIN="${BUILD_DIR}/g1_ctrl"

usage() {
  cat <<'EOF'
Usage:
  run_ctrl.sh [--mode sim|real] [--network <nic>] [--log]

Options:
  --mode <sim|real>   Convenience mode. "sim" defaults network to lo.
  --network <nic>     DDS network interface, such as lo or enp5s0.
  --log               Enable controller log output.
  --help              Show this message.
EOF
}

MODE="real"
NETWORK=""
ENABLE_LOG=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --network)
      NETWORK="${2:-}"
      shift 2
      ;;
    --log)
      ENABLE_LOG=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ "${MODE}" == "sim" && -z "${NETWORK}" ]]; then
  NETWORK="lo"
fi

if [[ -z "${NETWORK}" ]]; then
  echo "--network is required for real robot deployment." >&2
  usage >&2
  exit 1
fi

if [[ ! -x "${CTRL_BIN}" ]]; then
  echo "Controller binary not found: ${CTRL_BIN}" >&2
  echo "Build first with: ${ROBOT_DIR}/scripts/build.sh" >&2
  exit 1
fi

CMD=("${CTRL_BIN}" "--network=${NETWORK}")
if [[ "${ENABLE_LOG}" -eq 1 ]]; then
  CMD+=("--log")
fi

echo "Starting G1 controller..."
echo "  mode    : ${MODE}"
echo "  network : ${NETWORK}"
echo
echo "Robot checklist:"
echo "  - Robot is suspended or otherwise protected for first bring-up"
echo "  - Robot is in zero-torque mode"
echo "  - Debug mode has been entered with L2 + R2"
echo "  - Transition path: L2 + Up -> FixStand, then R2 + A -> policy control"
echo

exec "${CMD[@]}"
