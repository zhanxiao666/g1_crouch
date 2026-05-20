#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROBOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
POLICY_ROOT="${ROBOT_DIR}/config/policy"

usage() {
  cat <<'EOF'
Usage:
  prepare_policy.sh --source <training-output-or-policy.onnx> [options]

Options:
  --source <path>         Training output directory, deploy directory, or policy.onnx file.
  --policy-group <name>   Policy group under config/policy. Default: velocity
  --target-name <name>    Target folder name. Default: current timestamp
  --deploy-yaml <path>    Explicit deploy.yaml to copy. Default: auto-detect or use velocity/v0 template
  --force                 Overwrite existing target directory
  --help                  Show this message

Examples:
  ./scripts/prepare_policy.sh --source /path/to/logs/rsl_rl/g1_velocity/2026-05-07_12-00-00
  ./scripts/prepare_policy.sh --source /path/to/policy.onnx --target-name v20260507
EOF
}

SOURCE_PATH=""
POLICY_GROUP="velocity"
TARGET_NAME="$(date +%Y%m%d_%H%M%S)"
DEPLOY_YAML=""
FORCE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source)
      SOURCE_PATH="${2:-}"
      shift 2
      ;;
    --policy-group)
      POLICY_GROUP="${2:-}"
      shift 2
      ;;
    --target-name)
      TARGET_NAME="${2:-}"
      shift 2
      ;;
    --deploy-yaml)
      DEPLOY_YAML="${2:-}"
      shift 2
      ;;
    --force)
      FORCE=1
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

if [[ -z "${SOURCE_PATH}" ]]; then
  echo "--source is required." >&2
  usage >&2
  exit 1
fi

if [[ ! -e "${SOURCE_PATH}" ]]; then
  echo "Source does not exist: ${SOURCE_PATH}" >&2
  exit 1
fi

find_policy_onnx() {
  local source="$1"
  if [[ -f "${source}" ]]; then
    echo "${source}"
    return 0
  fi
  if [[ -f "${source}/policy.onnx" ]]; then
    echo "${source}/policy.onnx"
    return 0
  fi
  if [[ -f "${source}/exported/policy.onnx" ]]; then
    echo "${source}/exported/policy.onnx"
    return 0
  fi
  return 1
}

resolve_deploy_yaml() {
  local source_dir="$1"
  local default_template="${ROBOT_DIR}/config/policy/velocity/v0/params/deploy.yaml"

  if [[ -n "${DEPLOY_YAML}" ]]; then
    echo "${DEPLOY_YAML}"
    return 0
  fi
  if [[ -f "${source_dir}/params/deploy.yaml" ]]; then
    echo "${source_dir}/params/deploy.yaml"
    return 0
  fi
  if [[ -f "${source_dir}/deploy.yaml" ]]; then
    echo "${source_dir}/deploy.yaml"
    return 0
  fi
  echo "${default_template}"
}

POLICY_ONNX=$(find_policy_onnx "${SOURCE_PATH}") || {
  echo "Could not find policy.onnx from source: ${SOURCE_PATH}" >&2
  exit 1
}

SOURCE_DIR=$(dirname "${POLICY_ONNX}")
POLICY_DATA="${POLICY_ONNX}.data"
DEPLOY_YAML_PATH=$(resolve_deploy_yaml "${SOURCE_DIR}")

if [[ ! -f "${DEPLOY_YAML_PATH}" ]]; then
  echo "deploy.yaml not found: ${DEPLOY_YAML_PATH}" >&2
  exit 1
fi

TARGET_DIR="${POLICY_ROOT}/${POLICY_GROUP}/${TARGET_NAME}"
TARGET_EXPORTED_DIR="${TARGET_DIR}/exported"
TARGET_PARAMS_DIR="${TARGET_DIR}/params"

if [[ -e "${TARGET_DIR}" && "${FORCE}" -ne 1 ]]; then
  echo "Target already exists: ${TARGET_DIR}" >&2
  echo "Use --force to overwrite, or choose a different --target-name." >&2
  exit 1
fi

rm -rf "${TARGET_DIR}"
mkdir -p "${TARGET_EXPORTED_DIR}" "${TARGET_PARAMS_DIR}"

cp "${POLICY_ONNX}" "${TARGET_EXPORTED_DIR}/policy.onnx"
cp "${DEPLOY_YAML_PATH}" "${TARGET_PARAMS_DIR}/deploy.yaml"

if [[ -f "${POLICY_DATA}" ]]; then
  cp "${POLICY_DATA}" "${TARGET_EXPORTED_DIR}/policy.onnx.data"
fi

echo "Prepared G1 deploy policy:"
echo "  policy source : ${POLICY_ONNX}"
echo "  deploy yaml   : ${DEPLOY_YAML_PATH}"
echo "  target dir    : ${TARGET_DIR}"
echo
echo "Next steps:"
echo "  1. Review ${TARGET_PARAMS_DIR}/deploy.yaml and confirm joint order / scales match your trained policy."
echo "  2. Build controller: ${ROBOT_DIR}/scripts/build.sh"
echo "  3. Run simulator deploy: ${ROBOT_DIR}/scripts/run_ctrl.sh --mode sim"
echo "  4. Run real robot deploy: ${ROBOT_DIR}/scripts/run_ctrl.sh --network <your_nic>"
