#!/usr/bin/env bash
set -euo pipefail

# Motion-capable: start only with on-site supervision and a clear test area.
ACTION="${1:-status}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_WORKTREE="${DIMOS_WORKTREE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
DIMOS_VENV="${DIMOS_VENV:-${DIMOS_WORKTREE}/.venv}"
CONDA_SH="/home/unitree/.local/share/dimos-nx-onboard/miniforge3/etc/profile.d/conda.sh"
GOMP_LIBRARY="${DIMOS_VENV}/lib/libgomp.so.1"
QWEN_ENV_FILE="${HOME}/.config/dimos/qwen.env"
CLI="${DIMOS_VENV}/bin/python"

case "${ACTION}" in
  start|status|stop|logs) ;;
  *) echo "Usage: $0 [start|status|stop|logs]" >&2; exit 2 ;;
esac

[[ -r "${CONDA_SH}" ]] || { echo "Conda setup missing: ${CONDA_SH}" >&2; exit 1; }
[[ -f "${GOMP_LIBRARY}" ]] || { echo "OpenMP preload missing: ${GOMP_LIBRARY}" >&2; exit 1; }
set +u
source "${CONDA_SH}"
conda activate "${DIMOS_VENV}"
set -u
cd "${DIMOS_WORKTREE}"
unset PYTHONPATH ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${DIMOS_NX_DDS_URI:-file:///home/unitree/cyclonedds_ws/cyclonedds.xml}"
export LD_PRELOAD="${GOMP_LIBRARY}" CI=1
export LCM_DEFAULT_URL="udpm://239.255.76.67:7791?ttl=0"
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export DIMOS_NX_LIDAR_TOPIC=/dimos/registered_scan DIMOS_NX_ODOM_TOPIC=/dimos/odom
export DIMOS_NX_WORLD_FRAME_ID="${DIMOS_NX_WORLD_FRAME_ID:-odom}"
export DIMOS_NX_COLOR_IMAGE_TOPIC=/dimos/color_image
export DIMOS_NX_CMD_VEL_TOPIC="${DIMOS_NX_CMD_VEL_TOPIC:-/dimos/cmd_vel}"
export DIMOS_CLIP_MODEL_PATH="${DIMOS_CLIP_MODEL_PATH:-${DIMOS_WORKTREE}/data/models_clip/model.onnx}"
export DIMOS_NX_VOXEL_SIZE="${DIMOS_NX_VOXEL_SIZE:-0.1}"
export DIMOS_NX_COSTMAP_ALGO="${DIMOS_NX_COSTMAP_ALGO:-height_cost}"
export DIMOS_NX_COSTMAP_MARK_FREE_RADIUS="${DIMOS_NX_COSTMAP_MARK_FREE_RADIUS:-0.25}"
export DIMOS_NX_PLANNER_LINEAR_SPEED="${DIMOS_NX_PLANNER_LINEAR_SPEED:-0.4}"
export DIMOS_NX_PLANNER_ANGULAR_SPEED="${DIMOS_NX_PLANNER_ANGULAR_SPEED:-0.55}"

if [[ "${ACTION}" == start ]]; then
  [[ -f "${DIMOS_CLIP_MODEL_PATH}" ]] || { echo "CLIP model missing: ${DIMOS_CLIP_MODEL_PATH}" >&2; exit 1; }
  [[ -f "${DIMOS_WORKTREE}/dimos/web/command-center-extension/dist-standalone/index.html" ]] || { echo "Command Center build missing." >&2; exit 1; }
  [[ -r "${QWEN_ENV_FILE}" ]] || { echo "Qwen env file missing: ${QWEN_ENV_FILE}" >&2; exit 1; }
  set -a
  source "${QWEN_ENV_FILE}"
  set +a
  export OPENAI_API_KEY="${QWEN_API_KEY:-${OPENAI_API_KEY:-}}"
  export OPENAI_BASE_URL="${QWEN_BASE_URL:-${QWEN_API_BASE_URL:-${OPENAI_BASE_URL:-}}}"
  [[ -n "${OPENAI_API_KEY}" && -n "${OPENAI_BASE_URL}" ]] || { echo "Qwen env must define an API key and base URL." >&2; exit 1; }
fi
case "${ACTION}" in
  start) exec "${CLI}" -m dimos.robot.cli.dimos --viewer none run unitree-go2-nx-closed-loop --daemon ;;
  status) exec "${CLI}" -m dimos.robot.cli.dimos status ;;
  stop) exec "${CLI}" -m dimos.robot.cli.dimos stop ;;
  logs) exec "${CLI}" -m dimos.robot.cli.dimos log -f ;;
esac
