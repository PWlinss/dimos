#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Start or inspect the Pusai side of the isolated Go2/NX DimOS bridge.

set -euo pipefail

ACTION="start"
DIMOS_NX_BRIDGE_MODE="${DIMOS_NX_BRIDGE_MODE:-raw}"
REQUIRE_CAMERA="${DIMOS_NX_REQUIRE_CAMERA:-false}"
AGENTIC_MODE=false
CLOSED_LOOP_MODE=false
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_WORKTREE="${DIMOS_WORKTREE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
DIMOS_VENV="${DIMOS_VENV:-${DIMOS_WORKTREE}/.venv}"
DIMOS_VIEWER="${DIMOS_VIEWER:-}"
DIMOS_CLI="${DIMOS_VENV}/bin/dimos"
PUSAI_DDS_URI="${DIMOS_PUSAI_DDS_URI:-file://${SCRIPT_DIR}/cyclonedds_pusai_nx.xml}"
NX_DATA_VERIFY_DURATION_SEC="${DIMOS_NX_DATA_VERIFY_DURATION_SEC:-20}"
PUSAI_STATE_DIR="${DIMOS_PUSAI_NX_STATE_DIR:-/tmp/dimos-pusai-nx-bridge}"
PUSAI_LOG_DIR="${PUSAI_STATE_DIR}/logs"
CAMERA_DECODER_INPUT_TOPIC="${DIMOS_NX_COLOR_IMAGE_COMPRESSED_TOPIC:-/dimos/color_image/compressed}"
CAMERA_DECODER_OUTPUT_TOPIC="${DIMOS_NX_CAMERA_DECODER_OUTPUT_TOPIC:-/dimos/color_image}"
QWEN_ENV_FILE="${HOME}/.config/dimos/qwen.env"
AGENTIC_CONFIG_FILE="${DIMOS_NX_AGENTIC_CONFIG:-${HOME}/.config/dimos/nx-agentic.env}"

usage() {
  cat <<'EOF'
Usage: start_pusai_nx_bridge.sh [start|status|verify|stop|logs] [--raw|--map] [--with-camera] [--agentic|--closed-loop]

  start   Start the selected Go2/NX DimOS bridge in daemon mode if no instance exists.
  status  Show the current DimOS instance.
  verify  Check the running blueprint and the command-center/Rerun HTTP endpoints.
  stop    Gracefully stop the current DimOS instance.
  logs    Follow the current DimOS logs.
  --raw   Use raw MID360 mapping mode. This is the default.
  --map   Use mature /map input mode: /dimos/map -> DimOS global_costmap.
  --with-camera
          Require /dimos/color_image frames during verification.
  --agentic
          Run unitree-go2-nx-agentic (raw pointcloud, odom, camera and memory web).
          Publish velocity commands directly to /dimos/cmd_vel through the existing Go2 gates. Cannot combine with --map.
          Optional local settings: ~/.config/dimos/nx-agentic.env. Qwen credentials are read only from ~/.config/dimos/qwen.env.
  --closed-loop
          Opt in to unitree-go2-nx-closed-loop: candidate retrieval, bounded navigation
          segments and fresh-image feedback to the Agent. Implies --agentic.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    start|status|verify|stop|logs)
      ACTION="$1"
      ;;
    --raw)
      DIMOS_NX_BRIDGE_MODE=raw
      ;;
    --map)
      DIMOS_NX_BRIDGE_MODE=map
      ;;
    --with-camera)
      REQUIRE_CAMERA=true
      ;;
    --agentic)
      AGENTIC_MODE=true
      ;;
    --closed-loop)
      AGENTIC_MODE=true
      CLOSED_LOOP_MODE=true
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown action or option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

case "${DIMOS_NX_BRIDGE_MODE}" in
  raw)
    BRIDGE_BLUEPRINT="unitree-go2-nx-bridge"
    ;;
  map)
    BRIDGE_BLUEPRINT="unitree-go2-nx-map-bridge"
    ;;
  *)
    echo "Unsupported DIMOS_NX_BRIDGE_MODE=${DIMOS_NX_BRIDGE_MODE}. Use raw or map." >&2
    exit 2
    ;;
esac

if [[ "${AGENTIC_MODE}" == true ]]; then
  if [[ "${DIMOS_NX_BRIDGE_MODE}" != raw ]]; then
    echo "--agentic requires raw pointcloud mode; --map is unsupported." >&2
    exit 2
  fi
  BRIDGE_BLUEPRINT="unitree-go2-nx-agentic"
  if [[ "${CLOSED_LOOP_MODE}" == true ]]; then
    BRIDGE_BLUEPRINT="unitree-go2-nx-closed-loop"
  fi
  if [[ -z "${DIMOS_NX_REQUIRE_CAMERA+x}" ]]; then
    REQUIRE_CAMERA=true
  fi
  if [[ -r "${AGENTIC_CONFIG_FILE}" ]]; then
    # Local non-secret host paths; API credentials have a separate fixed file.
    # shellcheck disable=SC1090
    source "${AGENTIC_CONFIG_FILE}"
  fi
fi

if [[ -z "${DIMOS_VIEWER}" ]]; then
  if [[ "${AGENTIC_MODE}" == true ]]; then
    DIMOS_VIEWER=none
  else
    DIMOS_VIEWER=rerun-web
  fi
fi
DIMOS_CLI="${DIMOS_VENV}/bin/dimos"

[[ -x "${DIMOS_CLI}" ]] || {
  echo "DimOS CLI not found: ${DIMOS_CLI}. Set DIMOS_VENV in ${AGENTIC_CONFIG_FILE} or the environment." >&2
  exit 1
}

# Worker processes inherit this path and must load the bridge worktree, not the main checkout.
export PYTHONPATH="${DIMOS_WORKTREE}${DIMOS_EXTRA_PYTHONPATH:+:${DIMOS_EXTRA_PYTHONPATH}}${PYTHONPATH:+:${PYTHONPATH}}"
export DIMOS_CLIP_MODEL_PATH="${DIMOS_CLIP_MODEL_PATH:-}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${PUSAI_DDS_URI}"
if [[ "${AGENTIC_MODE}" == true ]]; then
  export LCM_DEFAULT_URL="udpm://239.255.76.67:7791?ttl=0"
  export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
fi

# ROS Foxy/Humble setup scripts may read optional variables while unset.
set +u
source /opt/ros/humble/setup.bash
set -u
export DIMOS_NX_LIDAR_TOPIC=/dimos/registered_scan
export DIMOS_NX_ODOM_TOPIC=/dimos/odom
export DIMOS_NX_WORLD_FRAME_ID="${DIMOS_NX_WORLD_FRAME_ID:-odom}"
export DIMOS_NX_COLOR_IMAGE_TOPIC=/dimos/color_image
export DIMOS_NX_CMD_VEL_TOPIC="${DIMOS_NX_CMD_VEL_TOPIC:-/dimos/cmd_vel}"
export DIMOS_NX_MAP_TOPIC="${DIMOS_NX_MAP_TOPIC:-/dimos/map}"
export DIMOS_NX_VOXEL_SIZE="${DIMOS_NX_VOXEL_SIZE:-0.1}"
export DIMOS_NX_COSTMAP_ALGO="${DIMOS_NX_COSTMAP_ALGO:-height_cost}"
export DIMOS_NX_COSTMAP_MARK_FREE_RADIUS="${DIMOS_NX_COSTMAP_MARK_FREE_RADIUS:-0.25}"
export DIMOS_NX_COSTMAP_MIN_HEIGHT="${DIMOS_NX_COSTMAP_MIN_HEIGHT:-0.1}"
export DIMOS_NX_COSTMAP_MAX_HEIGHT="${DIMOS_NX_COSTMAP_MAX_HEIGHT:-2.0}"
export DIMOS_NX_COSTMAP_CAN_CLIMB="${DIMOS_NX_COSTMAP_CAN_CLIMB:-0.15}"
export DIMOS_NX_COSTMAP_SMOOTHING="${DIMOS_NX_COSTMAP_SMOOTHING:-1.0}"
export DIMOS_NX_COSTMAP_MAX_RAY_LENGTH="${DIMOS_NX_COSTMAP_MAX_RAY_LENGTH:-8.0}"
export DIMOS_NX_RERUN_HEAVY_INTERVAL="${DIMOS_NX_RERUN_HEAVY_INTERVAL:-5.0}"
export DIMOS_NX_RERUN_IMAGE_INTERVAL="${DIMOS_NX_RERUN_IMAGE_INTERVAL:-0.2}"
export DIMOS_NX_RERUN_POINTCLOUD_INTERVAL="${DIMOS_NX_RERUN_POINTCLOUD_INTERVAL:-5.0}"
export DIMOS_NX_PLANNER_LINEAR_SPEED="${DIMOS_NX_PLANNER_LINEAR_SPEED:-0.4}"
export DIMOS_NX_PLANNER_ANGULAR_SPEED="${DIMOS_NX_PLANNER_ANGULAR_SPEED:-0.55}"

verify_deployment() {
  local failed=0
  local data_mode="${DIMOS_NX_BRIDGE_MODE}"
  local data_label="pointcloud"
  local verify_args
  local status_output
  if [[ "${data_mode}" == "map" ]]; then
    data_label="map"
  fi
  verify_args=(--duration-sec "${NX_DATA_VERIFY_DURATION_SEC}" --mode "${data_mode}")
  if [[ "${REQUIRE_CAMERA}" == true ]]; then
    verify_args+=(--require-image)
    data_label="${data_label}, camera"
  fi
  status_output="$("${DIMOS_CLI}" status 2>&1 || true)"
  echo "== Pusai DimOS deployment verification =="

  if [[ "${status_output}" == *"Run ID:"* && "${status_output}" == *"Blueprint: ${BRIDGE_BLUEPRINT}"* ]]; then
    echo "[PASS] DimOS blueprint: ${BRIDGE_BLUEPRINT}"
    if "${DIMOS_VENV}/bin/python" "${SCRIPT_DIR}/verify_pusai_nx_data.py" "${verify_args[@]}"; then
      echo "[PASS] NX data: ${data_label} and odom received"
    else
      echo "[FAIL] NX data: expected ${data_mode} data and odom frames" >&2
      failed=1
    fi
  else
    echo "[FAIL] expected running ${BRIDGE_BLUEPRINT} instance" >&2
    printf '%s\n' "${status_output}" >&2
    failed=1
  fi

  if curl -fsS --max-time 5 -o /dev/null http://localhost:7779; then
    echo "[PASS] command center: http://localhost:7779"
  else
    echo "[FAIL] command center is unavailable on port 7779" >&2
    failed=1
  fi

  if [[ "${AGENTIC_MODE}" == true ]]; then
    echo "[SKIP] Rerun Web: disabled for agentic blueprint"
  elif curl -fsS --max-time 5 -o /dev/null http://localhost:9090; then
    echo "[PASS] Rerun Web: http://localhost:9090"
  else
    echo "[FAIL] Rerun Web is unavailable on port 9090" >&2
    failed=1
  fi

  if [[ "${failed}" -eq 0 ]]; then
    echo "[PASS] Pusai DimOS deployment is healthy"
  else
    echo "[FAIL] Pusai DimOS deployment is incomplete" >&2
  fi
  return "${failed}"
}

cleanup_dimos_processes() {
  local pids
  local live_pids

  pids="$(pgrep -f "${DIMOS_VENV}/bin/python" || true)"
  if [[ -z "${pids}" ]]; then
    return 0
  fi

  kill -TERM ${pids} 2>/dev/null || true
  sleep 2

  live_pids="$(pgrep -f "${DIMOS_VENV}/bin/python" || true)"
  if [[ -n "${live_pids}" ]]; then
    kill -KILL ${live_pids} 2>/dev/null || true
  fi
}

load_agentic_model_env() {
  if [[ ! -r "${QWEN_ENV_FILE}" ]]; then
    echo "QWEN environment file is not readable: ${QWEN_ENV_FILE}" >&2
    return 1
  fi

  unset QWEN_API_KEY QWEN_BASE_URL QWEN_API_BASE_URL
  set +u
  # shellcheck disable=SC1090
  source "${QWEN_ENV_FILE}"
  set -u

  local name
  for name in ${!QWEN_@}; do
    export "${name}"
  done
  if [[ -z "${QWEN_API_KEY:-}" ]]; then
    echo "QWEN_API_KEY is missing from ${QWEN_ENV_FILE}" >&2
    return 1
  fi
  export OPENAI_API_KEY="${QWEN_API_KEY}"
  local api_base_url="${QWEN_BASE_URL:-${QWEN_API_BASE_URL:-}}"
  if [[ -n "${api_base_url}" ]]; then
    export OPENAI_BASE_URL="${api_base_url}"
  fi
}

is_live_pid() {
  local pid="$1"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  [[ -r "/proc/${pid}/stat" ]] || return 1
  [[ "$(awk '{print $3}' "/proc/${pid}/stat")" != "Z" ]] || return 1
  kill -0 "${pid}" 2>/dev/null
}

start_process() {
  local name="$1"
  shift
  local pid_file="${PUSAI_STATE_DIR}/${name}.pid"
  local log_file="${PUSAI_LOG_DIR}/${name}.log"

  mkdir -p "${PUSAI_LOG_DIR}"
  if [[ -f "${pid_file}" ]] && is_live_pid "$(<"${pid_file}")"; then
    echo "${name} already running (pid $(<"${pid_file}"))"
    return
  fi

  rm -f "${pid_file}"
  nohup setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  echo $! >"${pid_file}"
  echo "started ${name} (pid $(<"${pid_file}"), log ${log_file})"
}

stop_process() {
  local name="$1"
  local pid_file="${PUSAI_STATE_DIR}/${name}.pid"

  [[ -f "${pid_file}" ]] || return 0
  local pid
  pid="$(<"${pid_file}")"
  if is_live_pid "${pid}"; then
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 1
    if is_live_pid "${pid}"; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi
  rm -f "${pid_file}"
}

find_matching_camera_decoder() {
  local process_line pid args
  while IFS= read -r process_line; do
    pid="${process_line%% *}"
    args="${process_line#* }"
    [[ "${args}" == *compressed_image_to_image.py* ]] || continue
    [[ "${args}" == *"input_topic:=${CAMERA_DECODER_INPUT_TOPIC}"* ]] || continue
    [[ "${args}" == *"output_topic:=${CAMERA_DECODER_OUTPUT_TOPIC}"* ]] || continue
    printf '%s\n' "${pid}"
    return 0
  done < <(pgrep -af -- 'compressed_image_to_image[.]py' || true)
  return 1
}

start_camera_decoder() {
  if [[ "${AGENTIC_MODE}" == true ]]; then
    local existing_pid
    existing_pid="$(find_matching_camera_decoder || true)"
    if [[ -n "${existing_pid}" ]]; then
      echo "reusing camera decoder (pid ${existing_pid}); it will not be managed by this script"
      return 0
    fi
  fi
  start_process camera_decoder env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${PUSAI_DDS_URI}" \
    PYTHONPATH="${PYTHONPATH}" \
    "${DIMOS_VENV}/bin/python" "${SCRIPT_DIR}/compressed_image_to_image.py" --ros-args \
    -p input_topic:="${CAMERA_DECODER_INPUT_TOPIC}" \
    -p output_topic:="${CAMERA_DECODER_OUTPUT_TOPIC}"
}

stop_camera_decoder() {
  stop_process camera_decoder
}

wait_for_bridge_registration() {
  local status_output

  for _ in $(seq 1 20); do
    status_output="$("${DIMOS_CLI}" status 2>&1 || true)"
    if [[ "${status_output}" == *"Run ID:"* && "${status_output}" == *"Blueprint: ${BRIDGE_BLUEPRINT}"* ]]; then
      return 0
    fi
    sleep 1
  done

  return 1
}

pusai_lcm_system_config_ok() {
  ip link show lo | grep -q "MULTICAST" || return 1
  ip route show 224.0.0.0/4 dev lo | grep -q "^224.0.0.0/4" || return 1
  [[ "$(sysctl -n net.core.rmem_max)" -ge 67108864 ]] || return 1
  [[ "$(sysctl -n net.core.rmem_default)" -ge 67108864 ]] || return 1
}

ensure_pusai_lcm_system_config() {
  if pusai_lcm_system_config_ok; then
    return 0
  fi

  if ! sudo -n true 2>/dev/null; then
    cat >&2 <<'EOF'
Pusai LCM system config is missing and passwordless sudo is not available.
Run these once, then start DimOS again:

  sudo ip link set lo multicast on
  sudo ip route replace 224.0.0.0/4 dev lo
  sudo sysctl -w net.core.rmem_max=67108864
  sudo sysctl -w net.core.rmem_default=67108864
EOF
    return 1
  fi

  sudo -n ip link set lo multicast on
  sudo -n ip route replace 224.0.0.0/4 dev lo
  sudo -n sysctl -w net.core.rmem_max=67108864 >/dev/null
  sudo -n sysctl -w net.core.rmem_default=67108864 >/dev/null
}

case "${ACTION}" in
  start)
    if [[ "${AGENTIC_MODE}" == true ]]; then
      if [[ -z "${DIMOS_CLIP_MODEL_PATH}" || ! -f "${DIMOS_CLIP_MODEL_PATH}" ]]; then
        echo "DIMOS_CLIP_MODEL_PATH must name an existing local ONNX model file; set it in ${AGENTIC_CONFIG_FILE}." >&2
        exit 1
      fi
      if [[ ! -f "${DIMOS_WORKTREE}/dimos/web/command-center-extension/dist-standalone/index.html" ]]; then
        echo "Command Center build missing. Run scripts/prepare_pusai_nx_agentic.sh once." >&2
        exit 1
      fi
      load_agentic_model_env
      export LCM_DEFAULT_URL="udpm://239.255.76.67:7791?ttl=0"
    fi
    ensure_pusai_lcm_system_config
    status_output="$("${DIMOS_CLI}" status 2>&1 || true)"
	    if [[ "${status_output}" == *"Run ID:"* ]]; then
	      echo "A DimOS instance is already running; refusing to create a duplicate."
	      printf '%s\n' "${status_output}"
	      start_camera_decoder
	      verify_deployment
	      exit $?
	    fi
	    if [[ "${AGENTIC_MODE}" != true ]]; then
	      cleanup_dimos_processes
	    fi
	    start_camera_decoder
	    cd "${DIMOS_WORKTREE}"
	    "${DIMOS_CLI}" --viewer "${DIMOS_VIEWER}" run "${BRIDGE_BLUEPRINT}" --daemon
    wait_for_bridge_registration || true
    verify_deployment
    ;;
  status)
    exec "${DIMOS_CLI}" status
    ;;
  verify)
    verify_deployment
    ;;
	  stop)
	    stop_camera_decoder
	    "${DIMOS_CLI}" stop || true
	    if [[ "${AGENTIC_MODE}" != true ]]; then
	      cleanup_dimos_processes
	    fi
	    ;;
  logs)
    exec "${DIMOS_CLI}" log -f
    ;;
  --help|-h)
    usage
    ;;
  *)
    echo "Unknown action: ${ACTION}" >&2
    usage >&2
    exit 2
    ;;
esac
