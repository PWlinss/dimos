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
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_WORKTREE="${DIMOS_WORKTREE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
DIMOS_HOST_ROOT="$(cd "${DIMOS_WORKTREE}/../.." && pwd)"
DIMOS_VENV="${DIMOS_VENV:-${DIMOS_HOST_ROOT}/.venv_ros310}"
DIMOS_VIEWER="${DIMOS_VIEWER:-rerun-web}"
DIMOS_CLI="${DIMOS_VENV}/bin/dimos"
PUSAI_DDS_URI="${DIMOS_PUSAI_DDS_URI:-file://${SCRIPT_DIR}/cyclonedds_pusai_nx.xml}"
NX_DATA_VERIFY_DURATION_SEC="${DIMOS_NX_DATA_VERIFY_DURATION_SEC:-20}"
PUSAI_STATE_DIR="${DIMOS_PUSAI_NX_STATE_DIR:-/tmp/dimos-pusai-nx-bridge}"
PUSAI_LOG_DIR="${PUSAI_STATE_DIR}/logs"
CAMERA_DECODER_INPUT_TOPIC="${DIMOS_NX_COLOR_IMAGE_COMPRESSED_TOPIC:-/dimos/color_image/compressed}"
CAMERA_DECODER_OUTPUT_TOPIC="${DIMOS_NX_CAMERA_DECODER_OUTPUT_TOPIC:-/dimos/color_image}"

usage() {
  cat <<'EOF'
Usage: start_pusai_nx_bridge.sh [start|status|verify|stop|logs] [--raw|--map] [--with-camera]

  start   Start the selected Go2/NX DimOS bridge in daemon mode if no instance exists.
  status  Show the current DimOS instance.
  verify  Check the running blueprint and the command-center/Rerun HTTP endpoints.
  stop    Gracefully stop the current DimOS instance.
  logs    Follow the current DimOS logs.
  --raw   Use raw MID360 mapping mode. This is the default.
  --map   Use mature /map input mode: /dimos/map -> DimOS global_costmap.
  --with-camera
          Require /dimos/color_image frames during verification.
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

[[ -x "${DIMOS_CLI}" ]] || {
  echo "DimOS CLI not found: ${DIMOS_CLI}. Set DIMOS_VENV explicitly." >&2
  exit 1
}

# Worker processes inherit this path and must load the bridge worktree, not the main checkout.
export PYTHONPATH="${DIMOS_WORKTREE}${PYTHONPATH:+:${PYTHONPATH}}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${PUSAI_DDS_URI}"

# ROS Foxy/Humble setup scripts may read optional variables while unset.
set +u
source /opt/ros/humble/setup.bash
set -u
export DIMOS_NX_LIDAR_TOPIC=/dimos/registered_scan
export DIMOS_NX_ODOM_TOPIC=/dimos/odom
export DIMOS_NX_WORLD_FRAME_ID="${DIMOS_NX_WORLD_FRAME_ID:-odom}"
export DIMOS_NX_COLOR_IMAGE_TOPIC=/dimos/color_image
export DIMOS_NX_CMD_VEL_TOPIC=/dimos/cmd_vel
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

  if curl -fsS --max-time 5 -o /dev/null http://localhost:9090; then
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

start_camera_decoder() {
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
    ensure_pusai_lcm_system_config
    status_output="$("${DIMOS_CLI}" status 2>&1 || true)"
	    if [[ "${status_output}" == *"Run ID:"* ]]; then
	      echo "A DimOS instance is already running; refusing to create a duplicate."
	      printf '%s\n' "${status_output}"
	      start_camera_decoder
	      verify_deployment
	      exit $?
	    fi
	    cleanup_dimos_processes
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
	    cleanup_dimos_processes
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
