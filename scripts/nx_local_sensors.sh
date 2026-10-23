#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
BRIDGE_DIR="${ROOT_DIR}/docker/navigation/nx_bridge"
STATE_DIR="${ROOT_DIR}/.local/nx-local-sensors"
LOG_DIR="${STATE_DIR}/logs"
FOXY_SETUP="/opt/ros/foxy/setup.bash"
CYCLONEDDS_SETUP="/home/unitree/cyclonedds_ws/install/setup.bash"
UNITREE_SETUP="/home/unitree/ros2projects/go2_control_driver/install/setup.bash"
CYCLONEDDS_URI="file:///home/unitree/cyclonedds_ws/cyclonedds.xml"

adapter_script="${BRIDGE_DIR}/nx_local_sensor_adapter.py"
decoder_script="${ROOT_DIR}/scripts/compressed_image_to_image.py"

usage() {
  echo "Usage: $0 {start|status|stop}"
}

ros_python() (
  set +u
  unset PYTHONPATH ROS_DOMAIN_ID
  source "${FOXY_SETUP}"
  source "${CYCLONEDDS_SETUP}"
  source "${UNITREE_SETUP}"
  set -u
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI
  export PYTHONPATH="${BRIDGE_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
  /usr/bin/python3 "$@"
)

process_cmdline() {
  local pid="$1"
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' <"/proc/${pid}/cmdline"
}

process_state() {
  local name="$1" script="$2" pid_file="${STATE_DIR}/${1}.pid" pid cmdline
  if [[ ! -f "${pid_file}" ]]; then
    echo "${name}: stopped"
    return 1
  fi
  pid="$(<"${pid_file}")"
  if [[ ! "${pid}" =~ ^[1-9][0-9]*$ ]] || ! kill -0 "${pid}" 2>/dev/null; then
    echo "${name}: stopped (stale pid file)"
    return 1
  fi
  cmdline="$(process_cmdline "${pid}")" || {
    echo "${name}: PID ${pid} is not inspectable; refusing to signal it" >&2
    return 2
  }
  if [[ "${cmdline}" != *"${script}"* ]]; then
    echo "${name}: PID ${pid} belongs to another command; refusing to signal it" >&2
    return 2
  fi
  echo "${name}: running (pid ${pid})"
}

start_process() {
  local name="$1" script="$2" log_file="${LOG_DIR}/${1}.log"
  shift 2

  if process_state "${name}" "${script}"; then
    return 0
  else
    local result=$?
    [[ "${result}" -eq 1 ]] || return "${result}"
  fi

  mkdir -p "${LOG_DIR}"
  rm -f "${STATE_DIR}/${name}.pid"
  nohup bash -c '
    set -e
    unset PYTHONPATH ROS_DOMAIN_ID
    source "$1"
    source "$2"
    source "$3"
    shift 3
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="$1"
    shift
    export PYTHONPATH="$1${PYTHONPATH:+:$PYTHONPATH}"
    shift
    exec /usr/bin/python3 "$@"
  ' nx-local-sensors "${FOXY_SETUP}" "${CYCLONEDDS_SETUP}" "${UNITREE_SETUP}" \
    "${CYCLONEDDS_URI}" "${BRIDGE_DIR}" "$@" >"${log_file}" 2>&1 </dev/null &
  echo "$!" >"${STATE_DIR}/${name}.pid"

  sleep 0.2
  if process_state "${name}" "${script}"; then
    echo "  log: ${log_file}"
  else
    echo "${name} failed to start; see ${log_file}" >&2
    return 1
  fi
}

stop_process() {
  local name="$1" script="$2" pid_file="${STATE_DIR}/${1}.pid" pid cmdline i
  [[ -f "${pid_file}" ]] || { echo "${name}: stopped"; return 0; }
  pid="$(<"${pid_file}")"
  if [[ ! "${pid}" =~ ^[1-9][0-9]*$ ]] || ! kill -0 "${pid}" 2>/dev/null; then
    rm -f "${pid_file}"
    echo "${name}: stopped (stale pid file cleared)"
    return 0
  fi
  cmdline="$(process_cmdline "${pid}")" || {
    echo "${name}: PID ${pid} is not inspectable; refusing to signal it" >&2
    return 1
  }
  if [[ "${cmdline}" != *"${script}"* ]]; then
    echo "${name}: PID ${pid} belongs to another command; refusing to signal it" >&2
    return 1
  fi

  kill -TERM "${pid}"
  for ((i = 0; i < 50; i++)); do
    kill -0 "${pid}" 2>/dev/null || break
    cmdline="$(process_cmdline "${pid}" 2>/dev/null || true)"
    [[ "${cmdline}" == *"${script}"* ]] || break
    sleep 0.1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    cmdline="$(process_cmdline "${pid}" 2>/dev/null || true)"
    if [[ "${cmdline}" == *"${script}"* ]]; then
      kill -KILL "${pid}"
    fi
  fi
  rm -f "${pid_file}"
  echo "${name}: stopped"
}

status() {
  local result=0
  process_state adapter "${adapter_script}" || { [[ "$?" -eq 1 ]] || result=1; }
  process_state image_decoder "${decoder_script}" || { [[ "$?" -eq 1 ]] || result=1; }
  return "${result}"
}

action="${1:-start}"
case "${action}" in
  start)
    [[ $# -le 1 ]] || { usage >&2; exit 2; }
    [[ -f "${FOXY_SETUP}" && -f "${CYCLONEDDS_SETUP}" && -f "${UNITREE_SETUP}" ]] || {
      echo "Required Foxy/DDS/Unitree setup file is missing" >&2
      exit 1
    }
    mkdir -p "${STATE_DIR}"
    if ! ros_python -c 'import sys; assert sys.version_info[:2] == (3, 8); import nx_local_sensor_adapter'; then
      echo "Foxy Python cannot import the NX local sensor adapter" >&2
      exit 1
    fi
    start_process adapter "${adapter_script}" "${adapter_script}"
    if ros_python -c 'import sys; assert sys.version_info[:2] == (3, 8); import cv2, numpy, rclpy; from sensor_msgs.msg import CompressedImage, Image'; then
      start_process image_decoder "${decoder_script}" "${decoder_script}" --ros-args \
        -p input_topic:=/camera/image_raw/compressed \
        -p output_topic:=/dimos/color_image
    else
      echo "image_decoder: skipped (Foxy Python lacks cv2/numpy/ROS image dependencies)" >&2
    fi
    ;;
  status)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    status
    ;;
  stop)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    stop_process image_decoder "${decoder_script}"
    stop_process adapter "${adapter_script}"
    ;;
  -h|--help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
