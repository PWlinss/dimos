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

# Start the NX adapter nodes inside vln_ros2_foxy_dev. It never enables Go2 motion.

set -euo pipefail

ACTION="start"
WITH_CONTROL=false
MAP_POSE=false
WITH_CAMERA_SOURCE=false
LIVOX_COMMAND=""
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="${DIMOS_NX_STATE_DIR:-/tmp/dimos-nx-bridge}"
LOG_DIR="${STATE_DIR}/logs"
WIFI_DDS_URI="${DIMOS_NX_WIFI_DDS_URI:-file:///workspace/cyclonedds_ws/cyclonedds.xml}"
ROBOT_DDS_URI="${DIMOS_NX_ROBOT_DDS_URI:-file://${SCRIPT_DIR}/cyclonedds_robot_bus.xml}"
UNITREE_INTERFACE="${DIMOS_NX_UNITREE_INTERFACE:-}"
UNITREE_INTERFACE_ADDRESS="${DIMOS_NX_UNITREE_INTERFACE_ADDRESS:-192.168.123.222/24}"
UNITREE_CONTROLLER_ADDRESS="${DIMOS_NX_UNITREE_CONTROLLER_ADDRESS:-192.168.123.161}"
GO2_NAV_SETUP="${DIMOS_NX_GO2_NAV_SETUP:-/workspace/projects/go2/go2_ros2_nav/install/local_setup.bash}"
GO2_DRIVER_PARAMS="${DIMOS_NX_GO2_DRIVER_PARAMS:-/workspace/projects/go2/go2_ros2_nav/install/go2_driver_py/share/go2_driver_py/params/go2_driver.yaml}"
GO2_DRIVER_TF_PUBLISH_RATE="${DIMOS_NX_GO2_DRIVER_TF_PUBLISH_RATE:-100.0}"
LIVOX_WORKSPACE="${DIMOS_NX_LIVOX_WORKSPACE:-/workspace/tmp/livox_ros_driver2_clean_ws}"
LIVOX_SETUP="${LIVOX_WORKSPACE}/install/setup.bash"
LIVOX_CONFIG="${DIMOS_NX_LIVOX_CONFIG:-${LIVOX_WORKSPACE}/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json}"
CAMERA_SOURCE_TOPIC="${DIMOS_NX_CAMERA_SOURCE_TOPIC:-/camera/image_raw/compressed}"
CAMERA_RELAY_TOPIC="${DIMOS_NX_CAMERA_RELAY_TOPIC:-/dimos/color_image/compressed}"
CAMERA_RELAY_MAX_RATE_HZ="${DIMOS_NX_CAMERA_RELAY_MAX_RATE_HZ:-0.5}"
CAMERA_INTERFACE="${DIMOS_NX_CAMERA_INTERFACE:-}"
CAMERA_FPS="${DIMOS_NX_CAMERA_FPS:-1}"
CAMERA_WIDTH="${DIMOS_NX_CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${DIMOS_NX_CAMERA_HEIGHT:-360}"
CAMERA_JPEG_QUALITY="${DIMOS_NX_CAMERA_JPEG_QUALITY:-60}"
CAMERA_FRAME_ID="${DIMOS_NX_CAMERA_FRAME_ID:-camera_optical_frame}"
CAMERA_UDP_ADDRESS="${DIMOS_NX_CAMERA_UDP_ADDRESS:-230.1.1.1}"
CAMERA_UDP_PORT="${DIMOS_NX_CAMERA_UDP_PORT:-1720}"
CAMERA_LD_PRELOAD="${DIMOS_NX_CAMERA_LD_PRELOAD:-}"
MAP_POSE_UDP_PORT="${DIMOS_NX_MAP_POSE_UDP_PORT:-39102}"

usage() {
  cat <<'EOF'
Usage: start_nx_bridge.sh [start|status|verify|stop] [--with-control] [--map-pose] [--with-camera-source] [--livox-command COMMAND]

  start             Start the NX data relays. This is the default.
  status            Show tracked adapter processes.
  verify            Check tracked processes, startup logs, and disabled control gates.
  stop              Stop tracked adapter processes.
  --with-control    Also start the three control nodes with both gates disabled.
  --map-pose        Use /amcl_pose as /dimos/odom for saved-map navigation.
  --with-camera-source
                    Also start the Go2 H264 camera source for compressed JPEG frames.
  --livox-command   Override the default Livox source command under tracking.

The script never enables either Go2 movement gate.
It also starts a /map -> /dimos/map relay; publish /map separately with map_server/Nav2.
It also starts a rate-limited compressed camera relay to /dimos/color_image/compressed.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    start|status|verify|stop)
      ACTION="$1"
      ;;
    --with-control)
      WITH_CONTROL=true
      ;;
    --map-pose)
      MAP_POSE=true
      ;;
    --with-camera-source)
      WITH_CAMERA_SOURCE=true
      ;;
    --livox-command)
      shift
      [[ $# -gt 0 ]] || { echo "--livox-command requires a command" >&2; exit 2; }
      LIVOX_COMMAND="$1"
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

start_process() {
  local name="$1"
  shift
  local pid_file="${STATE_DIR}/${name}.pid"
  local log_file="${LOG_DIR}/${name}.log"

  if [[ -f "${pid_file}" ]] && is_live_pid "$(<"${pid_file}")"; then
    echo "${name} already running (pid $(<"${pid_file}"))"
    return
  fi

  rm -f "${pid_file}"
  nohup setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  echo $! >"${pid_file}"
  echo "started ${name} (pid $(<"${pid_file}"), log ${log_file})"
}

is_live_pid() {
  local pid="$1"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  [[ -r "/proc/${pid}/stat" ]] || return 1
  [[ "$(awk '{print $3}' "/proc/${pid}/stat")" != "Z" ]] || return 1
  kill -0 "${pid}" 2>/dev/null
}

unitree_interface_host_address() {
  printf '%s\n' "${UNITREE_INTERFACE_ADDRESS%%/*}"
}

unitree_interface_candidates() {
  if [[ -n "${UNITREE_INTERFACE}" ]]; then
    printf '%s\n' "${UNITREE_INTERFACE}"
    return
  fi

  if command -v ip >/dev/null 2>&1; then
    ip -o link show | awk -F': ' '$2 !~ /^(lo|docker|br-|veth|dummy)/ {print $2}' | sed 's/@.*//'
  else
    printf '%s\n' eth0 eth1 wlan0
  fi
}

select_unitree_interface() {
  local host_address
  host_address="$(unitree_interface_host_address)"

  if [[ -n "${UNITREE_INTERFACE}" ]]; then
    printf '%s\n' "${UNITREE_INTERFACE}"
    return
  fi

  if command -v ip >/dev/null 2>&1; then
    local current_interface
    current_interface="$(
      ip -o -4 addr show 2>/dev/null | awk -v addr="${host_address}" '$4 ~ "^" addr "/" {print $2; exit}'
    )"
    if [[ -n "${current_interface}" ]] && [[ "$(cat "/sys/class/net/${current_interface}/operstate" 2>/dev/null || true)" != "down" ]]; then
      printf '%s\n' "${current_interface}"
      return
    fi
  fi

  local candidate
  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] || continue
    if ping -c 1 -W 1 -I "${candidate}" "${UNITREE_CONTROLLER_ADDRESS}" >/dev/null 2>&1; then
      printf '%s\n' "${candidate}"
      return
    fi
  done < <(unitree_interface_candidates)

  if command -v ip >/dev/null 2>&1; then
    ip route get "${UNITREE_CONTROLLER_ADDRESS}" 2>/dev/null | awk '{for (i = 1; i <= NF; i++) if ($i == "dev") {print $(i + 1); exit}}'
    return
  fi

  printf '%s\n' eth0
}

unitree_interface_address_present() {
  local selected_interface="$1"
  local host_address
  host_address="$(unitree_interface_host_address)"

  command -v ip >/dev/null 2>&1 || return 1
  ip -br addr show "${selected_interface}" 2>/dev/null | grep -q "${host_address}"
}

ensure_unitree_interface_address() {
  local selected_interface
  selected_interface="$(select_unitree_interface)"
  [[ -n "${selected_interface}" ]] || selected_interface="eth0"

  if unitree_interface_address_present "${selected_interface}"; then
    echo "Unitree robot-bus address ${UNITREE_INTERFACE_ADDRESS} is present on ${selected_interface}"
    return 0
  fi

  if ! command -v ip >/dev/null 2>&1; then
    echo "[WARN] ip command not found; cannot verify ${UNITREE_INTERFACE_ADDRESS} on ${selected_interface}" >&2
    return 0
  fi

  local host_address
  host_address="$(unitree_interface_host_address)"
  local existing_interface
  existing_interface="$(ip -o -4 addr show 2>/dev/null | awk -v addr="${host_address}" '$4 ~ "^" addr "/" {print $2}')"
  for existing_interface in ${existing_interface}; do
    if [[ "${existing_interface}" != "${selected_interface}" ]]; then
      ip addr del "${UNITREE_INTERFACE_ADDRESS}" dev "${existing_interface}" 2>/dev/null || true
    fi
  done

  if ip addr add "${UNITREE_INTERFACE_ADDRESS}" dev "${selected_interface}" 2>/dev/null; then
    echo "added ${UNITREE_INTERFACE_ADDRESS} to ${selected_interface}"
    return 0
  fi

  cat >&2 <<EOF
[FAIL] Unitree robot-bus address ${UNITREE_INTERFACE_ADDRESS} is missing on ${selected_interface}.
Run this once on the NX host after reboot, then start the bridge again:

  sudo ip addr add ${UNITREE_INTERFACE_ADDRESS} dev ${selected_interface}

EOF
  return 1
}

camera_source_dependencies_ready() {
  local preload
  preload="$(camera_source_ld_preload)"

  if ! LD_PRELOAD="${preload}" python3 - <<'PY' >/dev/null 2>&1
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: F401
PY
  then
    cat >&2 <<'EOF'
[FAIL] Go2 camera source requires GStreamer Python introspection.
Install these packages inside the Foxy container, then restart with --with-camera-source:

  apt-get install -y \
    gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 python3-gst-1.0 \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav

EOF
    return 1
  fi

  if command -v gst-inspect-1.0 >/dev/null 2>&1; then
    local element
    for element in udpsrc rtph264depay h264parse avdec_h264 videoconvert videoscale jpegenc appsink; do
      if ! LD_PRELOAD="${preload}" gst-inspect-1.0 "${element}" >/dev/null 2>&1; then
        echo "[FAIL] Go2 camera source missing GStreamer element: ${element}" >&2
        return 1
      fi
    done
  fi
}

camera_source_ld_preload() {
  if [[ -n "${CAMERA_LD_PRELOAD}" ]]; then
    printf '%s\n' "${CAMERA_LD_PRELOAD}"
    return
  fi

  # Jetson/ARM Python processes can fail to load gst-libav after numpy/rclpy with:
  # "libgomp.so.1: cannot allocate memory in static TLS block".
  local libgomp="/lib/aarch64-linux-gnu/libgomp.so.1"
  [[ -f "${libgomp}" ]] && printf '%s\n' "${libgomp}"
}

stop_process() {
  local name="$1"
  local pid_file="${STATE_DIR}/${name}.pid"

  [[ -f "${pid_file}" ]] || return 0
  local pid
  pid="$(<"${pid_file}")"
  if is_live_pid "${pid}"; then
    if kill -0 "-${pid}" 2>/dev/null; then
      kill -INT "-${pid}" 2>/dev/null || true
    else
      kill -INT "${pid}" 2>/dev/null || true
    fi
    sleep 1
    if is_live_pid "${pid}"; then
      if kill -0 "-${pid}" 2>/dev/null; then
        kill -TERM "-${pid}" 2>/dev/null || true
      else
        kill -TERM "${pid}" 2>/dev/null || true
      fi
    fi
    echo "stopped ${name} (pid ${pid})"
  fi
  rm -f "${pid_file}"
}

show_status() {
  local name
  for name in livox_source pointcloud_relay map_server_wifi map_relay camera_source camera_relay \
    map_localization_display go2_driver odom_tf_broadcaster go2_pointcloud_process map_localization \
    map_odom_tf_keeper amcl_pose_to_udp map_tf_odom_to_udp sport_state_to_udp udp_to_pose \
    amcl_pose_to_pose cmd_vel_relay guarded_cmd_to_udp udp_to_sport_request; do
    local pid_file="${STATE_DIR}/${name}.pid"
    if [[ -f "${pid_file}" ]] && is_live_pid "$(<"${pid_file}")"; then
      echo "${name}: running (pid $(<"${pid_file}"))"
    else
      echo "${name}: stopped"
    fi
  done
}

check_process() {
  local name="$1"
  local pid_file="${STATE_DIR}/${name}.pid"
  if [[ -f "${pid_file}" ]] && is_live_pid "$(<"${pid_file}")"; then
    echo "[PASS] ${name}: running (pid $(<"${pid_file}"))"
    return 0
  fi
  echo "[FAIL] ${name}: not running" >&2
  return 1
}

control_gates_logged_disabled() {
  for _ in $(seq 1 20); do
    if grep -q "enabled=False" "${LOG_DIR}/cmd_vel_relay.log" 2>/dev/null && \
      grep -q "enabled=False" "${LOG_DIR}/udp_to_sport_request.log" 2>/dev/null; then
      return 0
    fi
    sleep 0.25
  done

  return 1
}

startup_logs_have_exceptions() {
  local name
  for name in livox_source pointcloud_relay map_server_wifi map_relay camera_source camera_relay \
    map_localization_display go2_driver odom_tf_broadcaster go2_pointcloud_process map_localization \
    map_odom_tf_keeper amcl_pose_to_udp map_tf_odom_to_udp sport_state_to_udp udp_to_pose \
    amcl_pose_to_pose cmd_vel_relay guarded_cmd_to_udp udp_to_sport_request; do
    local pid_file="${STATE_DIR}/${name}.pid"
    local log_file="${LOG_DIR}/${name}.log"
    if [[ -f "${pid_file}" ]] && is_live_pid "$(<"${pid_file}")" && [[ -f "${log_file}" ]] && \
      grep -E "Traceback|TypeError|SyntaxError" "${log_file}" >/dev/null 2>&1; then
      echo "[FAIL] startup log contains a Python exception: ${log_file}" >&2
      return 0
    fi
  done

  return 1
}

amcl_initialpose_subscription_ready() {
  local output
  if ! output="$(
    timeout 8 bash -lc "
      set +u
      source /opt/ros/foxy/setup.bash
      source /workspace/cyclonedds_ws/install/setup.bash
      [[ -f \"${GO2_NAV_SETUP}\" ]] && source \"${GO2_NAV_SETUP}\"
      set -u
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export CYCLONEDDS_URI=\"${ROBOT_DDS_URI}\"
      ros2 topic info /initialpose
    " 2>&1
  )"; then
    printf '%s\n' "${output}" >&2
    return 1
  fi

  grep -q "Subscription count: [1-9]" <<<"${output}"
}

verify_deployment() {
  local failed=0
  local name
  echo "== NX bridge deployment verification =="

  for name in pointcloud_relay map_relay camera_relay; do
    check_process "${name}" || failed=1
  done

  if [[ -f "${STATE_DIR}/camera_source.pid" ]]; then
    check_process camera_source || failed=1
  fi

  if [[ -f "${STATE_DIR}/amcl_pose_to_pose.pid" ]]; then
    check_process amcl_pose_to_pose || failed=1
  elif [[ -f "${STATE_DIR}/amcl_pose_to_udp.pid" ]]; then
    for name in map_server_wifi map_localization_display go2_driver odom_tf_broadcaster go2_pointcloud_process \
      map_localization map_odom_tf_keeper amcl_pose_to_udp udp_to_pose; do
      check_process "${name}" || failed=1
    done
  elif [[ -f "${STATE_DIR}/map_tf_odom_to_udp.pid" ]]; then
    for name in map_server_wifi map_localization_display go2_driver odom_tf_broadcaster go2_pointcloud_process \
      map_localization map_odom_tf_keeper map_tf_odom_to_udp udp_to_pose; do
      check_process "${name}" || failed=1
    done
    if amcl_initialpose_subscription_ready; then
      echo "[PASS] AMCL initialpose subscription: ready"
    else
      echo "[FAIL] AMCL initialpose subscription: unavailable" >&2
      failed=1
    fi
  else
    for name in sport_state_to_udp udp_to_pose; do
      check_process "${name}" || failed=1
    done
  fi

  if [[ -f "${STATE_DIR}/cmd_vel_relay.pid" || -f "${STATE_DIR}/guarded_cmd_to_udp.pid" || -f "${STATE_DIR}/udp_to_sport_request.pid" ]]; then
    for name in cmd_vel_relay guarded_cmd_to_udp udp_to_sport_request; do
      check_process "${name}" || failed=1
    done

    if control_gates_logged_disabled; then
      echo "[PASS] control gates: disabled"
    else
      echo "[FAIL] control gates: expected enabled=False startup logs" >&2
      failed=1
    fi
  else
    echo "[INFO] control nodes: not requested"
  fi

  if startup_logs_have_exceptions; then
    failed=1
  else
    echo "[PASS] startup logs: no Python exception"
  fi

  if [[ -f "${STATE_DIR}/livox_source.pid" ]] && is_live_pid "$(<"${STATE_DIR}/livox_source.pid")"; then
    echo "[PASS] Livox source: tracked by this script"
    if grep -q "PointCloud2" "${LOG_DIR}/livox_source.log"; then
      echo "[PASS] Livox source: PointCloud2 mode configured"
    else
      echo "[WARN] Livox source: waiting for PointCloud2 startup log"
    fi
  else
    echo "[FAIL] Livox source: not running" >&2
    failed=1
  fi

  if [[ "${failed}" -eq 0 ]]; then
    echo "[PASS] NX bridge process deployment is healthy"
  else
    echo "[FAIL] NX bridge process deployment is incomplete" >&2
  fi
  return "${failed}"
}

case "${ACTION}" in
  status)
    show_status
    exit 0
    ;;
  verify)
    verify_deployment
    exit $?
    ;;
  stop)
    # Stop the Sport API gateway first so it emits STOPMOVE before other relays stop.
    stop_process udp_to_sport_request
    stop_process guarded_cmd_to_udp
    stop_process cmd_vel_relay
    stop_process amcl_pose_to_pose
    stop_process amcl_pose_to_udp
    stop_process map_tf_odom_to_udp
    stop_process map_odom_tf_keeper
    stop_process map_localization
    stop_process go2_pointcloud_process
    stop_process odom_tf_broadcaster
    stop_process go2_driver
    stop_process map_localization_display
    stop_process udp_to_pose
    stop_process sport_state_to_udp
    stop_process camera_relay
    stop_process camera_source
    stop_process map_relay
    stop_process map_server_wifi
    stop_process pointcloud_relay
    stop_process livox_source
    exit 0
    ;;
esac

# ROS setup scripts can read optional variables while they are unset.
set +u
source /opt/ros/foxy/setup.bash
source /workspace/cyclonedds_ws/install/setup.bash
source /root/go2_control_driver_install/setup.bash
[[ -f "${GO2_NAV_SETUP}" ]] && source "${GO2_NAV_SETUP}"
[[ -f "${LIVOX_SETUP}" ]] || { echo "Livox setup not found: ${LIVOX_SETUP}" >&2; exit 1; }
source "${LIVOX_SETUP}"
set -u
mkdir -p "${LOG_DIR}"

if [[ -n "${LIVOX_COMMAND}" ]]; then
  start_process livox_source bash -lc "${LIVOX_COMMAND}"
else
  start_process livox_source env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${WIFI_DDS_URI}" \
    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -p xfer_format:=0 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p publish_freq:=10.0 \
    -p output_data_type:=0 \
    -p frame_id:=utlidar_lidar \
    -p user_config_path:="${LIVOX_CONFIG}" \
    -r /livox/lidar:=/registered_scan
fi

start_process pointcloud_relay env \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  CYCLONEDDS_URI="${WIFI_DDS_URI}" \
  python3 "${SCRIPT_DIR}/topic_relay.py" --ros-args \
  -p input_topic:=/registered_scan \
  -p output_topic:=/dimos/registered_scan \
  -p msg_module:=sensor_msgs.msg \
  -p msg_type:=PointCloud2 \
  -p max_rate_hz:=1.0 \
  -p input_reliability:=best_effort \
  -p output_reliability:=best_effort

start_process map_relay env \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  CYCLONEDDS_URI="${WIFI_DDS_URI}" \
  python3 "${SCRIPT_DIR}/topic_relay.py" --ros-args \
  -p input_topic:=/map \
  -p output_topic:=/dimos/map \
  -p msg_module:=nav_msgs.msg \
  -p msg_type:=OccupancyGrid \
  -p input_durability:=transient_local \
  -p output_durability:=transient_local

start_process camera_relay env \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  CYCLONEDDS_URI="${WIFI_DDS_URI}" \
  python3 "${SCRIPT_DIR}/topic_relay.py" --ros-args \
  -p input_topic:=${CAMERA_SOURCE_TOPIC} \
  -p output_topic:=${CAMERA_RELAY_TOPIC} \
  -p msg_module:=sensor_msgs.msg \
  -p msg_type:=CompressedImage \
  -p max_rate_hz:=${CAMERA_RELAY_MAX_RATE_HZ} \
  -p input_reliability:=best_effort \
  -p output_reliability:=best_effort

if [[ "${WITH_CAMERA_SOURCE}" == true ]]; then
  camera_source_dependencies_ready
  if [[ -z "${CAMERA_INTERFACE}" ]]; then
    CAMERA_INTERFACE="$(select_unitree_interface)"
  fi
  [[ -n "${CAMERA_INTERFACE}" ]] || CAMERA_INTERFACE="eth0"
  CAMERA_LD_PRELOAD="$(camera_source_ld_preload)"
  start_process camera_source env \
    LD_PRELOAD="${CAMERA_LD_PRELOAD}" \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${WIFI_DDS_URI}" \
    python3 "${SCRIPT_DIR}/go2_camera_image_source.py" --ros-args \
    -p image_topic:=${CAMERA_SOURCE_TOPIC} \
    -p udp_address:=${CAMERA_UDP_ADDRESS} \
    -p udp_port:=${CAMERA_UDP_PORT} \
    -p interface:=${CAMERA_INTERFACE} \
    -p target_fps:=${CAMERA_FPS} \
    -p width:=${CAMERA_WIDTH} \
    -p height:=${CAMERA_HEIGHT} \
    -p jpeg_quality:=${CAMERA_JPEG_QUALITY} \
    -p frame_id:=${CAMERA_FRAME_ID}
fi

if [[ "${MAP_POSE}" == true ]]; then
  ensure_unitree_interface_address
  stop_process sport_state_to_udp
  start_process map_server_wifi env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${WIFI_DDS_URI}" \
    ros2 launch mycar_navigation2 map_server.launch.py

  start_process map_localization_display env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    ros2 launch go2_description display.launch.py use_joint_state_publisher:=false

  start_process go2_driver env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    ros2 run go2_driver_py driver_2 --ros-args \
    --params-file "${GO2_DRIVER_PARAMS}" \
    -p publish_tf:=false

  start_process odom_tf_broadcaster env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    python3 "${SCRIPT_DIR}/odom_tf_broadcaster.py" --ros-args \
    -p publish_rate:="${GO2_DRIVER_TF_PUBLISH_RATE}"

  start_process go2_pointcloud_process env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    ros2 launch go2_perception go2_pointcloud_process.launch.py

  start_process map_localization env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    ros2 launch mycar_localization mycar_loca.launch.py

  start_process map_odom_tf_keeper env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    ros2 run go2_driver_py map_odom_tf_keeper --ros-args \
    -p parent_frame:=map \
    -p child_frame:=odom \
    -p publish_rate:=10.0

  stop_process amcl_pose_to_udp
  start_process map_tf_odom_to_udp env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    python3 "${SCRIPT_DIR}/map_tf_odom_to_udp.py" --ros-args \
    -p udp_port:="${MAP_POSE_UDP_PORT}"

  stop_process sport_state_to_udp
  stop_process amcl_pose_to_pose
  start_process udp_to_pose env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${WIFI_DDS_URI}" \
    python3 "${SCRIPT_DIR}/udp_to_pose.py" --ros-args \
    -p udp_port:="${MAP_POSE_UDP_PORT}" \
    -p output_topic:=/dimos/odom \
    -p frame_id:=map
else
  stop_process amcl_pose_to_pose
  stop_process amcl_pose_to_udp
  stop_process map_tf_odom_to_udp
  stop_process map_odom_tf_keeper
  stop_process map_localization
  stop_process go2_pointcloud_process
  stop_process odom_tf_broadcaster
  stop_process go2_driver
  stop_process map_localization_display
  stop_process map_server_wifi
  ensure_unitree_interface_address
  start_process sport_state_to_udp env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    python3 "${SCRIPT_DIR}/sport_state_to_udp.py"

  start_process udp_to_pose env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${WIFI_DDS_URI}" \
    python3 "${SCRIPT_DIR}/udp_to_pose.py" --ros-args \
    -p output_topic:=/dimos/odom \
    -p frame_id:=odom
fi

if [[ "${WITH_CONTROL}" == true ]]; then
  ensure_unitree_interface_address
  start_process cmd_vel_relay env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${WIFI_DDS_URI}" \
    python3 "${SCRIPT_DIR}/cmd_vel_relay.py"

  start_process guarded_cmd_to_udp env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${WIFI_DDS_URI}" \
    python3 "${SCRIPT_DIR}/guarded_cmd_to_udp.py"

  start_process udp_to_sport_request env \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI="${ROBOT_DDS_URI}" \
    python3 "${SCRIPT_DIR}/udp_to_sport_request.py"
fi

verify_deployment
