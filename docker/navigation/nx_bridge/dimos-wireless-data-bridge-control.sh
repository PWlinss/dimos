#!/usr/bin/env bash
set -euo pipefail

action="${1:-start}"
state=/tmp/dimos-wireless-data-bridge
logs="${state}/logs"
scripts=/home/unitree/projects/dimos
robot_dds=file:///home/unitree/cyclonedds_ws/cyclonedds.xml
wifi_dds=file:///home/unitree/cyclonedds_ws/cyclonedds_pusai_wifi.xml
camera_relay_max_rate_hz="${DIMOS_NX_CAMERA_RELAY_MAX_RATE_HZ:-5.0}"
clock_sync_file="${state}/robot-clock.json"
names=(pointcloud_receiver pointcloud_sender udp_to_pose sport_state_to_udp camera_relay)

is_live() {
  [[ -f "${state}/$1.pid" ]] && kill -0 "$(<"${state}/$1.pid")" 2>/dev/null
}

stop_all() {
  local name pid
  for name in "${names[@]}"; do
    [[ -f "${state}/${name}.pid" ]] || continue
    pid="$(<"${state}/${name}.pid")"
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 2
  for name in "${names[@]}"; do
    [[ -f "${state}/${name}.pid" ]] || continue
    pid="$(<"${state}/${name}.pid")"
    kill -KILL "${pid}" 2>/dev/null || true
    rm -f "${state}/${name}.pid"
  done
}

if [[ "${action}" == stop ]]; then
  stop_all
  exit 0
fi

if [[ "${action}" == status ]]; then
  for name in "${names[@]}"; do
    if is_live "${name}"; then
      printf '%s running pid=%s\n' "${name}" "$(<"${state}/${name}.pid")"
    else
      printf '%s stopped\n' "${name}"
    fi
  done
  exit 0
fi

[[ "${action}" == start ]] || { echo "usage: $0 [start|status|stop]" >&2; exit 2; }

for file in pointcloud_tcp_bridge.py robot_clock_sync.py udp_to_pose.py sport_state_to_udp.py topic_relay.py; do
  [[ -f "${scripts}/${file}" ]] || { echo "missing ${scripts}/${file}" >&2; exit 1; }
done
for file in /home/unitree/cyclonedds_ws/cyclonedds.xml /home/unitree/cyclonedds_ws/cyclonedds_pusai_wifi.xml; do
  [[ -f "${file}" ]] || { echo "missing ${file}" >&2; exit 1; }
done

mkdir -p "${logs}"
for name in "${names[@]}"; do
  if is_live "${name}"; then
    echo "refusing duplicate ${name}" >&2
    exit 3
  fi
  rm -f "${state}/${name}.pid"
done
if pgrep -af 'pointcloud_tcp_bridge.py|sport_state_to_udp.py|udp_to_pose.py|topic_relay.py' >/dev/null; then
  echo "refusing to overlap existing bridge process" >&2
  pgrep -af 'pointcloud_tcp_bridge.py|sport_state_to_udp.py|udp_to_pose.py|topic_relay.py' >&2
  exit 3
fi
if ss -ltn | grep -q ':39102 '; then
  echo "TCP port 39102 is already in use" >&2
  exit 3
fi
if ss -lun | grep -q ':39100 '; then
  echo "UDP port 39100 is already in use" >&2
  exit 3
fi

# Strict robot-clock mode must never inherit a previous process/session estimate.
rm -f "${clock_sync_file}"

set +u
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
source /home/unitree/ros2projects/go2_control_driver/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

start_node() {
  local name="$1" dds="$2"
  shift 2
  nohup setsid env CYCLONEDDS_URI="${dds}" python3 "$@" >"${logs}/${name}.log" 2>&1 < /dev/null &
  echo $! >"${state}/${name}.pid"
}

trap 'stop_all' ERR
start_node pointcloud_receiver "${wifi_dds}" "${scripts}/pointcloud_tcp_bridge.py" --role receiver --ros-args -p clock_sync_file:="${clock_sync_file}"
sleep 1
start_node pointcloud_sender "${robot_dds}" "${scripts}/pointcloud_tcp_bridge.py" --role sender --ros-args -p input_reliability:=best_effort -p clock_sync_file:="${clock_sync_file}"
start_node udp_to_pose "${wifi_dds}" "${scripts}/udp_to_pose.py" --ros-args -p output_topic:=/dimos/odom -p frame_id:=odom -p timestamp_mode:=robot_clock -p clock_sync_file:="${clock_sync_file}"
start_node sport_state_to_udp "${robot_dds}" "${scripts}/sport_state_to_udp.py" --ros-args -p clock_sync_file:="${clock_sync_file}"
start_node camera_relay "${wifi_dds}" "${scripts}/topic_relay.py" --ros-args -p input_topic:=/camera/image_raw/compressed -p output_topic:=/dimos/color_image/compressed -p msg_module:=sensor_msgs.msg -p msg_type:=CompressedImage -p max_rate_hz:="${camera_relay_max_rate_hz}" -p input_reliability:=best_effort -p output_reliability:=best_effort
sleep 4

for name in "${names[@]}"; do
  is_live "${name}" || { echo "${name} failed; see ${logs}/${name}.log" >&2; false; }
  printf '%s running pid=%s\n' "${name}" "$(<"${state}/${name}.pid")"
done
trap - ERR
