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

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source_ros_env() {
  set +u
  # Foxy setup files can reference optional unset variables such as
  # AMENT_TRACE_SETUP_FILES, so do not source them under nounset.
  source "$1"
  set -u
}

if [[ $# -ne 3 ]]; then
  cat >&2 <<'EOF'
Usage: ./publish_initialpose.sh X Y YAW_RAD

Examples:
  ./publish_initialpose.sh 0 0 0
  ./publish_initialpose.sh 1.2 -0.5 1.5708
EOF
  exit 2
fi

source_ros_env /opt/ros/foxy/setup.bash
source_ros_env /workspace/cyclonedds_ws/install/setup.bash
source_ros_env /root/go2_control_driver_install/setup.bash
source_ros_env /workspace/projects/go2/go2_ros2_nav/install/local_setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${DIMOS_NX_ROBOT_DDS_URI:-file://${SCRIPT_DIR}/cyclonedds_robot_bus.xml}"

python3 "${SCRIPT_DIR}/publish_initialpose.py" --x "$1" --y "$2" --yaw "$3"
