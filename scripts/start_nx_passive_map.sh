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

# Start only the passive NX mapping blueprint. This script has no motion path.
# Outputs: /dimos/nx_passive/global_map and /dimos/nx_passive/costmap.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DIMOS_WORKTREE="${DIMOS_WORKTREE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
DIMOS_VENV="${DIMOS_VENV:-${DIMOS_WORKTREE}/.venv}"
CONDA_SH="/home/unitree/.local/share/dimos-nx-onboard/miniforge3/etc/profile.d/conda.sh"
GOMP_LIBRARY="${DIMOS_VENV}/lib/libgomp.so.1"

[[ -r "${CONDA_SH}" ]] || {
  echo "Conda activation script not found: ${CONDA_SH}" >&2
  exit 1
}
[[ -f "${GOMP_LIBRARY}" ]] || {
  echo "OpenMP preload library not found: ${GOMP_LIBRARY}" >&2
  exit 1
}

set +u
source "${CONDA_SH}"
conda activate "${DIMOS_VENV}"
set -u
cd "${DIMOS_WORKTREE}"
unset PYTHONPATH ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${DIMOS_NX_DDS_URI:-file:///home/unitree/cyclonedds_ws/cyclonedds.xml}"
export LD_PRELOAD="${GOMP_LIBRARY}"

exec python -m dimos.robot.unitree.go2.blueprints.unitree_go2_nx_passive_map
