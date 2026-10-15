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
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
FRONTEND_DIR="${REPO_ROOT}/dimos/web/command-center-extension"

command -v npm >/dev/null || { echo "npm is required for the one-time Command Center build." >&2; exit 1; }
[[ -f "${FRONTEND_DIR}/package-lock.json" ]] || { echo "Missing frontend package-lock.json." >&2; exit 1; }

cd "${FRONTEND_DIR}"
npm ci
npm run build:standalone
[[ -f dist-standalone/index.html ]] || { echo "Standalone build did not produce dist-standalone/index.html." >&2; exit 1; }
echo "Command Center prepared: ${FRONTEND_DIR}/dist-standalone"
