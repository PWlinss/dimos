#!/usr/bin/env python3
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

from dimos.agents.mcp.mcp_client import mcp_client
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.streamvln_navigation import streamvln_navigation_skill
from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.core.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import unitree_go2_spatial

STREAMVLN_SYSTEM_PROMPT = SYSTEM_PROMPT + """

# STREAMVLN NAVIGATION PRIORITY
- For navigation-related requests, first call `streamvln_navigate`.
- If `streamvln_navigate` is unavailable, degraded, or fails, fallback to `navigate_with_text`.
- Use `streamvln_navigation_status` to inspect health and progress.
- Use `stop_streamvln_navigation` to stop an active StreamVLN navigation task.
"""

unitree_go2_agentic_mcp_streamvln = autoconnect(
    unitree_go2_spatial,
    McpServer.blueprint(),
    mcp_client(system_prompt=STREAMVLN_SYSTEM_PROMPT),
    _common_agentic,
    streamvln_navigation_skill(),
)

__all__ = ["unitree_go2_agentic_mcp_streamvln"]
