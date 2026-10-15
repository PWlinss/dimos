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

"""Constrained indoor-navigation Agent blueprint for the Unitree Go2 NX stack."""

import os
from math import radians

from dimos.agents.agent import agent
from dimos.agents.skills.navigation import navigation_skill
from dimos.core.blueprints import autoconnect
from dimos.mapping.odom_tf_relay import odom_tf_relay
from dimos.navigation.frontier_exploration import WavefrontFrontierExplorer
from dimos.perception.spatial_perception import spatial_memory
from dimos.web.websocket_vis.agent_memory import NXAgentMemoryWebModule
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge import (
    make_unitree_go2_nx_bridge,
)

UNITREE_GO2_NX_AGENTIC_SYSTEM_PROMPT = """\
You are a constrained NX indoor navigation agent for a Unitree Go2.
Use only these tools: tag_location, navigate_with_text, stop_navigation, and get_navigation_state.
Saved location tags may use their exact original names, including Chinese names. The semantic visual-memory
search uses English CLIP descriptions only; translate a visual request into one concise English description.
Navigation goals use the odom frame; visual navigation is disabled. Never invent or call another tool.
Use at most one action tool per user request. If any tool rejects a request or returns an error, report that
result and end the request; do not retry, rephrase, or call another action tool.
Report only tool-confirmed state; starting navigation does not mean the destination was reached.
"""

unitree_go2_nx_agentic = autoconnect(
    make_unitree_go2_nx_bridge("odom").disabled_modules(
        WavefrontFrontierExplorer, WebsocketVisModule, RerunBridgeModule
    ),
    NXAgentMemoryWebModule.blueprint(),
    spatial_memory(
        new_memory=False,
        world_frame_id="odom",
        base_frame_id="base_link",
        transform_tolerance_sec=0.25,
        min_distance_threshold=0.2,
        min_rotation_threshold=radians(15),
        similarity_threshold=0.95,
    ),
    odom_tf_relay(world_frame_id="odom", base_frame_id="base_link"),
    navigation_skill(
        goal_frame_id="odom",
        enable_visual_navigation=False,
        max_odom_age_sec=1.0,
        require_coordinate_frame=True,
        semantic_query_ascii_only=True,
    ),
    agent(
        system_prompt=UNITREE_GO2_NX_AGENTIC_SYSTEM_PROMPT,
        model="openai:" + os.getenv("QWEN_MODEL", "qwen3.8-omni-flash"),
        model_kwargs={"streaming": True, "reasoning_effort": "none", "timeout": 30, "max_retries": 0},
    ),
).remappings([(NXAgentMemoryWebModule, "cmd_vel", "cmd_vel")]).global_config(n_workers=9)

__all__ = ["UNITREE_GO2_NX_AGENTIC_SYSTEM_PROMPT", "unitree_go2_nx_agentic"]
