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

"""Static wiring checks; never build modules or connect to the robot."""

import asyncio

from dimos.agents.agent import Agent
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.nx_closed_loop import NXClosedLoopNavigationSkillContainer
from dimos.mapping.odom_tf_relay import OdomTFRelay
from dimos.navigation.frontier_exploration import WavefrontFrontierExplorer
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_nx_closed_loop import (
    NXClosedLoopWebModule,
    unitree_go2_nx_closed_loop,
)
from dimos.spec.utils import spec_structural_compliance
from dimos.visualization.rerun.bridge import RerunBridgeModule


def test_closed_loop_composes_existing_stack_without_legacy_motion_skills() -> None:
    blueprint = unitree_go2_nx_closed_loop
    blueprint._verify_no_name_conflicts()
    atoms = {atom.module: atom for atom in blueprint._active_blueprints}
    assert NXClosedLoopNavigationSkillContainer in atoms
    assert NavigationSkillContainer not in atoms
    assert WavefrontFrontierExplorer not in atoms
    assert RerunBridgeModule not in atoms
    assert NXClosedLoopWebModule in atoms
    assert atoms[SpatialMemory].kwargs["new_memory"] is False
    assert atoms[SpatialMemory].kwargs["world_frame_id"] == "odom"
    assert atoms[OdomTFRelay].kwargs["world_frame_id"] == "odom"
    assert atoms[NXClosedLoopNavigationSkillContainer].kwargs["goal_frame_id"] == "odom"


def test_closed_loop_exposes_only_task_tools_and_prompt_describes_feedback() -> None:
    atoms = unitree_go2_nx_closed_loop._active_blueprints
    skills = {
        name
        for atom in atoms
        for name in dir(atom.module)
        if getattr(getattr(atom.module, name), "__skill__", False)
    }
    assert skills == {
        "navigate_with_text",
        "search_memory",
        "navigate_to_candidate",
        "observe",
        "finish_navigation_task",
        "stop_navigation",
        "get_navigation_state",
        "tag_location",
    }
    prompt = next(atom for atom in atoms if atom.module is Agent).kwargs["system_prompt"]
    assert all(name in prompt for name in skills)
    assert "WAIT" in prompt and "NEW camera image" in prompt
    assert "Never silently restart" in prompt


def test_all_typed_module_refs_resolve_to_one_active_implementation() -> None:
    atoms = unitree_go2_nx_closed_loop._active_blueprints
    navigation = next(atom for atom in atoms if atom.module is NXClosedLoopNavigationSkillContainer)
    assert len(navigation.module_refs) >= 3  # memory, planner and Agent callback
    assert {stream.name for stream in navigation.streams} >= {"odom", "color_image"}
    for atom in atoms:
        for reference in atom.module_refs:
            candidates = [
                other.module
                for other in atoms
                if other.module is not atom.module
                and spec_structural_compliance(other.module, reference.spec)
            ]
            assert len(candidates) == 1, (atom.module, reference, candidates)


def test_web_stop_button_cancels_task_without_agent_or_ros() -> None:
    class FakeNavigation:
        calls = 0

        def stop_navigation(self) -> str:
            self.calls += 1
            return "Stopped"

    web = object.__new__(NXClosedLoopWebModule)
    navigation = FakeNavigation()
    web._navigation = navigation
    web._create_server()
    asyncio.run(web.sio.handlers["/"]["stop_explore"]("test-client"))
    assert navigation.calls == 1
