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

"""Offline/static checks for the constrained Go2 NX Agent blueprint."""

from importlib import import_module

from dimos.agents.agent import Agent
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.mapping.odom_pointcloud_transform import OdomPointCloudTransformer
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.frontier_exploration import WavefrontFrontierExplorer
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.test_all_blueprints_generation import _scan_for_blueprints
from dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge import (
    unitree_go2_nx_bridge,
)
from dimos.web.websocket_vis.agent_memory import NXAgentMemoryWebModule
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

EXPECTED_SKILLS = {
    "tag_location",
    "navigate_with_text",
    "stop_navigation",
    "get_navigation_state",
}


def _blueprint():
    return import_module(
        "dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_nx_agentic"
    ).unitree_go2_nx_agentic


def _atoms_by_module(blueprint):
    return {atom.module: atom for atom in blueprint.blueprints}


def test_nx_agentic_blueprint_is_scannable_and_registered() -> None:
    all_blueprints, _ = _scan_for_blueprints(DIMOS_PROJECT_ROOT / "dimos")
    assert all_blueprints["unitree-go2-nx-agentic"] == (
        "dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_nx_agentic:unitree_go2_nx_agentic"
    )


def test_nx_agentic_is_static_and_uses_restricted_persistent_navigation() -> None:
    blueprint = _blueprint()
    atoms = _atoms_by_module(blueprint)
    active_atoms = [
        atom for atom in blueprint.blueprints if atom.module not in blueprint.disabled_modules_tuple
    ]
    active_modules = {atom.module for atom in active_atoms}

    assert blueprint.global_config_overrides["n_workers"] == 9
    bridge_modules = {atom.module for atom in unitree_go2_nx_bridge.blueprints} - {
        WebsocketVisModule,
        RerunBridgeModule,
    }
    assert bridge_modules <= active_modules | {
        WavefrontFrontierExplorer
    }
    assert WavefrontFrontierExplorer in blueprint.disabled_modules_tuple
    assert WavefrontFrontierExplorer not in active_modules
    assert NXAgentMemoryWebModule in active_modules
    assert WebsocketVisModule not in active_modules
    assert RerunBridgeModule in blueprint.disabled_modules_tuple
    assert RerunBridgeModule not in active_modules
    assert blueprint.remapping_map[(NXAgentMemoryWebModule, "cmd_vel")] == "cmd_vel"

    spatial_kwargs = atoms[SpatialMemory].kwargs
    assert spatial_kwargs["new_memory"] is False
    assert spatial_kwargs["world_frame_id"] == "odom"
    assert spatial_kwargs["base_frame_id"] == "base_link"
    assert spatial_kwargs["transform_tolerance_sec"] == 0.25

    navigation_kwargs = atoms[NavigationSkillContainer].kwargs
    assert navigation_kwargs == {
        "goal_frame_id": "odom",
        "enable_visual_navigation": False,
        "max_odom_age_sec": 1.0,
        "require_coordinate_frame": True,
        "semantic_query_ascii_only": True,
    }

    assert atoms[OdomPointCloudTransformer].kwargs["world_frame_id"] == "odom"
    assert atoms[VoxelGridMapper].kwargs["frame_id"] == "odom"

    odom_atoms = [
        atom for atom in active_atoms if atom.module.__module__ == "dimos.mapping.odom_tf_relay"
    ]
    assert len(odom_atoms) == 1
    assert odom_atoms[0].kwargs == {"world_frame_id": "odom", "base_frame_id": "base_link"}


def test_nx_agentic_exposes_only_four_allowed_skills_without_robot_control_modules() -> None:
    blueprint = _blueprint()
    active_atoms = [
        atom for atom in blueprint.blueprints if atom.module not in blueprint.disabled_modules_tuple
    ]
    skill_names = {
        name
        for atom in active_atoms
        for name in dir(atom.module)
        if getattr(getattr(atom.module, name), "__skill__", False)
    }
    assert skill_names == EXPECTED_SKILLS

    forbidden_fragments = (
        "GO2Connection",
        "UnitreeSkillContainer",
        "PersonFollow",
        "ObjectTracking",
    )
    assert not any(
        fragment in atom.module.__name__
        for atom in active_atoms
        for fragment in forbidden_fragments
    )

    agent_atom = _atoms_by_module(blueprint)[Agent]
    prompt = agent_atom.kwargs["system_prompt"]
    assert all(skill_name in prompt for skill_name in EXPECTED_SKILLS)
    assert "English CLIP" in prompt
    assert "Chinese names" in prompt
    assert "at most one action tool" in prompt
    assert "do not retry" in prompt


def test_nx_agentic_static_wiring_has_no_stream_or_rpc_contract_gaps() -> None:
    blueprint = _blueprint()
    blueprint._verify_no_name_conflicts()

    active_modules = [atom.module for atom in blueprint._active_blueprints]
    concrete_rpcs = {
        f"{module.__name__}.{method_name}"
        for module in active_modules
        for method_name in module.rpcs
    }
    interface_rpcs = {
        f"{base.__name__}.{method_name}"
        for module in active_modules
        for method_name in module.rpcs
        for base in module.mro()
        if method_name in getattr(base, "__dict__", {})
    }
    available_rpcs = concrete_rpcs | interface_rpcs
    navigation_kwargs = _atoms_by_module(blueprint)[NavigationSkillContainer].kwargs
    requested_rpcs = set(NavigationSkillContainer.navigation_rpc_calls)
    if navigation_kwargs["enable_visual_navigation"]:
        requested_rpcs.update(NavigationSkillContainer.visual_navigation_rpc_calls)

    assert {
        "SpatialMemory.tag_location",
        "SpatialMemory.query_tagged_location",
        "SpatialMemory.query_by_text",
        "NavigationInterface.set_goal",
        "NavigationInterface.cancel_goal",
        "NavigationInterface.get_state",
    } <= available_rpcs
    assert requested_rpcs <= available_rpcs
    assert not requested_rpcs & set(NavigationSkillContainer.visual_navigation_rpc_calls)

    class FakeProxy:
        def __init__(self, requested: set[str]) -> None:
            self.requested = requested
            self.bound: dict[str, object] = {}

        def get_rpc_method_names(self) -> list[str]:
            return list(self.requested)

        def set_rpc_method(self, name: str, method: object) -> None:
            self.bound[name] = method

        def __getattr__(self, name: str):  # type: ignore[no-untyped-def]
            return lambda *args, **kwargs: None

    proxies = {
        module: FakeProxy(requested_rpcs if module is NavigationSkillContainer else set())
        for module in active_modules
    }

    class FakeCoordinator:
        def get_instance(self, module):  # type: ignore[no-untyped-def]
            return proxies[module]

    blueprint._connect_rpc_methods(FakeCoordinator())  # type: ignore[arg-type]
    assert proxies[NavigationSkillContainer].bound.keys() == requested_rpcs
