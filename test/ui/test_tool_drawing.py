"""Tests for `FastViserUrdf.add_tools`: a robot's tools drawn from its model, hanging on their flange frames."""

from __future__ import annotations


import numpy as np
import pytest
import viser

from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.ui.visualization import COLLISION_COLOR, FastViserUrdf, shared_urdf
from scaffolding_assembly_core.urdf import urdf_links


@pytest.fixture(scope="module")
def server():
    """One viser server on a free port for the whole module."""
    server = viser.ViserServer(port=0, verbose=False)
    yield server
    server.stop()


@pytest.fixture(scope="module")
def alice():
    """Alice's config: one arm with the Robotiq gripper."""
    return robot_config_from_serial("0804", DATA)


def test_model_urdf_has_no_tool_links(alice):
    """The robot's URDF is copied with the links it has, no tool joined on: the tool is only in `model.tools`."""
    original = (DATA / "husky_urdf/mt_husky_moveit_config/urdf"
                / "husky_ur5_e_no_base_joint_Alice_Calibrated.urdf")
    assert alice.model.links == urdf_links(original)
    assert "ur_arm_tool0" in alice.model.tools


def test_tools_hang_on_their_flange_in_both_trees(server, alice):
    """Visual and collision shapes go below the flange frame of their own tree, so they move and switch with it."""
    drawn = FastViserUrdf(server, shared_urdf(alice.model.urdf), root_node_name="/alice",
                          load_collision_meshes=True, collision_mesh_color_override=COLLISION_COLOR)
    before = len(drawn._meshes)
    drawn.add_tools(alice.model.tools)
    tool = alice.model.tools["ur_arm_tool0"]
    added = [mesh.name for mesh in drawn._meshes[before:]]
    assert len(added) == len(tool.geometry.visual) + len(tool.geometry.collision)
    flanges = {frame.name for joint, frame in zip(drawn._joint_map_values, drawn._joint_frames)
               if joint.child == "ur_arm_tool0"}
    assert {name.rsplit("/", 1)[0] for name in added} == flanges
    assert any(name.startswith("/alice/visual/") for name in added)
    assert any(name.startswith("/alice/collision/") for name in added)


def test_bare_robot_gets_no_tool_shapes(server, alice):
    """No mounted tools: nothing is added."""
    drawn = FastViserUrdf(server, shared_urdf(alice.model.urdf), root_node_name="/bare")
    before = len(drawn._meshes)
    drawn.add_tools({})
    assert len(drawn._meshes) == before


def test_tool_follows_the_joints(server, alice):
    """Moving the arm moves the flange frame, which carries the tool: nothing of the tool is posed apart."""
    drawn = FastViserUrdf(server, shared_urdf(alice.model.urdf), root_node_name="/moving")
    drawn.add_tools(alice.model.tools)
    names = drawn.get_actuated_joint_names()
    wrist = next(frame for joint, frame in zip(drawn._joint_map_values, drawn._joint_frames)
                 if joint.name == "ur_arm_wrist_3_joint")
    turned = np.array([0.5 if name == "ur_arm_wrist_3_joint" else 0.0 for name in names])
    before = tuple(wrist.wxyz)
    drawn.update_cfg(turned)
    assert tuple(wrist.wxyz) != before
    tool_nodes = [mesh for mesh in drawn._meshes if mesh.name.endswith("/tool_0")]
    assert tool_nodes and all(wrist.name in mesh.name for mesh in tool_nodes)
