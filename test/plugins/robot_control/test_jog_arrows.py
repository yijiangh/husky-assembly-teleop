"""Tests for jog_arrows: the 3D arrows show the way each jog button turns its joint."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from plugin_fakes import FakeContext

from husky_assembly_teleop.colors import BLUE, GREEN, RED, rgb
from husky_assembly_teleop.plugins.robot_control.jog_arrows import add_jog_arrows, draw_jog_arrows
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose


def arm_and_context() -> tuple[SimpleNamespace, FakeContext]:
    """An arm under the joint controller, on a known robot whose links all sit at the origin."""
    ctx = FakeContext(robots=(SimpleNamespace(serial="0804"),))
    ctx.kinematics = SimpleNamespace(link_pose=lambda _serial, _link: Pose())
    arm = SimpleNamespace(config=SimpleNamespace(name="ur_arm"), controllers=SimpleNamespace(is_active=lambda _c: True))
    return arm, ctx


def test_the_red_head_turns_as_plus_and_the_green_as_minus():
    """Red turns right-handed about the link's z axis, as a positive joint value does; green the other way."""
    arm, ctx = arm_and_context()
    add_jog_arrows(ctx, "0804", arm)
    heads = [node for node in ctx.view.scene.of_kind("arrows")
             if node.name.endswith("/head") and "/tool0/" not in node.name]
    assert len(heads) == 12
    for head in heads:
        start, end = head.args[0][0]
        plus = np.cross(start, end - start)[2] > 0
        assert head.args[1] == rgb(RED if plus else GREEN) and head.name.endswith("plus/head" if plus else "minus/head")


def test_tool0s_arrows_move_and_turn_the_plus_way_in_their_axis_colour():
    """Straight arrows point along their + axis, curved ones turn right-handed about it: x red, y green, z blue."""
    arm, ctx = arm_and_context()
    add_jog_arrows(ctx, "0804", arm)
    scene = ctx.view.scene
    nodes = {node.name.split("/tool0/")[1]: node for node in scene.nodes if "/tool0/" in node.name}
    for axis, (name, color) in enumerate(zip("xyz", (RED, GREEN, BLUE))):
        start, end = nodes[name].args[0][0]
        assert nodes[name].args[1] == rgb(color) and np.argmax(end - start) == axis and start[axis] == 0.0
        frame, head = nodes[f"r{name}"], nodes[f"r{name}/head"]
        w, x, y, z = frame.wxyz
        turn = Rotation.from_quat((x, y, z, w))
        head_start, head_end = (turn.apply(point) for point in head.args[0][0])
        assert head.args[1] == rgb(color)
        assert np.cross(head_start, head_end - head_start)[axis] > 0, f"turn about {name} goes the + way"


def test_each_box_shows_its_own_arrows():
    """Joint and tool0 arrows are hidden until their own box is ticked, and hidden again once it is unticked."""
    arm, ctx = arm_and_context()
    arrows = add_jog_arrows(ctx, "0804", arm)
    assert not arrows.joint_group.visible and not arrows.tool0.visible
    draw_jog_arrows(ctx, "0804", arm, arrows, True, False)
    assert arrows.joint_group.visible and not arrows.tool0.visible
    draw_jog_arrows(ctx, "0804", arm, arrows, False, True)
    assert not arrows.joint_group.visible and arrows.tool0.visible
    draw_jog_arrows(ctx, "0804", arm, arrows, False, False)
    assert not arrows.joint_group.visible and not arrows.tool0.visible


def test_in_the_robots_frame_tool0s_arrows_keep_the_robots_axes_at_tool0():
    """Frame at Robot: tool0's arrows sit at tool0 but turned as the robot's base, not as the flange."""
    arm, ctx = arm_and_context()
    base = Pose(orientation=tuple(Rotation.from_euler("z", 30, degrees=True).as_quat()))
    tool0 = Pose((0.5, 0.1, 0.8), tuple(Rotation.from_euler("xy", (90, 20), degrees=True).as_quat()))
    ctx.kinematics = SimpleNamespace(link_pose=lambda _serial, link: tool0 if link.endswith("tool0") else Pose(),
                                     base_pose=lambda _serial: base)
    arrows = add_jog_arrows(ctx, "0804", arm)
    draw_jog_arrows(ctx, "0804", arm, arrows, False, True, robot_frame=True)
    np.testing.assert_allclose(arrows.tool0.position, tool0.position)
    turned = Rotation.from_quat(np.roll(arrows.tool0.wxyz, -1)) * Rotation.from_quat(base.orientation).inv()
    assert turned.magnitude() < 1e-9, "the robot's axes, not tool0's"
