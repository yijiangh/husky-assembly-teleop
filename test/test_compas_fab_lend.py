"""CompasFabMirror.lend: tamp plans on the mirror's planner, then the next sync writes the full state again."""

from __future__ import annotations

import sys
from pathlib import Path

import pybullet as p
import pybullet_planning as pp
import pytest

from bar_assembly_core.design_io.pose import Pose
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
from bar_assembly_core.robot import RobotObject
from bar_assembly_core.scene import SceneSnapshot
from bar_assembly_core.ur import UR_JOINT_NAMES
from husky_assembly_teleop.config import robot_config_from_serial

DATA = Path(__file__).resolve().parent.parent / "data"
CINDY = "robots/a200-0806"


def _pp_clients() -> dict:
    """The client every loaded pybullet_planning module holds, by module name."""
    return {name: module.CLIENT for name, module in list(sys.modules.items())
            if name.split(".")[0] == "pybullet_planning" and hasattr(module, "CLIENT")}


@pytest.mark.slow
def test_tamp_plans_on_a_lent_planner_and_sync_restores_the_world():
    """plan_free_dual_arm runs in the mirror's world (not client 0); the next sync writes every joint back."""
    api = pytest.importorskip("husky_assembly_tamp.motion_planner.api")
    cindy = robot_config_from_serial("0806", DATA)
    stow = {f"{arm.name}_{name}": value for arm in cindy.arms for name, value in zip(UR_JOINT_NAMES, arm.stow_joints)}
    scene = SceneSnapshot(robots={CINDY: RobotObject(CINDY, cindy.model, Pose(), dict(stow))})
    # ? Another world first, so the mirror's is not client 0: pp's functions default to 0 at import.
    other = p.connect(p.DIRECT)
    mirror = CompasFabMirror(CINDY)
    try:
        assert mirror.client.client_id != 0
        mirror.sync(scene)
        before = _pp_clients()
        names = [f"{arm}_{joint}" for arm in ("left_ur_arm", "right_ur_arm") for joint in UR_JOINT_NAMES]
        # ? A plain list: tamp reads a numpy array as a Configuration first and fails on the IndexError.
        # Both shoulders turned 0.3 rad outwards: clear of each other.
        goal = [stow[name] + (-0.3 if name == names[0] else 0.3 if name == names[6] else 0.0) for name in names]

        with mirror.lend() as planner:
            assert set(_pp_clients().values()) == {mirror.client.client_id}
            assert pp.get_bodies() == list(range(p.getNumBodies(physicsClientId=mirror.client.client_id)))
            path, info = api.plan_free_dual_arm(planner, mirror.state, goal, max_time=20.0)
            assert path is not None, info
            # * Leave the arms elsewhere, as a borrower may.
            robot = planner.client.robot_puid
            pp.set_joint_positions(robot, pp.joints_from_names(robot, names), goal)

        assert _pp_clients() == before, "every pybullet_planning module points where it did before"
        mirror.sync(scene)
        robot, client = mirror.client.robot_puid, mirror.client.client_id
        for name in names:
            joint = mirror.client.robot_joint_puids[name]
            assert p.getJointState(robot, joint, physicsClientId=client)[0] == pytest.approx(stow[name], abs=1e-9)
        assert mirror.collisions() == []
    finally:
        mirror.close()
        p.disconnect(physicsClientId=other)
