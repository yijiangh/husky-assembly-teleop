"""Stand-ins for robots that are not measured: where last seen, else at the default pose, each with a warning."""

from __future__ import annotations

from planning_fixtures import ALICE, BELLE, CINDY, robot_config

from husky_assembly_teleop.world.stand_ins import DEFAULT, LAST_SEEN, LIVE, with_stand_ins
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

CONFIGS = tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))
ALICE_ID, BELLE_ID, CINDY_ID = (robot_id(config.serial) for config in CONFIGS)


def _robot(config, x: float, **fields) -> RobotObject:
    """A robot at (x, 0), arms at zero, measured at t = 1 s; `fields` override."""
    return RobotObject(robot_id(config.serial), config.model, Pose((x, 0.0, 0.0)), {},
                       **{"base_time": 1.0, "joints_time": 1.0, **fields})


def test_all_live_is_the_snapshot_itself():
    """Nothing to stand in: the same snapshot, every other robot live, no warnings; the acting one is not judged."""
    snapshot = Scene(robots={robot.id: robot for robot in (_robot(CONFIGS[0], 0.0),
                                                           _robot(CONFIGS[1], 3.0, base_tracked=False))})
    found = with_stand_ins(snapshot, {BELLE_ID})
    assert found.scene is snapshot and found.sources == {ALICE_ID: LIVE} and found.warnings == ()


def test_last_seen_and_never_seen_robots_stand_where_the_snapshot_has_them():
    """Belle lost mocap: last seen. Cindy's arm never reported: default. Both checked as they are, each warned about.

    An absent robot is left out quietly; the snapshot itself is not changed.
    """
    arms = frozenset(name for name in CONFIGS[2].model.movable_joints if name.startswith("left_ur_arm_"))
    belle = _robot(CONFIGS[1], 3.0, base_tracked=False)
    cindy = _robot(CONFIGS[2], 5.0, unmeasured=arms, joints_time=None)
    alice = _robot(CONFIGS[0], 0.0, enabled=False)
    snapshot = Scene(robots={robot.id: robot for robot in (alice, belle, cindy)})
    found = with_stand_ins(snapshot, set())
    assert found.sources == {BELLE_ID: LAST_SEEN, CINDY_ID: DEFAULT}
    assert not any(robot.acting_problems() for key, robot in found.scene.robots.items() if key != ALICE_ID)
    assert found.scene.robots[BELLE_ID].base.position[0] == 3.0 and found.scene.robots[ALICE_ID] is alice
    belle_line, cindy_line = found.warnings
    assert belle_line == f"{BELLE_ID} is not measured (its base is not tracked): checked where last seen"
    assert cindy_line.startswith(f"{CINDY_ID} is not measured (joints not known: left_ur_arm_")
    assert cindy_line.endswith("checked at its default pose or stowed")
    assert not snapshot.robots[BELLE_ID].base_tracked and snapshot.robots[CINDY_ID].unmeasured == arms
