"""Tests for design_io carry: joints assumed for movements without authored start joints."""

from design_io_fixtures import WRITER

from husky_assembly_teleop.design_io import (Action, Design, Movement, Pose, RobotState, State, Target,
                                             assumed_joints)
from husky_assembly_teleop.design_io.carry import assumed_start_all

ROBOTS = ("robots/a", "robots/b")


def _movement(movement_id: str, robot: str, start=None, target=None) -> Movement:
    """A free movement of `robot`: its start joints (or None) and target joints (or None)."""
    robots = {r: RobotState(Pose(), start if r == robot else None) for r in ROBOTS}
    return Movement(id=movement_id, type="free", controller="joint_tracking", arms=(f"{robot}/tool0",),
                    start=State(robots=robots, present=frozenset(), poses={}, attached={}),
                    target=Target(joints={robot: target}, links={}) if target else None)


def _design(*actions) -> Design:
    """A design in memory with only a schedule of actions (carry needs nothing else)."""
    return Design(folder=None, writer=WRITER, robots={}, tools={}, bodies={},
                  schedule=tuple(a.id for a in actions), actions={a.id: a for a in actions})


def _action(action_id: str, robot: str, *movements) -> Action:
    """An action of `robot`."""
    return Action(id=action_id, type="bar_jointing", robot=robot, bar="bars/B1", movements=movements)


def test_authored_and_carried_from_previous_target():
    """An authored start is used as is; the next movement starts where it ended (start + target)."""
    design = _design(_action("A1", "robots/a",
                             _movement("M0", "robots/a", start={"j1": 1.0, "j2": 2.0}, target={"j2": 5.0}),
                             _movement("M1", "robots/a")))
    assert assumed_joints(design, "A1", "M0") == (None, "")
    assert assumed_joints(design, "A1", "M1") == ({"j1": 1.0, "j2": 5.0}, "M0")


def test_own_target_before_anything_known():
    """Before the robot's first known pose, a movement uses its own target."""
    design = _design(_action("A1", "robots/a", _movement("M0", "robots/a", target={"j1": 3.0}),
                             _movement("M1", "robots/a")))
    assert assumed_joints(design, "A1", "M0") == ({"j1": 3.0}, "own target")
    assert assumed_joints(design, "A1", "M1") == ({"j1": 3.0}, "M0")


def test_look_ahead_and_robots_kept_apart():
    """With nothing before it, a movement takes the robot's next known start; other robots don't count."""
    design = _design(_action("A1", "robots/a", _movement("M0", "robots/a")),
                     _action("A2", "robots/b", _movement("M1", "robots/b", start={"k": 9.0})),
                     _action("A3", "robots/a", _movement("M2", "robots/a", start={"j1": 7.0})))
    everything = assumed_start_all(design)
    assert everything[("A1", "M0")] == ({"j1": 7.0}, "M2 (later)")
    assert everything[("A2", "M1")] == (None, "")


def test_nothing_known():
    """A robot with no start or target anywhere has nothing to assume."""
    design = _design(_action("A1", "robots/a", _movement("M0", "robots/a"), _movement("M1", "robots/a")),
                     _action("A2", "robots/b", _movement("M2", "robots/b", start={"k": 1.0})))
    assert assumed_joints(design, "A1", "M0") == (None, "")
    assert assumed_joints(design, "A1", "M1") == (None, "")
