"""
Stand-ins for other robots that are not measured, so every collision check treats them the same way.

The snapshot already holds a value for everything: a base not tracked stands where it was last seen, else at its
default pose; arm joints not measured are where they were last seen, else stowed (`Kinematics`). `with_stand_ins`
takes those values as known for every robot but the ones checked, and says which robots it guessed.

- Used by the base and arm planners, clearances and the process planner (`scene_for`, which lays the process design
  over them first); the robots they act for are left as they are (`with_guesses`).
- ! Every robot not live gets a warning: show it to the operator. A disabled robot is absent on purpose: left out.
- Pure: runs on any thread.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Collection

from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

#: Where a robot's base and joints come from, from measured to guessed; a robot's source is the most guessed it uses.
SOURCES = ("live", "last seen", "default")
LIVE, LAST_SEEN, DEFAULT = SOURCES


@dataclass(frozen=True)
class StandIns:
    """A scene with every other robot taken as known, and how.

    Attributes:
        scene: The scene; robots that stand in are copies, everything else is shared with the snapshot.
        sources: Id of each other robot present -> where its base and joints come from (`SOURCES`).
        warnings: One line per robot not live.
    """

    scene: Scene
    sources: dict[str, str]
    warnings: tuple[str, ...]


def with_stand_ins(snapshot: Scene, acting: Collection[str]) -> StandIns:
    """The snapshot with every robot but `acting` taken as known where it is; the snapshot itself if all are live.

    Args:
        snapshot: The tick's snapshot; it is not changed.
        acting: Ids of the robots planned or checked for; left as they are.
    """
    sources, warnings, robots = {}, [], dict(snapshot.robots)
    for key, robot in snapshot.robots.items():
        if key in acting or not robot.enabled:
            continue
        sources[key] = stand_in_source(robot)
        if sources[key] != LIVE:
            warnings.append(f"{key} is not measured ({'; '.join(robot.acting_problems())}): checked "
                            f"{'where last seen' if sources[key] == LAST_SEEN else 'at its default pose or stowed'}")
            robots[key] = replace(robot, base_tracked=True, unmeasured=frozenset())
    if not warnings:
        return StandIns(snapshot, sources, ())
    return StandIns(replace(snapshot, robots=robots), sources, tuple(warnings))


def unknown_robots(snapshot: Scene, acting: Collection[str]) -> frozenset[str]:
    """Ids of the robots present but not measured now, besides `acting`: those `with_stand_ins` would stand in."""
    return frozenset(key for key, robot in snapshot.robots.items()
                     if key not in acting and robot.enabled and robot.acting_problems())


def unknown_parts(robot: RobotObject) -> str:
    """Its parts not measured, short, e.g. "base, ur_arm"; arms by name (joints "<arm>_<UR joint>")."""
    arms = sorted({next((name[:-len(joint) - 1] for joint in UR_JOINT_NAMES if name.endswith(f"_{joint}")), name)
                   for name in robot.unmeasured})
    return ", ".join((["base"] if not robot.base_tracked else []) + arms)


def stand_in_source(robot: RobotObject) -> str:
    """Where a robot's values come from: LIVE, LAST_SEEN (all measured once), or DEFAULT (some never measured)."""
    if not robot.acting_problems():
        return LIVE
    never = (not robot.base_tracked and robot.base_time is None) or (robot.unmeasured and robot.joints_time is None)
    return DEFAULT if never else LAST_SEEN
