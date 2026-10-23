"""
What a plan was made from (`PlanBasis`), and why it no longer fits (`stale_reason`).

A plan is stale once anything it was planned on changed: the process, the current step, the transfer workaround or
'ignore built bars', the Rhino obstacles box or the statuses, the acting robot's arm or base, or another robot. Checked
by the step panel every tick and by the executor after Confirm and just before sending.

Pure: any thread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Mapping

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id

from ..assembly_planning.route_result import pose_error

if TYPE_CHECKING:
    from scaffolding_assembly_core.scene import Scene

    from ..assembly_planning.process_steps import LoadedProcess, Step

#: The acting arm may move this far, radians, before a plan is stale.
STALE_ARM = math.radians(2.0)
#: The acting base may move this far, metres and radians.
STALE_BASE = (0.002, math.radians(0.5))
#: Another robot's base may move this far, metres and radians, and each of its joints this far, radians.
STALE_OTHER = (0.01, math.radians(1.0), math.radians(1.0))


@dataclass(frozen=True)
class PlanBasis:
    """Everything a plan was made from, to tell when it no longer fits.

    Attributes:
        loaded: The loaded process, compared by identity.
        action: The action's id.
        movement_index: The planned movement's index in its action (the step's primary).
        serial: The acting robot.
        names: The planned joints.
        start: Their values the plan starts from (the live joints when planned).
        base: The acting robot's base when planned.
        others: Live id of every other robot in the snapshot -> its base and joints when planned.
        transfer_as_free: The transfer workaround when planned (the process plugin's).
        ignore_built: Whether built bars were left out.
        statuses: Every action's status in the run when planned.
        time: ROS time of planning, seconds.
        goal_base: Where the robot stood when the goal the plan reused was solved (Check goal), or None.
        obstacles_collide: Whether the process's obstacles from Rhino counted.
    """

    loaded: LoadedProcess
    action: str
    movement_index: int
    serial: str
    names: tuple[str, ...]
    start: Mapping[str, float]
    base: Pose
    others: Mapping[str, tuple[Pose, Mapping[str, float]]]
    transfer_as_free: bool
    ignore_built: bool
    statuses: Mapping[str, str]
    time: float
    goal_base: Pose | None = None
    obstacles_collide: bool = True

    def as_record(self) -> dict:
        """The basis as JSON values, for the run's records."""
        return {"process": str(self.loaded.source), "action": self.action, "movement_index": self.movement_index,
                "serial": self.serial, "names": list(self.names), "start": dict(self.start),
                "base": _pose(self.base),
                "others": {live: {"base": _pose(base), "joints": dict(joints)}
                           for live, (base, joints) in self.others.items()},
                "transfer_as_free": self.transfer_as_free, "ignore_built": self.ignore_built,
                "statuses": dict(self.statuses), "time": self.time,
                "goal_base": _pose(self.goal_base) if self.goal_base is not None else None,
                "obstacles_collide": self.obstacles_collide}


def basis_of(snapshot: Scene, loaded: LoadedProcess, step: Step, serial: str, names: tuple[str, ...],
             start: Mapping[str, float], transfer_as_free: bool, statuses: Mapping[str, str], ignore_built: bool,
             now: float, goal_base: Pose | None = None, obstacles_collide: bool = True) -> PlanBasis:
    """The basis of a plan made now from `snapshot`.

    Args:
        snapshot: The snapshot the plan was asked from.
        loaded: The loaded process.
        step: The planned movement's step.
        serial: The acting robot.
        names: The planned joints.
        start: Their start values.
        transfer_as_free: The transfer workaround (the process plugin's).
        statuses: Every action's status in the run.
        ignore_built: Whether built bars were left out.
        now: ROS time, seconds.
        goal_base: Where the goal the plan reused was solved, or None.
        obstacles_collide: Whether the process's obstacles from Rhino counted.

    Returns:
        PlanBasis: The basis.
    """
    acting = robot_id(serial)
    others = {live: (robot.base, dict(robot.joints)) for live, robot in snapshot.robots.items() if live != acting}
    return PlanBasis(loaded, step.action.id, step.movement_index, serial, tuple(names),
                     {name: float(start[name]) for name in names}, snapshot.robots[acting].base, others,
                     transfer_as_free, ignore_built, dict(statuses), now, goal_base, obstacles_collide)


def stale_reason(basis: PlanBasis, snapshot: Scene, *, loaded: LoadedProcess | None,
                 current: tuple[str, int] | None, transfer_as_free: bool, ignore_built: bool,
                 statuses: Mapping[str, str], obstacles_collide: bool = True) -> str:
    """Why a plan no longer fits; "" if it still does. The first reason found, in this order.

    The process, the step, the workaround or 'ignore built bars', the Rhino obstacles box, the statuses, the acting
    robot (there, its arm, its base, where its reused goal was solved), then each other robot.

    Args:
        basis: What the plan was made from.
        snapshot: This tick's snapshot.
        loaded: The loaded process now.
        current: The current (action id, primary movement index) now, or None.
        transfer_as_free: The transfer workaround now.
        ignore_built: Whether built bars are left out now.
        statuses: Every action's status in the run now.
        obstacles_collide: Whether the process's obstacles from Rhino count now.

    Returns:
        str: E.g. "cindy's arm moved 2.3° since planning (replan beyond 2.0°)", or "".
    """
    if loaded is not basis.loaded:
        return "the process was loaded again since planning"
    if current != (basis.action, basis.movement_index):
        return "the current step changed since planning"
    if transfer_as_free != basis.transfer_as_free or ignore_built != basis.ignore_built:
        return "the transfer workaround or 'ignore built bars' changed since planning"
    if obstacles_collide != basis.obstacles_collide:
        return f"the Rhino obstacles were switched {'on' if obstacles_collide else 'off'} since planning"
    if dict(statuses) != dict(basis.statuses):
        return "an action was marked done or reopened since planning"
    robot = snapshot.robots.get(robot_id(basis.serial))
    if robot is None:
        return f"{basis.serial} is no longer in the scene"
    moved = max((abs(float(robot.joints.get(name, math.inf)) - value) for name, value in basis.start.items()),
                default=0.0)
    if moved > STALE_ARM:
        return (f"{basis.serial}'s arm moved {math.degrees(moved):.1f}° since planning "
                f"(replan beyond {math.degrees(STALE_ARM):.1f}°)")
    distance, turn = pose_error(robot.base, basis.base)
    if distance > STALE_BASE[0] or turn > STALE_BASE[1]:
        return (f"{basis.serial}'s base moved {distance * 1e3:.1f} mm, {math.degrees(turn):.2f}° since planning "
                f"(replan beyond {STALE_BASE[0] * 1e3:.0f} mm or {math.degrees(STALE_BASE[1]):.1f}°)")
    if basis.goal_base is not None:
        distance, turn = pose_error(robot.base, basis.goal_base)
        if distance > STALE_BASE[0] or turn > STALE_BASE[1]:
            return (f"{basis.serial}'s base is {distance * 1e3:.1f} mm, {math.degrees(turn):.2f}° from where its goal "
                    f"was solved (Check goal again and replan beyond {STALE_BASE[0] * 1e3:.0f} mm or "
                    f"{math.degrees(STALE_BASE[1]):.1f}°)")
    for live, (base, joints) in basis.others.items():
        other = snapshot.robots.get(live)
        if other is None:
            return f"{live} left the scene since planning"
        distance, turn = pose_error(other.base, base)
        bent = max((abs(float(other.joints[name]) - value) for name, value in joints.items() if name in other.joints),
                   default=0.0)
        if distance > STALE_OTHER[0] or turn > STALE_OTHER[1] or bent > STALE_OTHER[2]:
            return (f"{live} moved since planning: base {distance * 1e2:.1f} cm, {math.degrees(turn):.1f}°, "
                    f"joints up to {math.degrees(bent):.1f}° (replan beyond {STALE_OTHER[0] * 1e2:.0f} cm, "
                    f"{math.degrees(STALE_OTHER[1]):.0f}° or {math.degrees(STALE_OTHER[2]):.0f}°)")
    return ""


def _pose(pose: Pose) -> list[float]:
    """A pose as [x, y, z, qx, qy, qz, qw]."""
    return [*map(float, pose.position), *map(float, pose.orientation)]
