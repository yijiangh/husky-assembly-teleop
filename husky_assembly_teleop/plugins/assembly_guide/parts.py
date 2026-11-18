"""
The parts the guide walks through, action by action.

Each action's parking of the robots it does not use, its drive to its start, the executor's parts of each step, Mark
done.

The executor's split (`step_parts`) is the one source of what a step runs; the guide adds only the parking, the drive
and the mark, and folds the ungrasp into the retreat it runs with. Pure: any thread.

* A robot holding a bar at its action's start gets no drive: it stays at the bar (a release, a hold), as in the old
  procedure; Plan › Base moves it by hand if it must.
* Parking is the simple rule for now: before an action of a robot, every other configured robot that holds no bar at
  the action's start stows its arms and drives to its parked pose (park.yaml), so it never stands in the way. A robot
  parked already has nothing to do. Actions by hand park nothing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scaffolding_assembly_core.geometry import Pose

from ...config import find_robot_serial, short_serial
from ...shared.assembly_planning.movement import Routine, UnknownMovement
from ...shared.assembly_planning.steps import OperatorStep, operator_steps
from ...shared.movement_exec_contract import (BASE_PART, BY_HAND_PART, HANDOFF_PART, INSERT_PART, MOTION_PART,
                                              OPEN_PART, REFUSED_PART, RETREAT_PART, SERVO_INDEX, SERVO_PART,
                                              WITH_MOTION_PART, PartPlan, step_parts)
from ...ui.pose_input import yaw_from_xyzw

#: The kinds of part, each with its own controls in the panel (`kinds.py`).
DRIVE, MOVE, HAND, SERVO, INSERT, RETREAT, GRIPPER, MARK, REFUSED, PARK = (
    "drive", "move", "by hand", "servo", "insert", "retreat", "gripper", "mark done", "not supported", "park")
#: The executor's part kinds, as the guide's.
KIND_OF = {MOTION_PART: MOVE, INSERT_PART: INSERT, RETREAT_PART: RETREAT, OPEN_PART: GRIPPER, HANDOFF_PART: GRIPPER,
           WITH_MOTION_PART: GRIPPER, BY_HAND_PART: HAND, BASE_PART: HAND, SERVO_PART: SERVO, REFUSED_PART: REFUSED}
#: The kinds the executor runs; the drive is the base planner's, Mark done the run's.
EXECUTED = (MOVE, HAND, SERVO, INSERT, RETREAT, GRIPPER)
#: A drive counts as at its end within these: the follower's own tolerance (crl_husky pure_pursuit).
DRIVE_ARRIVED, DRIVE_YAW_ARRIVED = 0.06, math.radians(6.0)
#: An arm counts as at its end within this, per joint, for a Done the operator presses; a move the guide skips by
#: itself needs the arms within NOTHING_TO_MOVE (what a finished trajectory leaves: about 1.4 mm at 0.8 m reach).
ARM_ARRIVED = math.radians(1.0)
NOTHING_TO_MOVE = math.radians(0.1)
#: What each routine's move is called.
MOVE_NAMES = {Routine.FREE_TO_LOAD: "travel to load", Routine.TRANSFER_FREE: "transfer, empty tools",
              Routine.TRANSFER_HELD: "transfer with the bar", Routine.FREE_HOME: "home",
              Routine.SUPPORT_APPROACH: "approach", Routine.SUPPORT_LINEAR: "straight line",
              Routine.SUPPORT_LEAVE: "leave"}


@dataclass(frozen=True)
class GuidePart:
    """One part of an action, as the guide runs it.

    Attributes:
        action: Its action's id.
        kind: DRIVE, MOVE, HAND, SERVO, INSERT, RETREAT, GRIPPER, MARK, REFUSED or PARK.
        label: What happens, for the operator, e.g. "B21_J_insert: insert".
        movement: The movement selected for it in the process plugin (the step's primary for a servo; the action's
            last for Mark done), so the panels' Load (Step; Step joints, Step IK) loads its target.
        movement_index: That movement's index in its action.
        step: The operator step it belongs to; None for the drive and Mark done.
        plans: The executor's parts it sends, in order (a retreat with its ungrasp).
        base: (x, y, yaw) the drive (or the parking) goes to, metres and radians; None for every other part.
        robot: The serial of the robot a PARK part moves; "" for every other part (the action's robot acts).
        stow: A PARK part's arms and the six joints each stows at, radians, UR driver order.
        why: Why a REFUSED part is not supported (the classifier's words); "" for every other part.
    """

    action: str
    kind: str
    label: str
    movement: str
    movement_index: int
    step: OperatorStep | None = None
    plans: tuple[PartPlan, ...] = ()
    base: tuple[float, float, float] | None = None
    robot: str = ""
    stow: tuple[tuple[str, tuple[float, ...]], ...] = ()
    why: str = ""

    @property
    def done_keys(self) -> frozenset[tuple[int, int]]:
        """(step primary, movement index) the executor reports finished once this part ran (`finished`)."""
        if self.step is None:
            return frozenset()
        return frozenset((self.step.primary, index) for plan in self.plans for index in (plan.index, *plan.also))

    @property
    def routine(self) -> Routine | None:
        """Routine | None: The routine of its first executor part."""
        return self.plans[0].routine if self.plans else None


def floor_pose(pose: Pose) -> tuple[float, float, float]:
    """(x, y, yaw) of a pose on the floor."""
    return float(pose.position[0]), float(pose.position[1]), yaw_from_xyzw(pose.orientation)


def action_parts(loaded, action_id: str, robots, transfer_as_free: bool, park: bool = False) -> list[GuidePart]:
    """One action's parts in run order.

    Parking the robots it does not use (with `park`), the drive to its start, each step's executor parts, Mark done.

    Args:
        loaded: The loaded process (`LoadedProcess`).
        action_id: The action.
        robots: The configured robots.
        transfer_as_free: The process plugin's transfer workaround.
        park: Park the robots the action does not use first (`park_parts`; the guide's "Park idle robots").
    """
    action = loaded.process.actions[action_id]
    movements = action.movements
    parts: list[GuidePart] = []
    start = movements[0].start.robots.get(action.robot) if action.robot and movements else None
    try:
        steps = operator_steps(loaded.process, action, transfer_as_free)
    except UnknownMovement as error:
        steps = []
        parts.append(GuidePart(action_id, REFUSED, f"{action_id}: not supported", movements[0].id if movements else "",
                               0, why=str(error)))
    first = steps[0].primary if steps else 0
    if park and movements:
        parts += park_parts(loaded, action_id, robots, movements[first].id, first)
    holds = action.robot in holding_robots(loaded.process, action_id)
    if start is not None and start.base is not None and not holds:
        parts.append(GuidePart(action_id, DRIVE, f"drive to {action_id}'s start", movements[first].id, first,
                               base=floor_pose(start.base)))
    for step in steps:
        plans = step_parts(loaded, action_id, step, robots, transfer_as_free)
        # * One part each, but the ungrasp runs during its retreat: it goes with it, in one request. A ground bar's
        #   servo after its insert is a part of its own: the executor keeps the insert's planned end for it.
        retreat = next((plan for plan in plans if plan.kind == RETREAT_PART), None)
        folded = {id(plan) for plan in plans if plan.kind == WITH_MOTION_PART and retreat is not None}
        for plan in plans:
            if id(plan) in folded:
                continue
            index = step.primary if plan.index == SERVO_INDEX else plan.index
            sent = tuple(each for each in plans if each is plan or (id(each) in folded and plan is retreat))
            parts.append(GuidePart(action_id, KIND_OF[plan.kind], _label(movements[index].id, plan),
                                   movements[index].id, index, step, sent))
    last = len(movements) - 1
    parts.append(GuidePart(action_id, MARK, f"mark {action_id} done", movements[last].id if movements else "", last))
    return parts


def holding_robots(process, action_id: str) -> set[str]:
    """The robots (process ids, e.g. "robots/cindy") holding a bar at an action's start, as the export has it."""
    action = process.actions[action_id]
    if not action.movements:
        return set()
    start = action.movements[0].start
    return {"/".join(holder.to.split("/")[:2]) for holders in start.attached.values() for holder in holders}


def park_parts(loaded, action_id: str, robots, movement: str, movement_index: int) -> list[GuidePart]:
    """One PARK part per configured robot an action of a robot does not use: not its robot, holding no bar at its start.

    Args:
        loaded: The loaded process (`LoadedProcess`).
        action_id: The action.
        robots: The configured robots (`RobotConfig`), each with its parked pose and its arms' stow joints.
        movement: The movement selected while a park part is shown (the action's first step).
        movement_index: Its index in the action.

    Returns:
        list[GuidePart]: In the configured robots' order; none for an action by hand.
    """
    process = loaded.process
    action = process.actions[action_id]
    if not action.robot or not action.movements:
        return []
    holding = holding_robots(process, action_id)
    busy = {find_robot_serial(robots, process.robots[robot].serial or "")
            for robot in (action.robot, *holding) if robot in process.robots}
    parts = []
    for config in robots:
        stow = tuple((arm.name, tuple(arm.stow_joints)) for arm in config.arms if arm.stow_joints is not None)
        if config.serial in busy or config.park_pose is None:
            continue
        name = next((robot.name for robot in process.robots.values()
                     if find_robot_serial(robots, robot.serial or "") == config.serial), "")
        who = f"{name} ({short_serial(config.serial)})" if name else short_serial(config.serial)
        parts.append(GuidePart(action_id, PARK, f"park {who}: out of {action_id}'s way",
                               movement, movement_index,
                               base=tuple(float(value) for value in config.park_pose), robot=config.serial,
                               stow=stow))
    return parts


def schedule_parts(loaded, robots, transfer_as_free: bool, park: bool = False) -> list[GuidePart]:
    """Every scheduled action's parts, in schedule order; `park` as for `action_parts`."""
    return [part for action_id in loaded.process.schedule
            for part in action_parts(loaded, action_id, robots, transfer_as_free, park)]


def _label(movement: str, plan: PartPlan) -> str:
    """A part's label: its movement and what it does, e.g. "B21_H_approach: approach"."""
    kind = KIND_OF[plan.kind]
    if kind == MOVE:
        what = MOVE_NAMES.get(plan.routine, plan.routine.value if plan.routine else "move")
    elif kind == GRIPPER:
        names = {OPEN_PART: "open the gripper", HANDOFF_PART: "close the gripper, arm compliant"}
        what = names.get(plan.kind, "tool step")
    elif kind == HAND:
        what = "by hand"
    else:
        what = kind
    return f"{movement}: {what}"


def arm_arrived(joints, goal, tolerance: float = ARM_ARRIVED) -> bool:
    """Whether measured joints are within `tolerance` of `goal`, per joint; False without a reading."""
    if joints is None or any(have is None for have in joints):
        return False
    return max(abs(float(have) - float(want)) for have, want in zip(joints, goal)) <= tolerance


def base_arrived(pose: Pose, goal: tuple[float, float, float]) -> bool:
    """Whether a base stands within the follower's tolerance of floor pose `goal`."""
    x, y, yaw = floor_pose(pose)
    turn = (yaw - goal[2] + math.pi) % (2 * math.pi) - math.pi
    return math.hypot(x - goal[0], y - goal[1]) <= DRIVE_ARRIVED and abs(turn) <= DRIVE_YAW_ARRIVED
