"""
What one movement of a process asks of its robot: the routine, the arms, the goal, and the drives really sent.

`classify` reads only the movement's parts (`Movement`, `Target`), never class names or ids. The action type is a
cross-check only: a mismatch goes into `warnings`. Pure: any thread.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import split_link_id
from scaffolding_assembly_core.process import Action, LineSpec, Movement, Process, State
from scaffolding_assembly_core.process.plan_check import end_state
from scaffolding_assembly_core.process.relations import bar_of
from scaffolding_assembly_core.process.types import GROUND_PREFIX, HALF_PREFIX
from scaffolding_assembly_core.robot import FLANGE_SUFFIX

from .joints import arm_joint_names, arm_side

#: Why a drive is not sent (`MatingDrives.dropped`).
GROUND_HALF = "ground half"
NO_MATE = "no mate (one-sided bar)"
RECEIVER_NOT_BUILT = "receiver not built"
ON_NOTHING = "tool on nothing"
LOOSEN_NEVER_SENT = "loosen on release: never sent"


class Routine(str, Enum):
    """How a movement is run; one per kind of movement, whatever its id or action."""

    FREE_TO_LOAD = "free_to_load"
    TRANSFER_HELD = "transfer_held"
    TRANSFER_FREE = "transfer_free"
    INSERT = "insert"
    RETREAT_LINEAR = "retreat_linear"
    FREE_HOME = "free_home"
    SUPPORT_APPROACH = "support_approach"
    SUPPORT_LINEAR = "support_linear"
    SUPPORT_LEAVE = "support_leave"
    TOOL_ONLY = "tool_only"
    OPERATOR = "operator"
    BASE_ONLY = "base_only"


class GoalSource(str, Enum):
    """Where a movement's goal comes from."""

    LINK_POSES = "link_poses"
    LINE = "line"
    JOINTS = "joints"
    NEXT_START = "next_start"
    STOW = "stow"
    NONE = "none"


#: The routines each action type is expected to hold: anything else is warned about, never refused.
_EXPECTED = {
    "bar_jointing": {Routine.FREE_TO_LOAD, Routine.TOOL_ONLY, Routine.TRANSFER_HELD, Routine.TRANSFER_FREE,
                     Routine.INSERT},
    "bar_release": {Routine.TOOL_ONLY, Routine.RETREAT_LINEAR, Routine.FREE_HOME},
    "bar_holding": {Routine.SUPPORT_APPROACH, Routine.TOOL_ONLY, Routine.SUPPORT_LINEAR},
    "bar_holding_release": {Routine.TOOL_ONLY, Routine.SUPPORT_LINEAR, Routine.SUPPORT_LEAVE},
    "bar_placing": set(),
}
#: Routines any action may hold: an operator's confirmation, a base drive.
_ANYWHERE = {Routine.OPERATOR, Routine.BASE_ONLY}


class UnknownMovement(ValueError):
    """A movement whose parts match no routine; the message lists the parts."""


@dataclass(frozen=True)
class ArmPart:
    """One moving arm of a movement.

    Attributes:
        flange: Its flange link id, e.g. "robots/cindy/left_ur_arm_tool0", as the movement names it.
        arm: The arm's name, e.g. "left_ur_arm".
        side: "left", "right", or "arm" for a one-arm robot.
        joints: Its six joint names, in the UR driver's order.
        line: Direction and distance of a linear path, or None.
        target: World pose the target gives the flange, or None.
    """

    flange: str
    arm: str
    side: str
    joints: tuple[str, ...]
    line: LineSpec | None = None
    target: Pose | None = None


@dataclass(frozen=True)
class MatingDrives:
    """The drives of a movement the monitor sends, and those it leaves out with the reason.

    Attributes:
        kept: Tool id -> "tighten" or "loosen", to send.
        dropped: Tool id -> why its drive is not sent.
        grounded: The insert's bar stands on the ground: a tighten was dropped for a ground half, or a straight
            coupled move has a tool on a ground half. The insert runs rigid.
        warnings: Diagnostics only, e.g. a kept tighten on a half that is not a male one.
    """

    kept: dict[str, str] = field(default_factory=dict)
    dropped: dict[str, str] = field(default_factory=dict)
    grounded: bool = False
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class MovementSpec:
    """One movement, classified: what the planners and the executor need, read from its parts.

    Attributes:
        action, movement: Their ids.
        action_type, robot, bar: The action's.
        routine: How it is run.
        arms: The moving arms, in the movement's order.
        coupled, controller, ends_on: The movement's.
        goal: Where the goal comes from.
        goal_joints: The acting robot's joints at the goal, if known: the target's, or the next start's.
        goal_from: The id of the movement `goal_joints` (or the search for them) came from; empty if none.
        grips: Tool id -> the grip it ends with.
        drives: The drives really sent.
        warnings: Mismatches with the action type, and drive diagnostics.
    """

    action: str
    movement: str
    action_type: str
    robot: str
    bar: str
    routine: Routine
    arms: tuple[ArmPart, ...]
    coupled: bool
    controller: str | None
    ends_on: str
    goal: GoalSource
    goal_joints: dict[str, float] | None
    goal_from: str
    grips: dict[str, str]
    drives: MatingDrives
    warnings: tuple[str, ...] = ()


def classify(process: Process, action: Action, movement: Movement, transfer_as_free: bool = False) -> MovementSpec:
    """Classify one movement by its parts; the first routine that matches wins (`_routine`).

    Args:
        process: The process.
        action: The action the movement belongs to.
        movement: The movement.
        transfer_as_free: Run a coupled free movement without the bar (the transfer workaround).

    Returns:
        MovementSpec: The classified movement.

    Raises:
        UnknownMovement: If its parts match no routine.
    """
    routine = _routine(action, movement, transfer_as_free)
    goal, goal_joints, goal_from = _goal(process, action, movement, routine)
    drives = mating_drives(process, movement)
    warnings = list(drives.warnings)
    if routine not in _EXPECTED.get(action.type, set()) | _ANYWHERE:
        warnings.append(f"{movement.id}: a {routine.value} movement in a {action.type} action")
    return MovementSpec(action.id, movement.id, action.type, action.robot, action.bar, routine,
                        tuple(_arm_part(movement, flange) for flange in movement.arms), movement.coupled,
                        movement.controller, movement.ends_on, goal, goal_joints, goal_from,
                        dict(movement.grip_change), drives, tuple(warnings))


def classify_action(process: Process, action: Action, transfer_as_free: bool = False) -> tuple[MovementSpec, ...]:
    """Classify every movement of an action, in order (`classify`).

    Args:
        process: The process.
        action: The action.
        transfer_as_free: Run a coupled free movement without the bar (the transfer workaround).

    Returns:
        tuple[MovementSpec, ...]: One per movement.

    Raises:
        UnknownMovement: If one of them matches no routine.
    """
    return tuple(classify(process, action, movement, transfer_as_free) for movement in action.movements)


def routine_or_none(action: Action, movement: Movement, transfer_as_free: bool = False) -> Routine | None:
    """A movement's routine by its parts alone, or None if they match none; for labels, planners use `classify`.

    Args:
        action: The action the movement belongs to.
        movement: The movement.
        transfer_as_free: Run a coupled free movement without the bar (the transfer workaround).

    Returns:
        Routine | None: The routine, or None.
    """
    try:
        return _routine(action, movement, transfer_as_free)
    except UnknownMovement:
        return None


def next_start(process: Process, action: Action, movement: Movement) -> tuple[dict | None, str]:
    """The acting robot's joints at the next start that gives them, walking forward in schedule order.

    Later movements where no arm moves and the robot's joints are null are skipped.

    Args:
        process: The process.
        action: The action of `movement`; its robot is the one looked for.
        movement: The movement to start after.

    Returns:
        tuple[dict | None, str]: The joints and the id of the movement they start; (None, id) at an arm movement
            whose start joints are null, or where the robot is absent; (None, "") after the last movement.
    """
    after = False
    for other_action, other in process.movements():
        if not after:
            after = other_action.id == action.id and other.id == movement.id
            continue
        state = other.start.robots.get(action.robot)
        if state is None:
            return None, other.id
        if state.joints is not None:
            return dict(state.joints), other.id
        if other.arms:
            return None, other.id
    return None, ""


def mating_drives(process: Process, movement: Movement) -> MatingDrives:
    """The drives the monitor sends for a movement, read from the process mates, not body names.

    - A tighten is kept only when its tool is on a joint half mated to another half whose bar is built and present.
      A half mated to the ground is never tightened, and makes the insert run rigid (`grounded`), also when the
      insert has no drives at all.
    - A loosen is kept only when the movement unbuilds the bar the tool is on; on a release it is never sent.

    Args:
        process: The process.
        movement: The movement.

    Returns:
        MatingDrives: Kept and dropped drives; drives already correct pass through unchanged.
    """
    start = movement.start
    kept, dropped, warnings = {}, {}, []
    grounded = False
    unbuilt = None
    for tool, drive in movement.drives.items():
        tool_state = start.tools.get(tool)
        on = tool_state.on if tool_state is not None else None
        if drive == "tighten":
            reason = _tighten_problem(process, start, on)
            grounded = grounded or reason == GROUND_HALF
            if reason is None and "Male" not in process.bodies[on].part:
                warnings.append(f"{movement.id}: {tool} tightens {on}, whose part {process.bodies[on].part!r} "
                                f"is not a male half")
        else:
            if unbuilt is None:
                unbuilt = start.built - end_state(movement).built
            reason = None if on is not None and bar_of(process, on) in unbuilt else LOOSEN_NEVER_SENT
        if reason is None:
            kept[tool] = drive
        else:
            dropped[tool] = reason
    if not grounded and movement.coupled and movement.path == "linear":
        # * An insert without drives (a ground bar in newer exports): its tools' halves tell the ground.
        on = [start.tools[tool].on for tool in _flange_tools(process, movement) if start.tools.get(tool) is not None]
        grounded = any(_tighten_problem(process, start, body) == GROUND_HALF for body in on if body is not None)
    return MatingDrives(kept, dropped, grounded, tuple(warnings))


def _flange_tools(process: Process, movement: Movement) -> list[str]:
    """The tools on a movement's moving flanges."""
    tools = []
    for flange in movement.arms:
        robot, link = split_link_id(flange)
        spec = process.robots.get(robot)
        if spec is not None and link in spec.tools:
            tools.append(spec.tools[link])
    return tools


def _tighten_problem(process: Process, start: State, on: str | None) -> str | None:
    """Why a tighten on the body `on` is not sent, or None if it is."""
    if on is None:
        return ON_NOTHING
    others = [b if a == on else a for a, b in process.mates if on in (a, b)]
    if any(other.startswith(GROUND_PREFIX) for other in others):
        return GROUND_HALF
    halves = [other for other in others if on.startswith(HALF_PREFIX) and other.startswith(HALF_PREFIX)]
    if not halves:
        return NO_MATE
    receivers = [bar_of(process, half) for half in halves]
    if not any(bar in start.built and bar in start.present for bar in receivers):
        return RECEIVER_NOT_BUILT
    return None


def _routine(action: Action, movement: Movement, transfer_as_free: bool) -> Routine:
    """The routine of a movement, by its parts; first match wins.

    Raises:
        UnknownMovement: If its parts match no routine.
    """
    target = movement.target
    own = target.robots.get(action.robot) if target is not None else None
    builds = target is not None and target.built is not None and action.bar in target.built \
        and action.bar not in movement.start.built
    arms, coupled = len(movement.arms), movement.coupled
    # * Arms and a moving base together match no routine: no planner drives both.
    linear = arms and not movement.base and movement.path == "linear"
    free = arms and not movement.base and movement.path == "free"
    if not arms and movement.ends_on == "operator":
        return Routine.OPERATOR
    if not arms and movement.base:
        return Routine.BASE_ONLY
    if not arms and movement.has_tool_part:
        return Routine.TOOL_ONLY
    # * Any straight move holding a bar is an insert: a ground bar's has no drives and is built only later.
    if linear and coupled:
        return Routine.INSERT
    if linear and not coupled:
        return Routine.RETREAT_LINEAR if arms >= 2 else Routine.SUPPORT_LINEAR
    if free and coupled:
        return Routine.TRANSFER_FREE if transfer_as_free else Routine.TRANSFER_HELD
    if free and arms >= 2 and target is None:
        return Routine.FREE_TO_LOAD
    if free and arms >= 2 and own is not None and own.joints and not own.link_poses:
        return Routine.FREE_HOME
    if free and arms == 1:
        return Routine.SUPPORT_APPROACH if own is not None and own.link_poses else Routine.SUPPORT_LEAVE
    raise UnknownMovement(f"{movement.id} ({action.id}) matches no routine: arms {list(movement.arms)}, "
                          f"base {list(movement.base)}, path {movement.path}, coupled {coupled}, "
                          f"drives {movement.drives}, builds {action.bar}: {builds}, ends on {movement.ends_on}, "
                          f"target {'none' if target is None else 'given'}")


def _goal(process: Process, action: Action, movement: Movement,
          routine: Routine) -> tuple[GoalSource, dict | None, str]:
    """A movement's goal: where it comes from, the acting robot's joints there if known, and their movement id."""
    target = movement.target
    own = dict(target.joints.get(action.robot, {})) if target is not None else {}
    given = (own or None, movement.id if own else "")
    if routine in (Routine.OPERATOR, Routine.BASE_ONLY, Routine.TOOL_ONLY):
        return GoalSource.NONE, None, ""
    if routine in (Routine.INSERT, Routine.RETREAT_LINEAR, Routine.SUPPORT_LINEAR):
        return (GoalSource.LINE, *given)
    if routine in (Routine.TRANSFER_HELD, Routine.TRANSFER_FREE, Routine.SUPPORT_APPROACH):
        return (GoalSource.LINK_POSES, *given)
    if routine is Routine.FREE_HOME or (routine is Routine.SUPPORT_LEAVE and own):
        return (GoalSource.JOINTS, *given)
    joints, source = next_start(process, action, movement)
    if routine is Routine.FREE_TO_LOAD or joints is not None:
        return GoalSource.NEXT_START, joints, source
    return GoalSource.STOW, None, ""


def _arm_part(movement: Movement, flange: str) -> ArmPart:
    """One moving arm of a movement, from its flange link id."""
    _robot, link = split_link_id(flange)
    arm = link[:-len(FLANGE_SUFFIX)] if link.endswith(FLANGE_SUFFIX) else link
    target = movement.target.link_poses.get(flange) if movement.target is not None else None
    return ArmPart(flange, arm, arm_side(arm), tuple(arm_joint_names(arm)), movement.line.get(flange), target)
