"""
What the movement_exec plugin offers other plugins (`MovementExecContract`), its requests, and a step's parts.

Other plugins ask with an `ExecRequest`; `part_plan` says how a step's movements are run, `step_parts` adds the servo
after the step's primary movement where it gets one (`servo_kind`).

A part is a motion along a plan (a plain one, an insert or a retreat: `MOTION_KINDS`), a tool step (Robotiq open, the
compliant Robotiq handoff, a scaffolding grasp), an ungrasp its retreat runs, a step done by hand, or the servo
(SERVO_PART, index SERVO_INDEX: it is no movement of its own). Routines not supported yet are refused, naming what is
missing (`LATER`). While movement_exec has a robot's lease for a step, nothing that changes the step may change
(`step_running`). A transfer's bar servo leaves `Carried` for its insert's plan.

Pure: any thread, but `step_running` (main thread only: it reads the robots' leases).
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Mapping, Protocol

from scaffolding_assembly_core.robot import FLANGE_SUFFIX

from ..config import find_robot_serial
from .assembly_planning.movement import Routine, UnknownMovement, classify
from .assembly_planning.steps import OperatorStep, operator_steps, step_index_of
from .execution.servo import AFTER_INSERT, APPROACH, TRANSFER, servo_kind

if TYPE_CHECKING:
    from scaffolding_assembly_core.geometry import Pose

    from ..config import RobotConfig
    from ..robot_interface.robot import HuskyRobotInterface
    from .assembly_run_contract import RunView
    from .execution.basis import PlanBasis
    from .execution.operator import Prompt
    from .assembly_planning.route_result import RouteResult
    from .assembly_planning.process_steps import LoadedProcess

#: Seconds a motion takes by default, per routine (the old export's values); the operator may slow it, up to
#: MAX_TRAJ_TIME.
DEFAULT_TRAJ_TIME = {Routine.FREE_TO_LOAD: 30.0, Routine.TRANSFER_FREE: 10.0, Routine.TRANSFER_HELD: 10.0,
                     Routine.FREE_HOME: 10.0, Routine.SUPPORT_APPROACH: 15.0, Routine.SUPPORT_LEAVE: 15.0,
                     Routine.SUPPORT_LINEAR: 5.0, Routine.INSERT: 5.0, Routine.RETREAT_LINEAR: 5.0}
MAX_TRAJ_TIME = 90.0
#: The routines run as a plain motion along the plan.
MOTION = frozenset({Routine.FREE_TO_LOAD, Routine.TRANSFER_FREE, Routine.TRANSFER_HELD, Routine.FREE_HOME,
                    Routine.SUPPORT_APPROACH, Routine.SUPPORT_LEAVE, Routine.SUPPORT_LINEAR})
#: Routines not supported yet, and what each is in the refusal.
LATER: dict[Routine, str] = {}

#: The plugin that runs a step on the robots: while it has their leases, nothing that moves the step may change.
STEP_RUNNER = "movement_exec"

#: What a part does.
MOTION_PART, OPEN_PART, GRASP_PART, BY_HAND_PART, BASE_PART, REFUSED_PART, SERVO_PART = (
    "motion", "robotiq_open", "grasp", "operator", "base", "refused", "servo")
INSERT_PART, RETREAT_PART, HANDOFF_PART, WITH_MOTION_PART = "insert", "retreat", "robotiq_handoff", "with_motion"
#: The parts that move the arms along the step's plan.
MOTION_KINDS = frozenset({MOTION_PART, INSERT_PART, RETREAT_PART})
#: ! The servo part's index: the servo is no movement of its own, so `ExecResult.done` and the step panel's done set
#:   tell it from the primary movement by this value.
SERVO_INDEX = -1
#: What each servo kind aims at, in the servo part's text.
SERVO_TARGET = {TRANSFER: "the insert start", APPROACH: "the approach pose", AFTER_INSERT: "the insert end"}

#: How a request ended.
DONE, DRY_RUN, REFUSED, CANCELLED, FAILED, HOLD_NOT_SENT = (
    "done", "dry_run", "refused", "cancelled", "failed", "hold_not_sent")


@dataclass(frozen=True)
class PartPlan:
    """One movement of a step, as the executor runs it.

    Attributes:
        index: The movement's index in its action; SERVO_INDEX for the servo after the primary movement.
        movement: Its id (the primary's, for the servo).
        routine: Its routine; None if it can't be classified.
        kind: MOTION_PART, INSERT_PART, RETREAT_PART, OPEN_PART, HANDOFF_PART, GRASP_PART, WITH_MOTION_PART,
            BY_HAND_PART, BASE_PART, REFUSED_PART or SERVO_PART.
        tools: The arms whose tools it drives; for a motion, a retreat or the servo, the arms it moves; for an
            insert, the arms whose joint screw tightens (none: a ground bar, rigid).
        text: What it does; for a refused part, why.
    """

    index: int
    movement: str
    routine: Routine | None
    kind: str
    tools: tuple[str, ...]
    text: str


@dataclass(frozen=True)
class ExecRequest:
    """One step to run, built by the step panel on the main thread.

    Attributes:
        loaded: The loaded process.
        action: The action's id.
        step: The step.
        parts: The step's parts still to run, in order.
        serial: The acting robot.
        robot_name: Its process name, e.g. "cindy".
        route: The plan of the step's motion, or None when no motion part is left.
        route_movement: The movement index the route was planned for, or None.
        basis: What the route was planned from, or None.
        traj_time: Seconds the motion should take (the arm's limits may need longer).
        label: Names the request, e.g. the primary movement's id.
        ignore_built: Reads the requester's "Ignore built bars" now, so a plan made otherwise is refused as stale;
            by default not ticked.
        obstacles_collide: Reads the process plugin's "Rhino obstacles collide" now, likewise; by default ticked.
        transfer_as_free: Reads the process plugin's "Transfer workaround" now, likewise; by default ticked.
    """

    loaded: LoadedProcess
    action: str
    step: OperatorStep
    parts: tuple[PartPlan, ...]
    serial: str
    robot_name: str
    route: RouteResult | None
    route_movement: int | None
    basis: PlanBasis | None
    traj_time: float
    label: str
    ignore_built: Callable[[], bool] = lambda: False
    obstacles_collide: Callable[[], bool] = lambda: True
    transfer_as_free: Callable[[], bool] = lambda: True


@dataclass(frozen=True)
class ExecResult:
    """How a request ended.

    Attributes:
        status: DONE, DRY_RUN (every check passed, nothing sent), REFUSED (nothing sent), CANCELLED, FAILED, or
            HOLD_NOT_SENT (it ended and an arm refused its hold: stop it with the pendant or the e-stop).
        reason: One line saying what happened.
        done: The movement indices finished for real, in order, SERVO_INDEX for a servo that converged (or had
            nothing left to plan); never any in a dry run.
        end: The acting arms' measured joints after the last motion, by name, or None.
        base: The acting robot's base then, or None.
    """

    status: str
    reason: str = ""
    done: tuple[int, ...] = ()
    end: dict[str, float] | None = None
    base: Pose | None = None

    @property
    def ok(self) -> bool:
        """bool: Every part finished for real."""
        return self.status == DONE


@dataclass(frozen=True)
class Carried:
    """The MoCap-bar correction a transfer's servo leaves for its insert's plan (`servo.carried_insert_goals`).

    Attributes:
        loaded: The loaded process it was measured on, compared by identity.
        action: The action's id.
        insert: The insert's movement index in the action.
        serial: The acting robot.
        at: ROS time the servo ended, seconds.
    """

    loaded: LoadedProcess
    action: str
    insert: int
    serial: str
    at: float


class MovementExecContract(Protocol):
    """What the movement_exec plugin offers: type `ctx.require("movement_exec")` as this.

    ! Call the requests on the main thread. A request is refused (a resolved future) rather than raised.
    """

    @property
    def active(self) -> str:
        """str: "<robot> · <label>" while a request runs, else ""."""

    @property
    def prompt(self) -> Prompt | None:
        """Prompt | None: The question waiting for Confirm or Cancel."""

    @property
    def dry_run(self) -> bool:
        """bool: Whether requests run every check and send nothing."""

    @property
    def carried(self) -> Carried | None:
        """Carried | None: What the last transfer's bar servo left for its insert's plan.

        None once that transfer runs again, another action starts, or the process is loaded again.
        """

    def request_execute(self, request: ExecRequest) -> asyncio.Future[ExecResult]:
        """Run a step's parts with every robot leased until done; the future gives how it ended."""

    def request_confirm(self) -> None:
        """Confirm the open prompt."""

    def request_cancel(self) -> None:
        """Cancel the open prompt, else the running request (every arm is held)."""

    def request_dry_run(self, on: bool) -> None:
        """Turn the dry run on or off; refused while a request runs."""


def step_running(robots: Mapping[str, HuskyRobotInterface]) -> str:
    """Why nothing that changes the current step may change now: STEP_RUNNER has a robot's lease; else "".

    ! Main thread only, where leases change.

    Args:
        robots: The robots by serial (`ctx.world.robots`); a lease's holder reads "<plugin>: <label>".

    Returns:
        str: E.g. "a200-0806 is leased by movement_exec: exec B20_J_transfer: wait until it is done, or Cancel it".
    """
    for serial, robot in robots.items():
        lease = robot.lease_slot.lease
        if lease is not None and lease.holder.startswith(f"{STEP_RUNNER}: "):
            return f"{serial} is leased by {lease.holder}: wait until it is done, or Cancel it"
    return ""


def serial_of(loaded: LoadedProcess, action_id: str, robots: tuple[RobotConfig, ...]) -> str | None:
    """The serial of the configured robot an action is for, or None (by hand, or not configured)."""
    robot = loaded.process.actions[action_id].robot
    return find_robot_serial(robots, loaded.process.robots[robot].serial or "") if robot else None


def part_plan(loaded: LoadedProcess, action_id: str, index: int, robots: tuple[RobotConfig, ...],
              transfer_as_free: bool, during: PartPlan | None = None) -> PartPlan:
    """How the executor runs one movement of an action, or why it does not.

    - A tool must sit on a flange of the acting robot whose configured end effector is the process tool's kind; an
      insert's tightening tools must be scaffolding tools. A v1 tool, and a drive outside an insert, are refused.
    - An insert of a ground bar runs rigid, no tool driven; one with neither a ground bar nor a tighten kept is
      refused (W:5333-5338).
    - A scaffolding open runs only with the retreat it goes with (`during`), on the same arms.

    Args:
        loaded: The loaded process.
        action_id: The action.
        index: The movement's index in it.
        robots: The configured robots.
        transfer_as_free: The transfer workaround.
        during: The part whose motion this movement runs during (`OperatorStep.with_motion`), or None.

    Returns:
        PartPlan: The part.
    """
    process, action = loaded.process, loaded.process.actions[action_id]
    movement = action.movements[index]

    def part(routine: Routine | None, kind: str, text: str, tools: tuple[str, ...] = ()) -> PartPlan:
        return PartPlan(index, movement.id, routine, kind, tools, text)

    try:
        spec = classify(process, action, movement, transfer_as_free)
    except UnknownMovement as error:
        return part(None, REFUSED_PART, str(error))
    routine = spec.routine
    moving = tuple(arm.arm for arm in spec.arms)
    if routine in MOTION:
        return part(routine, MOTION_PART, f"move {', '.join(moving)} along the plan", moving)
    if routine in LATER:
        return part(routine, REFUSED_PART, f"{LATER[routine]} is not supported yet")
    if routine is Routine.OPERATOR:
        return part(routine, BY_HAND_PART, f"do {movement.label or movement.id} by hand")
    if routine is Routine.BASE_ONLY:
        return part(routine, BASE_PART, f"drive the base for {movement.label or movement.id} by hand")
    if routine is Routine.RETREAT_LINEAR:
        return part(routine, RETREAT_PART, f"retreat {', '.join(moving)}: open, compliant break-out, then rigid",
                    moving)
    if routine is Routine.INSERT and spec.drives.grounded:
        return part(routine, INSERT_PART, "rigid, ground bar, no tighten")
    tightened = [tool for tool, drive in spec.drives.kept.items() if drive == "tighten"]
    if routine is Routine.INSERT and not tightened:
        return part(routine, REFUSED_PART, "no mating arm: no joint screw tightens and the bar is not grounded "
                                           "(an export error)")
    if routine is not Routine.INSERT and (spec.drives.kept or not spec.grips):
        return part(routine, REFUSED_PART, "a tool movement with a drive, or without a grip, is not supported yet")
    serial = serial_of(loaded, action_id, robots)
    config = next((config for config in robots if config.serial == serial), None)
    if config is None:
        return part(routine, REFUSED_PART, f"{action.robot} is not a configured robot")
    flanges = {tool: flange for flange, tool in process.robots[action.robot].tools.items()}

    def arm_of(tool: str, kind: str) -> tuple[str, str]:
        """The arm whose flange carries `tool`, and why not if the robot has no `kind` configured there ("" if so)."""
        arm = flanges.get(tool, "")[:-len(FLANGE_SUFFIX)]
        fitted = next((each.end_effector for each in config.arms if each.name == arm), None)
        return arm, "" if fitted == kind else (f"{tool} is a {kind} on {arm or 'no flange'}, but {serial} has "
                                               f"{fitted or 'nothing'} there")

    if routine is Routine.INSERT:
        found = [arm_of(tool, "scaffolding_v3") for tool in tightened]
        refused = next((why for _arm, why in found if why), "")
        if refused:
            return part(routine, REFUSED_PART, refused)
        mating = tuple(arm for arm, _why in found)
        return part(routine, INSERT_PART, f"insert, the joint screw of {' + '.join(mating)} tightening", mating)
    arms, kinds = [], set()
    for tool in spec.grips:
        kind = process.tools[tool].kind
        arm, refused = arm_of(tool, kind)
        if refused:
            return part(routine, REFUSED_PART, refused)
        arms.append(arm)
        kinds.add(kind)
    grips = set(spec.grips.values())
    if len(kinds) != 1 or len(grips) != 1:
        return part(routine, REFUSED_PART, f"mixed tools or grips in one movement: {spec.grips}")
    kind, grip = kinds.pop(), grips.pop()
    if kind == "robotiq" and grip == "open":
        return part(routine, OPEN_PART, "Robotiq open", tuple(arms))
    if kind == "robotiq":
        return part(routine, HANDOFF_PART, "Robotiq close, the arm compliant (handoff)", tuple(arms))
    if kind == "scaffolding_v3" and grip == "closed":
        return part(routine, GRASP_PART, "scaffolding grasp", tuple(arms))
    if kind == "scaffolding_v3" and during is not None and during.kind == RETREAT_PART:
        if set(arms) != set(during.tools):
            return part(routine, REFUSED_PART, f"opens the tools on {', '.join(arms)}, but {during.movement} moves "
                                               f"{', '.join(during.tools)}")
        return part(routine, WITH_MOTION_PART, f"the gripper screws open in {during.movement}", tuple(arms))
    if kind == "scaffolding_v3":
        return part(routine, REFUSED_PART, "a scaffolding open outside a retreat is not supported yet")
    return part(routine, REFUSED_PART, f"a {kind} tool is not run by the executor")


def servo_part(loaded: LoadedProcess, action_id: str, primary: int, transfer_as_free: bool) -> PartPlan | None:
    """The servo after a step's primary movement, or None when it gets none (`servo_kind`).

    Args:
        loaded: The loaded process.
        action_id: The action.
        primary: The step's primary movement index.
        transfer_as_free: The transfer workaround.
    """
    process, action = loaded.process, loaded.process.actions[action_id]
    movement = action.movements[primary]
    try:
        spec = classify(process, action, movement, transfer_as_free)
    except UnknownMovement:
        return None
    kind = servo_kind(loaded, action_id, primary, transfer_as_free)
    if kind is None:
        return None
    arms = tuple(part.arm for part in spec.arms)
    return PartPlan(SERVO_INDEX, movement.id, spec.routine, SERVO_PART, arms,
                    f"servo {', '.join(arms)} onto {SERVO_TARGET[kind]}")


def step_parts(loaded: LoadedProcess, action_id: str, step: OperatorStep, robots: tuple[RobotConfig, ...],
               transfer_as_free: bool) -> tuple[PartPlan, ...]:
    """Every movement a step runs, as parts in order, then its servo where the primary gets one.

    One run during the motion is refused (not supported yet), unless it is the ungrasp its retreat runs
    (WITH_MOTION_PART).
    """
    movements = loaded.process.actions[action_id].movements
    primary = part_plan(loaded, action_id, step.primary, robots, transfer_as_free)

    def during(index: int) -> PartPlan:
        """A movement run during the primary's motion."""
        found = part_plan(loaded, action_id, index, robots, transfer_as_free, during=primary)
        if found.kind in (WITH_MOTION_PART, REFUSED_PART):
            return found
        return PartPlan(index, movements[index].id, None, REFUSED_PART, (),
                        "running during the motion is not supported yet")

    parts = (primary, *(during(index) for index in step.with_motion),
             *(part_plan(loaded, action_id, index, robots, transfer_as_free) for index in step.after_motion))
    servo = servo_part(loaded, action_id, step.primary, transfer_as_free)
    return parts if servo is None else (*parts, servo)


def current_step(view: RunView, transfer_as_free: bool) -> tuple[str, list[OperatorStep], int] | None:
    """The run's current action, its steps and the step holding its movement.

    Args:
        view: The run now.
        transfer_as_free: The transfer workaround (the process plugin's).

    Returns:
        tuple[str, list[OperatorStep], int] | None: None without a process, a current action, or steps.
    """
    if view.process is None or view.current is None or view.current[0] not in view.process.process.actions:
        return None
    action_id, movement = view.current
    try:
        steps = operator_steps(view.process.process, view.process.process.actions[action_id], transfer_as_free)
    except UnknownMovement:
        return None
    index = step_index_of(steps, movement)
    return None if index is None else (action_id, steps, index)


def current_primary(view: RunView, transfer_as_free: bool) -> tuple[str, int] | None:
    """The current (action id, primary movement index) a plan is checked against, or None (`current_step`)."""
    found = current_step(view, transfer_as_free)
    return None if found is None else (found[0], found[1][found[2]].primary)
