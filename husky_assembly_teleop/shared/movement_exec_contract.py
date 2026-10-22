"""
What the movement_exec plugin offers other plugins (`MovementExecContract`), its requests, and a step's parts.

Other plugins ask with an `ExecRequest`; `part_plan` says how a step's movements are run.

A part is a motion along a plan, a tool step (Robotiq open or close, a scaffolding grasp), or a step done by hand.
Routines run in later milestones are refused, naming the milestone (`LATER`).

Pure: any thread.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Protocol

from scaffolding_assembly_core.robot import FLANGE_SUFFIX

from ..config import find_robot_serial
from .assembly_planning.movement import Routine, UnknownMovement, classify
from .assembly_planning.steps import OperatorStep, operator_steps, step_index_of

if TYPE_CHECKING:
    from scaffolding_assembly_core.geometry import Pose

    from ..config import RobotConfig
    from .assembly_run_contract import RunView
    from .execution.basis import PlanBasis
    from .execution.operator import Prompt
    from .assembly_planning.route_result import RouteResult
    from .assembly_planning.process_steps import LoadedProcess

#: Seconds a motion takes by default, per routine (the old export's values); the operator may slow it, up to
#: MAX_TRAJ_TIME. The routines in it are the motions M5 runs.
DEFAULT_TRAJ_TIME = {Routine.FREE_TO_LOAD: 30.0, Routine.TRANSFER_FREE: 10.0, Routine.FREE_HOME: 10.0,
                     Routine.SUPPORT_APPROACH: 15.0, Routine.SUPPORT_LEAVE: 15.0, Routine.SUPPORT_LINEAR: 5.0}
MAX_TRAJ_TIME = 90.0
MOTION = frozenset(DEFAULT_TRAJ_TIME)
#: Routines not run yet, and the milestone that runs them.
LATER = {Routine.INSERT: "M6", Routine.RETREAT_LINEAR: "M6", Routine.TRANSFER_HELD: "M7"}

#: What a part does.
MOTION_PART, OPEN_PART, CLOSE_PART, GRASP_PART, BY_HAND_PART, BASE_PART, REFUSED_PART = (
    "motion", "robotiq_open", "robotiq_close", "grasp", "operator", "base", "refused")

#: How a request ended.
DONE, DRY_RUN, REFUSED, CANCELLED, FAILED, HOLD_NOT_SENT = (
    "done", "dry_run", "refused", "cancelled", "failed", "hold_not_sent")


@dataclass(frozen=True)
class PartPlan:
    """One movement of a step, as the executor runs it.

    Attributes:
        index: The movement's index in its action.
        movement: Its id.
        routine: Its routine; None if it can't be classified.
        kind: MOTION_PART, OPEN_PART, CLOSE_PART, GRASP_PART, BY_HAND_PART, BASE_PART or REFUSED_PART.
        tools: The arms whose tools (or, for a motion, which) it drives.
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


@dataclass(frozen=True)
class ExecResult:
    """How a request ended.

    Attributes:
        status: DONE, DRY_RUN (every check passed, nothing sent), REFUSED (nothing sent), CANCELLED, FAILED, or
            HOLD_NOT_SENT (it ended and an arm refused its hold: stop it with the pendant or the e-stop).
        reason: One line saying what happened.
        done: The movement indices finished for real, in order; never any in a dry run.
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

    def request_execute(self, request: ExecRequest) -> asyncio.Future[ExecResult]:
        """Run a step's parts, holding its robot until done; the future gives how it ended."""

    def request_confirm(self) -> None:
        """Confirm the open prompt."""

    def request_cancel(self) -> None:
        """Cancel the open prompt, else the running request (every arm is held)."""

    def request_dry_run(self, on: bool) -> None:
        """Turn the dry run on or off; refused while a request runs."""


def serial_of(loaded: LoadedProcess, action_id: str, robots: tuple[RobotConfig, ...]) -> str | None:
    """The serial of the configured robot an action is for, or None (by hand, or not configured)."""
    robot = loaded.process.actions[action_id].robot
    return find_robot_serial(robots, loaded.process.robots[robot].serial or "") if robot else None


def part_plan(loaded: LoadedProcess, action_id: str, index: int, robots: tuple[RobotConfig, ...],
              transfer_as_free: bool) -> PartPlan:
    """How the executor runs one movement of an action, or why it does not.

    A tool movement's tool must sit on a flange of the acting robot whose configured end effector is the process
    tool's kind; a scaffolding open (outside a retreat), a v1 tool and a drive are refused.

    Args:
        loaded: The loaded process.
        action_id: The action.
        index: The movement's index in it.
        robots: The configured robots.
        transfer_as_free: The transfer workaround.

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
    if routine in MOTION:
        arms = tuple(arm.arm for arm in spec.arms)
        return part(routine, MOTION_PART, f"move {', '.join(arms)} along the plan", arms)
    if routine in LATER:
        return part(routine, REFUSED_PART, f"{routine.value} runs in {LATER[routine]}")
    if routine is Routine.OPERATOR:
        return part(routine, BY_HAND_PART, f"do {movement.label or movement.id} by hand")
    if routine is Routine.BASE_ONLY:
        return part(routine, BASE_PART, f"drive the base for {movement.label or movement.id} by hand")
    if spec.drives.kept or not spec.grips:
        return part(routine, REFUSED_PART, "a tool movement with a drive, or without a grip, runs in M6")
    serial = serial_of(loaded, action_id, robots)
    config = next((config for config in robots if config.serial == serial), None)
    if config is None:
        return part(routine, REFUSED_PART, f"{action.robot} is not a configured robot")
    flanges = {tool: flange for flange, tool in process.robots[action.robot].tools.items()}
    arms, kinds = [], set()
    for tool in spec.grips:
        arm = flanges.get(tool, "")[:-len(FLANGE_SUFFIX)]
        mounted = next((each.end_effector for each in config.arms if each.name == arm), None)
        kind = process.tools[tool].kind
        if mounted != kind:
            return part(routine, REFUSED_PART, f"{tool} is a {kind} on {arm or 'no flange'}, but {serial} has "
                                               f"{mounted or 'nothing'} there")
        arms.append(arm)
        kinds.add(kind)
    grips = set(spec.grips.values())
    if len(kinds) != 1 or len(grips) != 1:
        return part(routine, REFUSED_PART, f"mixed tools or grips in one movement: {spec.grips}")
    kind, grip = kinds.pop(), grips.pop()
    if kind == "robotiq":
        return part(routine, OPEN_PART if grip == "open" else CLOSE_PART, f"Robotiq {grip}", tuple(arms))
    if kind == "scaffolding_v3" and grip == "closed":
        return part(routine, GRASP_PART, "scaffolding grasp", tuple(arms))
    if kind == "scaffolding_v3":
        return part(routine, REFUSED_PART, "a scaffolding open outside a retreat runs in M6")
    return part(routine, REFUSED_PART, f"a {kind} tool is not run by the executor")


def step_parts(loaded: LoadedProcess, action_id: str, step: OperatorStep, robots: tuple[RobotConfig, ...],
               transfer_as_free: bool) -> tuple[PartPlan, ...]:
    """Every movement a step runs, as parts in order; one run during the motion is refused (M6)."""
    movements = loaded.process.actions[action_id].movements
    return tuple(PartPlan(index, movements[index].id, None, REFUSED_PART, (), "runs during its motion in M6")
                 if index in step.with_motion else part_plan(loaded, action_id, index, robots, transfer_as_free)
                 for index in step.indices)


def current_step(view: RunView) -> tuple[str, list[OperatorStep], int] | None:
    """The run's current action, its steps (with the run's transfer workaround) and the step holding its movement.

    Returns:
        tuple[str, list[OperatorStep], int] | None: None without a process, a current action, or steps.
    """
    if view.process is None or view.current is None or view.current[0] not in view.process.process.actions:
        return None
    action_id, movement = view.current
    try:
        steps = operator_steps(view.process.process, view.process.process.actions[action_id],
                               view.settings.transfer_as_free)
    except UnknownMovement:
        return None
    index = step_index_of(steps, movement)
    return None if index is None else (action_id, steps, index)


def current_primary(view: RunView) -> tuple[str, int] | None:
    """The current (action id, primary movement index) a plan is checked against, or None."""
    found = current_step(view)
    return None if found is None else (found[0], found[1][found[2]].primary)
