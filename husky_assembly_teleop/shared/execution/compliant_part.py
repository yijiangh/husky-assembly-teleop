"""
What the compliant parts (insert, retreat, handoff) do alike: arms, checks, record, compliance in and out, outcome.

Also the dry run's check of every streamed target. ! The async helpers: main thread, from a plugin task.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Callable, Collection, Iterable, Mapping, Sequence

from scaffolding_assembly_core.geometry import Pose

from ...robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER
from ...world.tcp import tool0_in_base_link
from ..assembly_planning.joints import arm_joint_names
from .controllers import hold_compliant, leave_compliance, switch_arms
from .precheck import part_problem
from .stream import (MAX_CLOCK_STEP, PartOutcome, Recorder, Segment, fk_tcp_gap, gap_record, plan_stream,
                     target_problem, zero_wrench)

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.end_effectors import ScaffoldingV3
    from ...robot_interface.robot import HuskyRobotInterface

#: A phase's record: (name, ROS time it started, how it ended) -> kept with its times from the part's start.
Phase = Callable[[str, float, str], None]


def moving_arms(robot: HuskyRobotInterface, names: Collection[str]) -> tuple[str, ...]:
    """The robot's arms, in its order, whose every joint is a column of a path with joint names `names`."""
    return tuple(arm for arm in robot.arms if set(arm_joint_names(arm)) <= set(names))


def tool0_by_arm(urdf: Path, arms: Iterable[str], names: Sequence[str], row: Sequence[float]) -> dict[str, Pose]:
    """Each arm's tool0 in its base_link at one row of a path with joint names `names`."""
    joints = dict(zip(names, row))
    return {arm: tool0_in_base_link(urdf, arm, joints) for arm in arms}


def check_part(ctx: PluginContext, robot: HuskyRobotInterface, arms: Collection[str],
               tools: Mapping[str, ScaffoldingV3], record: dict) -> str:
    """Keep each arm's `fk_tcp_gap` in `record`, then say why the part may not start (`part_problem`), or ""."""
    record["fk_tcp_gap"] = {name: gap_record(fk_tcp_gap(robot, name)) for name in arms}
    return part_problem(ctx, robot, arms, tools)


def started_part(ctx: PluginContext, robot: HuskyRobotInterface,
                 recorder: Recorder | None) -> tuple[float, Recorder]:
    """The part's start (ROS time now) and its recorder: `recorder`, or a new one counting from now."""
    started = ctx.now()
    return started, Recorder(robot, started) if recorder is None else recorder


def phase_keeper(ctx: PluginContext, started: float, phases: list[dict]) -> Phase:
    """A function that keeps a phase in `phases`: its name, its times from `started`, how it ended."""
    def phase(name: str, since: float, end: str) -> None:
        """Keep a phase in the record, its times from the part's start."""
        phases.append({"name": name, "t0": round(since - started, 3), "t1": round(ctx.now() - started, 3),
                       "end": end})

    return phase


async def enter_compliance(ctx: PluginContext, arms: Collection[ArmInterface], record: dict, *,
                           phase: Phase | None = None, zero: bool = True) -> str:
    """Switch `arms` to the compliance controller, then send each a zero target wrench; why either failed, or "".

    `record["switch"]` gets the switch's seconds and result; `phase` (if given) a "switch" phase.

    Args:
        ctx: The plugin's context.
        arms: The part's arms.
        record: The part's record.
        phase: Keeps the switch as a phase, or None.
        zero: Send the zero target wrench (`zero_wrench`); False when the caller sends its own after a first target.
    """
    since = ctx.now()
    switched = await switch_arms(ctx, arms, CARTESIAN_COMPLIANCE_CONTROLLER)
    record.setdefault("switch", {}).update(to_s=switched.seconds, ok=switched.ok)
    if phase is not None:
        phase("switch", since, switched.reason)
    if not switched.ok:
        return switched.reason
    return zero_wrench(arms) if zero else ""


async def stop_then_leave(ctx: PluginContext, stop: Callable[[], None], arms: Collection[ArmInterface], *,
                          label: str, record: dict | None = None, cancelled: bool = False) -> bool:
    """Stop the part's tools, then bring `arms` back under joint control (`leave_compliance`), even if the stop raised.

    ! Call it in the part's `finally`. No arms: only the stop. `cancelled` (Cancel, Stop all, the plugin closing):
    only the stop and a hold, sent and forgotten (`hold_compliant`); the arms stay compliant.

    Args:
        ctx: The plugin's context.
        stop: Stops every tool the part drives.
        arms: The arms to bring back.
        label: Who leaves, for the log.
        record: The part's record: its "switch" gets the way back's seconds and result; None: not kept.

    Returns:
        bool: Whether every arm is confirmed back (`leave_compliance`); True without arms, False when cancelled.
    """
    try:
        stop()
    finally:
        restored = True
        if arms and cancelled:
            hold_compliant(ctx, arms, label=label)
            restored = False
        elif arms:
            since = ctx.now()
            restored = await leave_compliance(ctx, arms, label=label)
            if record is not None:
                record.setdefault("switch", {}).update(back_s=ctx.now() - since, back_ok=restored)
    return restored


def first_refused_target(robot: HuskyRobotInterface, lines: Sequence[tuple[Segment, Callable | None]],
                         tcp: Mapping[str, Pose] | None = None) -> tuple[str, float, str] | None:
    """Check every target of a dry run's lines as each arm would, a stream's largest step apart, the TCP following.

    Each line's targets are taken MAX_CLOCK_STEP apart (`plan_stream`) and turned into the arm's base_link with the
    live base, as `stream` sends them; each is checked against the target before it, where the TCP will be.

    Args:
        robot: The robot.
        lines: (segment, lateral offset or None) in the order they would run.
        tcp: Arm name -> its TCP before the first target, in its base_link; None: at the first target.

    Returns:
        tuple[str, float, str] | None: The first refusal as (arm, seconds into its line, why), or None.
    """
    previous: dict[str, Pose] = {}
    for segment, lateral in lines:
        for k, goals in enumerate(plan_stream(segment, MAX_CLOCK_STEP, lateral)):
            for name, goal in goals.items():
                in_husky = robot.from_world(goal.position, goal.orientation)
                target = Pose.from_arrays(*robot.arms[name].from_husky(*in_husky))
                start = (tcp or {}).get(name, target)
                problem = target_problem(previous.get(name), target, previous.get(name, start))
                if problem:
                    return name, k * MAX_CLOCK_STEP, problem
                previous[name] = target
    return None


def part_outcome(status: str, reason: str, seconds: float, record: dict) -> PartOutcome:
    """The outcome, its record starting with the status, the reason and the seconds."""
    return PartOutcome(status, reason, seconds, {"status": status, "reason": reason, "seconds": seconds, **record})
