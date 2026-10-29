"""
Stream Cartesian targets to a robot's arms, one per tick, along lines fixed in the world: a compliant part's move.

Each tick every arm gets the point of its line at the stream's time `e`, plus an optional sideways offset (the
spiral's), turned into its base_link with the live mocap base; both arms in the same tick (W:3992-4024). Also here: the
split of a path into its rigid and compliant parts, the frames the targets are built from, and a part's record.

- The stream's clock advances at most MAX_CLOCK_STEP per tick: a late tick slows the move, never makes it jump.
- It stops before publishing: `keep_going` says no, its time is up, mocap lost the base (`pose_lost`), the base moved
  over BASE_DRIFT since it started, or an arm refused a target (the arm logs why).
- ! It only stops sending: on every ending, a cancel included, the caller holds the arms and switches them back.
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Collection, Iterable, Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import FLANGE_SUFFIX

from ...robot_interface.arm import cartesian_target_refusal
from ...robot_interface.end_effectors import ScaffoldingV3State
from ...robot_interface.mocap_state import pose_lost
from ...robot_interface.soft_stop import MotionRefused
from ..assembly_planning.route_result import pose_error
from .basis import STALE_BASE

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.robot import HuskyRobotInterface

#: The most the stream's clock advances in one tick, seconds.
MAX_CLOCK_STEP = 0.1
#: The base may move this far from where it was when the stream started, metres and radians, before it stops.
BASE_DRIFT = STALE_BASE
#: Every arm this close to its end, metres, has arrived (W:4697).
ARRIVAL = 0.002
#: Seconds a stream runs past its line and its wait before it gives up (W:128).
END_WAIT = 1.0
#: Seconds a compliant end is held waiting for every mating screw to stall (W:4701).
HOLD_FOR_STALL = 30.0
#: The URDF's tool0 and the measured TCP must agree this well, metres and radians, before a part goes compliant.
FK_TCP_TOL = (0.010, math.radians(2.0))
#: A leading part shorter than this, millimetres, is left out: one mode the whole way (W:4407).
MIN_LEADING_MM = 0.5
#: The shortest a compliant part or a retreat's rigid tail takes, seconds (W:5419, W:5485).
MIN_PART_SECONDS = 0.5
#: Samples a `Recorder` keeps, the last ones: about a minute at 20 Hz.
MAX_SAMPLES = 1300
#: An arm has settled against the force once its measured TCP moved less than this, metres ...
SETTLE_TOL = 0.0005
#: ... over this long, seconds: the move is this tick's TCP against the one a window ago.
SETTLE_WINDOW = 0.5
#: Seconds after an arm's last target before it is switched back anyway, settled or not.
SETTLE_CEILING = 3.0

#: How a compliant part ended: ok, then not. HOLD_NOT_SENT is a rigid part's (`send.HOLD_NOT_SENT`), passed through.
STALLED, RIGID_DONE, RETREATED, CLOSED = "STALLED", "RIGID_DONE", "RETREATED", "CLOSED"
NO_STALL, STIFF, CANCELLED, REFUSED, FAILED, HOLD_NOT_SENT, DRY_RUN = (
    "NO_STALL", "STIFF", "CANCELLED", "REFUSED", "FAILED", "HOLD_NOT_SENT", "DRY_RUN")


@dataclass(frozen=True)
class Segment:
    """Where each arm's tool0 goes along a streamed line, fixed in the world.

    Attributes:
        start: Arm name -> tool0 at the start, world.
        end: Arm name -> tool0 at the end, world.
        seconds: How long the line takes.
        wait: Seconds the end is held after that before the stream gives up; math.inf holds until `keep_going` says no.
    """

    start: Mapping[str, Pose]
    end: Mapping[str, Pose]
    seconds: float
    wait: float


@dataclass(frozen=True)
class PartOutcome:
    """How a compliant part (insert, retreat, handoff) ended.

    Attributes:
        status: STALLED, RIGID_DONE, RETREATED or CLOSED (ok); NO_STALL, STIFF, CANCELLED, REFUSED (nothing sent),
            FAILED, HOLD_NOT_SENT (a rigid part ended and an arm refused its hold: stop it with the pendant or the
            e-stop) or DRY_RUN.
        reason: One line saying what happened.
        seconds: From the first command to the end.
        record: Everything the run's record keeps about the part, as JSON values.
    """

    status: str
    reason: str = ""
    seconds: float = 0.0
    record: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        """bool: The part did what it was for."""
        return self.status in (STALLED, RIGID_DONE, RETREATED, CLOSED)


# --- --- --- --- --- SPLIT --- --- --- --- ---

def split_rows(rows: Sequence[Sequence[float]], points: Callable[[np.ndarray], Sequence[Sequence[float]]],
               split_mm: float, from_start: bool = False) -> tuple[np.ndarray | None, np.ndarray | None, float, float]:
    """Cut a path where every arm's tool0 is `split_mm` from its end, or from its start (W:4446-4527).

    The distance of a row is the largest over the arms; the cut is interpolated between the two rows around it, and
    both parts hold the cut row, so they join.

    Args:
        rows: The path (N x joints).
        points: A row -> each moving arm's tool0 position, metres, in any frame fixed to its arm.
        split_mm: Where to cut, millimetres from the end (an insert: the last `split_mm` are compliant), or from the
            start with `from_start` (a retreat: the first `split_mm` are).
        from_start: Measure `split_mm` from the start.

    Returns:
        tuple: (leading, trailing, achieved_mm, total_mm): the parts before and after the cut, the cut's distance from
            the end and the path's. Leading is None when it would be under MIN_LEADING_MM (one mode the whole way),
            and for fewer than 2 rows (then both distances are 0); trailing is None when the cut is at the end.
    """
    rows = np.asarray(rows, dtype=float)
    if len(rows) < 2:
        return None, rows, 0.0, 0.0
    at = [np.asarray(points(row), dtype=float) for row in rows]

    def to_end(here: np.ndarray) -> float:
        """The largest distance of an arm's tool0 from its end, millimetres."""
        return float(np.max(np.linalg.norm(at[-1] - here, axis=-1))) * 1e3

    gaps = [to_end(here) for here in at]
    total = gaps[0]
    goal = total - split_mm if from_start else split_mm
    if goal >= total - MIN_LEADING_MM:
        return None, rows, total, total
    if goal <= 0.0:
        return rows, None, 0.0, total
    index = next(i for i, gap in enumerate(gaps) if gap <= goal)
    span = gaps[index - 1] - gaps[index]
    fraction = 0.0 if span <= 1e-9 else min(max((gaps[index - 1] - goal) / span, 0.0), 1.0)
    cut = rows[index - 1] + fraction * (rows[index] - rows[index - 1])
    return (np.vstack([rows[:index], cut]), np.vstack([cut, rows[index:]]),
            to_end(np.asarray(points(cut), dtype=float)), total)


def split_seconds(traj_time: float, achieved_mm: float, total_mm: float,
                  from_start: bool = False) -> tuple[float, float]:
    """The leading and trailing parts' seconds: the path's time shared by distance (W:5416-5419, W:5484-5486).

    The compliant part (an insert's trailing one, a retreat's leading one) and a retreat's rigid tail take at least
    MIN_PART_SECONDS.

    Args:
        traj_time: Seconds for the whole path.
        achieved_mm: The cut's distance from the end (`split_rows`).
        total_mm: The path's.
        from_start: A retreat's split.

    Returns:
        tuple[float, float]: Seconds before and after the cut.
    """
    leading = traj_time * (total_mm - achieved_mm) / total_mm if total_mm > 0.0 else 0.0
    trailing = max(traj_time - leading, MIN_PART_SECONDS)
    return (max(leading, MIN_PART_SECONDS) if from_start else leading), trailing


# --- --- --- --- --- FRAMES --- --- --- --- ---

def tool0_fk(urdf: Path, arm: str, joints: Mapping[str, float]) -> Pose:
    """An arm's tool0 in its base_link (the controller's frame) by the URDF's kinematics.

    Args:
        urdf: The robot's URDF.
        arm: The arm's name, e.g. "left_ur_arm".
        joints: Joint values by URDF name; missing ones count as 0.
    """
    return compose(invert(link_pose(urdf, Pose(), joints, f"{arm}_base_link")),
                   link_pose(urdf, Pose(), joints, f"{arm}{FLANGE_SUFFIX}"))


def tool0_world_path(urdf: Path, base: Pose, names: Sequence[str], rows: Sequence[Sequence[float]],
                     arms: Iterable[str]) -> dict[str, list[Pose]]:
    """Each arm's tool0 in the world at every row of a joint path, the robot's base at `base`.

    Args:
        urdf: The robot's URDF.
        base: The base pose the path was planned at.
        names: The path's joint names.
        rows: The path's joint rows.
        arms: The arms wanted, e.g. "left_ur_arm".

    Returns:
        dict[str, list[Pose]]: Arm name -> its tool0 at each row.
    """
    arms = tuple(arms)
    paths: dict[str, list[Pose]] = {arm: [] for arm in arms}
    for row in rows:
        joints = dict(zip(names, (float(value) for value in row)))
        for arm in arms:
            paths[arm].append(link_pose(urdf, base, joints, f"{arm}{FLANGE_SUFFIX}"))
    return paths


def arm_base_in_world(robot: HuskyRobotInterface, arm: str) -> Pose | None:
    """An arm's base_link in the world from the live mocap base; None before the first mocap fix."""
    husky = robot.husky_in_world()
    return None if husky is None else compose(husky, robot.arms[arm].base_in_husky)


def measured_tool0(robot: HuskyRobotInterface, arm: str) -> Pose | None:
    """An arm's measured TCP (tool0) in the world; None without a TCP or a mocap fix."""
    base, state = arm_base_in_world(robot, arm), robot.arms[arm].state
    if base is None or state.tcp_position is None:
        return None
    return compose(base, Pose.from_arrays(state.tcp_position, state.tcp_orientation))


def fk_tcp_gap(robot: HuskyRobotInterface, arm: str) -> tuple[float, float] | None:
    """How far the measured TCP is from tool0 by the URDF's kinematics of the live joints, metres and radians.

    Over FK_TCP_TOL, the frames the targets are built in are wrong: no compliant part may start.

    Returns:
        tuple[float, float] | None: Distance and angle; None without joints or a TCP.
    """
    state = robot.arms[arm].state
    if robot.arms[arm].joint_vector() is None or state.tcp_position is None:
        return None
    return pose_error(tool0_fk(robot.config.model.urdf, arm, state.joint_positions),
                      Pose.from_arrays(state.tcp_position, state.tcp_orientation))


def gap_record(gap: tuple[float, float] | None) -> dict | None:
    """An arm's `fk_tcp_gap` as a part's record keeps it: {"mm", "deg"}, or None when it could not be measured."""
    return None if gap is None else {"mm": gap[0] * 1e3, "deg": math.degrees(gap[1])}


# --- --- --- --- --- TARGETS --- --- --- --- ---

def zero_wrench(arms: Collection[ArmInterface]) -> str:
    """Send every arm a zero target wrench (robot_control may have left one); why an arm refused it, or ""."""
    for arm in arms:
        try:
            sent = arm.send_target_wrench((0.0, 0.0, 0.0))
        except MotionRefused as refused:
            return f"{arm.config.name} refused the zero target wrench: {refused}"
        if not sent:
            return f"{arm.config.name} refused the zero target wrench (its log says why)"
    return ""


def line_target(start: Pose, end: Pose, seconds: float, elapsed: float,
                lateral: Sequence[float] | None = None) -> Pose:
    """The point of a line at `elapsed`: linear in time, the turn by the shortest slerp, `lateral` added (W:4141-4156).

    Args:
        start: Where the line starts.
        end: Where it ends; held once `seconds` passed.
        seconds: How long the line takes; 0 or less: at the end at once.
        elapsed: Seconds since it started.
        lateral: An offset added to the position, metres, e.g. the spiral's; None for none.
    """
    fraction = 1.0 if seconds <= 0.0 else min(elapsed / seconds, 1.0)
    position = np.add(start.position, fraction * np.subtract(end.position, start.position))
    if lateral is not None:
        position = position + np.asarray(lateral, dtype=float)
    first = Rotation.from_quat(start.orientation)
    turn = (first.inv() * Rotation.from_quat(end.orientation)).as_rotvec()
    return Pose.from_arrays(position, (first * Rotation.from_rotvec(fraction * turn)).as_quat())


def plan_stream(segment: Segment, dt: float,
                lateral: Callable[[float], Sequence[float]] | None = None) -> list[dict[str, Pose]]:
    """Every tick's world targets along a segment's line, offline (a dry run, a test); the wait is left out.

    Args:
        segment: The line.
        dt: Seconds per tick.
        lateral: Elapsed seconds -> the offset added to every arm's target, or None.

    Returns:
        list[dict[str, Pose]]: Per tick, from 0 until the end is reached: arm name -> its target; one tick for a line
            of 0 s.
    """
    ticks = max(math.ceil(round(segment.seconds / dt, 9)), 0)
    return [{name: line_target(segment.start[name], segment.end[name], segment.seconds, k * dt,
                               None if lateral is None else lateral(k * dt))
             for name in segment.start} for k in range(ticks + 1)]


def target_problem(previous: Pose | None, target: Pose, tcp: Pose) -> str:
    """Why the arm would refuse a Cartesian target, by its own checks that need no clock (`cartesian_target_refusal`).

    All poses in the arm's base_link.

    Args:
        previous: The target sent just before, or None for the first one.
        target: The target.
        tcp: Where the TCP is when it goes out.

    Returns:
        str: The arm's reason, or "".
    """
    def arrays(pose: Pose) -> tuple[np.ndarray, np.ndarray]:
        """A pose as (position, quaternion) arrays."""
        return np.asarray(pose.position), np.asarray(pose.orientation)

    return cartesian_target_refusal(*arrays(target), arrays(tcp), None if previous is None else arrays(previous)) or ""


def progress(robot: HuskyRobotInterface, segment: Segment) -> dict[str, dict[str, float] | None]:
    """How far each arm's measured TCP got along its line, millimetres (W:4891-4915).

    Returns:
        dict: Arm name -> {"travel_mm": start to end, "achieved_mm": start to the TCP, "remaining_mm": the TCP to the
            end}; None for an arm without a measured TCP.
    """
    result = {}
    for name in segment.start:
        here = measured_tool0(robot, name)
        if here is None:
            result[name] = None
            continue
        start, end = np.asarray(segment.start[name].position), np.asarray(segment.end[name].position)
        result[name] = {"travel_mm": float(np.linalg.norm(end - start)) * 1e3,
                        "achieved_mm": float(np.linalg.norm(np.subtract(here.position, start))) * 1e3,
                        "remaining_mm": float(np.linalg.norm(end - np.asarray(here.position))) * 1e3}
    return result


# --- --- --- --- --- RECORD --- --- --- --- ---

class Recorder:
    """One sample per tick of what every arm of a robot was sent and measured, the peak force per arm, the base drift.

    Attributes:
        phase: The phase the next samples belong to, e.g. "compliant"; the caller sets it.
        samples: The last MAX_SAMPLES samples, as JSON values.
        peak_force: Arm name -> the largest |F| measured, newtons, raw (the tool's and the bar's weight included).
        base_drift_max_mm: The furthest the base moved from where it stood at the first sample, millimetres.
    """

    def __init__(self, robot: HuskyRobotInterface, started: float, phase: str = ""):
        """Record `robot`'s arms; sample times count from ROS time `started`."""
        self.robot, self.started, self.phase = robot, started, phase
        self.samples: deque[dict] = deque(maxlen=MAX_SAMPLES)
        self.peak_force: dict[str, float] = {}
        self.base_drift_max_mm = 0.0
        self._base_start: Pose | None = None
        self._ticks = 0

    def tick(self, now: float) -> None:
        """Keep a sample of every arm at ROS time `now`: its last target, its TCP, its wrench and its tool's motors."""
        base = self.robot.husky_in_world()
        if base is not None:
            self._base_start = base if self._base_start is None else self._base_start
            self.base_drift_max_mm = max(self.base_drift_max_mm, pose_error(base, self._base_start)[0] * 1e3)
        arms, motors = {}, {}
        for name, arm in self.robot.arms.items():
            state = arm.state
            target = None if state.sent_target_position is None else [
                _rounded(state.sent_target_position), _rounded(state.sent_target_orientation)]
            arms[name] = {"target": target, "tcp": None if state.tcp_position is None else _rounded(state.tcp_position),
                          "wrench": None if state.wrench is None else _rounded(state.wrench)}
            if state.wrench is not None:
                self.peak_force[name] = max(self.peak_force.get(name, 0.0), float(np.linalg.norm(state.wrench[:3])))
            if isinstance(state.end_effector, ScaffoldingV3State):
                motors[name] = [state.end_effector.gripper_motor, state.end_effector.joint_motor]
        self.samples.append({"t": round(now - self.started, 3), "phase": self.phase, "arms": arms, "motors": motors})
        self._ticks += 1

    def as_record(self) -> dict:
        """The peak forces, the base drift and the samples as JSON values; `stream_dropped` counts samples not kept."""
        return {"peak_force_N": dict(self.peak_force), "base_drift_max_mm": self.base_drift_max_mm,
                "stream": list(self.samples), "stream_dropped": self._ticks - len(self.samples)}


def _rounded(values) -> list[float]:
    """Values rounded to a micrometre (or a millionth), for the record."""
    return np.round(np.asarray(values, dtype=float), 6).tolist()


# --- --- --- --- --- SETTLE --- --- --- --- ---

class Settling:
    """Whether every watched arm's measured TCP stopped moving (settled against the force) before the ceiling passed.

    Attributes:
        moved_mm: The furthest any arm's TCP moved against its sample a window earlier, millimetres; None while that
            sample is not there yet, or for an arm without a TCP (it never settles).
        seconds: Since the first tick.
    """

    def __init__(self, robot: HuskyRobotInterface, arms: Collection[str], tol: float = SETTLE_TOL,
                 window: float = SETTLE_WINDOW, ceiling: float = SETTLE_CEILING):
        """Watch `arms` of `robot`: settled once each moved under `tol` metres over `window` seconds."""
        self.robot, self.tol, self.window, self.ceiling = robot, tol, window, ceiling
        self.moved_mm: float | None = None
        self.seconds = 0.0
        self._first: float | None = None
        #: Arm name -> (ROS time, TCP position) samples: the newest at least a window old, and all since.
        self._kept: dict[str, deque[tuple[float, np.ndarray]]] = {name: deque() for name in arms}

    def tick(self, now: float) -> bool | None:
        """Keep every arm's TCP at ROS time `now`: True once all settled, False at the ceiling, None before either."""
        self._first = now if self._first is None else self._first
        self.seconds = now - self._first
        moved: list[float | None] = []
        for name, kept in self._kept.items():
            tcp = self.robot.arms[name].state.tcp_position
            if tcp is None:
                moved.append(None)
                continue
            kept.append((now, np.asarray(tcp, dtype=float)))
            # * The pair compared spans a whole window whatever the tick times: ROS ticks are not on a grid.
            while len(kept) > 1 and kept[1][0] <= now - self.window:
                kept.popleft()
            full = now - kept[0][0] >= self.window
            moved.append(float(np.linalg.norm(kept[-1][1] - kept[0][1])) if full else None)
        worst = None if None in moved else max(moved)
        self.moved_mm = None if worst is None else worst * 1e3
        if worst is not None and worst < self.tol:
            return True
        return False if self.seconds >= self.ceiling else None


# --- --- --- --- --- STREAM --- --- --- --- ---

async def stream(ctx: PluginContext, robot: HuskyRobotInterface, segment: Segment, *,
                 keep_going: Callable[[float], bool], lateral: Callable[[float], Sequence[float]] | None = None,
                 recorder: Recorder) -> str:
    """Send every arm of `segment` its line's target each tick, until something says stop; checked before each send.

    Each tick: the clock advances by at most MAX_CLOCK_STEP; then the stop tests; then each arm's target, the line's
    world point turned into its base_link with this tick's base, `arm.from_husky(*robot.from_world(G))`.

    Args:
        ctx: The plugin's context, for its ticks.
        robot: The robot; every arm in `segment` is one of its arms, under the compliance controller.
        segment: The lines, fixed in the world.
        keep_going: Elapsed seconds -> whether to go on; asked each tick before sending, e.g. not every screw stalled.
        lateral: Elapsed seconds -> an offset added to every arm's target, metres (the spiral), or None.
        recorder: Gets a sample after each tick's targets.

    Returns:
        str: "ended" (`keep_going` said no), "time" (over seconds + wait + END_WAIT), "base pose lost: …", "base
            moved …", or "refused: <arm>…"; nothing more is sent after any of them.
    """
    start, elapsed, last = None, 0.0, ctx.now()
    while True:
        now = ctx.now()
        elapsed += min(now - last, MAX_CLOCK_STEP)
        last = now
        if not keep_going(elapsed):
            return "ended"
        if elapsed > segment.seconds + segment.wait + END_WAIT:
            return "time"
        if lost := pose_lost(robot.base.mocap_id, robot.base.state, now):
            return f"base pose lost: {lost}"
        base = robot.husky_in_world()
        start = base if start is None else start
        distance, turn = pose_error(base, start)
        if distance > BASE_DRIFT[0] or turn > BASE_DRIFT[1]:
            return (f"base moved {distance * 1e3:.1f} mm, {math.degrees(turn):.2f}° since the stream started "
                    f"(stops beyond {BASE_DRIFT[0] * 1e3:.0f} mm or {math.degrees(BASE_DRIFT[1]):.1f}°)")
        offset = None if lateral is None else lateral(elapsed)
        for name in segment.start:
            arm = robot.arms[name]
            goal = line_target(segment.start[name], segment.end[name], segment.seconds, elapsed, offset)
            try:
                sent = arm.send_cartesian_target(*arm.from_husky(*robot.from_world(goal.position, goal.orientation)))
            except MotionRefused as refused:
                return f"refused: {name}: {refused}"
            if not sent:
                return f"refused: {name} (its log says why)"
        recorder.tick(now)
        await ctx.next_tick()
