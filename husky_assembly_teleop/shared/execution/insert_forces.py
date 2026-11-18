"""
The insert's target forces: the bar's weight taken off the arms, and a small push along each mating arm's line.

The weight is measured once, standing still at the insert's start before anything moves or the screws turn (the bar
clear of its joint): the sum of the arms' force readings in the world, where the squeeze between the arms cancels.
Only its vertical part is taken off, an equal share per arm.

- ! A sum with a sideways part over WEIGHT_MAX_SIDEWAYS, or a vertical one outside WEIGHT_RANGE, reads as a contact, not
  a weight: nothing is taken off, said loudly.
- The push goes along each mating arm's planned compliant line; it stops, and the arm's target freezes, once its joint
  screw is tight (`InsertForces.tight`).
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Collection, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ...robot_interface.refusal import newest_reason
from ...robot_interface.soft_stop import MotionRefused
from .stream import measured_tool0

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.robot import HuskyRobotInterface

#: Seconds the force readings are averaged over for the weight.
WEIGHT_SECONDS = 0.5
#: The oldest force reading the weight takes, seconds.
WRENCH_MAX_AGE = 0.3
#: A pushing arm's line further than this from its tool0 z, degrees, is warned: "along" and "across" in the stiffness
#: settings take tool0 z as the insert's direction.
LINE_OFF_TOOL_Z = 30.0
#: The most the readings' sum may point sideways, newtons, and still read as a weight (the sensors' offsets reach
#: about 2 N each).
WEIGHT_MAX_SIDEWAYS = 6.0
#: The vertical part of the sum that reads as a weight, newtons, up positive: a bar of up to 40 N, or a few newtons of
#: sensor offset upwards.
WEIGHT_RANGE = (-40.0, 3.0)
#: A planned compliant line shorter than this, metres, gives no direction to push along.
MIN_LINE = 1e-4
UP = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class InsertForceSettings:
    """The run's insert forces and stiffness (`RunSettings`).

    Attributes:
        cancel_weight: Take the bar's measured weight off the arms.
        push_n: The push along each mating arm's line, newtons; 0: none.
        stiffness: The compliance controller's stiffness along tool0's x, y and z, N/m.
    """

    cancel_weight: bool = True
    push_n: float = 4.0
    stiffness: tuple[float, float, float] = (1000.0, 1000.0, 500.0)

    def as_record(self) -> dict:
        """The settings as JSON values."""
        return {"cancel_weight": self.cancel_weight, "push_n": self.push_n, "stiffness": list(self.stiffness)}


@dataclass(frozen=True)
class BarWeight:
    """The arms' force readings summed in the world, standing still: the held bar's weight if nothing touches it.

    Attributes:
        total: The sum, world, newtons.
        samples: Ticks averaged.
        problem: Why it is not taken off ("" when it is).
    """

    total: np.ndarray
    samples: int
    problem: str

    @property
    def vertical(self) -> float:
        """float: The sum's vertical part, newtons, up positive (a weight reads negative)."""
        return float(self.total[2])

    @property
    def sideways(self) -> float:
        """float: The sum's horizontal part, newtons."""
        return float(np.linalg.norm(self.total[:2]))

    def as_record(self) -> dict:
        """The measurement as JSON values."""
        return {"total_N": np.round(self.total, 2).tolist(), "vertical_N": round(self.vertical, 2),
                "sideways_N": round(self.sideways, 2), "samples": self.samples, "problem": self.problem}


async def measure_weight(ctx: PluginContext, robot: HuskyRobotInterface, arms: Collection[str],
                         seconds: float = WEIGHT_SECONDS) -> BarWeight:
    """Average each arm's force reading in the world over `seconds`, then sum them: the bar's weight or a contact.

    ! The arms must stand still and hold the bar clear of everything. Each reading is taken in tool0, as the compliance
    controller takes it (`ft_sensor_ref_link`), and turned by the measured tool0; readings older than WRENCH_MAX_AGE
    are left out.
    """
    kept: dict[str, list[np.ndarray]] = {name: [] for name in arms}
    until = ctx.now() + seconds
    while True:
        for name in arms:
            state = robot.arms[name].state
            wrench, pose = state.wrench, measured_tool0(robot, name)
            fresh = state.wrench_update_time is not None and ctx.now() - state.wrench_update_time <= WRENCH_MAX_AGE
            if fresh and wrench is not None and pose is not None and np.all(np.isfinite(wrench[:3])):
                kept[name].append(Rotation.from_quat(pose.orientation).apply(np.asarray(wrench[:3], dtype=float)))
        if ctx.now() >= until:
            break
        await ctx.next_tick()
    missing = [name for name, readings in kept.items() if not readings]
    samples = min(len(readings) for readings in kept.values()) if kept else 0
    if missing:
        return BarWeight(np.zeros(3), samples, f"no fresh force reading or no TCP on {', '.join(missing)}")
    total = np.sum([np.mean(readings, axis=0) for readings in kept.values()], axis=0)
    weight = BarWeight(total, samples, "")
    if weight.sideways > WEIGHT_MAX_SIDEWAYS:
        problem = f"the readings sum {weight.sideways:.1f} N sideways (over {WEIGHT_MAX_SIDEWAYS:g}): a contact?"
    elif not WEIGHT_RANGE[0] <= weight.vertical <= WEIGHT_RANGE[1]:
        problem = (f"the readings sum {weight.vertical:+.1f} N vertically (a weight reads between "
                   f"{WEIGHT_RANGE[0]:g} and {WEIGHT_RANGE[1]:+g}): a contact?")
    else:
        return weight
    return BarWeight(total, samples, problem)


class InsertForces:
    """Each arm's target wrench in an insert: its share of the weight lifted, plus the push while its screw turns.

    Attributes:
        lift: Arm name -> the force taken off it, world, newtons (zero without a usable weight).
        push: Arm name -> its push, world, newtons; an arm whose screw is tight has none.
        tight_at: Arm name -> ROS time its screw was seen tight, or stalled when that ended the stream.
        line_from_tool_z: Pushing arm name -> its line's angle from its tool0 z at the first send, degrees.
    """

    def __init__(self, ctx: PluginContext, robot: HuskyRobotInterface, arms: Collection[str],
                 lines: Mapping[str, np.ndarray], settings: InsertForceSettings, weight: BarWeight | None,
                 label: str):
        """Work out the forces.

        Args:
            ctx: The plugin's context.
            robot: The robot.
            arms: Every arm of the insert.
            lines: Mating arm name -> its planned compliant line in the world (any length); only these push.
            settings: The run's settings.
            weight: The measurement, or None when cancelling is off (a ground bar, the setting).
            label: Who, for the log.
        """
        self._ctx, self._robot, self.label = ctx, robot, label
        self.arms, self.settings, self.weight = tuple(arms), settings, weight
        share = (-weight.vertical / len(self.arms)
                 if weight is not None and not weight.problem and self.arms else 0.0)
        self.lift = {name: share * UP for name in self.arms}
        self.push = {}
        for name, line in lines.items():
            length = float(np.linalg.norm(line))
            if name in self.arms and settings.push_n > 0.0 and length > MIN_LINE:
                self.push[name] = settings.push_n * np.asarray(line, dtype=float) / length
        #: The arms that pushed from the start.
        self.pushing = tuple(self.push)
        self.tight_at: dict[str, float] = {}
        self.line_from_tool_z: dict[str, float] = {}

    def send(self, names: Collection[str] | None = None) -> str:
        """Send each arm (`names`, or all) its wrench in tool0, turned by its measured tool0; why one refused, or ""."""
        for name in self.arms if names is None else names:
            pose = measured_tool0(self._robot, name)
            if pose is None:
                return f"{name}: no measured TCP to turn its target force into tool0"
            turn = Rotation.from_quat(pose.orientation)
            if name in self.push and name not in self.line_from_tool_z:
                along = turn.inv().apply(self.push[name])[2] / np.linalg.norm(self.push[name])
                # * Either way along z: the stiffness is the same both ways.
                angle = self.line_from_tool_z[name] = float(np.degrees(np.arccos(min(abs(along), 1.0))))
                if angle > LINE_OFF_TOOL_Z:
                    self._ctx.log_warn(f"{self.label}: {name}'s insert line is {angle:.0f}° off its tool0 z: the "
                                       f"stiffness along/across settings take tool0 z as the insert's direction")
            force = self.lift[name] + self.push.get(name, np.zeros(3))
            arm = self._robot.arms[name]
            try:
                sent = arm.send_target_wrench(turn.inv().apply(force))
            except MotionRefused as refused:
                return f"{name} refused its target force: {refused}"
            if not sent:
                return f"{name} refused its target force: {newest_reason(arm)}"
        return ""

    def tight(self, stall_times: Mapping[str, float]) -> Collection[str]:
        """The arms whose screw is tight; a newly tight one loses its push. For `stream`'s `held` arms.

        Args:
            stall_times: Arm name -> when its screw stalled (`StallGate.stall_times`).
        """
        for name in stall_times:
            if name in self.arms and name not in self.tight_at:
                self.tight_at[name] = self._ctx.now()
                if self.push.pop(name, None) is not None:
                    refused = self.send([name])
                    if refused:
                        self._ctx.log_warn(f"{self.label}: {name}'s push NOT taken off: {refused}")
                self._ctx.log_info(f"{self.label}: {name} screw TIGHT: its target holds where it was, no push")
        return self.tight_at.keys()

    def note_tight(self, stall_times: Mapping[str, float], since: float) -> None:
        """Record the screws that stalled as the stream ended, sending nothing: the last stall ends it before `tight`.

        Args:
            stall_times: Arm name -> seconds from `since` to its stall (`StallGate.stall_times`).
            since: ROS time the stall times count from (`StallGate.made`).
        """
        for name, stalled in stall_times.items():
            if name in self.arms and name not in self.tight_at:
                self.tight_at[name] = since + stalled
                self.push.pop(name, None)
                self._ctx.log_info(f"{self.label}: {name} screw TIGHT")

    def lift_only(self, names: Collection[str]) -> None:
        """Send `names` their lift alone, after a hold zeroed it (a cancel leaves them compliant); never raises.

        Each arm on its own: one that refuses (already back under joint control) must not cost the others their lift.
        """
        self.push.clear()
        refusals = []
        for name in names:
            try:
                refused = self.send([name])
            except Exception as error:  # noqa: BLE001 - in a finally: said, never raised
                refused = f"{name}: {error}"
            if refused:
                refusals.append(refused)
        if refusals:
            self._ctx.log_warn(f"{self.label}: the bar's weight NOT taken off again after the hold "
                               f"({'; '.join(refusals)}): it may sag under compliance")
        else:
            self._ctx.log_warn(f"{self.label}: left compliant with the bar's weight taken off: opening the jaws lifts "
                               f"each arm by its share / stiffness")

    def text(self) -> str:
        """What the arms get, for the log, e.g. "weight 9.6 N off (4.8 N per arm), push 4 N on left_ur_arm"."""
        if self.weight is None:
            weight = "weight not taken off (off in the run's settings)"
        elif self.weight.problem:
            weight = f"weight NOT taken off: {self.weight.problem}"
        else:
            lifted = -self.weight.vertical
            weight = f"weight {lifted:.1f} N off ({lifted / len(self.arms):.1f} N per arm)"
        push = (f"push {self.settings.push_n:g} N on {' + '.join(self.push)}" if self.push else "no push")
        return f"{weight}, {push}"

    def as_record(self) -> dict:
        """The forces as JSON values; the tight screws by ROS time."""
        return {"settings": self.settings.as_record(),
                "weight": None if self.weight is None else self.weight.as_record(),
                "lift_N": {name: np.round(force, 2).tolist() for name, force in self.lift.items()},
                "push_arms": list(self.pushing), "tight_at": dict(self.tight_at),
                "line_from_tool_z_deg": {name: round(angle, 1) for name, angle in self.line_from_tool_z.items()}}
