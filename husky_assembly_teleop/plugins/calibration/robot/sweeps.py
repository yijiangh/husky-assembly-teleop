"""
The calibration sweeps: which joint each take turns, from where, and its waypoints (`data/calibration/sweeps.yaml`).

The sweeps are predefined, taken from the old monitor; the file's order (from `short_moves`) only numbers them. At
run time each take is run as `nearest_take` turns it from where the arm stands (whole turns, direction), and the next
take is the first of `shortest_order`. Plain data; any thread.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass, replace
from itertools import permutations
from pathlib import Path

import numpy as np
import yaml

from scaffolding_assembly_core.urdf import UR_JOINT_NAMES, load_urdf

#: The sweeps file, below the data directory.
SWEEPS_FILE = Path("calibration") / "sweeps.yaml"
#: The calibration marker's Motive rigid body: Setup's default tool body for every arm, which the operator may change.
DEFAULT_TOOL_BODY = 1866
#: Fewest waypoints a take may have: a circle needs three, a good one many more.
MIN_STEPS = 8
#: A whole turn, radians: a joint at `a` and at `a ± TURN` stands the same.
TURN = 2 * np.pi
#: Farthest `short_moves` shifts a held joint from the stow pose, radians: further winds the tool's cables.
UNWOUND = 1.25 * np.pi


@dataclass(frozen=True)
class Sweep:
    """One take: one joint turned in equal steps, the others held.

    Attributes:
        joint: The swept joint, 0 (shoulder pan) or 1 (shoulder lift).
        joints: The six joint angles held, radians, UR driver order; the swept joint's entry is ignored.
        start: The swept joint's first angle, radians.
        end: Its last angle, radians.
        steps: Number of waypoints, both ends included.
    """

    joint: int
    joints: tuple[float, ...]
    start: float
    end: float
    steps: int

    def waypoints(self) -> np.ndarray:
        """Return the (steps, 6) joint angles of every waypoint, radians."""
        waypoints = np.tile(np.asarray(self.joints, dtype=float), (self.steps, 1))
        waypoints[:, self.joint] = np.linspace(self.start, self.end, self.steps)
        return waypoints

    def ends(self) -> tuple[np.ndarray, np.ndarray]:
        """Return the first and last waypoints' six joint angles, radians."""
        first, last = np.array(self.joints, dtype=float), np.array(self.joints, dtype=float)
        first[self.joint], last[self.joint] = self.start, self.end
        return first, last


@dataclass(frozen=True)
class ArmSweeps:
    """One arm's calibration sweeps.

    Attributes:
        sweeps: The takes, J0 ones first.
    """

    sweeps: tuple[Sweep, ...]


def load_sweeps(data_directory: Path, digits: str, arm: str) -> ArmSweeps:
    """Read one arm's sweeps.

    Args:
        data_directory: The monitor's data directory (`MonitorConfig.data_directory`).
        digits: The robot's four serial digits, e.g. "0806".
        arm: The arm, e.g. "left_ur_arm".

    Raises:
        KeyError: If the file has no sweeps for this arm.
    """
    robots = yaml.safe_load((data_directory / SWEEPS_FILE).read_text())
    entry = robots.get(digits, {}).get(arm)
    if entry is None:
        raise KeyError(f"{SWEEPS_FILE} has no sweeps for {digits} {arm}")
    sweeps = tuple(Sweep(int(take["joint"]), tuple(float(v) for v in take["joints"]), float(take["from"]),
                         float(take["to"]), int(take["steps"])) for take in entry["takes"])
    return ArmSweeps(tuple(sorted(sweeps, key=lambda sweep: sweep.joint)))


def load_validation_poses(data_directory: Path, digits: str, arm: str) -> list[np.ndarray]:
    """One arm's validation poses, radians in UR driver order; empty if the sweeps file lists none."""
    robots = yaml.safe_load((data_directory / SWEEPS_FILE).read_text())
    entry = robots.get(digits, {}).get(arm) or {}
    return [np.array(pose, dtype=float) for pose in entry.get("validation", [])]


def sweep_problems(sweeps: ArmSweeps, urdf: Path, arm: str) -> list[str]:
    """Why these sweeps cannot calibrate the arm, one line each; empty if they can.

    Checks what the math needs (two takes per joint, enough waypoints, J0 at 0 for J1, no repeated take) and that
    every waypoint is within the URDF's joint limits. Collisions are checked elsewhere.
    """
    problems = []
    for joint in (0, 1):
        count = sum(sweep.joint == joint for sweep in sweeps.sweeps)
        if count < 2:
            problems.append(f"J{joint}: {count} take(s), at least 2 needed (different heights or wrists)")
    lower, upper = joint_limits(urdf, arm)
    seen = set()
    for index, sweep in enumerate(sweeps.sweeps):
        label = f"take {index + 1} (J{sweep.joint})"
        if sweep.joint not in (0, 1):
            problems.append(f"{label}: sweeps joint {sweep.joint}; only 0 and 1 calibrate the base")
            continue
        if sweep.steps < MIN_STEPS:
            problems.append(f"{label}: {sweep.steps} waypoints, at least {MIN_STEPS} needed")
        if sweep.joint == 1 and abs(sweep.joints[0]) > 1e-6:
            problems.append(f"{label}: J0 is {sweep.joints[0]:g} rad; J1 takes need J0 at 0 (it sets the base yaw)")
        key = (sweep.joint, tuple(v for i, v in enumerate(sweep.joints) if i != sweep.joint))
        if key in seen:
            problems.append(f"{label}: holds the same joints as an earlier take; it adds no new circle")
        seen.add(key)
        waypoints = sweep.waypoints()
        for column, name in enumerate(UR_JOINT_NAMES):
            low, high = waypoints[:, column].min(), waypoints[:, column].max()
            if low < lower[column] or high > upper[column]:
                problems.append(f"{label}: {name} goes {low:.2f}..{high:.2f} rad, outside its limits "
                                f"{lower[column]:.2f}..{upper[column]:.2f}")
    return problems


def named(arm: str, values) -> dict[str, float]:
    """An arm's six joint values by URDF joint name; `values` in UR_JOINT_NAMES order, as arm plans are."""
    return {f"{arm}_{name}": float(value) for name, value in zip(UR_JOINT_NAMES, values)}


def joint_limits(urdf: Path, arm: str) -> tuple[np.ndarray, np.ndarray]:
    """Return the arm's lower and upper joint limits from its URDF, radians, UR driver order; ±inf if it has none."""
    limits = {joint.name: joint.limit for joint in load_urdf(urdf).robot.joints}
    lower, upper = np.full(len(UR_JOINT_NAMES), -np.inf), np.full(len(UR_JOINT_NAMES), np.inf)
    for column, name in enumerate(UR_JOINT_NAMES):
        limit = limits.get(f"{arm}_{name}")
        if limit is not None and limit.lower is not None and limit.upper is not None:
            lower[column], upper[column] = limit.lower, limit.upper
    return lower, upper


def nearest_turns(goal: Sequence[float], here: Sequence[float], lower: Sequence[float], upper: Sequence[float],
                  stow: Sequence[float]) -> np.ndarray:
    """Return `goal` with each joint shifted by whole turns nearest `here`: the same pose.

    Each joint stays within `UNWOUND` of `stow` where a whole turn of it can, else within its limits.
    """
    return np.array([min(_unwound_turns(float(value), low, high, home), key=lambda turned: abs(turned - at))
                     for value, at, low, high, home in zip(goal, here, lower, upper, stow)])


def nearest_pose(poses: Sequence[Sequence[float]], indices: Iterable[int], here: Sequence[float],
                 lower: Sequence[float], upper: Sequence[float], stow: Sequence[float]) -> tuple[int, np.ndarray]:
    """Return the pose among `indices` the arm reaches soonest from `here`, as `nearest_turns` turns it; and that.

    Ties go to the lower index.
    """
    here = np.asarray(here, dtype=float)
    turned = {index: nearest_turns(poses[index], here, lower, upper, stow) for index in sorted(indices)}
    index = min(turned, key=lambda index: _move_cost(turned[index] - here))
    return index, turned[index]


def short_moves(sweeps: ArmSweeps, lower: Sequence[float], upper: Sequence[float],
                begin: Sequence[float]) -> ArmSweeps:
    """Return the same takes, ordered, turned and shifted by whole turns so the arm moves little between them.

    The order `shortest_order` gives from `begin` (the stow pose), J0 takes first: how the sweeps file was ordered.
    """
    order = shortest_order(sweeps.sweeps, range(len(sweeps.sweeps)), begin, begin, lower, upper, joints=(0, 1))
    return ArmSweeps(tuple(sweep for _, sweep in order))


def shortest_order(sweeps: Sequence[Sweep], indices: Iterable[int], at: Sequence[float], stow: Sequence[float],
                   lower: Sequence[float], upper: Sequence[float],
                   joints: Sequence[int] | None = None) -> list[tuple[int, Sweep]]:
    """Return the takes `indices` in the order whose moves from `at` on take least time, each as `nearest_take` runs it.

    - One joint's takes after the other's; `joints` says which goes first, None tries both. Ties go to the lower
      take numbers first.
    - Exact: the best order of the takes left depends only on which are left and where the arm stands, which is
      remembered (a take ends in one of a few whole-turn variants).

    Returns:
        list: (index, the take turned and shifted) in order.
    """
    indices = sorted(indices)
    remembered: dict = {}

    def best_from(left: tuple[tuple[int, ...], ...], there: np.ndarray) -> tuple[float, list[tuple[int, Sweep]]]:
        left = tuple(group for group in left if group)
        if not left:
            return 0.0, []
        key = (left, tuple(np.round(there, 9)))
        if key not in remembered:
            best: tuple[float, list[tuple[int, Sweep]]] = (np.inf, [])
            for index in left[0]:
                sweep = nearest_take(sweeps[index], there, stow, lower, upper)
                first, last = sweep.ends()
                cost, order = best_from((tuple(i for i in left[0] if i != index), *left[1:]), last)
                cost += _move_cost(first - there)
                if cost < best[0]:
                    best = (cost, [(index, sweep), *order])
            remembered[key] = best
        return remembered[key]

    groups = sorted({sweeps[index].joint for index in indices})
    orders = [best_from(tuple(tuple(i for i in indices if sweeps[i].joint == joint) for joint in joint_order),
                        np.asarray(at, dtype=float))
              for joint_order in ([tuple(joints)] if joints is not None else permutations(groups))]
    return min(orders, key=lambda order: order[0], default=(0.0, []))[1]


def _move_cost(move: np.ndarray) -> float:
    """A move's time, up to a factor: its longest joint travel, ties broken by the total travel."""
    return float(np.abs(move).max() + 0.01 * np.abs(move).sum())


def _unwound_turns(value: float, low: float, high: float, stow: float) -> list[float]:
    """`value` and its whole-turn neighbours within `low`..`high` and `UNWOUND` of `stow`; else within the limits."""
    unwound = [v for v in (value - TURN, value, value + TURN)
               if max(low, stow - UNWOUND) <= v <= min(high, stow + UNWOUND)]
    return unwound or _whole_turns(value, low, high)


def _whole_turns(value: float, low: float, high: float) -> list[float]:
    """`value` and its whole-turn neighbours within `low`..`high`; `value` alone if none is."""
    return [v for v in (value - TURN, value, value + TURN) if low <= v <= high] or [value]


def nearest_take(sweep: Sweep, at: Sequence[float], stow: Sequence[float], lower: Sequence[float],
                 upper: Sequence[float]) -> Sweep:
    """Return `sweep`, maybe backwards and shifted by whole turns, starting as near `at` as limits and `UNWOUND` allow.

    The same take: every waypoint is the same pose. Held joints stay within `UNWOUND` of `stow` where they can; J0 of
    a J1 take stays as given.
    """
    held = list(sweep.joints)
    for column, value in enumerate(held):
        if column != sweep.joint and not (column == 0 and sweep.joint == 1):
            turns = np.array(_unwound_turns(value, lower[column], upper[column], stow[column]))
            held[column] = float(turns[np.argmin(np.abs(turns - at[column]))])
    low, high = lower[sweep.joint] - min(sweep.start, sweep.end), upper[sweep.joint] - max(sweep.start, sweep.end)
    options = [replace(sweep, joints=tuple(held), start=start + shift, end=end + shift)
               for start, end in ((sweep.start, sweep.end), (sweep.end, sweep.start))
               for shift in _whole_turns(0.0, low, high)]
    return min(options, key=lambda option: _move_cost(option.ends()[0] - at))
