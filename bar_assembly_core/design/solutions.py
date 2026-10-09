"""
Planner results, `solutions/<action id>.json` (format §8): read, write, staleness (A14) and the chaining rule.

A solution records the content hashes of `design.json` and of its action file as solved. When either file changed
since, the solution is stale: shown, never executed.

! Planners write; nothing else does. The design never refers to its solutions.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..geometry import Pose
from .read import check_keys, load_json
from .types import Design, DesignError
from .version import writer_info
from .write import _float, _joints, _pose, _write_json, _writer, content_hash

SOLUTION_FORMAT = "husky_design/solution"
#: A movement's result: solved; base and end joints only; failed (with a reason); not planned yet.
STATUSES: Tuple[str, ...] = ("solved", "keyframe_only", "failed", "not_planned")
#: How far a movement's solved start may be from the previous solved end, and a solved end from the design's target
#: joints, radians or metres: the planners' IK tolerance.
CHAIN_TOLERANCE = 1e-3


@dataclass(frozen=True)
class SolvedAgainst:
    """The content hashes (`content_hash`) of `design.json` and of the action file a solution was planned on."""

    design: str
    action: str


@dataclass(frozen=True)
class Planner:
    """Which planner made a solution: `artifacts` names the ssik artifact set or URDF it was built for."""

    name: str
    repo: str = ""
    commit: str = ""
    ik_backend: str = ""
    artifacts: str = ""
    settings: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Trajectory:
    """One robot's path over all its moving joints (12 on Cindy), never split per arm; `times` in seconds, or None."""

    robot: str
    joint_names: Tuple[str, ...]
    positions: Tuple[Tuple[float, ...], ...]
    times: Optional[Tuple[float, ...]] = None


@dataclass(frozen=True)
class MovementResult:
    """What a planner found for one movement.

    Attributes:
        status: One of STATUSES.
        reason: Why it failed; required for "failed".
        bases: Robot id -> the base chosen where the design had None.
        start: Robot id -> solved start joints.
        start_overridden: The planner moved a start the design had fixed; neighbours must agree with it.
        end: Robot id -> solved end joints.
        trajectory: The path, or None.
        path_poses: Link or body id -> its poses along the path, for showing an attached bar.
    """

    status: str
    reason: str = ""
    bases: Dict[str, Pose] = field(default_factory=dict)
    start: Dict[str, Dict[str, float]] = field(default_factory=dict)
    start_overridden: bool = False
    end: Dict[str, Dict[str, float]] = field(default_factory=dict)
    trajectory: Optional[Trajectory] = None
    path_poses: Dict[str, Tuple[Pose, ...]] = field(default_factory=dict)

    @staticmethod
    def still(joints: Dict[str, Dict[str, float]]) -> "MovementResult":
        """The result of a movement no arm moves in (a grip, a drive, a manual step): solved, ending where it starts.

        Args:
            joints: Robot id -> its joints during the movement.
        """
        return MovementResult(status="solved", start=joints, end=joints)


@dataclass(frozen=True)
class Solution:
    """The latest planner result for one action: one `MovementResult` per movement id."""

    action: str
    solved_against: SolvedAgainst
    planner: Planner
    movements: Dict[str, MovementResult]


def solved_against(design: Design, action_id: str) -> SolvedAgainst:
    """The hashes a solution planned now records: of `design.json` and of the action file, as written on disk.

    Raises:
        ValueError: If the design was never written (`folder` is None).
    """
    if design.folder is None:
        raise ValueError("the design is only in memory: write it first, a solution refers to its files")
    return SolvedAgainst(content_hash(design.folder / "design.json"),
                         content_hash(design.folder / "actions" / f"{action_id}.json"))


def is_stale(design: Design, solution: Solution) -> bool:
    """Whether the design or the action file changed since the solution was planned (A14)."""
    return solution.solved_against != solved_against(design, solution.action)


def write_solution(design: Design, solution: Solution) -> Path:
    """Write a solution to `solutions/<action id>.json` in the design folder, replacing the one there.

    Raises:
        DesignError: If it names an unknown action or movement, or breaks a solution rule (A2, A14).
        ValueError: If the design was never written.
    """
    if design.folder is None:
        raise ValueError("the design is only in memory: write it first")
    raw = {"format": SOLUTION_FORMAT, "writer": _writer(writer_info()), "planner": _planner(solution.planner),
           "action": solution.action,
           "solved_against": {"design": solution.solved_against.design, "action": solution.solved_against.action},
           "movements": {movement_id: _result(result) for movement_id, result in solution.movements.items()}}
    problems: List[str] = []
    _check(design, raw, f"solutions/{solution.action}.json", problems)
    if problems:
        raise DesignError(problems)
    folder = design.folder / "solutions"
    folder.mkdir(exist_ok=True)
    path = folder / f"{solution.action}.json"
    _write_json(path, raw)
    return path


def read_solutions(design: Design) -> Dict[str, Solution]:
    """Every solution in the design folder's `solutions/`, by action id; empty if there is none.

    Raises:
        DesignError: Listing every broken file (A1, A2, A14). Stale solutions are not errors: see `solution_warnings`.
    """
    if design.folder is None:
        return {}
    problems: List[str] = []
    raws = {}
    for path in sorted((design.folder / "solutions").glob("*.json")):
        name = f"solutions/{path.name}"
        raw = load_json(path, name, SOLUTION_FORMAT, problems)
        if raw is not None:
            before = len(problems)
            check_keys(raw, "solution", name, problems)
            if len(problems) == before:
                _check(design, raw, name, problems)
                if raw["action"] != path.stem:
                    problems.append(f"A14: {name}: names action {raw['action']!r}, not its file name")
            raws[path.stem] = raw
    if problems:
        raise DesignError(problems)
    return {name: _solution(raw) for name, raw in raws.items()}


def solution_warnings(design: Design, solutions: Dict[str, Solution]) -> List[str]:
    """Stale solutions (A14), every break of the chaining rule, and solved ends away from the design, one line each.

    - Chaining: each movement starts where the one before it in schedule order ended, robot by robot, within
      CHAIN_TOLERANCE; only results with joints (solved, keyframe_only) count.
    - The design wins: where a movement's target gives joints, its solved end must reach them within CHAIN_TOLERANCE.
    """
    warnings = [f"A14: solutions/{action_id}.json is stale: the design or the action changed since it was planned"
                for action_id, solution in sorted(solutions.items()) if is_stale(design, solution)]
    # Robot id -> (movement id, joints) of its last solved end.
    last: Dict[str, Tuple[str, Dict[str, float]]] = {}
    for action, movement in design.movements():
        solution = solutions.get(action.id)
        result = solution.movements.get(movement.id) if solution is not None else None
        if result is None or result.status not in ("solved", "keyframe_only"):
            continue
        for robot, joints in result.start.items():
            if robot in last:
                previous, ended = last[robot]
                shared = sorted(set(joints) & set(ended))
                gap = max((abs(joints[name] - ended[name]) for name in shared), default=0.0)
                if gap > CHAIN_TOLERANCE:
                    warnings.append(f"solutions: {robot} ends {previous} and starts {movement.id} "
                                    f"{gap:.3g} apart")
        for robot, joints in result.end.items():
            last[robot] = (movement.id, joints)
            wanted = movement.target.joints.get(robot, {}) if movement.target is not None else {}
            gap = max((abs(joints[name] - value) for name, value in wanted.items() if name in joints), default=0.0)
            if gap > CHAIN_TOLERANCE:
                warnings.append(f"solutions: {robot} ends {movement.id} {gap:.3g} from the design's target joints")
    return warnings


# --- --- --- --- --- CHECKS AND JSON --- --- --- --- ---

def _check(design: Design, raw: Dict[str, Any], name: str, problems: List[str]) -> None:
    """A solution names an existing action and its movements, with known statuses (A14)."""
    action = design.actions.get(raw["action"])
    if action is None:
        problems.append(f"A14: {name}: unknown action {raw['action']!r}")
        return
    known = {movement.id for movement in action.movements}
    for movement_id, result in raw["movements"].items():
        where = f"{name} {movement_id}"
        if movement_id not in known:
            problems.append(f"A14: {where}: not a movement of {action.id}")
        if result["status"] not in STATUSES:
            problems.append(f"A14: {where}: status must be one of {STATUSES}, not {result['status']!r}")
        elif result["status"] == "failed" and not result.get("reason"):
            problems.append(f"A14: {where}: a failed result needs a reason")
        for robot in (*result.get("bases", {}), *result.get("start", {}), *result.get("end", {})):
            if robot not in design.robots:
                problems.append(f"A14: {where}: unknown robot {robot!r}")
        trajectory = result.get("trajectory")
        if trajectory is not None and any(len(row) != len(trajectory["joint_names"])
                                          for row in trajectory["positions"]):
            problems.append(f"A14: {where}: every trajectory row has one value per joint name")


def _planner(planner: Planner) -> Dict[str, Any]:
    """The `planner` block; empty fields left out."""
    entry = {"name": planner.name, "repo": planner.repo, "commit": planner.commit, "ik_backend": planner.ik_backend,
             "artifacts": planner.artifacts, "settings": dict(planner.settings)}
    return {key: value for key, value in entry.items() if key == "name" or value}


def _result(result: MovementResult) -> Dict[str, Any]:
    """One movement's result; empty fields left out."""
    entry: Dict[str, Any] = {"status": result.status}
    if result.reason:
        entry["reason"] = result.reason
    if result.bases:
        entry["bases"] = {robot: _pose(pose) for robot, pose in sorted(result.bases.items())}
    if result.start:
        entry["start"] = {robot: _joints(joints) for robot, joints in sorted(result.start.items())}
    if result.start_overridden:
        entry["start_overridden"] = True
    if result.end:
        entry["end"] = {robot: _joints(joints) for robot, joints in sorted(result.end.items())}
    if result.trajectory is not None:
        trajectory = result.trajectory
        entry["trajectory"] = {"robot": trajectory.robot, "joint_names": list(trajectory.joint_names),
                               "positions": [[_float(v) for v in row] for row in trajectory.positions]}
        if trajectory.times is not None:
            entry["trajectory"]["times"] = [_float(v) for v in trajectory.times]
    if result.path_poses:
        entry["path_poses"] = {link: [_pose(pose) for pose in poses]
                               for link, poses in sorted(result.path_poses.items())}
    return entry


def _pose_of(values) -> Pose:
    """A pose from `[x, y, z, qx, qy, qz, qw]`."""
    return Pose.from_arrays(values[:3], values[3:])


def _solution(raw: Dict[str, Any]) -> Solution:
    """A Solution from a checked file."""
    planner = raw["planner"]
    movements = {}
    for movement_id, result in raw["movements"].items():
        trajectory = result.get("trajectory")
        movements[movement_id] = MovementResult(
            status=result["status"], reason=result.get("reason", ""),
            bases={robot: _pose_of(pose) for robot, pose in result.get("bases", {}).items()},
            start={robot: {k: float(v) for k, v in joints.items()}
                   for robot, joints in result.get("start", {}).items()},
            start_overridden=bool(result.get("start_overridden", False)),
            end={robot: {k: float(v) for k, v in joints.items()} for robot, joints in result.get("end", {}).items()},
            trajectory=None if trajectory is None else Trajectory(
                trajectory["robot"], tuple(trajectory["joint_names"]),
                tuple(tuple(float(v) for v in row) for row in trajectory["positions"]),
                tuple(float(v) for v in trajectory["times"]) if "times" in trajectory else None),
            path_poses={link: tuple(_pose_of(pose) for pose in poses)
                        for link, poses in result.get("path_poses", {}).items()})
    return Solution(action=raw["action"],
                    solved_against=SolvedAgainst(raw["solved_against"]["design"], raw["solved_against"]["action"]),
                    planner=Planner(planner["name"], planner.get("repo", ""), planner.get("commit", ""),
                                    planner.get("ik_backend", ""), planner.get("artifacts", ""),
                                    dict(planner.get("settings", {}))),
                    movements=movements)
