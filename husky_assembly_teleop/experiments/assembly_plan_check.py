"""
Plan every arm movement of a process's first actions headless, at the authored bases or perturbed ones.

    python -m husky_assembly_teleop.experiments.assembly_plan_check PROCESS [--actions 18|B20_J,B20_R]
        [--only id,...] [--workaround on|off] [--perturb-base M,DEG] [--seed 0] [--write-solutions DIR]
        [--data DIR] [--verbose]

No ROS: the real robots stand where the design has them (moved by a random offset with --perturb-base), each
robot's arms where its last planned movement left them, also after its base drove (else the authored start, else
stowed). Exit code 0 when nothing failed, 1 on a failed movement, 2 on bad input.

! --write-solutions copies the process (without solutions/ and runs/) and writes into the copy, never the source.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import shutil
import sys
import tempfile
import zlib
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Iterator, Sequence

import numpy as np
from husky_assembly_tamp.motion_planner.ssik_ik import ik_backend
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.process import Action, Movement, Process, read
from scaffolding_assembly_core.process.solutions import (MovementResult, Planner, Solution, read_solutions,
                                                         solution_warnings, solved_against, write_solution)
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Body, Scene

from ..config import RobotConfig, robot_config_from_serial
from ..plugins.process.loading import load_process
from ..shared.assembly_planning.movement import MovementSpec, Routine, classify_action
from ..shared.assembly_planning.route_result import RouteResult
from ..shared.assembly_planning.steps import operator_steps
from ..shared.assembly_planning.tamp_route import plan_movement
from ..shared.assembly_planning.process_steps import LoadedProcess, Step, loaded_process
from ..world.kinematics import stow_joints

#: The repository's data folder: robot URDFs and tools.
DATA = Path(__file__).resolve().parents[2] / "data"
#: The old monitor's fixed dual-arm home (utils.py:90 at 0e45758), left arm then right.
OLD_HOME = dict(zip(
    [f"{arm}_ur_arm_{joint}" for arm in ("left", "right") for joint in (
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")],
    (-1.381079037103113, -0.08674286382411818, -2.8050931738052864, -1.7444565873683324, 0.23963370629882144,
     1.4217452086745808, 1.3946926052686688, -3.0267499888085663, 2.8043950421044888, -1.727003294848389,
     -0.40561451816348215, -1.2402309664671707)))
#: A movement taking longer than this is flagged in the table, seconds.
SLOW = 60.0
#: Lines of a failed route's own output shown without --verbose.
TAIL = 20


@dataclass(frozen=True)
class Row:
    """One arm movement of the check.

    Attributes:
        action, movement: Their ids.
        status: "solved", "failed", "not_planned", or "skipped" (the transfer workaround does not run it).
        reason: Why it failed, was not planned or skipped.
        route: What planning gave; None when skipped.
        offset: The acting robot's base offset (dx, dy metres, dyaw radians); zeros unperturbed.
        output: What planning printed, when captured.
    """

    action: str
    movement: str
    status: str
    reason: str = ""
    route: RouteResult | None = None
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    output: str = ""


def perturbed(base: Pose, offset: tuple[float, float, float]) -> Pose:
    """A base moved by (dx, dy) on the floor and turned by dyaw about the world z axis; its height unchanged."""
    dx, dy, dyaw = offset
    turned = Rotation.from_rotvec((0.0, 0.0, dyaw)) * Rotation.from_quat(base.orientation)
    return Pose.from_arrays(np.add(base.position, (dx, dy, 0.0)), turned.as_quat())


class BaseOffsets:
    """One random base offset per robot and design base, drawn from the seed and that stand alone.

    * Any order and any subset of movements (`--only`, `--actions`) gives a stand the same offset as the full run.
    """

    def __init__(self, perturb: tuple[float, float] | None, seed: int):
        """Offsets within ±metres and ±degrees (`perturb`), none without it.

        Args:
            perturb: (metres, degrees), or None for no offsets.
            seed: The run's seed.
        """
        self.perturb, self.seed = perturb, seed

    def __call__(self, robot: str, base: Pose) -> tuple[float, float, float]:
        """The offset of one robot standing on one design base: (dx, dy) metres, dyaw radians.

        Args:
            robot: The process robot id.
            base: Its design base.

        Returns:
            tuple[float, float, float]: dx, dy and dyaw.
        """
        if self.perturb is None:
            return (0.0, 0.0, 0.0)
        metres, degrees = self.perturb
        rng = np.random.default_rng([self.seed, zlib.crc32(repr(base_key(robot, base)).encode())])
        dx, dy = rng.uniform(-metres, metres, 2)
        return (float(dx), float(dy), math.radians(float(rng.uniform(-degrees, degrees))))


def base_key(robot: str, base: Pose) -> tuple:
    """A robot on a design base, the base rounded to 1 mm and 0.1° of yaw: the same key is the same stand."""
    yaw = Rotation.from_quat(base.orientation).as_euler("zyx", degrees=True)[0]
    return (robot, *(round(value * 1000) for value in base.position), round(yaw * 10))


def run_check(loaded: LoadedProcess, robots: tuple[RobotConfig, ...], actions: Sequence[str], *,
              transfer_as_free: bool = True, perturb: tuple[float, float] | None = None, seed: int = 0,
              only: Iterable[str] | None = None, extra_bodies: Sequence[Body] = (),
              mirrors: dict[str, CompasFabMirror] | None = None, verbose: bool = True,
              log: Callable[[str], None] = print) -> list[Row]:
    """Plan every arm movement of some actions in schedule order, each robot's arms chained from its last planned end.

    Args:
        loaded: The loaded process.
        robots: The configured robots, one per process robot that has a serial.
        actions: Action ids, in schedule order.
        transfer_as_free: The transfer workaround: the transfer without the bar, the travel to load skipped.
        perturb: (metres, degrees): each robot's design base moved by a random offset within them, or None.
        seed: Seeds the offsets (per robot and design base) and the searches (per movement).
        only: Movement ids to plan; the others are left out (their robot's chain is broken).
        extra_bodies: Obstacles added to every scene (ids under "obstacles/").
        mirrors: Live robot id -> mirror to plan with; made (and closed) here where missing.
        verbose: Let planning print as it goes; otherwise its output is kept in `Row.output`.
        log: Gets one line per movement.

    Returns:
        list[Row]: One per arm movement planned or skipped, in the order run.
    """
    offsets = BaseOffsets(perturb, seed)
    process, only = loaded.process, set(only) if only is not None else None
    steps = {(step.action.id, step.movement.id): step for step in loaded.steps}
    planned = {spec.serial: robot for robot, spec in process.robots.items() if spec.serial}
    configs = {planned[config.serial[-4:]]: config for config in robots if config.serial[-4:] in planned}
    made = mirrors is None
    mirrors = {} if made else mirrors
    current: dict[str, dict[str, float]] = {}
    rows = []
    try:
        for action, movement, skipped in arm_movements(process, actions, transfer_as_free):
            if only is not None and movement.id not in only:
                current.pop(action.robot, None)
                continue
            if skipped:
                rows.append(Row(action.id, movement.id, "skipped", "transfer workaround: not run"))
                continue
            row = _plan_one(loaded, steps[action.id, movement.id], robots, configs, offsets, current, mirrors,
                            transfer_as_free, extra_bodies, verbose, seed)
            rows.append(row)
            seconds = f" in {row.route.seconds:.1f} s" if row.route is not None else ""
            log(f"{row.movement}: {row.status}{seconds}{f' ({row.reason})' if row.reason else ''}")
    finally:
        for mirror in mirrors.values() if made else ():
            mirror.close()
    return rows


def arm_movements(process: Process, actions: Sequence[str],
                  transfer_as_free: bool) -> list[tuple[Action, Movement, bool]]:
    """Every arm movement of some actions in the order the operator runs them, and whether the workaround skips it.

    Args:
        process: The process.
        actions: Action ids, in schedule order.
        transfer_as_free: The transfer workaround: the travel to load is skipped.

    Returns:
        list[tuple[Action, Movement, bool]]: (action, movement, skipped) per arm movement.
    """
    found = []
    for action_id in actions:
        action = process.actions[action_id]
        for step in operator_steps(process, action, transfer_as_free):
            for index in (*step.skipped, *step.indices):
                if action.movements[index].arms:
                    found.append((action, action.movements[index], index in step.skipped))
    return found


def _plan_one(loaded: LoadedProcess, step: Step, robots: tuple[RobotConfig, ...], configs: dict[str, RobotConfig],
              offsets: BaseOffsets, current: dict[str, dict[str, float]],
              mirrors: dict[str, CompasFabMirror], transfer_as_free: bool, extra_bodies: Sequence[Body],
              verbose: bool, seed: int) -> Row:
    """Plan one arm movement in its design snapshot, and move its robot's chain on.

    A robot's arms are where its last planned movement left them, wherever its base stands now: they do not move
    while the base drives. Without one (its first movement, or after a failed or left-out one), the authored start
    joints, else its stowed arms.

    Args:
        loaded: The loaded process.
        step: The movement's step.
        robots: The configured robots.
        configs: Process robot id -> its config.
        offsets: The base offsets.
        current: Process robot id -> its joints after its last planned movement; changed here.
        mirrors: Live robot id -> mirror; a missing one is made here.
        transfer_as_free: The transfer workaround.
        extra_bodies: Obstacles added to the snapshot.
        verbose: Let planning print; otherwise keep its output in the row.
        seed: The run's seed; the movement's searches are seeded from it and the movement id.

    Returns:
        Row: The movement's row.
    """
    action, movement = step.action, step.movement
    robot = action.robot
    snapshot_robots, placed = {}, set()
    for planned, config in configs.items():
        state = movement.start.robots.get(planned)
        if state is None or state.base is None:
            continue
        placed.add(planned)
        joints = stow_joints(config, config.model.movable_joints)
        joints.update(current.get(planned) or state.joints or {})
        live = robot_id(config.serial)
        snapshot_robots[live] = RobotObject(live, config.model, perturbed(state.base, offsets(planned, state.base)),
                                            joints)
    if robot not in placed:
        current.pop(robot, None)
        return Row(action.id, movement.id, "not_planned", f"{robot} has no design base at the start")
    snapshot = Scene(robots=snapshot_robots, bodies={body.id: body for body in extra_bodies},
                     world_poses={body.id: body.placement for body in extra_bodies})
    live = robot_id(configs[robot].serial)
    if live not in mirrors:
        mirrors[live] = CompasFabMirror(live)
    # * Seeded per movement: one planned alone (`--only`) searches as in the full run, given the same start.
    search_seed = zlib.crc32(f"{seed}/{movement.id}".encode())
    random.seed(search_seed)
    np.random.seed(search_seed)
    output: list[str] = []
    with _captured(output, not verbose):
        route = plan_movement(mirrors[live], loaded, step, snapshot, robots, transfer_as_free=transfer_as_free)
    result = route.result
    if result.status == "solved":
        current[robot] = result.end[robot]
    elif result.status == "failed":
        current.pop(robot, None)
    return Row(action.id, movement.id, result.status, result.reason, route, offsets(robot, movement.start.robots[
        robot].base), "".join(output))


@contextmanager
def _captured(into: list[str], capture: bool) -> Iterator[None]:
    """Keep what is printed meanwhile, Python's and the C libraries' alike, in `into`; nothing when not `capture`.

    ? At the file descriptor: PyBullet's helpers redirect it themselves, so `redirect_stdout` breaks them.
    """
    if not capture:
        yield
        return
    sys.stdout.flush()
    saved = os.dup(1)
    with tempfile.TemporaryFile(mode="w+") as file:
        os.dup2(file.fileno(), 1)
        try:
            yield
        finally:
            sys.stdout.flush()
            os.dup2(saved, 1)
            os.close(saved)
            file.seek(0)
            into.append(file.read())


def home_check(specs: Iterable[MovementSpec]) -> dict[str, float]:
    """How far each way home's target joints are from the old monitor's fixed home: movement id -> max |Δ| (rad)."""
    return {spec.movement: max(abs(spec.goal_joints[name] - value) for name, value in OLD_HOME.items())
            for spec in specs if spec.routine is Routine.FREE_HOME}


def format_table(rows: Sequence[Row]) -> str:
    """The rows as a text table, then one summary line."""
    lines = [f"{'movement':<18} {'routine':<17} {'ok':<5} {'tries':>5} {'s':>6} {'end error':>17} {'wp':>4}  note"]
    for row in rows:
        route = row.route
        error, note = "", _note(row)
        if route is not None and route.ok:
            if route.end_error:
                metres = max(value[0] for value in route.end_error.values())
                radians = max(value[1] for value in route.end_error.values())
                error = f"{metres * 1e3:.4f}mm/{math.degrees(radians):.4f}°"
            elif route.joint_error is not None:
                error = f"Δq {route.joint_error:.1e} rad"
        ok = {"solved": "ok", "failed": "FAIL", "skipped": "skip"}.get(row.status, "-")
        routine = route.routine.value if route is not None else "transfer skipped"
        tries, seconds = (route.attempts, f"{route.seconds:.1f}") if route is not None else ("", "")
        waypoints = route.waypoints if route is not None and route.ok else ""
        flag = " ! slow" if route is not None and route.seconds > SLOW else ""
        lines.append(f"{row.movement:<18} {routine:<17} {ok:<5} {tries:>5} {seconds:>6} {error:>17} {waypoints!s:>4}  "
                     f"{note}{flag}")
    counts = {status: sum(row.status == status for row in rows) for status in ("solved", "failed", "skipped",
                                                                               "not_planned")}
    slowest = max((row.route.seconds for row in rows if row.route is not None), default=0.0)
    lines.append(", ".join(f"{count} {status.replace('_', ' ')}" for status, count in counts.items())
                 + f"; slowest {slowest:.1f} s; total {sum(r.route.seconds for r in rows if r.route):.1f} s")
    return "\n".join(lines)


def _note(row: Row) -> str:
    """A row's note: the base offset; then why it is not solved, or a moved start and a way home's turns."""
    route, parts = row.route, []
    if any(row.offset):
        parts.append(f"base +({row.offset[0] * 1e3:.0f}, {row.offset[1] * 1e3:.0f}) mm "
                     f"{math.degrees(row.offset[2]):+.1f}°")
    if route is None or not route.ok:
        return "; ".join([*parts, row.reason])
    if route.result.start_overridden:
        parts.append("start moved off the process's")
    if route.routine is Routine.FREE_HOME:
        robot, = route.result.end
        start, end = route.result.start[robot], route.result.end[robot]
        # * The old run's long way home turned the right pan by about 5 rad: show it, and the largest turn.
        for name in dict.fromkeys(("right_ur_arm_shoulder_pan_joint", max(end, key=lambda n: abs(end[n] - start[n])))):
            if name in end:
                parts.append(f"{name.replace('_ur_arm_', ' ').replace('_joint', '')} {end[name] - start[name]:+.2f}")
    return "; ".join(parts)


def write_solutions(loaded: LoadedProcess, rows: Sequence[Row], folder: Path, settings: dict) -> Path:
    """Copy the process into `folder` (without solutions/ and runs/) and write the rows as its solutions.

    Args:
        loaded: The loaded process.
        rows: The check's rows; a skipped one is written as not planned.
        folder: Where the copy goes, as `folder/<process folder name>`; never inside the process.
        settings: The check's settings, recorded with the planner.

    Returns:
        Path: The copy.

    Raises:
        ValueError: If `folder` is inside the process folder.
        FileExistsError: If the copy is already there.
    """
    copy = solutions_copy(loaded, folder)
    source = loaded.source.resolve()
    shutil.copytree(source, copy, ignore=shutil.ignore_patterns("solutions", "runs"))
    process = read(copy)
    planner = Planner("husky_assembly_teleop.experiments.assembly_plan_check", "husky-assembly-teleop",
                      ik_backend=ik_backend(), settings=dict(settings))
    by_action: dict[str, dict[str, MovementResult]] = {}
    for row in rows:
        result = row.route.result if row.route is not None else MovementResult("not_planned", row.reason)
        by_action.setdefault(row.action, {})[row.movement] = result
    for action_id, movements in by_action.items():
        write_solution(process, Solution(action_id, solved_against(process, action_id), planner, movements))
    warnings = solution_warnings(process, read_solutions(process))
    print(f"wrote {len(by_action)} solutions to {copy / 'solutions'}; {len(warnings)} solution warnings")
    for line in warnings[:10]:
        print(f"  {line}")
    return copy


def solutions_copy(loaded: LoadedProcess, folder: Path) -> Path:
    """Where `write_solutions` puts its copy of the process, checked before anything is planned.

    Args:
        loaded: The loaded process.
        folder: The folder given to --write-solutions.

    Returns:
        Path: `folder/<process folder name>`.

    Raises:
        ValueError: If `folder` is the process folder or inside it.
        FileExistsError: If the copy is already there.
    """
    source, folder = loaded.source.resolve(), Path(folder).expanduser().resolve()
    if folder == source or source in folder.parents:
        raise ValueError(f"{folder} is inside the process folder {source}: write the solutions elsewhere")
    if (folder / source.name).exists():
        raise FileExistsError(f"{folder / source.name} is already there: give a new folder")
    return folder / source.name


def select_actions(loaded: LoadedProcess, text: str) -> list[str]:
    """The action ids `--actions` names: the first N of the schedule, or ids in schedule order.

    Raises:
        ValueError: If an id is not in the schedule.
    """
    schedule = list(loaded.process.schedule)
    if text.strip().isdigit():
        return schedule[:int(text)]
    wanted = [name.strip() for name in text.split(",") if name.strip()]
    unknown = [name for name in wanted if name not in schedule]
    if unknown:
        raise ValueError(f"not in the schedule: {', '.join(unknown)}")
    return [name for name in schedule if name in wanted]


def parser() -> argparse.ArgumentParser:
    """The command line."""
    parse = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    parse.add_argument("process", type=Path, help="the process folder (holding process.json)")
    parse.add_argument("--actions", default="18", help="how many actions from the schedule's start, or their ids")
    parse.add_argument("--only", default=None, help="movement ids to plan, comma-separated; the others are left out")
    parse.add_argument("--workaround", choices=("on", "off"), default="on",
                       help="the transfer workaround: the transfer without the bar, the travel to load skipped")
    parse.add_argument("--perturb-base", default=None, metavar="M,DEG",
                       help="move each design base by a random offset within ±M metres and ±DEG degrees")
    parse.add_argument("--seed", type=int, default=0, help="seed of the offsets and the searches")
    parse.add_argument("--write-solutions", type=Path, default=None, metavar="DIR",
                       help="copy the process into DIR and write the results there as its solutions")
    parse.add_argument("--data", type=Path, default=DATA, help="the data folder: robot URDFs and tools")
    parse.add_argument("--verbose", action="store_true", help="show what planning prints as it goes")
    return parse


def main(argv: Sequence[str] | None = None) -> int:
    """Run the check from the command line.

    Returns:
        int: 0 when every movement is solved, skipped or not planned; 1 on a failed one; 2 on bad input.
    """
    options = parser().parse_args(argv)
    try:
        perturb = tuple(float(value) for value in options.perturb_base.split(",")) if options.perturb_base else None
        if perturb is not None and len(perturb) != 2:
            raise ValueError("--perturb-base takes metres and degrees, e.g. 0.02,2")
        loaded = loaded_process(load_process(options.process, options.data))
        actions = select_actions(loaded, options.actions)
        only = [name.strip() for name in options.only.split(",")] if options.only else None
        known = {movement.id for action_id in actions for movement in loaded.process.actions[action_id].movements}
        if only is not None and not set(only) <= known:
            raise ValueError(f"not a movement of the actions checked: {', '.join(sorted(set(only) - known))}")
        robots = tuple(robot_config_from_serial(spec.serial, options.data)
                       for spec in loaded.process.robots.values() if spec.serial)
        if options.write_solutions is not None:
            solutions_copy(loaded, options.write_solutions)
    except (ValueError, FileNotFoundError, FileExistsError) as error:
        print(f"bad input: {error}", file=sys.stderr)
        return 2
    transfer_as_free = options.workaround == "on"
    rows = run_check(loaded, robots, actions, transfer_as_free=transfer_as_free, perturb=perturb, seed=options.seed,
                     only=only, verbose=options.verbose, log=lambda line: print(line, file=sys.stderr, flush=True))
    print(format_table(rows))
    for row in rows:
        if row.status == "failed" and row.output and not options.verbose:
            print(f"--- {row.movement}: the last {TAIL} lines planning printed ---")
            print("\n".join(row.output.rstrip().splitlines()[-TAIL:]))
    homes = home_check(spec for action_id in actions
                       for spec in classify_action(loaded.process, loaded.process.actions[action_id]))
    if homes:
        print(f"home check: {len(homes)} ways home, max |target - old home| {max(homes.values()):.2e} rad")
    if options.write_solutions is not None:
        settings = {"actions": list(actions), "workaround": options.workaround, "perturb_base": options.perturb_base,
                    "seed": options.seed}
        write_solutions(loaded, rows, options.write_solutions, settings)
    return 1 if any(row.status == "failed" for row in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
