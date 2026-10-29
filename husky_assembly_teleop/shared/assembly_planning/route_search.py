"""
The searches, checks and results a movement's route is built of, on the acting robot's synced mirror.

! Worker thread only, on the thread that created the mirror: every borrow, IK and check is followed by a sync.
"""

from __future__ import annotations

import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Iterator, Mapping, Sequence

import numpy as np
import pybullet as p
from husky_assembly_tamp.motion_planner.api import plan_free_dual_arm

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.process import Process
from scaffolding_assembly_core.process.solutions import CHAIN_TOLERANCE, MovementResult, Trajectory
from scaffolding_assembly_core.scene import Scene

from ..planner.rrt_connect import MAX_ITERATIONS, RESTARTS
from .process_steps import Step
from .arm import plan_arm
from .collision import collision_hit, in_process_terms, swept_check, world_frame_fk
from .joints import joint_limits, outside_limits
from .movement import ArmPart, GoalSource, MovementSpec
from .route_result import RouteResult, line_goal, pose_error
from .scene_for import PlanningScene
from .search_tree import RUNS, SearchTree

if TYPE_CHECKING:
    from compas_fab.backends import PyBulletPlanner

    from ...config import RobotConfig
    from ...world.collision_hit import CollisionHit

#: Tries of the held transfer's search before giving up, each a new random search.
ATTEMPTS = 3
#: Seconds per try of a support robot's one-arm search.
SUPPORT_TIME = 30.0
#: Goals given in joints: the result says how far its end is from them.
_JOINT_GOALS = (GoalSource.JOINTS, GoalSource.NEXT_START, GoalSource.STOW)


@dataclass(frozen=True)
class FreeSearch:
    """The operator's settings of the free searches (Advanced planning): tamp's free dual-arm search and the one-arm one.

    Attributes:
        max_time: Seconds per try of tamp's free search.
        max_iterations: Iterations per try of it.
        joint_resolution: The joint step of its collision checks, radians.
        tries: Tries before giving up, each a new random search; the one-arm search's too.
        swept_check: Check a found path between its waypoints (`swept_check`); off, the path is taken as found.
    """

    max_time: float = 120.0
    max_iterations: int = 50
    joint_resolution: float = 0.1
    tries: int = 3
    swept_check: bool = True

    def tamp(self) -> dict:
        """The keyword arguments of tamp's `plan_free_dual_arm`."""
        return {"max_time": self.max_time, "max_iterations": self.max_iterations,
                "joint_resolution": self.joint_resolution}


class Refused(Exception):
    """Planning a movement stops here; the message says why."""


class RouteSearch:
    """One movement being planned: its scenes, robot and start, the searches it may run, and what they found."""

    def __init__(self, mirror: CompasFabMirror, process: Process, step: Step, spec: MovementSpec,
                 planning: PlanningScene, scene: Scene, config: RobotConfig, goal_joints: Mapping[str, float] | None,
                 abort: threading.Event, log: Callable[[str], None], search: FreeSearch = FreeSearch(),
                 tree: SearchTree | None = None):
        """Hold what every routine needs; `names` and `start` are set once the mirror is synced.

        Args:
            mirror: The acting robot's mirror.
            process: The process.
            step: The movement's step.
            spec: Its classification.
            planning: Its planning scene, anything held by the robot shown.
            scene: The scene the searches run in (the planning scene, or it without the held bar).
            config: The acting robot's config.
            goal_joints: Joints to plan to instead of solving the goal, or None.
            abort: Stops between tries.
            log: Gets the lines of every try, the checks, and the fallbacks of IK and the straight moves.
            search: The free searches' settings.
            tree: Records the free searches' trees while they run (`SearchTree`), or None.
        """
        self.mirror, self.process, self.step, self.spec = mirror, process, step, spec
        self.planning, self.scene, self.config, self.abort, self.log = planning, scene, config, abort, log
        self.search, self.tree = search, tree
        self.override = dict(goal_joints) if goal_joints is not None else None
        self.robot = planning.scene.robots[planning.acting]
        self.names: list[str] = []
        self.start: dict[str, float] = {}
        self.planner, self.attempts = "", 0
        self.goal_joints: dict[str, float] | None = None
        self.colliding: list[tuple[str, str]] = []
        self.hits: list[CollisionHit] = []
        self.warnings = [*planning.warnings, *spec.warnings]
        self.swept = self.ik = None

    # --- --- --- --- --- GOALS --- --- --- --- ---

    def joints_of(self, given: Mapping[str, float] | None) -> dict[str, float]:
        """The planned joints' values out of a goal that may give more joints.

        Args:
            given: Values by joint name, or None.

        Returns:
            dict[str, float]: One value per name in `names`.

        Raises:
            Refused: If `given` is None or lacks one of the joints.
        """
        if given is None or any(name not in given for name in self.names):
            raise Refused(f"no goal joints for {self.spec.goal.value} ({self.spec.goal_from or 'nothing to read'})")
        return {name: float(given[name]) for name in self.names}

    def check_limits(self, goal: Mapping[str, float]) -> None:
        """Refuse a start or goal outside the URDF limits: the search would run its whole time for nothing.

        Args:
            goal: Goal values by joint name, one per name in `names`.

        Raises:
            Refused: Naming each joint outside its limits.
        """
        labelled = [(label, [values[name] for name in self.names])
                    for label, values in (("start", self.start), ("goal", goal))]
        outside = outside_limits(labelled, self.names, *joint_limits(self.robot.model.urdf, self.names))
        if outside:
            raise Refused(f"outside the URDF limits: {'; '.join(outside)}")

    def line_goals(self) -> dict[ArmPart, Pose]:
        """Each moving flange's line goal, from where the start has it.

        Returns:
            dict[ArmPart, Pose]: Moving arm -> world pose of its tool0 at the end of its line.

        Raises:
            Refused: If a moving flange has no line.
        """
        if any(part.line is None for part in self.spec.arms):
            raise Refused("a moving flange has no line")
        return {part: line_goal(self.flange(part), part.line) for part in self.spec.arms}

    def flange(self, part: ArmPart, joints: Mapping[str, float] | None = None) -> Pose:
        """World pose of a moving arm's tool0 at the start, some joints changed.

        Args:
            part: The moving arm.
            joints: Values by joint name to use instead of the start's, or None.

        Returns:
            Pose: Its tool0 in the world, at the robot's base in the snapshot.
        """
        return link_pose(self.robot.model.urdf, self.robot.base, {**self.robot.joints, **(joints or {})},
                         f"{part.arm}_tool0")

    # --- --- --- --- --- SEARCHES --- --- --- --- ---

    def free(self, goal: dict[str, float], goals: dict[ArmPart, Pose] | None = None) -> RouteResult:
        """The free dual-arm search of tamp with the `search` settings, swept-checked; tried again on no path or a hit.

        Args:
            goal: Goal values by joint name, one per name in `names` (a plain dict: tamp fails on numpy arrays).
            goals: Moving arm -> the pose its tool0 should end at, to report the end's error; None for a joint goal.

        Returns:
            RouteResult: Solved, with the first path clear between its waypoints.

        Raises:
            Refused: If the start or goal collides, or no try gives a clear path (with the straight path's verdict).
        """
        self.goal_joints = goal
        search, failures = self.search, []
        for _ in range(search.tries):
            self.next_attempt()
            self.note(f"max_time {search.max_time:.0f} s, max_iterations {search.max_iterations}, joint_resolution "
                      f"{search.joint_resolution} rad")
            started, path = time.monotonic(), None
            with self.mirror.lend() as planner, world_frame_fk(planner), \
                    self.recorded(goal, search.max_iterations, RUNS, planner) as drawn:
                try:
                    path, info = plan_free_dual_arm(planner, self.mirror.state, goal, **search.tamp(), **drawn)
                finally:
                    if self.tree is not None:
                        self.tree.end(found=path is not None)
            self.mirror.sync(self.scene)
            if path is None and info.get("failure_reason") == "start_or_goal_in_collision":
                self.refuse_endpoints(goal)
            failures.append(self.ended(path, started, f"tamp: {info.get('failure_reason', 'no reason')}"))
            if not failures[-1]:
                return self.solved(path, goals)
        raise Refused(self.gave_up(failures, search.tries))

    @contextmanager
    def recorded(self, goal: dict[str, float], max_iterations: int, runs: int,
                 planner: PyBulletPlanner | None = None) -> Iterator[dict]:
        """Record a birrt search into `tree`, begun for this try: yields the search's `draw_fn`; {} without a tree.

        - With tamp's `planner`, every check it runs is recorded too (`SearchTree.checked`), by wrapping this
          instance's `check_collision`, as `world_frame_fk` wraps its kinematics; put back on exit.
        - * Tool0s are read in the mirror's own PyBullet world, placed at the robot's base. A check leaves the arms
          where it checked them; the next one sets the searched joints again, so posing them changes nothing.
        """
        if self.tree is None:
            yield {}
            return
        tree, client = self.tree, self.mirror.client
        robot, client_id = client.robot_puid, client.client_id
        joints = [client.robot_joint_puids[name] for name in self.names]
        links = [client.robot_link_puids[f"{part.arm}_tool0"] for part in self.spec.arms]

        def tool0s() -> np.ndarray:
            return np.array([p.getLinkState(robot, link, computeForwardKinematics=True,
                                            physicsClientId=client_id)[4] for link in links])

        def locate(config) -> np.ndarray:
            for joint, value in zip(joints, config):
                p.resetJointState(robot, joint, float(value), physicsClientId=client_id)
            return tool0s()

        start = [float(self.mirror.state.robot_configuration[name]) for name in self.names]
        tree.begin(f"try {self.attempts}/{self.search.tries}", start, [goal[name] for name in self.names], locate,
                   max_iterations, runs)
        if planner is None:
            yield {"draw_fn": tree.draw_fn}
            return
        check = planner.check_collision

        def checking(*args, **options):
            collided = True
            try:
                answer = check(*args, **options)
                collided = False
                return answer
            finally:
                tree.checked(tool0s(), collided)

        had_own = "check_collision" in vars(planner)
        planner.check_collision = checking
        try:
            yield {"draw_fn": tree.draw_fn}
        finally:
            if had_own:
                planner.check_collision = check
            else:
                del planner.check_collision

    def one_arm(self, part: ArmPart, goal: dict[str, float], goals: dict[ArmPart, Pose] | None = None) -> RouteResult:
        """The one-arm search (`arm.plan_arm`), swept-checked; tried again on no path or a swept hit.

        Args:
            part: The moving arm.
            goal: Goal values by joint name, one per name in `names`.
            goals: Moving arm -> the pose its tool0 should end at, to report the end's error; None for a joint goal.

        Returns:
            RouteResult: Solved, with the first path clear between its waypoints.

        Raises:
            Refused: If the start or goal collides or is outside the URDF limits, the search is cancelled, or no try
                gives a clear path (then with the straight path's verdict).
        """
        self.goal_joints = goal
        self.check_limits(goal)
        limits = joint_limits(self.robot.model.urdf, self.names)
        failures = []
        for _ in range(self.search.tries):
            self.next_attempt()
            self.note(f"max_time {SUPPORT_TIME:.0f} s")
            started, found = time.monotonic(), None
            with self.recorded(goal, MAX_ITERATIONS, RESTARTS + 1) as drawn:
                try:
                    found = plan_arm(self.mirror, part.arm, [goal[name] for name in self.names], self.abort,
                                     limits=limits, label=self.label, time_limit=SUPPORT_TIME, **drawn)
                finally:
                    if self.tree is not None:
                        self.tree.end(found=found is not None and found.path is not None)
            self.mirror.sync(self.scene)
            if found.path is None and found.reason.startswith(("target is in collision", "start is in collision")):
                self.refuse_endpoints(goal)
            if found.path is None and not found.reason.startswith("no path"):
                raise Refused(f"{self.planner}: {found.reason}")
            failures.append(self.ended(found.path.points if found.path is not None else None, started, found.reason))
            if not failures[-1]:
                return self.solved(found.path.points, goals)
        raise Refused(self.gave_up(failures, self.search.tries))

    def ended(self, path: Sequence[Sequence[float]] | None, started: float, reason: str) -> str:
        """Log how a try ended; a path is checked between its waypoints when `search.swept_check` is on.

        Args:
            path: The try's waypoints in `names` order, or None.
            started: When the try started, `time.monotonic()`.
            reason: Why there is no path: tamp's reason as "tamp: …", or the one-arm search's whole line.

        Returns:
            str: Why the try is refused, starting "no path" when the search found none; "" when its path is clear.
        """
        seconds = time.monotonic() - started
        if path is None:
            failure = reason if reason.startswith("no path") else f"no path in {seconds:.0f} s ({reason})"
        else:
            self.note(f"path of {len(path)} waypoints in {seconds:.1f} s")
            failure = self.swept_failure(path) if self.search.swept_check else ""
        if failure:
            self.note(failure)
        return failure

    def gave_up(self, failures: Sequence[str], tries: int) -> str:
        """Why the search is refused after its tries; no path in any of them adds the straight path's verdict."""
        reason = f"{self.planner}: {'; '.join(failures)} ({tries} tries)"
        if all(failure.startswith("no path") for failure in failures):
            reason += f"; {self.straight_line(self.goal_joints)}"
        return reason

    def straight_line(self, goal: Mapping[str, float]) -> str:
        """The verdict on a search that found no path: a colliding endpoint, else the straight joint path's first hit.

        Both endpoints are checked in full and the straight path between them swept (`swept_check`); each hit is
        kept (`hit`), so the collisions panel draws the first one.

        Args:
            goal: Goal values by joint name, one per name in `names`.

        Returns:
            str: The verdict, logged too.
        """
        for where, joints in (("start", None), ("goal", goal)):
            pairs = self.endpoint(where, joints)
            if pairs:
                return f"the {where} collides: {pairs_text(pairs)}"
        self.mirror.sync(self.scene)
        start = [self.start[name] for name in self.names]
        report = swept_check(self.mirror, self.names, [start, [goal[name] for name in self.names]])
        self.mirror.sync(self.scene)
        budget = f"{self.search.tries}×{self.search.max_time:.0f} s"
        if report.ok:
            verdict = f"start, goal and the straight path between them are clear: the search found no way in {budget}"
        elif report.error:
            verdict = f"the straight path could not be checked: {report.error}"
        else:
            first = report.colliding_joints
            share = (max(abs(first[name] - self.start[name]) for name in self.names)
                     / max(abs(goal[name] - self.start[name]) for name in self.names))
            pairs = self.hit(f"the straight path at {share:.0%}", first)
            verdict = (f"start and goal clear; the straight path first hits {pairs_text(pairs)} at {share:.0%} — the "
                       f"search found no way around in {budget}")
        self.log(f"{self.step.label}: {verdict}")
        return verdict

    def linear(self, plan: Callable, goals: dict[ArmPart, Pose]) -> RouteResult:
        """One of tamp's straight dual-arm moves, run once on the lent mirror.

        Args:
            plan: `plan(planner, state, report)` -> a JointTrajectory, or None with why in `report`.
            goals: Moving arm -> the pose its tool0 should end at.

        Returns:
            RouteResult: Solved, the path in `names` order.

        Raises:
            Refused: With tamp's reason, the failing waypoint and the nearest colliding step's pairs.
        """
        self.next_attempt()
        report = {}
        with self.mirror.lend() as planner, world_frame_fk(planner):
            trajectory = plan(planner, self.mirror.state, report)
        self.mirror.sync(self.scene)
        if trajectory is None:
            where = f" at waypoint {report['waypoint']}/{report['n_waypoints'] - 1}" if "waypoint" in report else ""
            state = report.get("first_colliding_state")
            pairs = self.hit(f"the nearest step{where}", {
                name: float(state.robot_configuration[name]) for name in self.names}) if state is not None else []
            hit = f", the nearest step collides: {pairs_text(pairs)}" if pairs else ""
            raise Refused(f"{self.planner}: {report.get('reason', 'no path')}{where}{hit}")
        order = [list(trajectory.joint_names).index(name) for name in self.names]
        return self.solved([[point.joint_values[i] for i in order] for point in trajectory.points], goals)

    def swept_failure(self, path: Sequence[Sequence[float]]) -> str:
        """Why a searched path is refused between its waypoints (`swept_check`), or "" if it is clear.

        Args:
            path: Waypoints in `names` order, the start first.

        Returns:
            str: The first bad segment and its pairs (also kept in `colliding`), or "".
        """
        self.swept = swept_check(self.mirror, self.names, path)
        if self.swept.ok:
            self.note(f"swept check: {self.swept.samples} samples between {len(path)} waypoints, clear")
            return ""
        if self.swept.error:
            return f"the swept check failed: {self.swept.error}"
        segment, pairs = self.swept.bad_segments[0]
        found = self.hit(f"segment {segment} of {len(path) - 1}, try {self.attempts}", self.swept.colliding_joints)
        if not found:
            # ? The full check missed what the search check hit: name the swept check's pairs.
            found = self.terms(pairs)
            self.colliding += found
        return f"segment {segment} of {len(path) - 1} sweeps through {pairs_text(found)}"

    def refuse_endpoints(self, goal: Mapping[str, float]) -> None:
        """Stop where a search found its start or goal in collision, naming which one and what it hits.

        Args:
            goal: Goal values by joint name.

        Raises:
            Refused: Always; "the goal collides: …", else "the start collides: …", else too close for the search.
        """
        for where, joints in (("goal", goal), ("start", None)):
            pairs = self.endpoint(where, joints)
            if pairs:
                raise Refused(f"the {where} collides: {pairs_text(pairs)}")
        raise Refused("the start or the goal is too close to something for the search's checks")

    def endpoint(self, where: str, joints: Mapping[str, float] | None) -> list[tuple[str, str]]:
        """Check the start or the goal in full (`hit`), and log whether it is clear or what it collides with.

        Args:
            where: "start" or "goal".
            joints: The goal's values by joint name; None for the start (the synced state).

        Returns:
            list[tuple[str, str]]: The pairs in process ids, empty if clear.
        """
        pairs = self.hit(where, joints)
        self.log(f"{self.step.label}: " + (f"the {where} collides: {pairs_text(pairs)}" if pairs else f"{where} clear"))
        return pairs

    def hit(self, where: str, joints: Mapping[str, float] | None, scene: Scene | None = None) -> list[tuple[str, str]]:
        """Keep what a configuration collides with as a hit, and its pairs in process ids, from the synced mirror.

        ! Leaves the mirror at that configuration (`collision_hit`).

        Args:
            where: Which configuration, e.g. "goal".
            joints: The planned joints changed from the synced state, by name: only pairs with a moving side count;
                None: the synced state, checked in full.
            scene: The scene the mirror is synced to; None: the search scene.

        Returns:
            list[tuple[str, str]]: The pairs, each side in process ids where it maps (also kept in `colliding`);
                empty if clear.
        """
        found = collision_hit(self.mirror, scene or self.scene, where, joints,
                              moved=self.names if joints is not None else None)
        if found is None:
            return []
        self.hits.append(found)
        pairs = self.terms(found.pairs)
        self.colliding += pairs
        return pairs

    def next_attempt(self) -> None:
        """Count one more try, unless planning was cancelled; the mirror synced for it.

        Raises:
            Refused: "cancelled", when `abort` is set.
        """
        if self.abort.is_set():
            raise Refused("cancelled")
        self.attempts += 1
        self.mirror.sync(self.scene)

    def note(self, text: str, tries: int | None = None) -> None:
        """Log a line of the try running, so a long search shows its progress.

        Args:
            text: What happened, e.g. "no path in 5 s (tamp: birrt_failed)".
            tries: How many tries the search makes; None: the free searches' (`search.tries`).
        """
        self.log(f"{self.step.label}: {self.planner} try {self.attempts}/{tries or self.search.tries}: {text}")

    # --- --- --- --- --- RESULTS --- --- --- --- ---

    def solved(self, path: Sequence[Sequence[float]], goals: dict[ArmPart, Pose] | None = None) -> RouteResult:
        """The solved result of a path in `names` order, with how close its end comes to the goal.

        Args:
            path: Waypoints in `names` order, the start first.
            goals: Moving arm -> the pose its tool0 should end at, or None.

        Returns:
            RouteResult: Status "solved"; the start marked overridden where it moved off the authored one.
        """
        rows = np.asarray(path, dtype=float)
        end = dict(zip(self.names, rows[-1].tolist()))
        end_error = {part: pose_error(self.flange(part, end), goal) for part, goal in (goals or {}).items()}
        joint_error = max(abs(end[name] - self.goal_joints[name]) for name in self.names) \
            if self.spec.goal in _JOINT_GOALS and self.goal_joints is not None else None
        state = self.step.movement.start.robots.get(self.spec.robot)
        authored = state.joints if state is not None else None
        overridden = authored is not None and any(abs(authored[name] - value) > CHAIN_TOLERANCE
                                                  for name, value in self.start.items() if name in authored)
        result = MovementResult("solved", start={self.spec.robot: dict(self.start)}, start_overridden=overridden,
                                end={self.spec.robot: end},
                                trajectory=Trajectory(self.spec.robot, tuple(self.names),
                                                      tuple(tuple(row) for row in rows.tolist())))
        return self.route(result, len(rows), {part.flange: error for part, error in end_error.items()}, joint_error,
                          {part.flange: goal for part, goal in (goals or {}).items()})

    def failed(self, reason: str) -> RouteResult:
        """The failed result, with the start it was planned from.

        Args:
            reason: Why it failed.

        Returns:
            RouteResult: Status "failed", with what was found so far.
        """
        start = {self.spec.robot: dict(self.start)} if self.start else {}
        return self.route(MovementResult("failed", reason, start=start))

    def route(self, result: MovementResult, waypoints: int = 0, end_error: dict | None = None,
              joint_error: float | None = None, goals: dict[str, Pose] | None = None) -> RouteResult:
        """A RouteResult with what this route found; `seconds` is filled in by `plan_movement`.

        Args:
            result: The core's result.
            waypoints: Rows of the path.
            end_error: Process flange id -> (metres, radians) from its goal pose.
            joint_error: The largest gap to a joint goal, radians, or None.
            goals: Process flange id -> the world pose its tool0 was planned to end at, or None.

        Returns:
            RouteResult: The result with the planner, tries, goal, pairs, checks and warnings.
        """
        return RouteResult(result, self.spec.routine, self.planner, self.attempts, 0.0, waypoints, end_error or {},
                           joint_error, self.goal_joints, tuple(dict.fromkeys(self.colliding)), self.swept, self.ik,
                           tuple(self.warnings), tuple(self.hits), dict(goals or {}))

    def terms(self, pairs: Sequence[tuple[str, str]]) -> list[tuple[str, str]]:
        """Colliding pairs in process ids, where mapped.

        Args:
            pairs: Pairs of scene ids.

        Returns:
            list[tuple[str, str]]: The same pairs, each side in process ids where it maps.
        """
        return [in_process_terms(pair, self.planning.process_ids) for pair in pairs]

    def label(self, live: str) -> str:
        """A scene id in process terms, for the one-arm search's reasons.

        Args:
            live: A scene id.

        Returns:
            str: Its process id, or itself where it does not map.
        """
        return self.planning.process_ids.get(live) or live


def pairs_text(pairs: Sequence[tuple[str, str]]) -> str:
    """Colliding pairs as text, e.g. "a with b; c with d"."""
    return "; ".join(f"{a} with {b}" for a, b in pairs)
