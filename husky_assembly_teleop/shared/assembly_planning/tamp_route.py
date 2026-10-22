"""
Plan one process movement on the real robots: tamp's dual-arm planners, or one arm's search or line, by its routine.

The movement is classified, its planning scene made from the snapshot with the contacts of its end allowed, the goal
solved at the live base, and every searched path checked between its waypoints. `check_goal` only solves the goal.

- ! Worker thread only, on the thread that created the mirror; the mirror plans for the movement's robot.
- ! tamp and compas_fab borrow the mirror (`lend`) and leave it anywhere: it is synced again after each borrow, after
  IK, and before every check. tamp can't be cancelled: `abort` is checked between tries.
- Any error gives a failed result with its text: a path is never accepted unchecked.
- The searches, checks and results are `route_search.RouteSearch`'s; this module picks them by routine.
"""

from __future__ import annotations

import threading
import time
from dataclasses import replace
from typing import TYPE_CHECKING, Callable, Mapping

from husky_assembly_tamp.motion_planner.api import (_arm_joint_names, plan_constrained_dual_arm,
                                                    plan_constrained_dual_arm_linear, plan_dual_arm_linear_independent)

from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.mirrors.compas import frame_from_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.process.solutions import MovementResult
from scaffolding_assembly_core.scene import Scene

from ...world.kinematics import stow_joints
from .process_steps import LoadedProcess, Step
from .collision import collisions, world_frame_fk
from .ik import solve_ik
from .joints import arm_joint_names, joint_limits, unwrap_to_nearest
from .movement import GoalSource, MovementSpec, Routine, classify
from .route_result import GoalCheck, RouteResult, aims_at_line, line_goal, pose_error
from .route_search import ATTEMPTS, Refused, RouteSearch, pairs_text
from .scene_for import RobotBelief, scene_for
from .single_arm import plan_linear

if TYPE_CHECKING:
    from ...config import RobotConfig

#: tamp's free search for a transfer and the travel to load, and for the way home: seconds and iterations per try,
#: joint step of its checks (rad).
FAR = {"max_time": 120.0, "max_iterations": 50, "joint_resolution": 0.1}
HOME = {"max_time": 30.0, "joint_resolution": 0.1}
#: tamp's search with the bar held: the bar's step (m, rad), seconds per try; try k is seeded SEED_STEP * k.
HELD = {"stage": 3, "position_res": 0.01, "rotation_res": 0.025, "max_time": 120.0, "derive_start": False}
SEED_STEP = 9973
#: A held transfer's right flange may end this far from its target before the grasp counts as mismatched, metres.
GRASP_MISMATCH = 0.002
#: Routines no arm moves in, and the linear ones, whose start may touch (warned, not refused).
_NO_ARM = (Routine.TOOL_ONLY, Routine.OPERATOR, Routine.BASE_ONLY)
_LINEAR = (Routine.INSERT, Routine.RETREAT_LINEAR, Routine.SUPPORT_LINEAR)
#: Routines whose goal is solved by IK at the live base (`check_goal`).
IK_GOALS = (Routine.TRANSFER_FREE, Routine.TRANSFER_HELD, Routine.SUPPORT_APPROACH)


def plan_movement(mirror: CompasFabMirror, loaded: LoadedProcess, step: Step, snapshot: Scene,
                  robots: tuple[RobotConfig, ...], *, transfer_as_free: bool = False,
                  beliefs: Mapping[str, RobotBelief] | None = None, ignore_built: bool = False,
                  obstacles_collide: bool = True, goal_joints: Mapping[str, float] | None = None,
                  abort: threading.Event | None = None, log: Callable[[str], None] = print) -> RouteResult:
    """Plan one movement from where the snapshot has its robot, by its routine (`classify`).

    Args:
        mirror: The acting robot's mirror, on this thread.
        loaded: The loaded process.
        step: The movement's step.
        snapshot: The world to plan in; the acting robot's joints in it are the start.
        robots: The configured robots.
        transfer_as_free: The transfer workaround: the coupled free movement runs without the bar.
        beliefs: Live robot id -> where an unmeasured robot is believed to be (`scene_for`).
        ignore_built: Plan as if the built bars were not there (`scene_for`).
        obstacles_collide: Whether the process's obstacles from Rhino count (`scene_for`).
        goal_joints: Arm joints to plan to instead of solving the goal: a transfer, a travel to load, an approach.
        abort: Set from another thread to stop between tries.
        log: Gets the IK and linear fallback lines.

    Returns:
        RouteResult: Solved, failed with the reason, or not planned (no arm moves, no goal yet).

    Raises:
        ValueError: If the mirror plans for another robot than the movement's, or `scene_for` refuses the step.
    """
    started = time.monotonic()
    spec = classify(loaded.process, step.action, step.movement, transfer_as_free)
    if spec.routine in _NO_ARM:
        return _not_planned(spec, "no arm moves", started)
    if spec.routine is Routine.FREE_TO_LOAD and goal_joints is None and spec.goal_joints is None:
        return _not_planned(spec, "needs the transfer start, after M3", started)
    route = _route(mirror, loaded, step, snapshot, robots, spec, beliefs=beliefs, ignore_built=ignore_built,
                   obstacles_collide=obstacles_collide, goal_joints=goal_joints, abort=abort or threading.Event(),
                   log=log)
    try:
        result = route.run()
    except Refused as refused:
        result = route.failed(str(refused))
    except Exception as error:
        # ! Fail closed: whatever went wrong, nothing is solved.
        result = route.failed(f"{route.planner or spec.routine.value} raised {error!r}")
        problem = _sync_again(route)
        if problem:
            result = replace(result, warnings=(*result.warnings, problem))
    return replace(result, seconds=time.monotonic() - started)


def check_goal(mirror: CompasFabMirror, loaded: LoadedProcess, step: Step, snapshot: Scene,
               robots: tuple[RobotConfig, ...], *, transfer_as_free: bool = False,
               beliefs: Mapping[str, RobotBelief] | None = None, ignore_built: bool = False,
               obstacles_collide: bool = True, log: Callable[[str], None] = print) -> GoalCheck:
    """Solve a movement's goal by IK at the live base, as `plan_movement` would before searching, and plan nothing.

    Only the transfers and a support robot's approach have a goal solved by IK. Fails closed: any error gives a
    failed check with its text.

    Args:
        mirror: The acting robot's mirror, on this thread.
        loaded: The loaded process.
        step: The movement's step.
        snapshot: The world; the acting robot's base and joints in it are the base and the start.
        robots: The configured robots.
        transfer_as_free: The transfer workaround (`classify`).
        beliefs: Live robot id -> where an unmeasured robot is believed to be (`scene_for`).
        ignore_built: Solve as if the built bars were not there (`scene_for`).
        obstacles_collide: Whether the process's obstacles from Rhino count (`scene_for`).
        log: Gets the IK fallback line.

    Returns:
        GoalCheck: The goal and how it was found, or why there is none.
    """
    started = time.monotonic()
    movement, route = step.movement.id, None
    try:
        spec = classify(loaded.process, step.action, step.movement, transfer_as_free)
        if spec.routine not in IK_GOALS:
            return GoalCheck(movement, spec.routine, False, "goal not solved by IK", seconds=time.monotonic() - started)
        route = _route(mirror, loaded, step, snapshot, robots, spec, beliefs=beliefs, ignore_built=ignore_built,
                       obstacles_collide=obstacles_collide, goal_joints=None, abort=threading.Event(), log=log)
        route.prepare()
        goal, reason = route.ik_goal(unwrap=spec.routine is not Routine.SUPPORT_APPROACH), ""
    except Refused as refused:
        goal, reason = None, str(refused)
    except Exception as error:
        # ! Fail closed: whatever went wrong, there is no goal.
        goal, reason = None, f"the goal check raised {error!r}"
        problem = _sync_again(route) if route is not None else ""
        reason += f"; {problem}" if problem else ""
    if route is None:
        return GoalCheck(movement, None, False, reason, seconds=time.monotonic() - started)
    authored = route.spec.goal_joints or {}
    return GoalCheck(movement, route.spec.routine, goal is not None, reason, route.planning.acting, route.robot.base,
                     dict(route.start) or None, route.ik, goal,
                     {name: float(authored[name]) for name in route.names if name in authored} or None,
                     dict(route.planning.sources), tuple(route.warnings), time.monotonic() - started)


def _route(mirror: CompasFabMirror, loaded: LoadedProcess, step: Step, snapshot: Scene,
           robots: tuple[RobotConfig, ...], spec: MovementSpec, *, beliefs: Mapping[str, RobotBelief] | None,
           ignore_built: bool, obstacles_collide: bool, goal_joints: Mapping[str, float] | None,
           abort: threading.Event, log: Callable[[str], None]) -> _Route:
    """A movement's route: its planning scene, and the scene its searches run in (without the bar for the workaround).

    Raises:
        ValueError: If the mirror plans for another robot than the movement's, or `scene_for` refuses the step.
    """
    options = {"beliefs": beliefs, "ignore_built": ignore_built, "obstacles_collide": obstacles_collide,
               "end_contacts": True}
    planning = scene_for(loaded, step, snapshot, robots, **options)
    if mirror.robot_id != planning.acting:
        raise ValueError(f"{step.label}: the mirror plans for {mirror.robot_id}, the movement is {planning.acting}'s")
    # * The workaround searches without the bar, but solves its goal with the bar held there.
    search = scene_for(loaded, step, snapshot, robots, hide_held=True, **options).scene \
        if spec.routine is Routine.TRANSFER_FREE else planning.scene
    config = next(config for config in robots if robot_id(config.serial) == planning.acting)
    return _Route(mirror, loaded.process, step, spec, planning, search, config, goal_joints, abort, log)


def _sync_again(route: _Route) -> str:
    """Sync a route's mirror after an error left it anywhere; why it failed, or "" if it synced."""
    try:
        route.mirror.sync(route.scene)
    except Exception as again:
        return f"the mirror did not sync again: {again!r}"
    return ""


def _not_planned(spec: MovementSpec, reason: str, started: float) -> RouteResult:
    """A movement left unplanned, and why."""
    return RouteResult(MovementResult("not_planned", reason), spec.routine, seconds=time.monotonic() - started,
                       warnings=spec.warnings)


class _Route(RouteSearch):
    """One movement planned by its routine, after its start is checked once."""

    # --- --- --- --- --- ROUTINES --- --- --- --- ---

    def prepare(self) -> None:
        """Sync the mirror to the search scene, and take the planned joints' names and their start values."""
        self.mirror.sync(self.scene)
        dual = self.spec.routine not in (Routine.SUPPORT_APPROACH, Routine.SUPPORT_LINEAR, Routine.SUPPORT_LEAVE)
        # ? tamp's 12-joint order: left arm, then right.
        self.names = [name for arm in _arm_joint_names(self.mirror.cell) for name in arm] if dual \
            else arm_joint_names(self.spec.arms[0].arm)
        self.start = {name: float(self.robot.joints[name]) for name in self.names}

    def run(self) -> RouteResult:
        """Check the start once, then plan by the routine.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not: a free move's start collides, or the routine's own reason.
        """
        self.prepare()
        hits = self.terms(collisions(self.mirror, full_report=True))
        if hits:
            if self.spec.routine not in _LINEAR:
                self.colliding += hits
                raise Refused(f"the start collides: {pairs_text(hits)}")
            self.warnings.append(f"the start touches: {pairs_text(hits)}")
        return getattr(self, f"_{self.spec.routine.value}")()

    def _transfer_free(self) -> RouteResult:
        """The transfer without the bar: tamp's free search to the insertion start solved with the bar held.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        self.planner = "tamp.plan_free_dual_arm"
        return self.free(self.ik_goal(), FAR, {part: part.target for part in self.spec.arms})

    def _free_to_load(self) -> RouteResult:
        """The travel to load: tamp's free search to the transfer's start.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        self.planner = "tamp.plan_free_dual_arm"
        goal = self.joints_of(self.override or self.spec.goal_joints)
        self.check_limits(goal)
        return self.free(goal, FAR)

    def _free_home(self) -> RouteResult:
        """The way home: tamp's free search to the process's target joints, exactly (never turned by whole turns).

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        self.planner = "tamp.plan_free_dual_arm"
        goal = self.joints_of(self.spec.goal_joints)
        self.check_limits(goal)
        return self.free(goal, HOME)

    def _transfer_held(self) -> RouteResult:
        """The transfer with the bar held: tamp's constrained search, each try seeded anew.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        self.planner = "tamp.plan_constrained_dual_arm"
        goal, bar = self.ik_goal(), self.planning.ids(self.spec.bar)
        failures = []
        for attempt in range(ATTEMPTS):
            self.next_attempt()
            with self.mirror.lend() as planner, world_frame_fk(planner):
                path, info = plan_constrained_dual_arm(planner, self.mirror.state, active_bar_id=bar, goal_conf=goal,
                                                       random_seed=SEED_STEP * attempt, **HELD)
            self.mirror.sync(self.scene)
            if path is not None:
                right = next((part for part in self.spec.arms if part.side == "right"), None)
                if right is not None and right.target is not None:
                    missed = pose_error(self.flange(right, dict(zip(self.names, path[-1]))), right.target)[0]
                    if missed > GRASP_MISMATCH:
                        self.warnings.append(f"the right flange ends {missed * 1e3:.1f} mm from its target: the "
                                             f"grasp differs from the authored one")
                return self.solved(path, {part: part.target for part in self.spec.arms})
            failures.append(str(info.get("failure_reason", "no path")))
        raise Refused(f"{self.planner}: {'; '.join(failures)} ({ATTEMPTS} tries of {HELD['max_time']:.0f} s)")

    def _insert(self) -> RouteResult:
        """The insert: tamp's straight move with the bar held, to the target poses, else along the line.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        self.planner = "tamp.plan_constrained_dual_arm_linear"
        goals = {part: part.target for part in self.spec.arms}
        if None in goals.values():
            goals = self.line_goals()
        frames = {part.side: frame_from_pose(goal) for part, goal in goals.items()}
        bar = self.planning.ids(self.spec.bar)
        return self.linear(lambda planner, state, report: plan_constrained_dual_arm_linear(
            planner, state, active_bar_id=bar, goal_ee_frames=frames, skip_env_collisions=False, report=report), goals)

    def _retreat_linear(self) -> RouteResult:
        """The retreat: each flange along its line from where it is, tamp's straight move per arm.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        self.planner = "tamp.plan_dual_arm_linear_independent"
        goals = self.line_goals()
        frames = {part.side: frame_from_pose(goal) for part, goal in goals.items()}
        return self.linear(lambda planner, state, report: plan_dual_arm_linear_independent(
            planner, state, goal_ee_frames=frames, skip_env_collisions=False, report=report), goals)

    def _support_approach(self) -> RouteResult:
        """A support robot's approach: its target pose solved by IK at the live base, then the one-arm search.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        self.planner = "arm.plan_arm"
        part = self.spec.arms[0]
        return self.one_arm(part, self.ik_goal(unwrap=False), {part: part.target})

    def _support_linear(self) -> RouteResult:
        """A support robot's straight move: along the line when it leaves a contact, else to its target pose.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        part = self.spec.arms[0]
        if part.line is not None and (part.target is None or aims_at_line(self.process, self.step.movement, self.spec)):
            goal = line_goal(self.flange(part), part.line)
        elif part.target is not None:
            goal = part.target
        else:
            raise Refused(f"{part.flange} has neither a line nor a target pose")
        self.next_attempt()
        path = plan_linear(self.mirror, self.robot, part.arm, goal, log=self.log)
        self.mirror.sync(self.scene)
        self.planner = f"single_arm.plan_linear/{path.backend}"
        if path.points is None:
            self.colliding += self.terms([path.colliding] if path.colliding else [])
            raise Refused(f"{self.planner}: {path.reason}")
        return self.solved(path.points, {part: goal})

    def _support_leave(self) -> RouteResult:
        """A support robot leaving: the one-arm search to the next start, the target joints, or its stowed arm.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: Why it is not, as the failed result's reason.
        """
        self.planner = "arm.plan_arm"
        part = self.spec.arms[0]
        given = stow_joints(self.config, self.names) if self.spec.goal is GoalSource.STOW else self.spec.goal_joints
        return self.one_arm(part, self.joints_of(given))

    # --- --- --- --- --- GOALS --- --- --- --- ---

    def ik_goal(self, unwrap: bool = True) -> dict[str, float]:
        """The arms' joints at their target poses, by IK at the live base with anything held there; or the override.

        Args:
            unwrap: Turn each goal joint by whole turns next to the start, within the URDF limits (`unwrap_to_nearest`),
                and refuse a goal outside them; the dual-arm routines do, the approach does not.

        Returns:
            dict[str, float]: Goal values by joint name, one per name in `names`.

        Raises:
            Refused: If a flange has no target pose, IK finds no collision-free solution, or a joint is outside its
                limits.
        """
        if self.override is not None:
            goal = self.joints_of(self.override)
        else:
            targets = {part.arm: part.target for part in self.spec.arms}
            if None in targets.values():
                raise Refused("a moving flange has no target pose to solve the goal for")
            # * IK in the scene with the held bar: it must be clear where the goal puts it.
            self.mirror.sync(self.planning.scene)
            self.ik = solve_ik(self.mirror, self.robot, targets, reference=self.spec.goal_joints, log=self.log)
            self.mirror.sync(self.scene)
            if self.ik.joints is None:
                self.colliding += self.terms([self.ik.colliding] if self.ik.colliding else [])
                raise Refused(f"no IK for the goal ({self.ik.backend}): {self.ik.reason}")
            goal = self.joints_of(self.ik.joints)
        if unwrap:
            values = unwrap_to_nearest([goal[name] for name in self.names], [self.start[name] for name in self.names],
                                       *joint_limits(self.robot.model.urdf, self.names))
            goal = dict(zip(self.names, values.tolist()))
            self.check_limits(goal)
        return goal
