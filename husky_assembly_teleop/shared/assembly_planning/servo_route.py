"""
One servo pass on the motion planner: IK at the live base for the pass's targets, then the correction move.

The step is the insert (the servo before it, at the transfer's end, or after it) or a support robot's approach. The
move is tamp's straight line with the bar held, its sampling search once when no line is found and sampling is
allowed, or one arm's two-row joint move, swept-checked. A correction under `ask.stop_below` plans nothing.

- ! Worker thread only, on the thread that created the mirror (`tamp_route`'s rules); the mirror plans for the
  step's robot.
- Any error gives a failed step with its text: a path is never accepted unchecked.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Callable, Mapping

import numpy as np
from husky_assembly_tamp.motion_planner.api import plan_constrained_dual_arm, plan_constrained_dual_arm_linear

from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.mirrors.compas import frame_from_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.process.solutions import MovementResult

from .process_steps import LoadedProcess, Step
from .collision import collisions, world_frame_fk
from .ik import solve_ik
from .joints import joint_limits, unwrap_to_nearest
from .movement import classify
from .route_result import RouteResult, pose_error
from .route_search import Refused, pairs_text
from .scene_for import RobotBelief, scene_for
from .tamp_route import GRASP_MISMATCH, HELD, _Route, _sync_again

if TYPE_CHECKING:
    from scaffolding_assembly_core.scene import Scene

    from ...config import RobotConfig
    from ..motion_planner_contract import ServoAsk

#: The one-arm correction's planner name: the two rows [live, goal] as one joint move (W:6154-6156).
TWO_ROWS = "servo.two_rows"


@dataclass(frozen=True)
class ServoStep:
    """What planning one servo pass gave.

    Attributes:
        route: The correction move, as a planned movement; "not_planned" when nothing is left to plan.
        correction: Process flange id -> how far the move takes it, metres (W:1868-1898); empty when it failed
            before IK.
        nothing_left: Every correction is under the ask's `stop_below`: no move was planned, the pass is done.
    """

    route: RouteResult
    correction: dict[str, float]
    nothing_left: bool = False

    @property
    def ok(self) -> bool:
        """bool: Whether the pass has a move to send, or nothing left to plan."""
        return self.route.ok or self.nothing_left


def plan_servo_step(mirror: CompasFabMirror, loaded: LoadedProcess, step: Step, snapshot: Scene,
                    robots: tuple[RobotConfig, ...], ask: ServoAsk, *, transfer_as_free: bool = False,
                    beliefs: Mapping[str, RobotBelief] | None = None, ignore_built: bool = False,
                    obstacles_collide: bool = True, abort: threading.Event | None = None,
                    log: Callable[[str], None] = print) -> ServoStep:
    """Plan one servo pass from where the snapshot has the step's robot: IK for the targets, then the correction.

    Args:
        mirror: The acting robot's mirror, on this thread.
        loaded: The loaded process.
        step: The step the pass aims at: the insert, or a support robot's approach.
        snapshot: The world to plan in; the acting robot's base and joints in it are the live base and the start.
        robots: The configured robots.
        ask: The targets and how to move to them.
        transfer_as_free: The transfer workaround (`classify`); the step's routine does not change with it.
        beliefs: Live robot id -> where an unmeasured robot is believed to be (`scene_for`).
        ignore_built: Plan as if the built bars were not there (`scene_for`).
        obstacles_collide: Whether the process's obstacles from Rhino count (`scene_for`).
        abort: Set from another thread to stop before a search.
        log: Gets the IK fallback line.

    Returns:
        ServoStep: The move and each flange's correction, nothing left to plan, or a failed route with the reason.

    Raises:
        ValueError: If the mirror plans for another robot than the step's, or `scene_for` refuses the step.
    """
    started = time.monotonic()
    spec = classify(loaded.process, step.action, step.movement, transfer_as_free)
    # * The planning scene itself is searched: the bar is in the tools, so it is never hidden.
    planning = scene_for(loaded, step, snapshot, robots, beliefs=beliefs, ignore_built=ignore_built,
                         obstacles_collide=obstacles_collide, end_contacts=True)
    if mirror.robot_id != planning.acting:
        raise ValueError(f"{step.label}: the mirror plans for {mirror.robot_id}, the step is {planning.acting}'s")
    config = next(config for config in robots if robot_id(config.serial) == planning.acting)
    route = ServoRoute(mirror, loaded.process, step, spec, planning, planning.scene, config, None,
                       abort or threading.Event(), log)
    correction, nothing_left = {}, False
    try:
        route.check_start()
        goal = route.servo_goal(ask)
        correction = route.correction_of(goal)
        if max(correction.values()) < ask.stop_below:
            result, nothing_left = route.route(MovementResult("not_planned", "nothing left to plan")), True
        else:
            result = route.held_move(ask, goal) if ask.held else route.one_arm_move(ask, goal)
    except Refused as refused:
        result = route.failed(str(refused))
    except Exception as error:
        # ! Fail closed: whatever went wrong, nothing is solved.
        result = route.failed(f"{route.planner or spec.routine.value} raised {error!r}")
        problem = _sync_again(route)
        if problem:
            result = replace(result, warnings=(*result.warnings, problem))
    return ServoStep(replace(result, seconds=time.monotonic() - started), correction, nothing_left)


class ServoRoute(_Route):
    """One servo pass: the goal by IK at the live base from the live joints, and the correction move to it."""

    def check_start(self) -> None:
        """Sync and take the start (`prepare`); a touching start is a warning only: the bar may stand on the floor."""
        self.prepare()
        touches = self.terms(collisions(self.mirror, full_report=True))
        if touches:
            self.warnings.append(f"the start touches: {pairs_text(touches)}")

    # --- --- --- --- --- GOAL --- --- --- --- ---

    def servo_goal(self, ask: ServoAsk) -> dict[str, float]:
        """The arms' joints at the pass's targets, by IK at the live base, turned by whole turns next to the start.

        Args:
            ask: The pass; `targets` must name every moving flange of the step.

        Returns:
            dict[str, float]: Goal values by joint name, one per name in `names`.

        Raises:
            Refused: If a moving flange has no target, IK finds no collision-free solution, or a joint is outside
                its limits.
        """
        missing = [part.flange for part in self.spec.arms if part.flange not in ask.targets]
        if missing:
            raise Refused(f"no target for {', '.join(missing)}")
        targets = {part.arm: ask.targets[part.flange] for part in self.spec.arms}
        # ! The reference is the LIVE joints (W:3553-3555), not the authored goal `ik_goal` stays near.
        self.ik = solve_ik(self.mirror, self.robot, targets, log=self.log)
        if self.ik.joints is None and self.ik.colliding_joints is not None:
            self.mirror.sync(self.scene)  # ? IK may have lent the mirror
            self.hit("goal by IK", self.ik.colliding_joints)
        self.mirror.sync(self.scene)
        if self.ik.joints is None:
            raise Refused(f"no IK for the servo target ({self.ik.backend}): {self.ik.reason}")
        goal = self.joints_of(self.ik.joints)
        # * Unwrapped to the turn nearest the live joints within the URDF limits (M:5534-5544).
        values = unwrap_to_nearest([goal[name] for name in self.names], [self.start[name] for name in self.names],
                                   *joint_limits(self.robot.model.urdf, self.names))
        goal = dict(zip(self.names, values.tolist()))
        self.check_limits(goal)
        self.goal_joints = goal
        return goal

    def correction_of(self, goal: Mapping[str, float]) -> dict[str, float]:
        """How far the goal moves each flange from where it is, metres: a pose question, never joints (W:1868-1898).

        Args:
            goal: Goal values by joint name, one per name in `names`.

        Returns:
            dict[str, float]: Process flange id -> distance between its tool0 at the start and at the goal.
        """
        return {part.flange: float(np.linalg.norm(np.subtract(self.flange(part, goal).position,
                                                              self.flange(part).position)))
                for part in self.spec.arms}

    # --- --- --- --- --- MOVES --- --- --- --- ---

    def held_move(self, ask: ServoAsk, goal: dict[str, float]) -> RouteResult:
        """The correction with the bar in both tools: tamp's straight line to the targets, else its sampling search.

        The line interpolates the bar from where it is to the targets, so the grasp really held stays
        (M:5545-5560). With no line and sampling allowed, one try of the sampling search to the goal joints
        (M:5580-5597): it explores, so the result warns.

        Args:
            ask: The pass; `linear_only` forbids the sampling search.
            goal: Goal values by joint name, one per name in `names`.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: No line and `linear_only` (W:2208-2214), the sampling search finds no path, or cancelled.
        """
        self.planner = "tamp.plan_constrained_dual_arm_linear"
        goals = {part: ask.targets[part.flange] for part in self.spec.arms}
        frames = {part.side: frame_from_pose(pose) for part, pose in goals.items()}
        bar = self.planning.ids(self.spec.bar)
        try:
            return self.linear(lambda planner, state, report: plan_constrained_dual_arm_linear(
                planner, state, active_bar_id=bar, goal_ee_frames=frames, skip_env_collisions=False, report=report),
                goals)
        except Refused as no_line:
            if self.abort.is_set():
                raise
            if ask.linear_only:
                # ! The bar stands on the floor: the sampling search explores, which would drag it (W:2208-2214).
                raise Refused(f"{no_line}; straight lines only after a grounded insert: nothing is planned")
            self.warnings.append(f"no straight line ({no_line}): the sampling search ran instead; it explores, so "
                                 f"check the travel before confirming")
        self.planner = "tamp.plan_constrained_dual_arm"
        self.next_attempt()
        with self.mirror.lend() as planner, world_frame_fk(planner):
            path, info = plan_constrained_dual_arm(planner, self.mirror.state, active_bar_id=bar, goal_conf=goal,
                                                   random_seed=0, **HELD)
        self.mirror.sync(self.scene)
        if path is None:
            raise Refused(f"{self.planner}: {info.get('failure_reason', 'no path')} (1 try of "
                          f"{HELD['max_time']:.0f} s)")
        right = next((part for part in self.spec.arms if part.side == "right"), None)
        if right is not None:
            missed = pose_error(self.flange(right, dict(zip(self.names, path[-1]))), goals[right])[0]
            if missed > GRASP_MISMATCH:
                self.warnings.append(f"the right flange ends {missed * 1e3:.1f} mm from its target: the grasp "
                                     f"differs from the authored one")
        return self.solved(path, goals)

    def one_arm_move(self, ask: ServoAsk, goal: dict[str, float]) -> RouteResult:
        """One arm's correction: the rows [live, goal] as one joint move (W:6154-6156), swept-checked (A6).

        Args:
            ask: The pass, for the target the end is measured against.
            goal: Goal values by joint name, one per name in `names`.

        Returns:
            RouteResult: Solved, two rows.

        Raises:
            Refused: If the move sweeps through something, or cancelled.
        """
        self.planner = TWO_ROWS
        rows = [[self.start[name] for name in self.names], [goal[name] for name in self.names]]
        self.next_attempt()
        problem = self.swept_failure(rows)
        if problem:
            raise Refused(f"{self.planner}: {problem}")
        return self.solved(rows, {part: ask.targets[part.flange] for part in self.spec.arms})
