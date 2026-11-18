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
from husky_assembly_tamp.motion_planner.api import plan_constrained_dual_arm_linear

from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.mirrors.compas import frame_from_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.process.solutions import MovementResult

from .process_steps import LoadedProcess, Step
from .collision import collisions
from .movement import classify
from .route_result import RouteResult
from .route_search import Refused, pairs_text
from .scene_for import scene_for
from .tamp_route import HELD, Route

if TYPE_CHECKING:
    from scaffolding_assembly_core.scene import Scene

    from ...config import RobotConfig
    from ..motion_planner_contract import ServoAsk

#: The one-arm correction's planner name: the two rows [live, goal] as one joint move.
TWO_ROWS = "servo.two_rows"


@dataclass(frozen=True)
class ServoStep:
    """What planning one servo pass gave.

    Attributes:
        route: The correction move, as a planned movement; "not_planned" when nothing is left to plan.
        correction: Process flange id -> how far the move takes it, metres; empty when it failed
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
                    ignore_built: bool = False, obstacles_collide: bool = True, abort: threading.Event | None = None,
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
    planning = scene_for(loaded, step, snapshot, robots, ignore_built=ignore_built,
                         obstacles_collide=obstacles_collide)
    if mirror.robot_id != planning.acting:
        raise ValueError(f"{step.label}: the mirror plans for {mirror.robot_id}, the step is {planning.acting}'s")
    config = next(config for config in robots if robot_id(config.serial) == planning.acting)
    route = ServoRoute(mirror, loaded.process, step, spec, planning, planning.scene, config, None,
                       abort or threading.Event(), log)
    correction, nothing_left = {}, False

    def plan() -> RouteResult:
        """The start checked, the goal by IK, then the correction move, or nothing when the correction is small."""
        nonlocal correction, nothing_left
        route.check_start()
        goal = route.servo_goal(ask)
        correction = route.correction_of(goal)
        if max(correction.values()) < ask.stop_below:
            result = route.route(MovementResult("not_planned", "nothing left to plan"))
            nothing_left = True
            return result
        return route.held_move(ask, goal) if ask.held else route.one_arm_move(ask, goal)

    result = route.planned(plan)
    return ServoStep(replace(result, seconds=time.monotonic() - started), correction, nothing_left)


class ServoRoute(Route):
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
        # ! Solved near the LIVE joints, not the authored goal, and unwrapped next to them.
        goal = self.ik_goal(targets={part.arm: ask.targets[part.flange] for part in self.spec.arms})
        self.goal_joints = goal
        return goal

    def correction_of(self, goal: Mapping[str, float]) -> dict[str, float]:
        """How far the goal moves each flange from where it is, metres: a pose question, never joints.

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

        The line interpolates the bar from where it is to the targets, so the grasp really held stays. With no line
        and sampling allowed, one try of the sampling search to the goal joints: it explores, so the result warns.

        Args:
            ask: The pass; `linear_only` forbids the sampling search.
            goal: Goal values by joint name, one per name in `names`.

        Returns:
            RouteResult: Solved.

        Raises:
            Refused: No line and `linear_only`, the sampling search finds no path, or cancelled.
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
                # ! The bar stands on the floor: the sampling search explores, which would drag it.
                raise Refused(f"{no_line}; straight lines only after a grounded insert: nothing is planned")
            self.warnings.append(f"no straight line ({no_line}): the sampling search ran instead; it explores, so "
                                 f"check the travel before confirming")
        self.planner = "tamp.plan_constrained_dual_arm"
        self.next_attempt()
        path, info = self.held_search(goal, 0, goals)
        if path is None:
            raise Refused(f"{self.planner}: {info.get('failure_reason', 'no path')} (1 try of "
                          f"{HELD['max_time']:.0f} s)")
        return self.solved(path, goals)

    def one_arm_move(self, ask: ServoAsk, goal: dict[str, float]) -> RouteResult:
        """One arm's correction: the rows [live, goal] as one joint move, swept-checked.

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
