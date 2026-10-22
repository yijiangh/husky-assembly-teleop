"""
The operator's step panel: the run's current step, its goal checked, its motion planned and previewed, then Exec.

The current step follows the run (`assembly_run`): its action, the step holding its movement (`operator_steps`, the
run's transfer workaround). Plan asks the motion planner from the live robot (only when it is live); the plan goes
stale when anything it was made from changes (`shared/execution/basis.py`). Exec hands the step to `movement_exec`,
which asks the operator to Confirm each part; once every part is done the next step becomes current, and after the
action's last step Mark done is offered.

- Check goal: the goal by IK at the live base, logged, shown as the target ghost and recorded; a plan of the same step
  reuses it while the base stays within 2 mm / 0.5° and the process plugin's "Rhino obstacles collide" is unchanged.
- A plan is used once: Exec takes it, and it is cleared once the step ran; only a request refused before sending
  anything, or a dry run, gives it back (still checked for staleness).
- ! EXPERIMENTAL, with movement_exec the first plugins that move robots.
"""

from __future__ import annotations

import asyncio
import math
import threading
from typing import TYPE_CHECKING

from scaffolding_assembly_core.ids import robot_id

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...shared.assembly_planning.movement import Routine, UnknownMovement, classify
from ...shared.assembly_planning.route_result import GoalCheck, RouteResult, goal_line
from ...shared.assembly_planning.route_search import ATTEMPTS
from ...shared.assembly_planning.tamp_route import FAR, IK_GOALS
from ...shared.assembly_run_contract import is_live
from ...shared.execution.basis import PlanBasis, basis_of, stale_reason
from ...shared.execution.retime import RetimeError, retime
from ...shared.motion_planner_contract import MotionPlanRequest
from ...shared.movement_exec_contract import (DEFAULT_TRAJ_TIME, DRY_RUN, MAX_TRAJ_TIME, MOTION_PART, REFUSED,
                                              ExecRequest, ExecResult, PartPlan, current_primary, current_step,
                                              serial_of, step_parts)
from ...shared.planner.planner_search import WAIT_MARGIN, PlanResult
from ...shared.planner.timed_path import TimedPath
from ...ui.ghost import RobotGhost
from .goal import StepGoal, same_step
from .panel import StepsPanel
from .step_panel_base import PlannerPlugin, Search

if TYPE_CHECKING:
    from ...shared.assembly_planning.process_steps import Step
    from ...shared.assembly_planning.steps import OperatorStep
    from ...shared.assembly_run_contract import AssemblyRunContract, RunView
    from ...shared.motion_planner_contract import MotionPlannerContract
    from ...shared.movement_exec_contract import MovementExecContract
    from ...shared.process_contract import ProcessContract

#: Seconds to wait for a plan: every try of tamp's longest search, the first plan's model loading, and a margin.
PLAN_TIME_LIMIT = ATTEMPTS * FAR["max_time"] + 60.0 + WAIT_MARGIN


@register
class AssemblyStepsPlugin(PlannerPlugin):
    """Walks the run's current action step by step: check the goal, plan, preview, Exec, next step."""

    name = "assembly_steps"
    requires = ("process", "assembly_run", "motion_planner", "movement_exec")
    experimental = True
    worker_name = "steps"

    def __init__(self):
        """No step yet."""
        super().__init__()
        self._ctx: PluginContext | None = None
        self._panel: StepsPanel | None = None
        #: The current step: (loaded process, action id, primary movement index, transfer workaround), or None.
        self._key: tuple | None = None
        self._steps: list[OperatorStep] = []
        self._index = 0
        self._robot_name = ""
        #: The workaround as last seen, to notice the operator toggling it.
        self._transfer: bool | None = None
        #: Seconds the motion should take, the operator's; the default per routine at each new step.
        self.traj_time = 10.0
        self.ignore_built = False
        #: Movement indices of the current step done by Exec; the action offered to Mark done after its last step.
        self._done: set[int] = set()
        self._offer: str | None = None
        #: The last goal check, and the step it is for.
        self._goal = StepGoal()
        #: The plan behind `path`, and what it was planned from.
        self._route: RouteResult | None = None
        self._basis: PlanBasis | None = None
        #: The step key an Exec runs for, until its result comes back.
        self._executing: tuple | None = None
        #: The shortest motion time the arms' limits allow for the plan, seconds; NaN without a plan.
        self.minimum = math.nan

    # --- --- --- --- --- SETUP AND TICK --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel and a target and a path ghost per robot with arms."""
        self._ctx = ctx
        robots = [config for config in ctx.config.robots if config.arms]
        if not robots:
            self._set_up_idle(ctx, "no robots with arms configured: no step to run")
            return
        self._panel = StepsPanel(ctx, self)
        self._add_ghosts(ctx, robots)

    def update(self, ctx: PluginContext) -> None:
        """Follow the run's current step, then advance the preview and check the plan for staleness."""
        if self.idle:
            return
        self._follow(ctx)
        super().update(ctx)

    def draw(self, ctx: PluginContext) -> None:
        """Fill the panel, the slider and the ghosts."""
        if self.idle or self._panel is None:
            return
        self._panel.draw(ctx, self)
        self._draw_slider()
        self._draw_ghosts(ctx)

    def _follow(self, ctx: PluginContext) -> None:
        """Take the run's current step; a new one starts clean. A toggled workaround moves to its step's primary."""
        view = self._assembly_run(ctx).view
        found = current_step(view)
        transfer = view.settings.transfer_as_free
        toggled, self._transfer = self._transfer is not None and transfer != self._transfer, transfer
        if found is None:
            if self._key is not None:
                self._new_step(ctx, None)
            return
        action, steps, index = found
        if toggled and view.current[1] != steps[index].primary:
            # * The workaround regroups the action's movements: go to the primary of the step now holding this one.
            self._assembly_run(ctx).request_set_current(action, steps[index].primary)
        key = (view.process, action, steps[index].primary, transfer)
        if not same_step(key, self._key):
            self._steps, self._index = steps, index
            self._new_step(ctx, key)

    def _new_step(self, ctx: PluginContext, key: tuple | None) -> None:
        """Start a step clean: no plan, nothing done, no offer, no goal; its routine's default motion time."""
        self._clear()
        self._key, self._done, self._offer = key, set(), None
        self.minimum = math.nan
        self.serial, self._robot_name, self._has_target = None, "", key is not None
        if key is None:
            return
        loaded, action = key[0], key[1]
        self.serial = serial_of(loaded, action, ctx.config.robots)
        robot = loaded.process.actions[action].robot
        self._robot_name = loaded.process.robots[robot].name if robot else ""
        routine = self.routine()
        self.traj_time = DEFAULT_TRAJ_TIME.get(routine, self.traj_time)

    # --- --- --- --- --- WHAT THE PANEL READS --- --- --- --- ---

    def current(self) -> tuple[RunView, str, OperatorStep, Step] | None:
        """The run's view, the current action, its step and the primary movement's Step; None without one."""
        if self._key is None:
            return None
        view = self._assembly_run(self._ctx).view
        loaded, action, primary = self._key[0], self._key[1], self._key[2]
        return view, action, self._steps[self._index], loaded.steps[loaded.index_of(action, primary)]

    def routine(self) -> Routine | None:
        """The current step's primary routine, or None."""
        current = self.current()
        if current is None:
            return None
        view, _action, _step, step = current
        try:
            return classify(view.process.process, step.action, step.movement,
                            view.settings.transfer_as_free).routine
        except UnknownMovement:
            return None

    def parts(self) -> tuple[PartPlan, ...]:
        """The current step's parts not done yet."""
        current = self.current()
        if current is None:
            return ()
        view, action, step, _primary = current
        return tuple(part for part in step_parts(view.process, action, step, self._ctx.config.robots,
                                                 view.settings.transfer_as_free) if part.index not in self._done)

    @property
    def position(self) -> tuple[int, int]:
        """tuple[int, int]: The current step's number from 1, and how many steps the action has."""
        return self._index + 1, len(self._steps)

    @property
    def offer(self) -> str | None:
        """The action offered to Mark done (every step of it ran), or None."""
        return self._offer

    @property
    def goal(self) -> GoalCheck | None:
        """The current step's goal check, or None if none was made."""
        return self._goal.of(self._key_of_goal())

    @property
    def route(self) -> RouteResult | None:
        """The plan behind the path, or None."""
        return self._route

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def go(self, ctx: PluginContext, by: int) -> None:
        """Make the previous (-1) or next (+1) step of the action current; refused while a step runs."""
        self._used.touch()
        busy = self._busy(ctx)
        if busy:
            self._say(f"{'Prev' if by < 0 else 'Next'} refused: {busy}", failed=True)
            return
        if self._key is None or not 0 <= self._index + by < len(self._steps):
            self._say("no step there in this action", failed=True)
            return
        self._assembly_run(ctx).request_set_current(self._key[1], self._steps[self._index + by].primary)

    def set_traj_time(self, seconds: float) -> None:
        """Set the motion time (1 to MAX_TRAJ_TIME s); the preview is timed again."""
        self.traj_time = min(max(float(seconds), 1.0), MAX_TRAJ_TIME)
        if self._route is not None and self.path is not None:
            timed = self._timed(self._route)
            if isinstance(timed, TimedPath):
                self.path = timed
                self._t = min(self._t, timed.duration)

    def set_ignore_built(self, ignore: bool) -> None:
        """Plan as if the built bars were not there (for tests in the air); a plan made otherwise goes stale."""
        self.ignore_built = bool(ignore)

    def set_workaround(self, ctx: PluginContext, on: bool) -> None:
        """Ask the run to change its transfer workaround."""
        self._assembly_run(ctx).request_settings(transfer_as_free=bool(on))

    def mark_done(self, ctx: PluginContext) -> None:
        """Mark the offered action done in the run."""
        if self._offer is None:
            self._say("nothing to mark yet: run every step of the action first", failed=True)
            return
        self._assembly_run(ctx).request_mark_done(self._offer)

    def check_goal(self, ctx: PluginContext) -> None:
        """Start a goal check of the current step, unless one runs or its routine has no IK goal."""
        self._used.touch()
        if self._goal.running:
            self._say("Check goal ignored: a check is running")
            return
        if self.routine() not in IK_GOALS:
            self._say("Check goal: only a transfer or an approach has a goal solved by IK", failed=True)
            return
        self._goal.task = ctx.spawn("check goal", self.checked_goal(ctx))

    async def checked_goal(self, ctx: PluginContext) -> GoalCheck | None:
        """Solve the current step's goal at the live base; keep it (for the ghost and the plan), show and record it.

        Returns:
            GoalCheck | None: The answer, or None without a step or an answer in time.
        """
        current = self.current()
        if current is None:
            self._say("no current step", failed=True)
            return None
        view, action, _step, step = current
        # * The snapshot is taken here, on the main thread; the planner's worker makes the scene from it.
        request = MotionPlanRequest(view.process, step, ctx.scene.snapshot, ctx.config.robots, beliefs=view.beliefs,
                                    transfer_as_free=view.settings.transfer_as_free, ignore_built=self.ignore_built,
                                    obstacles_collide=self._obstacles_collide(ctx), label=f"check goal {step.label}")
        self._say(f"solving the goal of {step.label}…")
        check = await self._goal.solve(ctx, self._key_of_goal(), request)
        if check is None:
            self._say("Check goal: no answer in time", failed=True)
            return None
        self._say(goal_line(check), failed=not check.ok)
        self._assembly_run(ctx).request_record("check_goal", action=action, movement=step.movement.id,
                                               data={**check.as_record(),
                                                     "obstacles_collide": request.obstacles_collide})
        return check

    # --- --- --- --- --- PLANNER PLUGIN --- --- --- --- ---

    def make_search(self, ctx: PluginContext, abort: threading.Event) -> Search:
        """A search through the motion planner from the live robot; refused without a motion or a live robot."""
        current = self.current()
        refusal = "no current step" if current is None else ""
        snapshot, now, request, goal_base = ctx.scene.snapshot, ctx.now(), None, None
        if current is not None:
            view, action, op_step, step = current
            live = snapshot.robots.get(robot_id(self.serial)) if self.serial else None
            if not any(part.kind == MOTION_PART and part.index == op_step.primary for part in self.parts()):
                refusal = f"{step.label} has no motion to plan"
            elif live is None or not is_live(live, now):
                refusal = f"{self._robot_name} ({self.serial}) is not live: plans start from the live robot only"
            else:
                goal_joints = self._goal.joints_near(self._key_of_goal(), live.base)
                goal_base = self._goal.of(self._key_of_goal()).base if goal_joints is not None else None
                request = MotionPlanRequest(view.process, step, snapshot, ctx.config.robots, beliefs=view.beliefs,
                                            transfer_as_free=view.settings.transfer_as_free,
                                            ignore_built=self.ignore_built,
                                            obstacles_collide=self._obstacles_collide(ctx), goal_joints=goal_joints,
                                            label=f"plan {step.label}")
        found: dict[str, RouteResult] = {}

        async def run() -> PlanResult:
            if request is None:
                return PlanResult(None, refusal)
            planner: MotionPlannerContract = ctx.require("motion_planner")
            route = await planner.request_plan(request)
            if not route.ok:
                return PlanResult(None, f"{route.result.status}: {route.result.reason}", route.seconds)
            timed = self._timed(route)
            if isinstance(timed, str):
                return PlanResult(None, timed, route.seconds)
            found["route"] = route
            return PlanResult(timed, "", route.seconds)

        def accepted(result: PlanResult) -> tuple[str, bool]:
            route = found["route"]
            view, action, op_step, step = current
            trajectory = route.result.trajectory
            self._route = route
            self._basis = basis_of(snapshot, view.process, step, self.serial, trajectory.joint_names,
                                   dict(zip(trajectory.joint_names, trajectory.positions[0])), view.settings,
                                   view.statuses, self.ignore_built, now, goal_base,
                                   obstacles_collide=request.obstacles_collide)
            self._assembly_run(ctx).request_record("plan", action=action, movement=step.movement.id, data={
                "planner": route.planner, "attempts": route.attempts, "seconds": route.seconds,
                "waypoints": route.waypoints, "goal_reused": request.goal_joints is not None,
                "warnings": list(route.warnings), "obstacles_collide": request.obstacles_collide})
            return (f"{route.planner}: {route.waypoints} waypoints in {route.seconds:.1f} s; the motion takes "
                    f"{self.path.duration:.1f} s"), bool(route.warnings)

        label = current[3].label if current is not None else "nothing"
        return Search(self.serial or "", f"plan {label}", f"planning {label}…", None, PLAN_TIME_LIMIT, accepted,
                      run=run)

    def stale_reason(self, ctx: PluginContext) -> str:
        """Why the ready plan no longer fits (`basis.stale_reason`); "" if it still does."""
        if self._basis is None:
            return "the plan has no basis"
        view = self._assembly_run(ctx).view
        return stale_reason(self._basis, ctx.scene.snapshot, loaded=view.process, current=current_primary(view),
                            transfer_as_free=view.settings.transfer_as_free, ignore_built=self.ignore_built,
                            statuses=view.statuses, obstacles_collide=self._obstacles_collide(ctx))

    def forget_plan(self) -> None:
        """Drop the route and its basis with the path."""
        self._route = self._basis = None

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The acting arms at the goal check's IK joints, else at the movement's authored target, where it stands."""
        current = self.current()
        if current is None:
            ghost.hide()
            return
        goal = self.goal
        _view, _action, _op_step, step = current
        target = step.movement.target
        joints = goal.ik.joints if goal is not None and goal.ok else (
            target.joints.get(step.action.robot) if target is not None else None)
        if not joints:
            ghost.hide()
            return
        ghost.show(ctx.kinematics.base_pose(self.serial), {**ctx.kinematics.joints(self.serial), **joints})

    def show_path(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The acting arms on the plan at the slider time, the base where it was planned."""
        if self._route is None or self._basis is None:
            ghost.hide()
            return
        names = self._route.result.trajectory.joint_names
        joints = {**ctx.kinematics.joints(self._path_serial), **dict(zip(names, self.path.sample(self._t)))}
        ghost.show(self._basis.base, joints)

    def _commit(self, ctx: PluginContext) -> None:
        """Exec: hand the step's parts left to movement_exec, the plan with them when the step's motion is left."""
        self._used.touch()
        current = self.current()
        if current is None:
            self._say("no current step", failed=True)
            return
        busy = self._busy(ctx)
        if busy:
            self._say(f"Exec refused: {busy}", failed=True)
            return
        view, action, op_step, step = current
        parts = self.parts()
        motion = any(part.kind == MOTION_PART for part in parts)
        if motion and self.plan_state != "ready":
            self._say(f"nothing to Exec: the plan is {self.plan_state}; Plan first", failed=True)
            return
        stale = self.stale_reason(ctx) if motion else ""
        if stale:
            # * Checked here too: an input in the same tick (e.g. Ignore built bars, Rhino obstacles collide) comes
            #   before the update's check.
            self.plan_state = "stale"
            self._say(f"Exec refused, the plan is stale: {stale}", failed=True)
            return
        request = ExecRequest(view.process, action, op_step, parts, self.serial or "", self._robot_name,
                              self._route if motion else None, op_step.primary if motion else None,
                              self._basis if motion else None, self.traj_time, step.label,
                              ignore_built=lambda: self.ignore_built,
                              obstacles_collide=lambda: self._obstacles_collide(ctx))
        if motion:
            self.plan_state = "sent"  # ! used once: never sent again
        executor: MovementExecContract = ctx.require("movement_exec")
        key = self._executing = self._key
        self._say(f"Exec {step.label}: confirm each part on the movement_exec panel or the phone")
        executor.request_execute(request).add_done_callback(
            lambda future: self._executed(ctx, key, motion, future))

    def _executed(self, ctx: PluginContext, key: tuple, carried_plan: bool, future: asyncio.Future) -> None:
        """Take an Exec's result: note the parts done; the step done moves on, the action done offers Mark done."""
        self._executing = None
        result: ExecResult = future.result()
        if not same_step(key, self._key):
            return  # the step changed meanwhile
        self._done.update(result.done)
        self._say(f"{result.status}: {result.reason}", failed=not result.ok and result.status != DRY_RUN)
        if carried_plan and result.status in (REFUSED, DRY_RUN) and key[2] not in result.done:
            # * Nothing of the motion was sent: the plan comes back, checked for staleness first.
            stale = self.stale_reason(ctx)
            self.plan_state = "stale" if stale else "ready"
            if stale:
                self._say(f"{result.status}; the plan is stale now: {stale}", failed=True)
        elif carried_plan:
            self._clear()
        if not result.ok or self.parts():
            return
        if self._index + 1 < len(self._steps):
            self._assembly_run(ctx).request_set_current(key[1], self._steps[self._index + 1].primary)
        else:
            self._offer = key[1]
            self._say(f"every step of {key[1]} done: Mark {key[1]} done")

    # --- --- --- --- --- HELPERS --- --- --- --- ---

    def _assembly_run(self, ctx: PluginContext) -> AssemblyRunContract:
        """The assembly_run plugin."""
        return ctx.require("assembly_run")

    def _busy(self, ctx: PluginContext) -> str:
        """Why the step may not change or run now: an Exec or a plan runs; else ""."""
        executor: MovementExecContract = ctx.require("movement_exec")
        if executor.active or self._executing is not None:
            return f"{executor.active or 'an Exec'} runs: wait for it, or Cancel it"
        if self.plan_state == "planning":
            return "a plan is being made: wait, or Clear"
        return ""

    def _obstacles_collide(self, ctx: PluginContext) -> bool:
        """Whether the process plugin counts its Rhino obstacles in collision checks now.

        Args:
            ctx: This plugin's context.

        Returns:
            bool: The process plugin's "Rhino obstacles collide" box.
        """
        process: ProcessContract = ctx.require("process")
        return process.obstacles_collide

    def _key_of_goal(self) -> tuple | None:
        """What a goal check is for: the current (loaded process, action, primary, Rhino obstacles collide).

        * With the box: a goal checked with the other setting is never shown or reused.
        """
        return None if self._key is None else (*self._key[:3], self._obstacles_collide(self._ctx))

    def _timed(self, route: RouteResult) -> TimedPath | str:
        """The route timed for the arms at the motion time (`retime`), for the preview; or why it can't be."""
        try:
            timed = retime(route.result.trajectory.positions, self.traj_time)
        except RetimeError as error:
            rows = route.result.trajectory.positions
            if rows[0] == rows[-1]:
                return "the arms are already at the goal: nothing to move (Next ▶ goes on)"
            return f"planned, but it can't be timed for the arms: {error}"
        self.minimum = timed.minimum
        return TimedPath(timed.times, timed.positions)
