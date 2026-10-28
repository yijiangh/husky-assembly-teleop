"""
The operator's steps in the Assembly tab: the current step, its goal checked, its motion planned and previewed, Exec.

The current step follows the run (`assembly_run`), which follows the process panel: its action, the step holding its
movement (`operator_steps`, the process plugin's transfer workaround). Plan asks the motion planner from the live robot
(only when it is live); the plan goes stale when anything it was made from changes (`shared/execution/basis.py`). Exec
hands the step to `movement_exec`, which runs a motion at once and asks the operator to Confirm the bar mount, a
Robotiq step and the servo's first pass; once every part is done the next step becomes current, and after the
action's last step Mark done (assembly_run's) is suggested.

- Live IK: the goal by IK at the live base, logged, shown as the target ghost and recorded; a plan of the same step
  reuses it while the base stays within 2 mm / 0.5° and the process plugin's "Rhino obstacles collide" is unchanged.
- After Live IK or a plan, the process panel draws the action's later movements with the joints found, on the live
  base they were found at, and turns to the step's target (`ProcessContract.request_carry`); a new action drops them.
- Plan on a movement with nothing to plan (an ungrasp, a grasp, an operator step) says so and selects the next robot
  movement instead (`_move_on`): the operator clicks Plan again.
- A plan is used once: Exec takes it, and it is cleared once the step ran; only a request refused before sending
  anything, or a dry run, gives it back (still checked for staleness).
- An insert after its transfer's MoCap-bar servo is planned from where the servo left the flanges (`executor.carried`).
- ! EXPERIMENTAL, with movement_exec the first plugins that run process movements on the robots.
"""

from __future__ import annotations

import asyncio
import math
import threading
from typing import TYPE_CHECKING

import viser
from scaffolding_assembly_core.ids import robot_id, split_link_id
from scaffolding_assembly_core.robot import FLANGE_SUFFIX
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...shared.assembly_planning.movement import Routine, UnknownMovement, classify
from ...shared.assembly_planning.route_result import GoalCheck, RouteResult, goal_line, pose_error
from ...shared.assembly_planning.route_search import ATTEMPTS, FreeSearch
from ...shared.assembly_planning.tamp_route import HELD, IK_GOALS
from ...shared.assembly_run_contract import is_live
from ...shared.execution.basis import PlanBasis, basis_of, stale_reason
from ...shared.execution.retime import RetimeError, retime
from ...shared.execution.servo import authored_start, carried_insert_goals, flange_poses, flange_side, tool0_errors
from ...shared.execution.stream import tool0_world_path
from ...shared.motion_planner_contract import MotionPlanRequest
from ...shared.movement_exec_contract import (DEFAULT_TRAJ_TIME, DRY_RUN, MAX_TRAJ_TIME, MOTION_KINDS, REFUSED,
                                              REFUSED_PART, ExecRequest, ExecResult, PartPlan, current_primary,
                                              current_step, serial_of, step_parts)
from ...shared.planner.planner_search import WAIT_MARGIN, PlanResult
from ...shared.planner.timed_path import TimedPath
from ...shared.process_contract import Carry
from ...ui.assembly_tab import ASSEMBLY_TAB
from ...ui.ghost import RobotGhost
from .diagnosis import StepDiagnosis, path_line, tool0_line
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

#: Seconds to wait for a plan beyond its tries: the first plan's model loading, and a margin.
PLAN_TIME_MARGIN = 60.0 + WAIT_MARGIN
#: Opacity of the real robots while the visual diagnosis is on (the old monitor's VISUAL_DIAG_ROBOT_ALPHA).
DIAGNOSIS_OPACITY = 0.3


@register
class AssemblyStepsPlugin(PlannerPlugin):
    """Walks the run's current action step by step: check the goal, plan, preview, Exec, next step."""

    name = "assembly_steps"
    requires = ("process", "assembly_run", "motion_planner", "movement_exec")
    experimental = True
    tab_path = (ASSEMBLY_TAB,)
    icon = viser.Icon.STAIRS_UP
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
        #: Visual diagnosis: the real robots see-through and each insert or retreat plan's tool0 path drawn.
        self.visual_diagnosis = False
        #: The diagnostic drawings (Check tool0, the plan's path), made on first use.
        self._diagnosis_nodes: StepDiagnosis | None = None
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
        """Fill the panel, the slider and the ghosts; the diagnostic drawings show while the panel is in use."""
        if self.idle or self._panel is None:
            return
        self._panel.draw(ctx, self)
        self._draw_slider()
        self._draw_ghosts(ctx)
        if self._diagnosis_nodes is not None and self._used is not None:
            self._diagnosis_nodes.visible = self._used.active

    def teardown(self, ctx: PluginContext) -> None:
        """Put the real robots back solid, then as the base."""
        self._fade_robots(ctx, False)
        super().teardown(ctx)

    def _follow(self, ctx: PluginContext) -> None:
        """Take the run's current step; a new one starts clean. A toggled workaround moves to its step's primary."""
        view = self._assembly_run(ctx).view
        transfer = self._transfer_as_free(ctx)
        found = current_step(view, transfer)
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
        """Start a step clean: no plan, nothing done, no offer, no goal; its routine's default motion time.

        Another action also drops the joints carried forward in the process panel.
        """
        self._clear()
        if self._diagnosis_nodes is not None:
            self._diagnosis_nodes.clear()
        if key is None or self._key is None or key[1] != self._key[1]:
            self._process(ctx).request_carry(None)
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
                            self._transfer_as_free(self._ctx)).routine
        except UnknownMovement:
            return None

    def parts(self) -> tuple[PartPlan, ...]:
        """The current step's parts not done yet."""
        current = self.current()
        if current is None:
            return ()
        view, action, step, _primary = current
        return tuple(part for part in step_parts(view.process, action, step, self._ctx.config.robots,
                                                 self._transfer_as_free(self._ctx)) if part.index not in self._done)

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

    def set_traj_time(self, seconds: float) -> None:
        """Set the motion time (1 to MAX_TRAJ_TIME s); the preview is timed again."""
        self.traj_time = min(max(float(seconds), 1.0), MAX_TRAJ_TIME)
        if self._route is not None and self.path is not None:
            timed = self._timed(self._route)
            if isinstance(timed, TimedPath):
                self.path = timed
                self._t = min(self._t, timed.duration)

    def set_visual_diagnosis(self, ctx: PluginContext, on: bool) -> None:
        """Switch the visual diagnosis: fade the real robots, and draw a ready insert or retreat plan's path."""
        self.visual_diagnosis = on
        self._used.touch()
        self._fade_robots(ctx, on)
        if on:
            self._draw_plan_path(ctx)
        elif self._diagnosis_nodes is not None:
            self._diagnosis_nodes.clear("path")

    def _fade_robots(self, ctx: PluginContext, on: bool) -> None:
        """Every configured robot with arms see-through, or solid again."""
        for config in ctx.config.robots:
            if config.arms:
                ctx.view.set_robot_opacity(config.serial, DIAGNOSIS_OPACITY if on else None)

    def _diagnosis(self, ctx: PluginContext) -> StepDiagnosis:
        """The diagnostic drawings, made on first use."""
        if self._diagnosis_nodes is None:
            self._diagnosis_nodes = StepDiagnosis(ctx)
        return self._diagnosis_nodes

    def check_tool0(self, ctx: PluginContext) -> None:
        """Draw each flange's live tool0 against the step's authored target, say the gap, and record it."""
        self._used.touch()
        current = self.current()
        if current is None:
            self._say("Check tool0: no current step", failed=True)
            return
        view, action, _op_step, step = current
        target = step.movement.target
        targets = target.robots[step.action.robot].link_poses if target is not None and step.action.robot in \
            target.robots else {}
        targets = {flange: pose for flange, pose in targets.items() if flange.endswith(FLANGE_SUFFIX)}
        if not targets:
            self._say(f"Check tool0: {step.label} has no authored tool0 target", failed=True)
            return
        live = ctx.scene.snapshot.robots.get(robot_id(self.serial)) if self.serial else None
        if live is None or not is_live(live, ctx.now()):
            self._say(f"Check tool0: {self._robot_name} ({self.serial}) is not live", failed=True)
            return
        poses = flange_poses(live, targets)
        errors = tool0_errors(poses, targets)
        self._diagnosis(ctx).show_tool0(poses, targets)
        line = tool0_line(errors)
        self._say(line)
        ctx.log_info(f"[tool0 check] {step.label}: {line}")
        self._assembly_run(ctx).request_record("check_tool0", action=action, movement=step.movement.id, data={
            side: {key: error[key] for key in ("pos_err_mm", "pos_norm_mm", "rot_err_deg")}
            for side, error in errors.items()})

    def _draw_plan_path(self, ctx: PluginContext) -> None:
        """With the visual diagnosis on and a ready insert or retreat plan: its tool0 path and the frames, logged."""
        current = self.current()
        if not self.visual_diagnosis or current is None or self._route is None or self._basis is None:
            return
        if self.routine() not in (Routine.INSERT, Routine.RETREAT_LINEAR):
            return
        view, action, _op_step, step = current
        robot = ctx.scene.snapshot.robots.get(robot_id(self.serial)) if self.serial else None
        if robot is None:
            return
        trajectory = self._route.result.trajectory
        arms = sorted({name[:-len(suffix) - 1] for name in trajectory.joint_names for suffix in UR_JOINT_NAMES
                       if name.endswith(f"_{suffix}")})
        paths = tool0_world_path(robot.model.urdf, self._basis.base, trajectory.joint_names, trajectory.positions,
                                 arms)
        target = step.movement.target
        ends = target.robots[step.action.robot].link_poses if target is not None else {}
        rhino_target, rhino_start = {}, {}
        for flange, pose in ends.items():
            link = split_link_id(flange)[1]
            if not link.endswith(FLANGE_SUFFIX):
                continue
            arm = link[:-len(FLANGE_SUFFIX)]
            rhino_target[arm] = pose
            try:
                rhino_start[arm] = authored_start(view.process.process.actions[action], step.movement_index, flange)
            except ValueError:
                pass
        self._diagnosis(ctx).show_path(paths, rhino_start, rhino_target)
        ctx.log_info(path_line(step.label, paths, rhino_start, rhino_target))

    def free_search(self, ctx: PluginContext) -> FreeSearch:
        """The free searches' settings for a plan (`MotionPlanRequest.search`): the run's Advanced planning ones."""
        settings = self._assembly_run(ctx).view.settings
        return FreeSearch(settings.plan_max_time, settings.plan_max_iterations, settings.plan_joint_resolution,
                          settings.plan_tries, settings.plan_swept_check)

    def set_ignore_built(self, ignore: bool) -> None:
        """Plan as if the built bars were not there (for tests in the air); a plan made otherwise goes stale."""
        self.ignore_built = bool(ignore)

    def goal_label(self) -> str:
        """The Live IK button's label: which start the current step's goal is, or that it has none.

        Returns:
            str: E.g. "Live IK: insert start" for a transfer.
        """
        routine = self.routine()
        if routine in (Routine.TRANSFER_FREE, Routine.TRANSFER_HELD):
            return "Live IK: insert start"
        if routine == Routine.SUPPORT_APPROACH:
            return "Live IK: linear start"
        return "Live IK (transfer or approach only)"

    def check_goal(self, ctx: PluginContext) -> None:
        """Start a goal check (Live IK) of the current step, unless one runs or its routine has no IK goal."""
        self._used.touch()
        if self._goal.running:
            self._say("Live IK ignored: a check is running")
            return
        if self.routine() not in IK_GOALS:
            self._say("Live IK: only a transfer (the move before the insert) or an approach has a goal solved by IK; "
                      "select that movement in the process panel", failed=True)
            return
        self._goal.task = ctx.spawn("check goal", self.checked_goal(ctx))

    async def checked_goal(self, ctx: PluginContext) -> GoalCheck | None:
        """Solve the current step's goal at the live base; keep it (for the ghost and the plan), show and record it.

        Solved, its joints are carried forward in the process panel; both only while its step is still current.

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
                                    transfer_as_free=self._transfer_as_free(ctx), ignore_built=self.ignore_built,
                                    obstacles_collide=self._obstacles_collide(ctx), label=f"check goal {step.label}")
        solving = f"solving the goal of {step.label}…"
        self._say(solving)
        key = self._key_of_goal()
        check = await self._goal.solve(ctx, key, request)
        # ! The step may have changed while the planner worked: say and carry only for the step it was solved for;
        #   `solve` logs the answer either way.
        still = same_step(key, self._key_of_goal())
        if not still and self._message == solving:
            # * Only this solve's own line is replaced: the new step may have said something since.
            self._say(f"Live IK for {step.label} came after the step changed: not used")
        if check is None:
            if still:
                self._say("Live IK: no answer in time", failed=True)
            return None
        if still:
            self._say(goal_line(check), failed=not check.ok)
        if still and check.ok and check.goal_joints:
            self._process(ctx).request_carry(Carry(action, step.movement.id, step.action.robot,
                                                   dict(check.goal_joints), base=check.base))
        self._assembly_run(ctx).request_record("check_goal", action=action, movement=step.movement.id,
                                               data={**check.as_record(),
                                                     "obstacles_collide": request.obstacles_collide})
        return check

    # --- --- --- --- --- PLANNER PLUGIN --- --- --- --- ---

    def _plan(self, ctx: PluginContext) -> None:
        """Plan the step's motion; on a movement with nothing to plan, say so and select the next robot movement.

        ! Never plans that next movement by itself: the operator clicks Plan again, on it.
        """
        moved_on = self._move_on(ctx)
        if moved_on:
            self._say(moved_on, failed=True)
            ctx.log_warn(moved_on)
            return
        super()._plan(ctx)

    def _move_on(self, ctx: PluginContext) -> str:
        """When the selected movement has no motion of its own to plan, select the next robot movement and say so.

        The next robot movement is the step's own when the selected one runs with it or is skipped for it (an ungrasp
        during its retreat, the travel to load before the transit), else the next step's that moves an arm.

        Returns:
            str: What to tell the operator; "" to plan as usual (the step's motion is selected, or an Exec or a plan
                runs).
        """
        current = self.current()
        if current is None or self._busy(ctx):
            return ""
        view, action, op_step, _step = current
        selected = view.current[1]
        undone = {part.index for part in self.parts()}
        moves = op_step.primary in undone and self._moves(view, action, op_step)
        if selected == op_step.primary and moves:
            return ""
        if selected in (*op_step.with_motion, *op_step.skipped) and moves:
            following = op_step.primary
        else:
            following = next((later.primary for later in self._steps[self._index + 1:]
                              if self._moves(view, action, later)), None)
        movements = view.process.process.actions[action].movements
        if following is None:
            return f"{movements[selected].id}: nothing to plan, and no robot movement follows in {action}"
        self._assembly_run(ctx).request_set_current(action, following)
        return f"{movements[selected].id}: nothing to plan; moved on to {movements[following].id}: click Plan again"

    def _moves(self, view: RunView, action: str, step: OperatorStep) -> bool:
        """Whether a step's primary movement moves an arm (a plan's to make), or is refused (Plan says why)."""
        parts = step_parts(view.process, action, step, self._ctx.config.robots, self._transfer_as_free(self._ctx))
        return any(part.index == step.primary and part.kind in (*MOTION_KINDS, REFUSED_PART) for part in parts)

    def make_search(self, ctx: PluginContext, abort: threading.Event) -> Search:
        """A search through the motion planner from the live robot; refused without a motion or a live robot."""
        current = self.current()
        refusal = "no current step" if current is None else ""
        snapshot, now, request, goal_base, goals = ctx.scene.snapshot, ctx.now(), None, None, None
        search = self.free_search(ctx)
        if current is not None:
            view, action, op_step, step = current
            live = snapshot.robots.get(robot_id(self.serial)) if self.serial else None
            primary = next((part for part in self.parts() if part.index == op_step.primary), None)
            if primary is not None and primary.kind == REFUSED_PART:
                # * Say why, e.g. "B20_J_transfer: the transfer with the bar held (M7) is not supported yet".
                refusal = f"{step.label}: {primary.text}"
            elif primary is None or primary.kind not in MOTION_KINDS:
                refusal = f"{step.label} has no motion to plan"
            elif live is None or not is_live(live, now):
                refusal = f"{self._robot_name} ({self.serial}) is not live: plans start from the live robot only"
            else:
                goal_joints = self._goal.joints_near(self._key_of_goal(), live.base)
                goal_base = self._goal.of(self._key_of_goal()).base if goal_joints is not None else None
                carried = ctx.require("movement_exec").carried if primary.routine is Routine.INSERT else None
                if carried is not None and carried.loaded is view.process and (carried.action, carried.insert) == (
                        action, step.movement_index):
                    # * The transfer's bar servo left the flanges off the authored start: the insert goes from there.
                    goals = carried_insert_goals(view.process, action, carried.insert, live)
                    flange, ends = next(iter(goals)), step.movement.target.robots[step.action.robot].link_poses
                    distance, turn = pose_error(goals[flange], ends[flange])
                    ctx.log_info(f"[insert plan] MoCap-bar correction carried: {step.label} {flange_side(flange)} "
                                 f"goal {distance * 1e3:.2f} mm / {math.degrees(turn):.3f}° from the authored end")
                request = MotionPlanRequest(view.process, step, snapshot, ctx.config.robots, beliefs=view.beliefs,
                                            transfer_as_free=self._transfer_as_free(ctx),
                                            ignore_built=self.ignore_built,
                                            obstacles_collide=self._obstacles_collide(ctx), goal_joints=goal_joints,
                                            goal_poses=goals, search=search, label=f"plan {step.label}")
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
                                   dict(zip(trajectory.joint_names, trajectory.positions[0])),
                                   request.transfer_as_free, view.statuses, self.ignore_built, now, goal_base,
                                   obstacles_collide=request.obstacles_collide)
            self._assembly_run(ctx).request_record("plan", action=action, movement=step.movement.id, data={
                "planner": route.planner, "attempts": route.attempts, "seconds": route.seconds,
                "waypoints": route.waypoints, "goal_reused": request.goal_joints is not None,
                "warnings": list(route.warnings), "obstacles_collide": request.obstacles_collide,
                "goal_carried": goals is not None})
            self._process(ctx).request_carry(Carry(action, step.movement.id, step.action.robot,
                                                   dict(zip(trajectory.joint_names, trajectory.positions[-1])),
                                                   "plan", self._basis.base))
            self._draw_plan_path(ctx)
            return (f"{route.planner}: {route.waypoints} waypoints in {route.seconds:.1f} s; the motion takes "
                    f"{self.path.duration:.1f} s"), bool(route.warnings)

        label = current[3].label if current is not None else "nothing"
        # * Long enough for every try of the free search at the run's settings.
        # * A held transfer (workaround off) runs HELD's own tries and time, whatever the settings say.
        limit = max(search.tries * search.max_time, ATTEMPTS * HELD["max_time"]) + PLAN_TIME_MARGIN
        return Search(self.serial or "", f"plan {label}", f"planning {label}…", None, limit, accepted,
                      run=run)

    def stale_reason(self, ctx: PluginContext) -> str:
        """Why the ready plan no longer fits (`basis.stale_reason`); "" if it still does."""
        if self._basis is None:
            return "the plan has no basis"
        view = self._assembly_run(ctx).view
        transfer = self._transfer_as_free(ctx)
        return stale_reason(self._basis, ctx.scene.snapshot, loaded=view.process,
                            current=current_primary(view, transfer), transfer_as_free=transfer,
                            ignore_built=self.ignore_built, statuses=view.statuses,
                            obstacles_collide=self._obstacles_collide(ctx))

    def forget_plan(self) -> None:
        """Drop the route and its basis with the path, and the path's diagnostic drawing."""
        self._route = self._basis = None
        if self._diagnosis_nodes is not None:
            self._diagnosis_nodes.clear("path")

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
        motion = any(part.kind in MOTION_KINDS for part in parts)
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
                              obstacles_collide=lambda: self._obstacles_collide(ctx),
                              transfer_as_free=lambda: self._transfer_as_free(ctx))
        if motion:
            self.plan_state = "sent"  # ! used once: never sent again
        executor: MovementExecContract = ctx.require("movement_exec")
        key = self._executing = self._key
        self._say(f"Exec {step.label}: running; Confirm below or on the phone when asked (the bar mount, a Robotiq "
                  f"step, the servo's first pass)")
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
            self._say(f"every step of {key[1]} done: Mark done below")

    # --- --- --- --- --- HELPERS --- --- --- --- ---

    def _assembly_run(self, ctx: PluginContext) -> AssemblyRunContract:
        """The assembly_run plugin."""
        return ctx.require("assembly_run")

    def _process(self, ctx: PluginContext) -> ProcessContract:
        """The process plugin."""
        return ctx.require("process")

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

    def _transfer_as_free(self, ctx: PluginContext) -> bool:
        """Whether the process plugin's transfer workaround is on now.

        Args:
            ctx: This plugin's context.

        Returns:
            bool: The process plugin's "Transfer workaround" box.
        """
        process: ProcessContract = ctx.require("process")
        return process.transfer_as_free

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
                return "the arms are already at the goal: nothing to move (mv ▶ in the process panel goes on)"
            return f"planned, but it can't be timed for the arms: {error}"
        self.minimum = timed.minimum
        return TimedPath(timed.times, timed.positions)
