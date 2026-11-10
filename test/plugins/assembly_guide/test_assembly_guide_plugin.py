"""The guide on fakes: the part shown, Redo from here, Done and Skip, out of order, the modes, a part by hand.

The executor is a fake that finishes a part at once, or asks a question first for a part by hand; the process plugin
a fake selection the run moves; the base planner a fake whose commit puts the base on the goal.
"""

from __future__ import annotations

import asyncio
import math
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import purged_process, robot_config
from robot_lab import FakeRun, RobotLab

from husky_assembly_teleop.shared.process_scene.live_scene import SceneNotShown
from husky_assembly_teleop.plugins.assembly_guide import plugin as guide_module
from husky_assembly_teleop.plugins.assembly_guide.parts import DRIVE, HAND, MARK, MOVE, NOTHING_TO_MOVE, PARK
from husky_assembly_teleop.world.checks import INFO, WARN
from husky_assembly_teleop.plugins.assembly_guide.plugin import AUTO, GOD, OFF, REDO, AssemblyGuidePlugin, \
    simulation_problem
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.steps import as_run
from husky_assembly_teleop.shared.execution.operator import Prompt
from husky_assembly_teleop.shared.movement_exec_contract import BY_HAND_PART, DONE as EXEC_DONE, ExecResult
from husky_assembly_teleop.shared.assembly_planning.route_result import GoalCheck
from husky_assembly_teleop.plugins.arm_planner.planner import ArmPath
from husky_assembly_teleop.shared.planner.plan import Plan
from husky_assembly_teleop.shared.planner.timed_path import TimedPath
from husky_assembly_teleop.shared.assembly_planning.steps import first_step
from husky_assembly_teleop.shared.process_contract import Selection
from husky_assembly_teleop.shared.runs.runs import DONE
from husky_assembly_teleop.world.checks import BAD
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

CINDY = "a200-0806"


def answered(value):
    """A future already holding `value`."""
    future = asyncio.get_running_loop().create_future()
    future.set_result(value)
    return future


class Process:
    """The process plugin: its boxes and its selection."""

    obstacles_collide = transfer_as_free = True

    def __init__(self, loaded, movement: str):
        """Select `movement`."""
        self.authored, self.shown = loaded.process, as_run(loaded.process, True)
        self.movements = list(self.authored.movements())
        self.request_select(next(index for index, (_a, m) in enumerate(self.movements) if m.id == movement))

    def request_select(self, index: int) -> None:
        """Select the authored movement `index`, shown as the panel shows it."""
        action, movement = self.movements[index]
        shown = self.shown.actions[action.id]
        movement = next((each for each in shown.movements if each.id == movement.id), shown.movements[0])
        self.selection = Selection(self.authored, index, shown, movement)
        # * The live scene shows it at once here; a test may hold it back.
        self.scene_shows = (action.id, movement.id)


class Run(FakeRun):
    """The run: marks, reopens and moves its current step at once, selecting it in the process plugin."""

    def __init__(self, loaded, current, process: Process):
        """Every action pending."""
        super().__init__(loaded, current)
        self.process, self.reopened = process, []

    def request_mark_done(self, action_id=None):
        """Mark it done, then make the first pending action current at its first step, as the real run does."""
        super().request_mark_done(action_id)
        self.view.statuses[action_id] = DONE
        process = self.view.process.process
        following = next((each for each in process.schedule if self.view.statuses[each] != DONE), None)
        if following is not None:
            self.request_set_current(following, first_step(process, process.actions[following], True))

    def request_reopen(self, action_id=None):
        """Put it back to pending, and select its first movement."""
        self.reopened.append(action_id)
        self.view.statuses[action_id] = "pending"
        self.request_set_current(action_id, 0)

    def request_set_current(self, action_id, movement_index=None):
        """Make it current and select it."""
        super().request_set_current(action_id, movement_index)
        self.process.request_select(self.view.process.index_of(action_id, movement_index or 0))


class Executor:
    """The executor: finishes a part at once; a part by hand asks first and finishes on Confirm."""

    def __init__(self, ctx):
        """Nothing finished."""
        self.ctx, self.done, self.requests, self.forgot, self.cancelled = ctx, {}, [], [], 0
        self.prompt, self.active, self.carried, self.dry_run = None, "", None, False
        self._waiting = None

    def finished(self, loaded, action):
        """What was finished for `action`."""
        return frozenset(self.done.get(action, ()))

    def request_forget(self, action, done=None):
        """Forget some or all of it."""
        self.forgot.append((action, frozenset(done or ())))
        kept = self.done.get(action, set())
        kept.difference_update(done if done is not None else set(kept))

    def request_execute(self, request):
        """Finish it, or ask first for a part by hand."""
        self.requests.append(request)
        future = asyncio.get_running_loop().create_future()
        if any(plan.kind == BY_HAND_PART for plan in request.parts):
            self.prompt = Prompt(len(self.requests), "cindy", "place the bar", False, self.ctx.now())
            self._waiting = (request, future)
        else:
            self._finish(request, future)
        return future

    def request_confirm(self):
        """Finish the part asked about."""
        request, future = self._waiting
        self.prompt, self._waiting = None, None
        self._finish(request, future)

    def request_cancel(self):
        """Answer the question Cancel: the request ends cancelled."""
        self.cancelled += 1
        if self._waiting is not None:
            _request, future = self._waiting
            self.prompt, self._waiting = None, None
            future.set_result(ExecResult("CANCELLED", "the prompt is answered Cancel"))

    def request_dry_run(self, on):
        """Keep it."""
        self.dry_run = on

    def _finish(self, request, future):
        """Every movement of the request done; nothing in a dry run."""
        if request.dry_run:
            future.set_result(ExecResult("DRY_RUN", "dry run: every check passed, nothing sent"))
            return
        indices = [index for plan in request.parts for index in (plan.index, *plan.also)]
        self.done.setdefault(request.action, set()).update((request.step.primary, index) for index in indices)
        future.set_result(ExecResult(EXEC_DONE, "every part done", tuple(indices)))


class BasePlanner:
    """Plans at once; its commit puts the base on the goal."""

    def __init__(self, bases):
        """No plan yet."""
        self.bases, self.plan, self.requests = bases, None, []

    def request_plan(self, request):
        """A path that stands at the goal."""
        self.requests.append(request)
        goal = np.asarray(request.goal, dtype=float)
        self.plan = Plan(1, request, "ready", TimedPath(np.array([0.0, 2.0]), np.array([goal, goal])), "found")
        return answered(self.plan)

    def commit(self, plan_id, unknown_robots=(), clearance=True):
        """Put the base on the goal."""
        x, y, yaw = self.plan.request.goal
        self.bases[self.plan.request.serial] = Pose((x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        return answered((True, "sent"))

    def cancel_commit(self, serial=None):
        """Nothing runs."""
        return False


class Guide(RobotLab):
    """Cindy far from B20_J's start, the guide at its transfer, on fakes."""

    def __init__(self, monkeypatch, movement: str = "B20_J_transfer"):
        """Build the robots, the fakes and the guide, without ghosts."""
        super().__init__()
        monkeypatch.setattr(guide_module, "robot_ghosts", lambda *_args: {})
        loaded = purged_process()
        self.bases[CINDY] = Pose((3.0, 1.0, 0.0))
        self.tick(self.ctx.t)
        self.process = Process(loaded, movement)
        self.run = Run(loaded, None, self.process)
        self.executor = Executor(self.ctx)
        self.base_planner = BasePlanner(self.bases)
        self.ctx.plugins = {"process": self.process, "assembly_run": self.run, "movement_exec": self.executor,
                            "base_planner": self.base_planner, "motion_planner": None}
        self.ctx.config.ghost_timeout = 0.0
        self.ctx.world.mocap_relay = None
        self.ctx.kinematics = SimpleNamespace(base_pose=lambda serial: self.bases[serial], joints=lambda serial: {})
        self.guide = AssemblyGuidePlugin()
        self.guide.setup(self.ctx)
        self.guide.park_idle = False  # * parking has its own tests (test_park_*)

    async def ticks(self, count: int = 3) -> None:
        """Let the guide's tasks run, updating and drawing it each tick."""
        for _ in range(count):
            await self.ctx.next_tick()
            self.guide.update(self.ctx)
            self.guide.draw(self.ctx)

    def finish(self, label: str) -> None:
        """The executor finishes the current part, as if run from the Assembly tab."""
        part = self.guide.part
        assert part.label == label
        self.executor.done.setdefault(part.action, set()).update(part.done_keys)

    @property
    def label(self) -> str:
        """str: The current part's label."""
        return self.guide.part.label


def test_a_drive_runs_through_the_base_planner_and_done_goes_on(monkeypatch):
    """Prepare plans, Go commits, the base at its goal: Off stays on the drive until Done; then the move."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        assert lab.guide.part.kind == DRIVE
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.prompt.startswith("Drive cindy (0806)")
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        assert lab.guide.part.kind == DRIVE and lab.guide.procedure.outcome.label == "done", "Off stays"
        assert lab.guide._done_button.label == "Done: next part"
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.guide.part.kind == MOVE and lab.process.selection.movement.id == "B20_J_transfer"

    asyncio.run(walk())


def test_a_drive_sent_from_the_base_planner_panel_while_the_guide_asks_counts(monkeypatch):
    """Commit in Plan › Base sends the guide's plan: Go waits for that drive instead of sending it again."""
    lab = Guide(monkeypatch)
    commits = []
    send = lab.base_planner.commit

    def counted(plan_id, unknown_robots=(), clearance=True):
        commits.append(plan_id)
        return send(plan_id, unknown_robots, clearance)

    lab.base_planner.commit = counted

    async def walk():
        await lab.ticks()
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.prompt.startswith("Drive cindy (0806)")
        plan = lab.base_planner.plan
        await send(plan.id)  # * the panel's Commit
        lab.base_planner.plan = Plan(plan.id, plan.request, "sent", plan.path, "sent to 0806's follower")
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        assert commits == [], "not sent twice"
        assert lab.guide.procedure.outcome.label == "done"

    asyncio.run(walk())


def test_a_part_finished_in_the_assembly_tab_counts(monkeypatch):
    """The executor's finished movements count, whoever sent them: Off stays on the part, one Done goes on."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.guide.done(lab.ctx)
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)  # * skip the drive: no result, a second press
        await lab.ticks()
        lab.finish("B20_J_transfer: transfer, empty tools")
        await lab.ticks()
        assert lab.label == "B20_J_transfer: transfer, empty tools"
        assert [check.label for check in lab.guide._results(lab.ctx, lab.guide.index)] == ["executor finished"]
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.label == "B20_J_place_bar: by hand"
        assert lab.process.selection.movement.id == "B20_J_place_bar"

    asyncio.run(walk())


def test_done_goes_on_at_once_at_the_end_and_asks_twice_without_a_result(monkeypatch):
    """The base put on its goal by hand: one Done. Nothing there: Done asks, a second one skips; Redo undoes it."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.guide.done(lab.ctx)
        assert lab.guide.part.kind == DRIVE
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)
        assert lab.guide.part.kind == MOVE
        events = [fields["data"]["event"] for event, fields in lab.run.records if event == "guide"]
        assert events == ["skipped"]
        lab.guide.look(lab.ctx, -1)
        lab.guide.redo(lab.ctx)
        lab.ctx.t += 0.5
        lab.guide.redo(lab.ctx)
        await lab.ticks()
        assert lab.guide.part.kind == DRIVE and not lab.guide.is_done(lab.ctx, lab.guide.part, 0)
        part = lab.guide.part
        lab.bases[CINDY] = Pose((part.base[0], part.base[1], 0.0),
                                (0.0, 0.0, math.sin(part.base[2] / 2), math.cos(part.base[2] / 2)))
        await lab.ticks()
        lab.guide.done(lab.ctx)
        assert lab.guide.part.kind == MOVE, "at its end: one press"

    asyncio.run(walk())


def test_a_drive_counts_as_done_once_a_later_part_of_its_action_ran(monkeypatch):
    """After a restart the guide's own record of the drive is gone; the executor's of the move after it says it."""
    lab = Guide(monkeypatch)

    async def look():
        await lab.ticks()
        index = next(i for i, part in enumerate(lab.guide.parts) if part.action == "B20_J" and part.kind == DRIVE)
        drive, move = lab.guide.parts[index], lab.guide.parts[index + 1]
        assert move.kind == MOVE and not lab.guide.is_done(lab.ctx, drive, index), "Cindy is far from its start"
        lab.executor.done["B20_J"] = set(move.done_keys)
        assert lab.guide.is_done(lab.ctx, drive, index)
        assert [check.label for check in lab.guide._results(lab.ctx, index)] == ["driven"]

    asyncio.run(look())


def test_back_and_next_show_another_part_and_act_on_nothing(monkeypatch):
    """Back and Next move the part shown, and its selection; they plan, send and mark nothing."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.guide.look(lab.ctx, 2)
        await lab.ticks()
        assert lab.guide.index == 2 and lab.process.selection.movement.id == "B20_J_place_bar"
        lab.guide.look(lab.ctx, -5)
        await lab.ticks()
        assert lab.guide.index == 0
        assert lab.base_planner.requests == [] and lab.executor.requests == []
        assert all(status != DONE for status in lab.run.view.statuses.values())

    asyncio.run(walk())


def test_redo_from_a_marked_action_reopens_every_later_one(monkeypatch):
    """B20_J done and marked, then back to its insert: Redo reopens B20_J, the executor forgets the insert on."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        while lab.guide.part.action == "B20_J":
            part = lab.guide.part
            if part.kind == DRIVE:
                lab.guide.done(lab.ctx)
                lab.ctx.t += 0.5
            elif part.kind == "mark done":
                lab.run.request_mark_done("B20_J")
            else:
                lab.finish(part.label)
            lab.guide.done(lab.ctx)
            await lab.ticks()
        assert lab.run.view.statuses["B20_J"] == DONE
        while lab.guide.part.label != "B20_J_insert: insert":
            lab.guide.look(lab.ctx, -1)
        lab.guide.redo(lab.ctx)
        lab.ctx.t += 0.5
        lab.guide.redo(lab.ctx)
        await lab.ticks()
        assert lab.run.reopened == ["B20_J"]
        assert lab.guide.part.label == "B20_J_insert: insert"
        assert (lab.process.selection.action.id, lab.process.selection.movement.id) == ("B20_J", "B20_J_insert")
        assert ("B20_J", frozenset({(4, 4), (4, -1)})) in lab.executor.forgot
        assert ("B20_R", frozenset()) in lab.executor.forgot, "every later action entirely"

    asyncio.run(walk())


def test_a_part_by_hand_is_done_once_every_box_is_ticked(monkeypatch):
    """Prepare and Go stay grey; one Done is refused until every box is ticked, then records it; the servo is next."""
    lab = Guide(monkeypatch)

    async def walk():
        await to_the_bar_by_hand(lab)
        guide, boxes = lab.guide, lab.guide.kinds[HAND].boxes
        assert [box.label for box in boxes[:3]] == ["Bar placed", "Screws clamped", "Hands clear"]
        assert guide._prepare.disabled and guide._go.disabled and not guide._done_button.disabled
        guide.prepare(lab.ctx)
        guide.done(lab.ctx)
        await lab.ticks()
        assert lab.executor.requests == [], "nothing before every box is ticked"
        assert guide.refusal(lab.ctx.t).label == "done: press again"
        assert "0 of 3 boxes ticked" in guide._guide_line.content
        lab.ctx.t += 10.0
        for box in boxes[:2]:
            box.value = True
        guide.done(lab.ctx)
        await lab.ticks()
        assert lab.executor.requests == [], "one box left"
        lab.ctx.t += 10.0
        boxes[2].value = True
        guide.done(lab.ctx)
        await lab.ticks(15)
        assert len(lab.executor.requests) == 1 and lab.executor.prompt is None
        assert lab.label == "B20_J_transfer: servo"

    asyncio.run(walk())


def test_god_needs_a_full_simulation_and_auto_on_real_robots_a_second_press(monkeypatch):
    """No simulated relay: God is refused; Auto asks twice; a pick in the process panel ends it."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        assert simulation_problem(lab.ctx) == "no mocap relay runs"
        lab.guide.set_mode(lab.ctx, GOD)
        assert lab.guide.mode == OFF
        lab.guide.set_mode(lab.ctx, AUTO)
        assert lab.guide.mode == OFF
        lab.ctx.t += 0.5
        lab.guide.set_mode(lab.ctx, AUTO)
        assert lab.guide.mode == AUTO
        await lab.ticks()
        assert lab.guide.procedure.prompt.startswith("Drive"), "Auto waits at a real robot's drive"
        lab.process.request_select(lab.process.selection.index + 1)
        await lab.ticks()
        assert lab.guide.mode == OFF

    asyncio.run(walk())


def test_the_gate_counts_every_part_of_every_robot():
    """A simulated relay, bases, arms and tools: open; one real tool closes it."""
    simulated = SimpleNamespace(kind="sim")
    arm = SimpleNamespace(simulation=simulated, config=SimpleNamespace(end_effector_namespace="/a/gripper"))
    robot = SimpleNamespace(base=SimpleNamespace(simulation=simulated), arms={"ur_arm": arm},
                            simulated=SimpleNamespace(get=lambda part: simulated))
    ctx = SimpleNamespace(world=SimpleNamespace(mocap_relay=SimpleNamespace(state=SimpleNamespace(simulated=True)),
                                                robots={"a200-0804": robot}))
    assert simulation_problem(ctx) == ""
    robot.simulated = SimpleNamespace(get=lambda part: None)
    assert simulation_problem(ctx) == "a200-0804 ur_arm's tool is real"


async def to_the_bar_by_hand(lab: Guide) -> None:
    """Skip the drive, finish the transfer, Done: the part by hand is current."""
    await lab.ticks()
    lab.guide.done(lab.ctx)
    lab.ctx.t += 0.5
    lab.guide.done(lab.ctx)
    await lab.ticks()
    lab.finish("B20_J_transfer: transfer, empty tools")
    lab.guide.done(lab.ctx)
    await lab.ticks()
    assert lab.guide.part.kind == HAND


async def ask_by_hand(lab: Guide) -> None:
    """Every box ticked, Done: the executor's question is open, not yet answered."""
    lab.guide.kinds[HAND].tick_all()
    lab.guide.done(lab.ctx)
    await lab.ticks(2)
    assert lab.executor.prompt is not None


def test_cancel_at_the_executors_question_answers_it_and_ends_auto(monkeypatch):
    """Cancel while the executor asks: it is told to cancel, and nothing is left asking."""
    lab = Guide(monkeypatch)

    async def walk():
        await to_the_bar_by_hand(lab)
        await ask_by_hand(lab)
        lab.guide.cancel(lab.ctx)
        await lab.ticks()
        assert lab.executor.cancelled == 1 and lab.executor.prompt is None
        assert not lab.guide.procedure.running and lab.guide.part.kind == HAND

    asyncio.run(walk())


def test_redo_is_refused_while_another_part_runs(monkeypatch):
    """The executor asks for the part by hand; Redo on the transfer shown says where it runs and does nothing."""
    lab = Guide(monkeypatch)

    async def walk():
        await to_the_bar_by_hand(lab)
        await ask_by_hand(lab)
        lab.guide.look(lab.ctx, -1)
        lab.guide.redo(lab.ctx)
        assert lab.guide.refusal(lab.ctx.t).label == "redo from here refused"
        assert lab.executor.forgot == []

    asyncio.run(walk())


def test_what_runs_belongs_to_its_part(monkeypatch):
    """The drive waits for Go; shown elsewhere, the guide line names it, and Go there is refused."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        before = lab.bases[CINDY]
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.prompt and "▶1" in lab.guide._overview.content
        lab.guide.look(lab.ctx, 1)
        await lab.ticks()
        assert "B20_J part 1 waits for Go" in lab.guide._guide_line.content
        lab.guide.press_go(lab.ctx)
        assert lab.guide.refusal(lab.ctx.t).label == "go refused" and lab.bases[CINDY] is before

    asyncio.run(walk())


def test_out_of_order_asks_twice(monkeypatch):
    """The bar by hand shown, the drive and transfer before it not done: the first Done is refused, the second goes."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.guide.look(lab.ctx, 2)
        await lab.ticks()
        assert lab.guide.part.kind == HAND and lab.guide.undone_before(lab.ctx, 2) == [0, 1]
        lab.guide.kinds[HAND].tick_all()
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.executor.requests == [] and "out of order" in lab.guide.refusal(lab.ctx.t).detail
        assert lab.guide._done_button.label == "Done?"
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)
        await lab.ticks(15)
        assert len(lab.executor.requests) == 1

    asyncio.run(walk())


def test_auto_works_on_the_next_part_to_do(monkeypatch):
    """B20_J and B20_R marked: Auto shows B21_J's first part, the next part to do, and waits there at the drive's Go."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.run.request_mark_done("B20_J")
        lab.run.request_mark_done("B20_R")
        await lab.ticks()
        lab.guide.set_mode(lab.ctx, AUTO)
        lab.ctx.t += 0.5
        lab.guide.set_mode(lab.ctx, AUTO)
        await lab.ticks(5)
        assert lab.guide.mode == AUTO and lab.guide.part.action == "B21_J" and lab.guide.part.kind == DRIVE
        assert lab.guide.procedure.prompt.startswith("Drive cindy (0806)")
        assert lab.guide.index == lab.guide.next_to_do(lab.ctx)

    asyncio.run(walk())


def test_a_new_run_forgets_what_the_guide_judged_done(monkeypatch):
    """The drive skipped in one run is to do again in the next."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.run.view.folder = "runs/1"
        await lab.ticks()
        lab.guide.done(lab.ctx)
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.guide.is_done(lab.ctx, lab.guide.parts[0], 0)
        lab.run.view.folder = "runs/2"
        await lab.ticks()
        assert not lab.guide.is_done(lab.ctx, lab.guide.parts[0], 0)

    asyncio.run(walk())


def test_every_part_of_an_action_the_run_marked_is_done(monkeypatch):
    """A run continued after a restart: B20_J marked, the executor's memory empty; all its parts are done."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.run.request_mark_done("B20_J")
        await lab.ticks()
        parts = [(index, part) for index, part in enumerate(lab.guide.parts) if part.action == "B20_J"]
        assert all(lab.guide.is_done(lab.ctx, part, index) for index, part in parts)
        assert [check.label for check in lab.guide._results(lab.ctx, parts[1][0])] == ["action marked"]

    asyncio.run(walk())


class ArmPlanner:
    """The arm planner: solves any goal, and finds a two-row path to it, or none."""

    def __init__(self, finds: bool, turned: dict[str, float] | None = None):
        """Whether its searches find a path; joints its IK answers turned by these radians from the reference."""
        self.finds, self.requests, self.solved, self.turned = finds, [], [], turned or {}

    def solve_goals(self, serial, targets, reference=None):
        """The reference itself as the goal of every arm, turned where asked."""
        self.solved.append((serial, tuple(targets)))
        goal = {name: value + self.turned.get(name, 0.0) for name, value in (reference or {}).items()}
        return answered((goal, "goal by IK at the live base (ssik, rank 1 of 8)"))

    def request_plan(self, request):
        """A path from zeros to the goal, or a failed plan."""
        self.requests.append(request)
        if not self.finds:
            return answered(Plan(len(self.requests), request, "failed", None, "no path in 10 s"))
        rows = np.array([(0.0,) * len(request.goals), request.goals])
        # * As the real planner's: its 0.2 s samples, and its waypoints apart from them.
        path = ArmPath.along([f"j{i}" for i in range(len(request.goals))], rows, [])
        return answered(Plan(len(self.requests), request, "ready", path, "found"))


def test_a_one_arm_move_is_planned_by_the_arm_planner_and_fails_without_a_path(monkeypatch):
    """Alice's approach: the arm planner's path becomes a route for the executor; no path fails, no motion planner."""
    lab = Guide(monkeypatch, "B21_H_approach")
    lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True)

    async def plan():
        await lab.ticks()
        part = next(each for each in lab.guide.parts if each.label == "B21_H_approach: approach")
        loaded = lab.run.view.process
        step = loaded.steps[loaded.index_of(part.action, part.step.primary)]
        return await lab.guide._plan_on_arm_planner(lab.ctx, part, "a200-0804", step)

    route, basis, path, warned = asyncio.run(plan())
    assert route.ok and route.planner == "arm_planner.rrt_connect" and not warned
    assert len(route.result.trajectory.positions) == 2, "the planner's two waypoints, not its 0.2 s samples"
    assert route.result.trajectory.robot == "robots/alice" and len(route.result.trajectory.joint_names) == 6
    assert (basis.serial, basis.action) == ("a200-0804", "B21_H") and path.duration > 0.0
    assert lab.ctx.plugins["arm_planner"].solved == [("a200-0804", ("ur_arm",))], "the approach's goal: IK at the base"

    lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=False)
    with pytest.raises(RuntimeError, match="no path \\(arm planner\\)"):
        asyncio.run(plan())
    assert len(lab.ctx.plugins["arm_planner"].requests) == 1, "Off: one search, no motion planner after it"
    lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=False)
    lab.guide.mode = AUTO
    with pytest.raises(RuntimeError, match="no path"):
        asyncio.run(plan())
    assert len(lab.ctx.plugins["arm_planner"].requests) == 3, "Auto searches again before it gives up"
    lab.guide.mode = OFF
    lab.guide.arm_planner_moves = False
    lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True)
    assert asyncio.run(plan()) is None and lab.ctx.plugins["arm_planner"].requests == [], "the box off"


def test_a_transfer_goal_turns_each_joint_to_the_value_nearest_the_arms_now(monkeypatch):
    """Cindy's B21_J transfer, IK a whole turn off on one wrist: the search gets it a turn back, as motion planner."""
    wrist = "left_ur_arm_wrist_3_joint"
    lab = Guide(monkeypatch, "B21_J_transfer")
    planner = lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True, turned={wrist: 2 * np.pi})

    async def plan():
        await lab.ticks()
        part = next(each for each in lab.guide.parts if each.movement == "B21_J_transfer" and each.kind != DRIVE)
        loaded = lab.run.view.process
        step = loaded.steps[loaded.index_of(part.action, part.step.primary)]
        return await lab.guide._plan_on_arm_planner(lab.ctx, part, "a200-0806", step)

    asyncio.run(plan())
    request, = planner.requests
    names = [*arm_joint_names(request.arm), *(name for arm, _goal in request.also for name in arm_joint_names(arm))]
    goal = dict(zip(names, request.goals))
    now = lab.ctx.scene.snapshot.robots[robot_id("a200-0806")].joints
    assert planner.solved == [("a200-0806", ("left_ur_arm", "right_ur_arm"))], "a transfer's goal: IK at the base"
    assert abs(goal[wrist] - now[wrist]) <= np.pi, "the turn nearest the arm now, not the one IK answered"


def test_the_needs_line_names_the_planner_and_turns_amber_on_a_fallback(monkeypatch):
    """A drive says base planner, a free move arm planner, then motion planner in amber once that planned it."""
    lab = Guide(monkeypatch, "B21_H_approach")
    asyncio.run(lab.ticks())
    approach = next(each for each in lab.guide.parts if each.label == "B21_H_approach: approach")
    drive = next(each for each in lab.guide.parts if each.kind == DRIVE)
    assert lab.guide._planner_check(drive).label == "base planner"
    assert (lab.guide._planner_check(approach).label, lab.guide._planner_check(approach).level) == ("arm planner", INFO)
    lab.guide._planned = (approach, "motion planner")
    assert (lab.guide._planner_check(approach).label, lab.guide._planner_check(approach).level) == ("motion planner",
                                                                                                    WARN)
    lab.guide.arm_planner_moves = False
    assert (lab.guide._planner_check(approach).label, lab.guide._planner_check(approach).level) == ("motion planner",
                                                                                                    INFO)


def test_done_goes_on_past_parts_done_already_and_next_steps_one(monkeypatch):
    """By hand and the servo ran from the Assembly tab: Done on the transfer shows the insert; Back shows the servo."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        for part in lab.guide.parts:
            if part.label in ("B20_J_place_bar: by hand", "B20_J_transfer: servo"):
                lab.executor.done.setdefault(part.action, set()).update(part.done_keys)
        lab.guide.done(lab.ctx)
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.label == "B20_J_transfer: transfer, empty tools"
        lab.finish("B20_J_transfer: transfer, empty tools")
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.label == "B20_J_insert: insert"
        assert any("B20_J_place_bar: by hand, B20_J_transfer: servo done already" in line
                   for line in lab.ctx.lines("info"))
        lab.guide.look(lab.ctx, -1)
        assert lab.label == "B20_J_transfer: servo"

    asyncio.run(walk())


def test_redo_unticks_the_boxes_of_a_part_by_hand(monkeypatch):
    """Two boxes ticked, then Redo from the transfer: the part by hand shows its boxes unticked again."""
    lab = Guide(monkeypatch)

    async def walk():
        await to_the_bar_by_hand(lab)
        boxes = lab.guide.kinds[HAND].boxes
        boxes[0].value = boxes[1].value = True
        await lab.ticks()
        lab.guide.look(lab.ctx, -1)
        assert lab.label == "B20_J_transfer: transfer, empty tools"
        lab.guide.redo(lab.ctx)
        lab.ctx.t += 0.5
        lab.guide.redo(lab.ctx)
        await lab.ticks()
        assert not lab.guide.is_done(lab.ctx, lab.guide.part, lab.guide.index)
        lab.guide.look(lab.ctx, 1)
        await lab.ticks()
        assert lab.guide.part.kind == HAND and lab.guide.kinds[HAND].count == (0, 3)

    asyncio.run(walk())


def test_redo_makes_a_park_standing_at_its_end_to_do_until_its_robot_left_and_came_back(monkeypatch):
    """Belle parked already: done. Redo from Alice's park: Belle's park is to do though she stands parked.

    Once she drove off and back it counts again.
    """
    lab = Guide(monkeypatch, "B25_J_transfer")
    lab.guide.park_idle = True
    belle = robot_config("a200-0805")
    x, y, yaw = belle.park_pose
    parked = Pose((x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    lab.bases["a200-0805"] = parked
    lab.belle.arms["ur_arm"].state.joint_positions.update(
        {f"ur_arm_{name}": float(value) for name, value in zip(UR_JOINT_NAMES, belle.arms[0].stow_joints)})

    async def walk():
        await lab.ticks()
        assert (lab.guide.part.kind, lab.guide.part.robot) == (PARK, "a200-0804")
        index = lab.guide.index + 1
        belle_part = lab.guide.parts[index]
        assert belle_part.robot == "a200-0805" and lab.guide.is_done(lab.ctx, belle_part, index)
        lab.guide.redo(lab.ctx)
        lab.ctx.t += 0.5
        lab.guide.redo(lab.ctx)
        await lab.ticks()
        assert not lab.guide.is_done(lab.ctx, belle_part, index), "redone, though she stands parked"
        lab.bases["a200-0805"] = Pose((x + 1.0, y, 0.0), parked.orientation)
        await lab.ticks()
        lab.bases["a200-0805"] = parked
        await lab.ticks()
        assert lab.guide.is_done(lab.ctx, belle_part, index), "parked again after the redo"

    asyncio.run(walk())


def test_redo_only_with_a_done_part_from_there_on(monkeypatch):
    """Nothing done from the part shown on: nothing to redo; B20_J marked, Redo on its first part reopens it."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        assert lab.guide._cannot(lab.ctx, REDO) == "nothing from this part on is done: nothing to redo"
        lab.run.request_mark_done("B20_J")
        await lab.ticks()
        lab.guide.look(lab.ctx, -lab.guide.index)
        assert lab.guide.index == 0 and not lab.guide._cannot(lab.ctx, REDO)
        lab.guide.redo(lab.ctx)
        lab.ctx.t += 0.5
        lab.guide.redo(lab.ctx)
        await lab.ticks()
        assert lab.run.reopened == ["B20_J"] and lab.guide.index == 0
        assert not lab.guide.is_done(lab.ctx, lab.guide.part, 0)

    asyncio.run(walk())


def test_both_arms_go_home_in_one_arm_planner_search(monkeypatch):
    """Cindy's B20_R way home: one request for both arms, their goals from the design, one route of twelve joints."""
    lab = Guide(monkeypatch, "B20_R_home")
    planner = lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True)

    async def plan():
        await lab.ticks()
        part = next(each for each in lab.guide.parts if each.label == "B20_R_home: home")
        loaded = lab.run.view.process
        step = loaded.steps[loaded.index_of(part.action, part.step.primary)]
        return await lab.guide._plan_on_arm_planner(lab.ctx, part, "a200-0806", step)

    route, _basis, _path, _warned = asyncio.run(plan())
    request, = planner.requests
    assert (request.arm, [arm for arm, _goal in request.also]) == ("left_ur_arm", ["right_ur_arm"])
    assert len(request.goals) == 12 and planner.solved == [], "home is a joint goal: no IK"
    assert len(route.result.trajectory.joint_names) == 12 and route.planner == "arm_planner.rrt_connect"


def test_the_arm_planner_waits_until_the_live_scene_shows_the_part(monkeypatch):
    """The live scene still shows the retreat (its bars, its contacts): nothing is planned until it shows the home."""
    lab = Guide(monkeypatch, "B20_R_home")
    planner = lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True)

    async def plan():
        await lab.ticks()
        part = next(each for each in lab.guide.parts if each.label == "B20_R_home: home")
        lab.process.scene_shows = ("B20_R", "B20_R_retreat")
        task = asyncio.ensure_future(lab.guide._plan(lab.ctx, part, "a200-0806"))
        await lab.ticks(3)
        assert planner.requests == planner.solved == [] and not task.done(), "not in the retreat's scene"
        lab.process.scene_shows = None
        await lab.ticks(3)
        assert planner.requests == planner.solved == [], "not while the bodies swap"
        lab.process.scene_shows = ("B20_R", "B20_R_home")
        return await task

    route, _basis, _path, _warned = asyncio.run(plan())
    assert route.ok and len(planner.requests) == 1


def test_a_plan_is_refused_when_the_live_scene_changes_meanwhile(monkeypatch):
    """Another movement selected while the arm planner works: its path was planned among the wrong bars, never used."""
    lab = Guide(monkeypatch, "B20_R_home")
    planner = lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True)
    asked = planner.request_plan

    async def meanwhile(request):
        lab.process.scene_shows = ("B20_R", "B20_R_retreat")
        return await asked(request)

    planner.request_plan = meanwhile

    async def plan():
        await lab.ticks()
        part = next(each for each in lab.guide.parts if each.label == "B20_R_home: home")
        await lab.guide._plan(lab.ctx, part, "a200-0806")

    with pytest.raises(SceneNotShown, match="changed while B20_R_home was planned"):
        asyncio.run(plan())


class StowPlanner:
    """Plans at once; its commit puts the arms on their goals."""

    def __init__(self, robots):
        """No plan yet."""
        self.robots, self.plan, self.requests = robots, None, []

    def request_plan(self, request):
        """A path that stands at the goal."""
        self.requests.append(request)
        goal = np.asarray(request.goals, dtype=float)
        self.plan = Plan(2, request, "ready", TimedPath(np.array([0.0, 2.0]), np.array([goal, goal])), "found")
        return answered(self.plan)

    def commit(self, plan_id, unknown_robots=(), clearance=True):
        """Put each planned arm on its goal."""
        request = self.plan.request
        for arm, goal in zip(request.arms, np.reshape(request.goals, (-1, 6))):
            state = self.robots[request.serial].arms[arm].state
            state.joint_positions.update({f"{arm}_{name}": float(value) for name, value in zip(UR_JOINT_NAMES, goal)})
        return answered((True, "sent"))

    def cancel_commit(self, serial=None):
        """Nothing runs."""
        return False


def test_park_stows_the_arm_then_drives_and_a_parked_robot_has_nothing_to_do(monkeypatch):
    """A park part stows its robot's arms, then drives it to its parked pose; one parked already is done.

    B25_J parks Alice, then Belle: Prepare plans the stow, Go stows, the drive's Go parks. Belle, parked already, is
    done without moving, and the next part is Cindy's drive.
    """
    lab = Guide(monkeypatch, "B25_J_transfer")
    lab.guide.park_idle = True
    planner = lab.ctx.plugins["arm_planner"] = StowPlanner(lab.robots)
    # * Alice's arm out of its stow, Belle parked and stowed already.
    lab.alice.arms["ur_arm"].state.joint_positions["ur_arm_shoulder_pan_joint"] = 0.5
    belle = robot_config("a200-0805")
    x, y, yaw = belle.park_pose
    lab.bases["a200-0805"] = Pose((x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    lab.belle.arms["ur_arm"].state.joint_positions.update(
        {f"ur_arm_{name}": float(value) for name, value in zip(UR_JOINT_NAMES, belle.arms[0].stow_joints)})

    async def walk():
        await lab.ticks()
        assert (lab.guide.part.kind, lab.guide.part.robot) == (PARK, "a200-0804")
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.prompt.startswith("Stow alice (0804)'s arms")
        lab.guide.press_go(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.prompt.startswith("Drive alice (0804), park alice (0804)")
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        assert lab.guide.procedure.outcome.label == "done"
        assert lab.guide.at_end(lab.ctx, lab.guide.part)
        belle_part = lab.guide.parts[lab.guide.index + 1]
        assert belle_part.robot == "a200-0805" and lab.guide.is_done(lab.ctx, belle_part, lab.guide.index + 1)
        lab.guide.done(lab.ctx)
        await lab.ticks()
        return lab.guide.next_to_do(lab.ctx, lab.guide.index)

    following = asyncio.run(walk())
    assert [request.serial for request in planner.requests] == ["a200-0804"]
    assert lab.base_planner.requests[0].goal == tuple(robot_config("a200-0804").park_pose)
    assert lab.guide.parts[following].kind == DRIVE and lab.guide.parts[following].robot == ""


class NoStow(StowPlanner):
    """Finds no stow path."""

    def request_plan(self, request):
        """A failed plan."""
        self.requests.append(request)
        return answered(Plan(2, request, "failed", None, "no path in 10 s"))


def test_a_park_that_fails_warns_and_holds_nothing_back(monkeypatch):
    """A park that fails warns and is left, one Done leaves another, and the drive after them goes on its first Go.

    Alice's stow finds no path: "not parked"; Belle's park is left by one Done; then Cindy's drive.
    """
    lab = Guide(monkeypatch, "B25_J_transfer")
    lab.guide.park_idle = True
    lab.ctx.plugins["arm_planner"] = NoStow(lab.robots)
    lab.alice.arms["ur_arm"].state.joint_positions["ur_arm_shoulder_pan_joint"] = 0.5

    async def walk():
        await lab.ticks()
        alice = lab.guide.index
        assert (lab.guide.part.kind, lab.guide.part.robot) == (PARK, "a200-0804")
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.outcome.label == "not parked" and lab.guide.is_done(lab.ctx, lab.guide.part, alice)
        assert lab.guide._state(lab.ctx, alice).label == "not parked"
        assert any("not parked (no stow path" in line for line in lab.ctx.lines("warn"))
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert (lab.guide.part.kind, lab.guide.part.robot) == (PARK, "a200-0805")
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.guide._results(lab.ctx, alice + 1)[0].label == "skipped"
        assert lab.guide.part.kind == DRIVE and lab.guide.part.robot == ""
        assert lab.guide.undone_before(lab.ctx, lab.guide.index) == []
        needs = {check.label: check for check in lab.guide._needs(lab.ctx, lab.guide.index)}
        assert "not parked" not in needs, "both parks were left: nothing left to do there"
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        assert lab.guide.procedure.outcome.label == "done"

    asyncio.run(walk())


def test_no_goal_at_the_live_base_flags_the_drive_before_and_it_drives_again(monkeypatch):
    """The transfer finds no goal where Cindy's drive ended: the drive is flagged not good, and Prepare drives again.

    The base stands at its goal, so without the flag the drive would have nothing to do.
    """
    lab = Guide(monkeypatch)
    lab.guide.arm_planner_moves = False
    lab.ctx.plugins["motion_planner"] = SimpleNamespace(request_ik=lambda request: answered(
        GoalCheck("B20_J_transfer", None, False, "left_ur_arm: ssik finds no solution")))

    async def walk():
        await lab.ticks()
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.guide.part.kind == MOVE
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        outcome = lab.guide.procedure.outcome
        assert outcome.level == BAD and "no goal at the live base" in outcome.detail and "flagged" in outcome.detail
        assert lab.guide._state(lab.ctx, 0).label == "not good"
        assert not lab.guide.is_done(lab.ctx, lab.guide.parts[0], 0)
        assert lab.guide.next_to_do(lab.ctx) == 0
        lab.guide.look(lab.ctx, -1)
        assert lab.guide.at_end(lab.ctx, lab.guide.part)
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert len(lab.base_planner.requests) == 2 and lab.guide.procedure.prompt.startswith("Drive cindy (0806)")
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        assert lab.guide.is_done(lab.ctx, lab.guide.part, 0) and 0 not in lab.guide._flagged

    asyncio.run(walk())


def test_done_twice_confirms_a_part_by_hand_with_its_boxes_unticked(monkeypatch):
    """Trust me: the second Done ticks every box, warns and records the part."""
    lab = Guide(monkeypatch)

    async def walk():
        await to_the_bar_by_hand(lab)
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.executor.requests == [] and lab.guide._done_button.label == "Done?"
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)
        await lab.ticks(15)
        assert len(lab.executor.requests) == 1 and lab.label == "B20_J_transfer: servo"
        assert any("Done anyway (only 0 of 3 boxes ticked" in line for line in lab.ctx.lines("warn"))

    asyncio.run(walk())


def test_prepare_twice_does_a_done_drive_again_and_keeps_later_parts(monkeypatch):
    """A done drive: one Prepare is refused, the second plans the drive again though the base stands at its goal."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        assert lab.guide.is_done(lab.ctx, lab.guide.part, 0) and len(lab.base_planner.requests) == 1
        lab.ctx.t += 10.0
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert len(lab.base_planner.requests) == 1 and "press Prepare again" in lab.guide.refusal(lab.ctx.t).detail
        assert lab.guide._prepare.label == "Prepare?"
        lab.ctx.t += 0.5
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert len(lab.base_planner.requests) == 2 and lab.guide.procedure.prompt.startswith("Drive cindy (0806)")
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        assert lab.guide.is_done(lab.ctx, lab.guide.part, 0)

    asyncio.run(walk())


def test_done_twice_marks_an_action_whose_robot_is_not_live(monkeypatch):
    """Cindy not live: one Done on Mark done is refused, the second asks the run twice (it assumes the export's end)."""
    lab = Guide(monkeypatch)
    monkeypatch.setattr(guide_module, "is_live", lambda robot, now: False)

    async def walk():
        await lab.ticks()
        mark = next(index for index, part in enumerate(lab.guide.parts) if part.kind == MARK)
        lab.guide._show(lab.ctx, mark)
        lab.guide.done(lab.ctx)
        await lab.ticks()
        assert lab.run.marked == [] and "cindy (0806) is not live" in lab.guide.refusal(lab.ctx.t).detail
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)
        await lab.ticks(5)
        action = lab.guide.parts[mark].action
        assert lab.run.marked == [action, action] and lab.run.view.statuses[action] == DONE

    asyncio.run(walk())


# --- --- --- --- --- review fixes (6 Nov): what is checked, and what the operator sees --- --- --- --- ---


def guide_events(lab: Guide) -> list[dict]:
    """The guide's events in the run, their data."""
    return [fields["data"] for event, fields in lab.run.records if event == "guide"]


async def auto_on_real_robots(lab: Guide) -> None:
    """Auto on real robots: two presses."""
    lab.guide.set_mode(lab.ctx, AUTO)
    assert lab.guide.refusal(lab.ctx.t).label == "mode: press again"
    lab.guide.draw(lab.ctx)
    assert lab.guide._mode._labels == (OFF, "Auto?", GOD), "Auto's second press looks like every other"
    lab.ctx.t += 0.5
    lab.guide.set_mode(lab.ctx, AUTO)
    assert lab.guide.mode == AUTO


def test_prepare_twice_runs_a_part_of_a_marked_action_and_the_mark_stays(monkeypatch):
    """B20_J marked: Prepare on its drive asks once (no Redo needed), the second plans it; the run keeps the mark."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.run.request_mark_done("B20_J")
        lab.guide._show(lab.ctx, 0)
        await lab.ticks()
        lab.guide.prepare(lab.ctx)
        refusal = lab.guide.refusal(lab.ctx.t)
        assert refusal.label == "prepare: press again" and "marked done" in refusal.detail
        lab.ctx.t += 0.5
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.prompt.startswith("Drive cindy (0806)")
        assert lab.run.view.statuses["B20_J"] == DONE and lab.run.reopened == []

    asyncio.run(walk())


def test_done_drops_a_plan_waiting_for_go_and_auto_goes_on(monkeypatch):
    """Auto waits at the drive's Go (real robots): Done twice (Skip) drops that plan; Auto plans the transfer next."""
    lab = Guide(monkeypatch)
    lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True)

    async def walk():
        await lab.ticks()
        await auto_on_real_robots(lab)
        await lab.ticks()
        assert lab.guide.part.kind == DRIVE and lab.guide.procedure.waiting
        lab.guide.done(lab.ctx)
        assert lab.guide.refusal(lab.ctx.t).label == "done: press again", "no result yet: Skip asks"
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)
        await lab.ticks(4)
        assert lab.guide.mode == AUTO, "dropping the plan on purpose is no reason to end Auto"
        assert lab.guide.part.label == "B20_J_transfer: transfer, empty tools"
        assert lab.guide.procedure.prompt.startswith("Run B20_J_transfer"), "Auto waits at a real robot's move"
        assert lab.bases[CINDY] == Pose((3.0, 1.0, 0.0)), "the drive never went"

    asyncio.run(walk())


def test_prepare_on_another_part_replaces_a_plan_waiting_for_go(monkeypatch):
    """Off, the drive waits for Go: Prepare on the transfer plans that instead (nothing refused), Auto unaffected."""
    lab = Guide(monkeypatch)
    lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True)

    async def walk():
        await lab.ticks()
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.waiting
        lab.guide.look(lab.ctx, 1)
        lab.guide.prepare(lab.ctx)
        await lab.ticks(3)
        assert lab.guide.refusal(lab.ctx.t) is None
        assert lab.guide.owner is lab.guide.part and lab.guide.procedure.prompt.startswith("Run B20_J_transfer")
        assert lab.guide._state(lab.ctx, 0).label == "to do", "the drive's cancel is no failure"

    asyncio.run(walk())


def test_why_auto_stopped_stays_on_the_guide_line_and_in_the_run(monkeypatch):
    """Alice soft stopped: Auto ends at once, the guide line and a warning below the buttons say why; the run too."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.alice.soft_stop_latch.stopped = True
        await auto_on_real_robots(lab)
        await lab.ticks()
        assert lab.guide.mode == OFF
        assert "auto stopped" in lab.guide._guide_line.content
        assert "alice (0804) is soft stopped" in lab.guide._notes.content
        assert guide_events(lab)[-1]["event"] == "mode" and "soft stopped" in guide_events(lab)[-1]["why"]
        lab.ctx.t += 10.0
        await lab.ticks()
        assert "auto stopped" in lab.guide._guide_line.content, "it stays until the mode is chosen again"
        lab.alice.soft_stop_latch.stopped = False
        await auto_on_real_robots(lab)
        await lab.ticks()
        assert "auto stopped" not in lab.guide._guide_line.content

    asyncio.run(walk())


class FailingBasePlanner(BasePlanner):
    """Finds no path."""

    def request_plan(self, request):
        """No path: the start is in collision."""
        self.requests.append(request)
        return answered(Plan(1, request, "failed", None, "no plan: the start is in collision"))


def test_a_failure_outlives_showing_another_part(monkeypatch):
    """The drive fails: Next and Back keep its red "failed" and why; Prepare again clears it."""
    lab = Guide(monkeypatch)
    lab.ctx.plugins["base_planner"] = FailingBasePlanner(lab.bases)

    async def walk():
        await lab.ticks()
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide._state(lab.ctx, 0).label == "failed"
        lab.guide.look(lab.ctx, 1)
        await lab.ticks()
        lab.guide.look(lab.ctx, -1)
        await lab.ticks()
        assert lab.guide._state(lab.ctx, 0).label == "failed"
        assert "no plan: the start is in collision" in lab.guide._notes.content
        assert "no path: no plan" not in lab.guide._notes.content, "one prefix"
        lab.ctx.plugins["base_planner"] = lab.base_planner
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide._state(lab.ctx, 0).label == "to do"

    asyncio.run(walk())


def test_auto_never_marks_past_a_doubt(monkeypatch):
    """Auto switched on at B20_J's Mark done, its parts not done: it ends and says why; nothing is marked."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        mark = next(index for index, part in enumerate(lab.guide.parts) if part.kind == MARK)
        lab.guide._show(lab.ctx, mark)
        await lab.ticks()
        await auto_on_real_robots(lab)
        await lab.ticks()
        assert lab.guide.mode == OFF and lab.run.marked == []
        assert "out of order" in lab.guide._auto_end.detail and "Done twice" in lab.guide._auto_end.detail

    asyncio.run(walk())


def test_parts_built_again_end_auto_and_wait_for_what_runs(monkeypatch):
    """Park idle robots switched on: not while the drive waits for Go (its index must hold); once it ends, Auto ends."""
    lab = Guide(monkeypatch)

    async def walk():
        await to_the_bar_by_hand(lab)
        await auto_on_real_robots(lab)
        await lab.ticks()
        assert lab.guide.mode == AUTO and "auto waits: tick the boxes" in lab.guide._guide_line.content
        lab.guide.look(lab.ctx, -1)
        await lab.ticks()
        assert "auto waits at B20_J part 3" in lab.guide._guide_line.content
        lab.guide.look(lab.ctx, 1)
        count = len(lab.guide.parts)
        lab.guide.park_idle = True
        await lab.ticks()
        assert len(lab.guide.parts) > count, "Belle and Alice park first"
        assert lab.guide.mode == OFF and "built again" in lab.guide._auto_end.detail

    asyncio.run(walk())


def test_parts_are_not_built_again_while_a_part_runs(monkeypatch):
    """Park idle robots switched on while the drive waits for Go: the parts stay until it ends."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        parts = lab.guide.parts
        lab.guide.park_idle = True
        await lab.ticks()
        assert lab.guide.parts is parts and lab.guide.procedure.waiting
        lab.guide.cancel(lab.ctx)
        await lab.ticks()
        assert lab.guide.parts is not parts

    asyncio.run(walk())


def test_out_of_order_go_with_an_unknown_robot_takes_one_second_press(monkeypatch):
    """The transfer before the drive, Alice unknown: the first Go asks for both, the second goes."""
    from husky_assembly_teleop.shared.guide import procedure as procedure_module
    from husky_assembly_teleop.shared.planner import planner_panel
    lab = Guide(monkeypatch)
    lab.ctx.plugins["arm_planner"] = ArmPlanner(finds=True)
    unknown = lambda _snapshot, _acting: frozenset({"robots/alice"})  # noqa: E731
    monkeypatch.setattr(planner_panel, "unknown_robots", unknown)
    monkeypatch.setattr(procedure_module, "unknown_robots", unknown)

    async def walk():
        await lab.ticks()
        lab.guide.look(lab.ctx, 1)
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.prompt.startswith("Run B20_J_transfer")
        lab.guide.press_go(lab.ctx)
        detail = lab.guide.refusal(lab.ctx.t).detail
        assert "out of order" in detail and "1 other robot unknown" in detail and lab.executor.requests == []
        lab.ctx.t += 0.5
        lab.guide.press_go(lab.ctx)
        await lab.ticks()
        assert len(lab.executor.requests) == 1 and lab.executor.requests[0].unknown_robots == {"robots/alice"}
        assert guide_events(lab)[-1]["event"] in ("out of order", "done")
        assert "out of order" in [event["event"] for event in guide_events(lab)]

    asyncio.run(walk())


def test_a_goal_solved_at_one_base_says_nothing_at_another(monkeypatch):
    """The transfer's goal was solved where Cindy stands: at its end there; 1 cm away it is to do, not the design's."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        transfer = next(part for part in lab.guide.parts if part.label == "B20_J_transfer: transfer, empty tools")
        joints = dict(lab.ctx.scene.snapshot.robots[f"robots/{CINDY}"].joints)
        names = [name for name in joints if name.startswith("left_ur_arm_")]
        lab.guide._goal = (transfer, {name: joints[name] for name in names}, lab.bases[CINDY])
        assert lab.guide.at_end(lab.ctx, transfer, NOTHING_TO_MOVE)
        lab.bases[CINDY] = Pose((3.01, 1.0, 0.0))
        await lab.ticks()
        assert not lab.guide.at_end(lab.ctx, transfer) and lab.guide._move_goal(lab.ctx, transfer) is None

    asyncio.run(walk())


def test_mark_done_waits_only_for_its_robot_and_done_twice_marks_one_never_still(monkeypatch):
    """Cindy's wheel states stale (counts as moving): the mark fails after the wait; Done twice then marks it."""
    lab = Guide(monkeypatch)
    monkeypatch.setattr(guide_module, "STILL_WAIT", 0.3)
    monkeypatch.setattr(guide_module, "robot_moving", lambda serial, robot, now: (
        "can't tell whether 0806 base is moving" if serial == CINDY else ""))

    async def walk():
        await lab.ticks()
        mark = next(index for index, part in enumerate(lab.guide.parts) if part.kind == MARK)
        for index in range(mark):
            lab.guide._done.add(index)
        lab.guide._show(lab.ctx, mark)
        lab.guide.done(lab.ctx)
        await lab.ticks(6)
        assert lab.run.marked == [] and "Done twice marks it anyway" in lab.guide._notes.content
        lab.guide.done(lab.ctx)
        assert "can't tell whether 0806 base is moving" in lab.guide.refusal(lab.ctx.t).detail
        lab.ctx.t += 0.5
        lab.guide.done(lab.ctx)
        await lab.ticks(3)
        assert lab.run.marked == ["B20_J"]

    asyncio.run(walk())


def test_a_drive_planned_before_the_scene_changed_is_refused_at_go(monkeypatch):
    """The drive waits for Go; another movement shows in the live scene meanwhile: Go refuses, nothing drives."""
    lab = Guide(monkeypatch)

    async def walk():
        await lab.ticks()
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        lab.process.scene_shows = ("B20_J", "B20_J_insert")
        lab.guide.press_go(lab.ctx)
        await lab.ticks()
        assert lab.bases[CINDY] == Pose((3.0, 1.0, 0.0))
        assert lab.guide._state(lab.ctx, 0).label == "failed" and "changed" in lab.guide._notes.content

    asyncio.run(walk())


class LiveArmPlanner(ArmPlanner):
    """Its paths start where the arms stand, as the real one's."""

    def __init__(self, lab: Guide):
        """Finds every path."""
        super().__init__(finds=True)
        self.lab = lab

    def request_plan(self, request):
        """From the live joints to the goal."""
        self.requests.append(request)
        joints = self.lab.ctx.scene.snapshot.robots[f"robots/{request.serial}"].joints
        arms = [request.arm, *(arm for arm, _goal in request.also)]
        start = [joints[name] for arm in arms for name in arm_joint_names(arm)]
        rows = np.array([start, list(request.goals)])
        return answered(Plan(len(self.requests), request, "ready", TimedPath(np.array([0.0, 3.0]), rows), "found"))


def test_a_plan_waiting_for_go_turns_stale_when_the_base_moves(monkeypatch):
    """The transfer waits for Go; Cindy's base moves 1 cm: the needs line says the plan is stale, as Go would."""
    lab = Guide(monkeypatch)
    lab.ctx.plugins["arm_planner"] = LiveArmPlanner(lab)

    async def walk():
        await lab.ticks()
        lab.guide.look(lab.ctx, 1)
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        assert lab.guide.procedure.waiting and "plan stale" not in lab.guide._chips.content
        lab.bases[CINDY] = Pose((3.01, 1.0, 0.0))
        await lab.ticks()
        assert "plan stale" in lab.guide._chips.content

    asyncio.run(walk())


def test_a_robot_holding_a_bar_at_its_actions_start_has_no_drive(monkeypatch):
    """B20_R: Cindy holds B20, so its parts start with the retreat; texts by hand follow the part."""
    lab = Guide(monkeypatch)
    asyncio.run(lab.ticks())
    release = [part for part in lab.guide.parts if part.action == "B20_R"]
    assert DRIVE not in [part.kind for part in release]
    hand = next(part for part in lab.guide.parts if part.kind == HAND)
    assert lab.guide.kinds[HAND].goal_of(hand).startswith("Place the bar")


class SoftStoppingStow(StowPlanner):
    """Its commit soft stops the robot before the arms get there (a clearance trip)."""

    def commit(self, plan_id, unknown_robots=(), clearance=True):
        """Soft stop the robot; the arms stay."""
        self.robots[self.plan.request.serial].soft_stop_latch.stopped = True
        return answered((True, "sent"))


def test_a_park_stopped_by_a_soft_stop_stops_and_is_no_mere_warning(monkeypatch):
    """Alice's stow trips a soft stop: the park fails (red), it is not left "not parked" for Auto to go past."""
    lab = Guide(monkeypatch, "B25_J_transfer")
    lab.guide.park_idle = True
    lab.ctx.plugins["arm_planner"] = SoftStoppingStow(lab.robots)
    monkeypatch.setattr(guide_module, "STOW_SETTLE", 0.2)
    lab.alice.arms["ur_arm"].state.joint_positions["ur_arm_shoulder_pan_joint"] = 0.5

    async def walk():
        await lab.ticks()
        alice = lab.guide.index
        assert (lab.guide.part.kind, lab.guide.part.robot) == (PARK, "a200-0804")
        lab.guide.prepare(lab.ctx)
        await lab.ticks()
        lab.guide.press_go(lab.ctx)
        await lab.ticks(5)
        assert lab.guide._state(lab.ctx, alice).label == "failed" and alice not in lab.guide._flagged

    asyncio.run(walk())
