"""The guide in God mode on the real motion planner and executor, the robots fakes that follow: whole actions.

The base planner is a fake that drives the base onto its goal at Commit; the process plugin is a fake selection that
follows `request_select`; the run marks at once. God's gate is passed (the fake robots announce no simulation).
"""

from __future__ import annotations

import asyncio
import math
from copy import copy
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, design_scene, purged_process, robot_config, step
from robot_lab import FakeRun, RobotLab

from husky_assembly_teleop.plugins.assembly_guide import plugin as guide_module
from husky_assembly_teleop.plugins.assembly_guide.parts import MARK
from husky_assembly_teleop.plugins.assembly_guide.plugin import GOD, AssemblyGuidePlugin
from husky_assembly_teleop.plugins.arm_planner.planner_plugin import ArmPlannerPlugin
from husky_assembly_teleop.plugins.motion_planner.plugin import MotionPlannerPlugin
from husky_assembly_teleop.plugins.movement_exec.plugin import MovementExecPlugin
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.scene_for import scene_for
from husky_assembly_teleop.shared.process_scene.scene_bodies import scene_bodies
from husky_assembly_teleop.world.stand_ins import with_stand_ins
from husky_assembly_teleop.shared.movement_exec_contract import serial_of
from husky_assembly_teleop.shared.planner.plan import Plan
from husky_assembly_teleop.shared.planner.timed_path import TimedPath
from husky_assembly_teleop.shared.assembly_planning.steps import first_step
from husky_assembly_teleop.shared.process_contract import Selection
from husky_assembly_teleop.shared.runs.runs import DONE
from husky_assembly_teleop.world.checks import BAD
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.scene import world_poses
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror

pytestmark = pytest.mark.slow


def answered(value):
    """A future already holding `value`."""
    future = asyncio.get_running_loop().create_future()
    future.set_result(value)
    return future


class Run(FakeRun):
    """The run: marks and reopens at once."""

    def request_mark_done(self, action_id=None):
        """Mark it done, then make the first pending action current at its first step, as the real run does."""
        super().request_mark_done(action_id)
        self.view.statuses[action_id] = DONE
        process = self.view.process.process
        following = next((each for each in process.schedule if self.view.statuses[each] != DONE), None)
        if following is not None:
            self.request_set_current(following, first_step(process, process.actions[following], True))

    def request_reopen(self, action_id=None):
        """Put it back to pending."""
        self.view.statuses[action_id] = "pending"

    def request_set_current(self, action_id, movement_index=None):
        """Make it current and select it in the process plugin, as the real run does."""
        super().request_set_current(action_id, movement_index)
        self.process.request_select(self.view.process.index_of(action_id, movement_index or 0))


class Process:
    """The process plugin: its boxes and its selection, moved by `request_select` as the panel shows it."""

    obstacles_collide = transfer_as_free = True
    #: The (action id, movement) put in the live scene a tick ago, and what the snapshot shows (`GuideLab.tick`).
    put = scene_shows = None

    def __init__(self, loaded, movement: str):
        """Select `movement`."""
        from husky_assembly_teleop.shared.assembly_planning.steps import as_run
        self.authored, self.shown = loaded.process, as_run(loaded.process, True)
        self.movements = list(self.authored.movements())
        self.carries = []
        self.request_select(next(index for index, (_a, m) in enumerate(self.movements) if m.id == movement))

    def request_carry(self, carry):
        """Keep it."""
        self.carries.append(carry)

    def request_select(self, index: int) -> None:
        """Select the authored movement `index`, shown as the panel shows it."""
        action, movement = self.movements[index]
        shown = self.shown.actions[action.id]
        movement = next((each for each in shown.movements if each.id == movement.id), shown.movements[0])
        self.selection = Selection(self.authored, index, shown, movement)


class BasePlanner:
    """Plans at once; its commit puts the base on the goal."""

    def __init__(self, bases):
        """No plan yet."""
        self.bases, self.plan, self.commits = bases, None, 0

    def request_plan(self, request):
        """A two-row path to the goal."""
        goal = np.asarray(request.goal, dtype=float)
        self.plan = Plan(1, request, "ready", TimedPath(np.array([0.0, 2.0]), np.array([goal, goal])), "found")
        return answered(self.plan)

    def commit(self, plan_id, unknown_robots=(), clearance=True):
        """Put the base on the goal; sent."""
        self.commits += 1
        x, y, yaw = self.plan.request.goal
        self.bases[self.plan.request.serial] = Pose((x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        return answered((True, "sent"))

    def cancel_commit(self, serial=None):
        """Nothing runs."""
        return False


class GuideLab(RobotLab):
    """The robots at `movement`'s start as the design has them, the guide in God mode on the real layers below."""

    def __init__(self, monkeypatch, movement: str):
        """Build everything at `movement`'s start."""
        loaded = purged_process()
        scene, at = design_scene(loaded, movement), step(loaded, movement)
        configs = tuple(robot_config(digits) for digits in (ALICE, BELLE, CINDY))
        acting = serial_of(loaded, at.action.id, configs)
        joints = {}
        for serial in ("a200-0804", "a200-0805", "a200-0806"):
            robot = scene.robots.get(robot_id(serial))
            if robot is not None:
                arms = ("left_ur_arm", "right_ur_arm") if serial == "a200-0806" else ("ur_arm",)
                joints[serial] = {arm: [robot.joints[name] for name in arm_joint_names(arm)] for arm in arms}
        if at.assumed_from == "own target":
            config = next(config for config in configs if config.serial == acting)
            joints[acting] = {arm.name: list(arm.stow_joints) for arm in config.arms}
        super().__init__(joints)
        for number, serial in enumerate(self.robots):
            robot = scene.robots.get(robot_id(serial))
            self.bases[serial] = robot.base if robot is not None else Pose((20.0 + 3.0 * number, 20.0, 0.0))
        self.tick(self.ctx.t)
        monkeypatch.setattr(guide_module, "robot_ghosts", lambda *_args: {})
        monkeypatch.setattr(guide_module, "simulation_problem", lambda _ctx: "")
        self.run = Run(loaded, (at.action.id, at.movement_index))
        self.planner = MotionPlannerPlugin()
        self.planner._log = self.planner._pool.log = lambda _line: None
        self.executor = MovementExecPlugin()
        self.process = Process(loaded, movement)
        self.run.process = self.process
        self.ctx.plugins = {"assembly_run": self.run, "motion_planner": self.planner, "movement_exec": self.executor,
                            "process": self.process, "base_planner": BasePlanner(self.bases),
                            "arm_planner": ArmPlannerPlugin()}
        self.executor.setup(self.ctx)
        self.ctx.plugins["arm_planner"].setup(self.ctx)
        self.ctx.kinematics = SimpleNamespace(base_pose=lambda serial: self.bases[serial], joints=lambda serial: {})
        self.ctx.config.ghost_timeout = 0.0
        self.guide = AssemblyGuidePlugin()
        self.guide.setup(self.ctx)
        self.guide.park_idle = False  # * parking has its own tests (test_park_*)
        self.guide.update(self.ctx)

    def close(self) -> None:
        """Close both planners' worlds."""
        self.planner.teardown(None)
        self.ctx.plugins["arm_planner"].teardown(self.ctx)

    def tick(self, t: float) -> None:
        """As the robots' tick, and the live snapshot gets the process bodies put a tick ago, as the process plugin's.

        The snapshot is taken before any plugin runs: a selection shows from the tick after its bodies were put.
        """
        super().tick(t)
        process = getattr(self, "process", None)
        if process is None:
            return
        shown = process.put
        process.scene_shows = (shown[0], shown[1].id) if shown is not None else None
        if shown is not None:
            snapshot = self.ctx.scene.snapshot
            for body in self.bodies_of(shown):
                snapshot.put(body)
            snapshot.world_poses = world_poses(snapshot.bodies, snapshot.robots, lambda robot, link: link_pose(
                robot.model.urdf, robot.base, robot.joints, link))
            for body_id in [body_id for body_id in snapshot.bodies if body_id not in snapshot.world_poses]:
                snapshot.remove(body_id)
        selection = process.selection
        process.put = (selection.action.id, selection.movement)

    def bodies_of(self, shown):
        """The process bodies of a shown movement, made once per movement."""
        cache = self.__dict__.setdefault("_bodies", {})
        key = (shown[0], shown[1].id)
        if key not in cache:
            cache[key] = scene_bodies(self.process.authored, shown[1], self.configs, "process/")
        return [copy(body) for body in cache[key]]

    async def until_marked(self, action: str, seconds: float = 400.0) -> list[str]:
        """Tick the guide until `action` is marked done; the labels of the parts it ran, in order.

        `after` keeps the guide's mode and part then: once the walk's loop closes, its tasks are cancelled, and a
        cancelled part ends God.
        """
        ran, until = [], self.ctx.t + seconds
        if self.guide.mode != GOD:
            self.guide.set_mode(self.ctx, GOD)
        while self.run.view.statuses.get(action) != DONE:
            assert self.ctx.t < until, f"stuck at {self.guide.part.label}: {self.guide.procedure.outcome}"
            outcome = self.guide.procedure.outcome
            assert outcome is None or outcome.level != BAD, f"{self.guide.part.label}: {outcome.detail}"
            if not ran or ran[-1] != self.guide.part.label:
                ran.append(self.guide.part.label)
            await self.ctx.next_tick()
            await asyncio.sleep(0.005)
            self.guide.update(self.ctx)
            self.guide.draw(self.ctx)
        self.after = (self.guide.mode, self.guide.part)
        return ran


def test_god_mode_runs_b21_h_through_the_real_planner_and_executor(monkeypatch):
    """Alice's hold: drive, approach, its servo, open, the straight line in, the compliant close, Mark done."""
    lab = GuideLab(monkeypatch, "B21_H_approach")
    lab.track("a200-0804")  # * the handoff needs Alice's frames
    try:
        ran = asyncio.run(lab.until_marked("B21_H"))
    finally:
        lab.close()
    assert ran == ["drive to B21_H's start", "B21_H_approach: approach", "B21_H_approach: servo",
                   "B21_H_open: open the gripper", "B21_H_to_grasp: straight line",
                   "B21_H_close: close the gripper, arm compliant", "mark B21_H done"]
    files = [file for file in lab.run.files() if "part" in file]
    assert [file["movement"] for file in files] == ["B21_H_approach", "B21_H_approach", "B21_H_open",
                                                    "B21_H_to_grasp", "B21_H_close"]
    # * The approach, a one-arm free move, was planned by the arm planner and run by the executor as any route; the
    #   straight line in by the motion planner.
    assert files[0]["plan"]["planner"] == "arm_planner.rrt_connect" and files[0]["outcome"]["status"] == "ARRIVED"
    assert files[3]["plan"]["planner"].startswith("single_arm.plan_linear")
    assert lab.guide.part.action == "B21_R" and lab.guide.part.kind != MARK


def test_god_mode_runs_b20_j_and_b20_r(monkeypatch):
    """Cindy joints the ground bar and releases it: every part, through the real planner and executor."""
    lab = GuideLab(monkeypatch, "B20_J_transfer")
    lab.track("a200-0806")  # * the compliant parts need Cindy's frames

    async def walk():
        return await lab.until_marked("B20_J"), await lab.until_marked("B20_R")

    try:
        joint, release = asyncio.run(walk())
    finally:
        lab.close()
    assert joint == ["drive to B20_J's start", "B20_J_transfer: transfer, empty tools", "B20_J_place_bar: by hand",
                     "B20_J_transfer: servo", "B20_J_insert: insert", "B20_J_insert: servo",
                     "mark B20_J done"]
    assert release[-2:] == ["B20_R_home: home", "mark B20_R done"] and "B20_R_retreat: retreat" in release
    planners = {file["movement"]: file["plan"]["planner"] for file in lab.run.files() if file.get("plan")}
    # * The free moves, both of Cindy's arms in one search, on the arm planner; the insert and retreat lines on tamp.
    assert planners["B20_J_transfer"] == planners["B20_R_home"] == "arm_planner.rrt_connect"
    assert planners["B20_J_insert"].startswith("tamp.") and planners["B20_R_retreat"].startswith("tamp.")
    # * The way home is clear in its own step's scene, B20 standing (it was planned in the retreat's scene once, where
    #   the tools may touch B20, and ran through the bar).
    home = next(file for file in lab.run.files() if file.get("movement") == "B20_R_home" and file.get("plan"))
    assert not home_rows_in_contact(lab, home["plan"]["joint_names"], home["plan"]["rows"])
    assert lab.after[0] == GOD and lab.after[1].action == "B21_J"


def home_rows_in_contact(lab: GuideLab, names, rows) -> list[int]:
    """The rows of a B20_R_home path that touch anything in that step's own planning scene (`scene_for`)."""
    loaded = lab.run.view.process
    serial = robot_config(CINDY).serial
    snapshot = design_scene(loaded, "B20_R_home")
    scene = with_stand_ins(scene_for(loaded, step(loaded, "B20_R_home"), snapshot, lab.configs).scene,
                           {robot_id(serial)}).scene
    mirror = CompasFabMirror(robot_id(serial))
    try:
        mirror.sync(scene)
        check = mirror.search_check(list(names), 0.0)
        return [index for index, row in enumerate(rows) if check(list(row)) is not None]
    finally:
        mirror.close()


@pytest.mark.timeout(600)
def test_god_mode_builds_the_first_three_bars(monkeypatch):
    """The real lab test's seven actions, B20_J to B22_R, one after the other without a hand on the panel."""
    lab = GuideLab(monkeypatch, "B20_J_transfer")
    lab.track("a200-0806")
    lab.track("a200-0804")
    actions = ["B20_J", "B20_R", "B21_J", "B21_H", "B21_R", "B22_J", "B22_R"]

    async def walk():
        return [await lab.until_marked(action) for action in actions]

    try:
        ran = asyncio.run(walk())
    finally:
        lab.close()
    assert [labels[-1] for labels in ran] == [f"mark {action} done" for action in actions]
    assert lab.after[0] == GOD, "God set once, never ended on the way"
    assert lab.run.marked == actions
    assert lab.after[1].action == "B34_J"
