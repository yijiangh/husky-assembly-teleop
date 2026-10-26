"""The B20 servo chain end to end: the real run, planner, executor and step panel on fake robots, in a scratch copy.

Spec §6 "test_servo_chain". B20_J's transfer step is planned on tamp and run with every part confirmed, the tools
stalling and Cindy's base measured 10 mm off: the servo converges on tool0 and is recorded in the run; then, with
"Servo by bar" and synthetic markers of B20 streamed from a world-frame error, it converges on the MoCap bar and
carries the correction for the insert. The run is written into a copy of the purged process: the Drive's folder is
never written.
"""

from __future__ import annotations

import asyncio
import json
import shutil
from math import radians
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, DATA, design_scene, purged_process, robot_config, step
from robot_lab import RobotLab

from husky_assembly_teleop.plugins.assembly_run.plugin import AssemblyRunPlugin
from husky_assembly_teleop.plugins.assembly_steps.panel import StepsPanel
from husky_assembly_teleop.plugins.assembly_steps.plugin import AssemblyStepsPlugin
from husky_assembly_teleop.plugins.motion_planner.plugin import MotionPlannerPlugin
from husky_assembly_teleop.plugins.movement_exec.plugin import MovementExecPlugin
from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.robot_interface.mocap_state import RawMarkers
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.process_steps import loaded_process
from husky_assembly_teleop.shared.execution.servo import flange_poses, servo_plan
from husky_assembly_teleop.shared.movement_exec_contract import DONE, MOTION_PART, REFUSED_PART, SERVO_PART, serial_of
from husky_assembly_teleop.shared.process_contract import Selection
from husky_assembly_teleop.shared.runs.runs import RunSettings, read_events
from husky_assembly_teleop.ui.ghost import RecentUse
from husky_assembly_teleop.world.bar_pose import transform_points
from husky_assembly_teleop.world.marker_layouts import bar_layouts
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.ids import robot_id

#: Wall-clock seconds the chain may take: tamp's transfer search alone may take two minutes.
WALL_LIMIT = 540.0
#: The step walked, Cindy, and the name the servo tracks B20's markers under.
MOVEMENT, SERIAL, TRACKED = "B20_J_transfer", "a200-0806", "servo_B20"


def world_error(dy: float, roll_deg: float) -> np.ndarray:
    """A small rigid error in the world frame: `dy` metres along y and a turn of `roll_deg` degrees about x."""
    T = np.eye(4)
    angle = radians(roll_deg)
    T[1:3, 1:3] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    T[1, 3] = dy
    return T


def run_files(folder: Path) -> list[tuple[str, int]]:
    """Every file under a process folder's runs/ with its size, in order; empty without one."""
    runs = folder / "runs"
    if not runs.is_dir():
        return []
    return sorted((str(path.relative_to(runs)), path.stat().st_size) for path in runs.rglob("*") if path.is_file())


class FakeProcess:
    """The process plugin as the run and the step panel see it: the scratch process, its selection, its two boxes."""

    def __init__(self, process):
        """Select the first movement; the workaround on, and the Rhino obstacles counted as the walk was planned."""
        self.loaded, self.index = loaded_process(process), 0
        self.transfer_as_free = self.obstacles_collide = True

    @property
    def selection(self) -> Selection:
        """The selected movement."""
        at = self.loaded.steps[self.index]
        return Selection(self.loaded.process, self.index, at.action, at.movement)

    def request_select(self, index: int) -> None:
        """Select at once."""
        self.index = index

    def request_marks(self, marks) -> None:
        """Nothing to show."""

    def request_carry(self, carry) -> None:
        """Nothing to draw."""


class Chain(RobotLab):
    """B20_J's transfer step on the real run, planner, executor and step panel, Cindy where the design has her.

    The run (assembly_run) writes into `scratch`, a copy of the purged process; the robots absent from the design
    stand far away; `stream(t)`, set by `markers`, feeds the servo's tracked bar its markers every tick.
    """

    stream = None

    def __init__(self, scratch: Path):
        """Build everything at the transfer's start."""
        process = load_process(scratch, DATA)
        loaded = loaded_process(process)
        scene, at = design_scene(loaded, MOVEMENT), step(loaded, MOVEMENT)
        configs = tuple(robot_config(digits) for digits in (ALICE, BELLE, CINDY))
        acting = serial_of(loaded, at.action.id, configs)
        joints = {}
        for serial in ("a200-0804", "a200-0805", "a200-0806"):
            robot = scene.robots.get(robot_id(serial))
            if robot is not None:
                arms = ("left_ur_arm", "right_ur_arm") if serial == "a200-0806" else ("ur_arm",)
                joints[serial] = {arm: [robot.joints[name] for name in arm_joint_names(arm)] for arm in arms}
        if at.assumed_from == "own target":
            # * No authored start: the acting robot comes with its arms stowed (as the M3 check plans it).
            config = next(config for config in configs if config.serial == acting)
            joints[acting] = {arm.name: list(arm.stow_joints) for arm in config.arms}
        super().__init__(joints)
        for number, serial in enumerate(self.robots):
            robot = scene.robots.get(robot_id(serial))
            self.bases[serial] = robot.base if robot is not None else Pose((20.0 + 3.0 * number, 20.0, 0.0))
        self.tick(self.ctx.t)
        self.loaded, self.at = loaded, at
        self.process = FakeProcess(process)
        self.run = AssemblyRunPlugin()
        self.run._ctx = self.ctx
        self.planner = MotionPlannerPlugin()
        self.planner._log = self.planner._pool.log = lambda _line: None
        self.executor = MovementExecPlugin()
        self.ctx.plugins = {"assembly_run": self.run, "motion_planner": self.planner, "movement_exec": self.executor,
                            "process": self.process}
        self.executor.setup(self.ctx)
        self.ctx.kinematics = SimpleNamespace(base_pose=lambda serial: self.bases[serial], joints=lambda serial: {})
        self.steps = AssemblyStepsPlugin()
        self.steps._ctx = self.ctx
        self.steps._panel = StepsPanel(self.ctx, self.steps)
        self.steps._used = RecentUse(0.0)
        self.steps._target_ghosts, self.steps._path_ghosts = {}, {}

    def tick(self, t: float) -> None:
        """As the lab's, then the bar's markers into the servo's tracked object."""
        super().tick(t)
        obj = self.ctx.world.tracked_objects.get(TRACKED)
        if obj is not None and self.stream is not None:
            obj.markers, obj.last_update_time = self.stream(t), t

    def shift(self, delta: tuple) -> None:
        """Measure Cindy's base `delta` metres off."""
        base = self.bases[SERIAL]
        self.bases[SERIAL] = Pose(tuple(np.add(base.position, delta)), base.orientation)
        self.tick(self.ctx.t)

    def markers(self, E: np.ndarray) -> None:
        """Stream B20's markers each tick from where the world-frame error `E` puts the bar relative to the FK bar."""
        plan = servo_plan(self.loaded, self.at.action.id, self.at.movement_index, RunSettings(servo_by_bar=True),
                          bar_layouts(self.loaded.process))
        holder, grasp = plan.holder
        count = len(plan.layout.points)

        def stream(t: float) -> RawMarkers:
            robot = self.ctx.scene.snapshot.robots[robot_id(SERIAL)]
            B_fk = compose(flange_poses(robot, [holder])[holder], grasp).matrix()
            return RawMarkers(ids=np.arange(1, count + 1), positions=transform_points(E @ B_fk, plan.layout.points),
                              measured=np.ones(count, dtype=bool), residuals=np.full(count, 5e-4),
                              sizes=np.full(count, 0.019), capture_time=t)
        self.stream = stream

    async def opened(self) -> None:
        """Open the run (no run folder yet: the first record starts one) and let it make the transfer step current."""
        self.run.update(self.ctx)
        while self.run.view.current is None:
            await self.ctx.next_tick()
        self.steps.update(self.ctx)

    async def settle(self, seconds: float = 2.5) -> None:
        """Let ROS time pass, the robots still (a lease needs every arm still for 2 s)."""
        until = self.ctx.t + seconds
        while self.ctx.t < until:
            await self.ctx.next_tick()

    async def run_step(self, shift_at: int | None = None) -> int:
        """Plan the step's motion if it has one, Exec, Confirm every prompt until the Exec ends; how many there were.

        As prompt `shift_at` (from 1) opens, Cindy's base is measured 10 mm off: at the grasp's prompt the motion has
        arrived, so only the servo after it sees the shift. The step panel and the run follow after.
        """
        steps = self.steps
        if any(part.kind == MOTION_PART for part in steps.parts()):
            steps._on_path(self.ctx, "Plan")
            await steps._plan_task
            assert steps.plan_state == "ready", steps._message
        steps._commit(self.ctx)
        confirmed = 0
        while steps._executing is not None:
            prompt = self.executor.prompt
            if prompt is not None:
                confirmed += 1
                if confirmed == shift_at:
                    self.shift((0.01, 0.0, 0.0))
                while self.ctx.t - prompt.opened_at < 0.6:
                    await self.ctx.next_tick()
                self.executor.request_confirm()
                while self.executor.prompt is prompt:
                    await asyncio.sleep(0)
            await asyncio.sleep(0.01)
        await self.settle()
        steps.update(self.ctx)
        self.run.update(self.ctx)
        return confirmed

    async def written(self) -> None:
        """Wait for the run plugin's tasks, then for its queued writes."""
        while self.run._tasks:
            await asyncio.gather(*list(self.run._tasks), return_exceptions=True)
        await asyncio.wrap_future(self.run._writer.submit(lambda: None))
        await asyncio.sleep(0)

    def records(self) -> tuple[list[dict], list[dict]]:
        """The run's events, and the exec records they name, in order."""
        folder = self.run.view.folder
        events = read_events(folder)
        files = [json.loads((folder / event["file"]).read_text()) for event in events if event["event"] == "exec"]
        return events, files

    def servo_record(self, number: int) -> dict:
        """The run's `servo/B20_J/B20_J_transfer.<number>.json`, its .png drawn beside it."""
        path = self.run.view.folder / "servo" / "B20_J" / f"{MOVEMENT}.{number}.json"
        assert path.with_suffix(".png").stat().st_size > 0
        return json.loads(path.read_text())


@pytest.mark.slow
@pytest.mark.timeout(600)
def test_the_b20_servo_chain_on_the_real_plugins(tmp_path, monkeypatch):
    """B20_J's transfer step with the real run, planner, executor and step panel: the servo on tool0, then by bar.

    Plan (tamp), Exec, Confirm the motion, the mount and the grasp (it stalls), Cindy's base measured 10 mm off: the
    servo's pass 1 is confirmed and converges on tool0, recorded as `servo/B20_J/B20_J_transfer.1.json` with its
    .png; the run's events hold new_run, plan, prompt, confirm, exec and servo. Then, "Servo by bar" on and B20's
    markers streamed 3 mm / 0.1° off in the world, Exec runs the servo alone: it converges on the MoCap bar within
    3 passes and carries the correction for B20_J_insert. The Drive's process folder is never written.
    """
    # * The run records the commits by running git: left out, so no git command runs in the shared worktree.
    monkeypatch.setattr("husky_assembly_teleop.shared.runs.runs.commits",
                        lambda: {"monitor": "", "core": "", "tamp": ""})
    source = Path(purged_process().process.folder)
    drive_runs = run_files(source)
    scratch = tmp_path / "purged"
    # * Without runs/: a run there would be continued or offered, and the chain must start its own.
    shutil.copytree(source, scratch, ignore=shutil.ignore_patterns("runs"))
    lab = Chain(scratch)
    transfer, insert = lab.at.movement_index, step(lab.loaded, "B20_J_insert").movement_index

    async def scenario():
        await lab.opened()
        assert lab.run.view.current == ("B20_J", transfer) and lab.run.view.folder is None
        prompts = await lab.run_step(shift_at=3)
        assert prompts == 4, (lab.steps._message, lab.executor._last)
        assert lab.steps.position == (2, 2) and lab.steps.parts()[0].kind == REFUSED_PART
        # * Back on the transfer step with its motion, mount and grasp done: Exec runs the servo alone, now by bar.
        lab.run.request_settings(servo_by_bar=True)
        lab.run.request_set_current("B20_J", transfer)
        lab.steps.update(lab.ctx)
        _view, _action, op_step, _primary = lab.steps.current()
        lab.steps._done = set(op_step.indices)
        assert [part.kind for part in lab.steps.parts()] == [SERVO_PART]
        lab.markers(world_error(0.003, 0.1))
        again = await lab.run_step()
        assert again >= 1, (lab.steps._message, lab.executor._last)
        await lab.written()
        return prompts, again

    try:
        prompts, again = asyncio.run(asyncio.wait_for(scenario(), WALL_LIMIT))
    finally:
        lab.planner.teardown(None)
        lab.run.teardown(lab.ctx)

    events, files = lab.records()
    names = [event["event"] for event in events]
    assert names[0] == "new_run" and {"plan", "prompt", "confirm", "exec", "servo", "settings"} <= set(names)
    assert [file["movement"] for file in files] == [MOVEMENT, "B20_J_place_bar", "B20_J_grasp", MOVEMENT, MOVEMENT]
    assert [file["outcome"]["status"] for file in files] == ["ARRIVED", "confirmed", "stalled", "converged",
                                                             "converged"]
    assert files[0]["plan"]["planner"] == "tamp.plan_free_dual_arm"
    assert files[3]["servo_file"] == f"servo/B20_J/{MOVEMENT}.1.json"
    assert files[4]["servo_file"] == f"servo/B20_J/{MOVEMENT}.2.json"
    assert json.loads((lab.run.view.folder / "run.json").read_text())["settings"]["servo_by_bar"] is True

    tool0 = lab.servo_record(1)
    assert tool0["servo_feedback"] == "tool0" and tool0["bar_name"] == "bars/B20"
    assert tool0["outcome"]["status"] == DONE and "converged on tool0" in tool0["outcome"]["reason"]
    assert [reading["iter"] for reading in tool0["servoing_data"]] == [0, 1] and len(tool0["moves"]) == 1
    before, after = (reading["tool0"] for reading in tool0["servoing_data"])
    assert all(error["pos_norm_mm"] == pytest.approx(10.0, abs=0.2) for error in before.values())
    assert all(error["pos_norm_mm"] < 0.2 for error in after.values())

    bar = lab.servo_record(2)
    assert bar["servo_feedback"] == "mocap_bar" and "converged on the MoCap bar" in bar["outcome"]["reason"]
    assert 1 <= again <= len(bar["moves"]) <= 3 and bar["servoing_data"][-1]["bar"]["usable"]
    carried = lab.executor.carried
    assert carried is not None and carried.loaded is lab.run.view.process
    assert (carried.action, carried.insert, carried.serial) == ("B20_J", insert, SERIAL)
    assert "[servo] the MoCap-bar correction is carried into the plan of B20_J_insert." in lab.ctx.lines("info")
    assert lab.steps.position == (2, 2) and "not supported yet" in lab.steps.parts()[0].text
    # ? after the M6 merge: the insert is a motion part then. Plan it here (keep the planner's request, e.g. by
    #   wrapping `lab.planner.request_plan`) and assert its `goal_poses` equal `apply_authored_move` from the live
    #   flanges to 1e-9, as test_assembly_steps' carry test checks them on the fake planner.

    assert run_files(source) == drive_runs and run_files(scratch)
