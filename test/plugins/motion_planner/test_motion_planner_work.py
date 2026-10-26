"""The motion planner's work (plugins/motion_planner/work.py): what it answers without planning, on a small process."""

from __future__ import annotations

import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from planning_fixtures import ALICE, BELLE, CINDY, DATA, design_scene, purged_process, robot_config, step
from process_fixtures import build_process

from husky_assembly_teleop.plugins.motion_planner import work as work_module
from husky_assembly_teleop.plugins.motion_planner.work import run_collisions, run_ik, run_plan, run_servo_step
from husky_assembly_teleop.shared.assembly_planning import servo_route, tamp_route
from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.shared.assembly_planning.process_steps import loaded_process
from husky_assembly_teleop.shared.motion_planner_contract import MotionPlanRequest, ServoAsk
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process import write
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

CINDY_ID = robot_id(f"a200-{CINDY}")
#: A servo ask that names no flange: the work refuses every request here before reading it.
ASK = ServoAsk({}, 0.0002, False, True)


class Pool:
    """A pool whose mirror fails, to show the work never raises; counts the mirrors asked for."""

    def __init__(self):
        """Ask for none yet."""
        self.asked = []

    def mirror(self, serial):
        """Count the request, then fail."""
        self.asked.append(serial)
        raise RuntimeError("no mirror here")


@pytest.fixture(scope="module")
def loaded(tmp_path_factory) -> object:
    """The small process, written and loaded; Cindy acts in its first action."""
    folder = tmp_path_factory.mktemp("service")
    return loaded_process(load_process(Path(write(build_process(folder), folder / "p").folder), DATA))


def request(loaded, robots=(), unmeasured=frozenset()) -> MotionPlanRequest:
    """A request for the first movement with Cindy in the snapshot, measured unless `unmeasured` names joints."""
    config = robot_config(CINDY)
    snapshot = Scene(robots={CINDY_ID: RobotObject(CINDY_ID, config.model, Pose(), {}, unmeasured=unmeasured)})
    return MotionPlanRequest(loaded, loaded.steps[2], snapshot, robots, servo=ASK)


WORK = (run_plan, run_ik, run_collisions, run_servo_step)


def reason(answer) -> str:
    """The reason of any answer."""
    if hasattr(answer, "route"):
        answer = answer.route
    return answer.result.reason if hasattr(answer, "result") else answer.reason


@pytest.mark.parametrize("work", WORK)
def test_answers_without_planning(loaded, work):
    """Cancelled, no configured robot, or an acting robot not measured: answered with the reason, no mirror asked."""
    pool, abort = Pool(), threading.Event()
    configs = (robot_config(CINDY),)
    assert reason(work(pool, request(loaded), abort, print)) == "no configured robot acts"
    unmeasured = work(pool, request(loaded, configs, frozenset({"left_ur_arm_elbow_joint"})), abort, print)
    assert reason(unmeasured).startswith("a200-0806 can't be planned for: joints not known")
    abort.set()
    assert reason(work(pool, request(loaded, configs), abort, print)) == "cancelled"
    assert pool.asked == []


@pytest.mark.parametrize("work", WORK)
def test_an_error_is_a_failed_answer(loaded, work):
    """Whatever raises on the worker comes back as a failed answer, never as an exception."""
    pool = Pool()
    answer = work(pool, request(loaded, (robot_config(CINDY),)), threading.Event(), print)
    assert reason(answer) == "the planner raised RuntimeError('no mirror here')" and pool.asked == ["a200-0806"]
    if work is run_plan:
        assert answer.result.status == "failed" and answer.routine is None and not answer.ok
    if work is run_ik:
        assert not answer.ok and answer.routine is None
    if work is run_servo_step:
        assert not answer.ok and not answer.nothing_left and answer.correction == {} and answer.route.routine is None


def test_a_servo_step_needs_its_ask(loaded):
    """A servo step asked without `servo` is refused with the reason, before any mirror is asked."""
    pool = Pool()
    asked = MotionPlanRequest(loaded, loaded.steps[2], request(loaded).snapshot, (robot_config(CINDY),))
    answer = run_servo_step(pool, asked, threading.Event(), print)
    assert reason(answer) == "the request carries no servo ask" and not answer.ok and pool.asked == []


@pytest.mark.parametrize("work, module", [(run_plan, tamp_route), (run_ik, tamp_route), (run_collisions, work_module),
                                          (run_servo_step, servo_route)],
                         ids=["run_plan", "run_ik", "run_collisions", "run_servo_step"])
def test_the_rhino_obstacles_box_reaches_the_planning_scene(work, module, monkeypatch):
    """A request with the box unticked asks `scene_for` to leave the Rhino obstacles out, from every work."""
    purged = purged_process()
    calls = []

    def scene_for(*_args, **options):
        calls.append(options)
        raise RuntimeError("kept what scene_for was asked")

    monkeypatch.setattr(module, "scene_for", scene_for)
    configs = tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))
    asked = MotionPlanRequest(purged, step(purged, "B20_J_transfer"), design_scene(purged, "B20_J_transfer"), configs,
                              transfer_as_free=True, obstacles_collide=False, servo=ASK)
    pool = SimpleNamespace(mirror=lambda serial: SimpleNamespace(robot_id=robot_id(serial)))
    answer = work(pool, asked, threading.Event(), print)
    assert "kept what scene_for was asked" in reason(answer)
    assert [call["obstacles_collide"] for call in calls] == [False]


def test_a_retreats_goal_poses_and_step_reach_plan_movement(loaded, monkeypatch):
    """`goal_poses` and `linear_step` of a request go to `plan_movement` unchanged."""
    asked = {}
    monkeypatch.setattr(work_module, "plan_movement", lambda *_args, **options: asked.update(options))
    goals = {"robots/cindy/left_ur_arm_tool0": Pose((1.0, 2.0, 3.0))}
    configs = (robot_config(CINDY),)
    snapshot = Scene(robots={CINDY_ID: RobotObject(CINDY_ID, configs[0].model, Pose(), {})})
    question = MotionPlanRequest(loaded, loaded.steps[2], snapshot, configs, goal_poses=goals, linear_step=0.001)
    run_plan(SimpleNamespace(mirror=lambda _serial: None), question, threading.Event(), print)
    assert asked["goal_poses"] is goals and asked["linear_step"] == 0.001
