"""The motion planner's work (plugins/motion_planner/work.py): what it answers without planning, on a small process."""

from __future__ import annotations

import threading
from pathlib import Path

import pytest
from planning_fixtures import CINDY, DATA, robot_config
from process_fixtures import build_process

from husky_assembly_teleop.plugins.motion_planner.work import run_collisions, run_ik, run_plan
from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.shared.assembly_planning.process_steps import loaded_process
from husky_assembly_teleop.shared.motion_planner_contract import MotionPlanRequest
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process import write
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

CINDY_ID = robot_id(f"a200-{CINDY}")


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
    return MotionPlanRequest(loaded, loaded.steps[2], snapshot, robots)


WORK = (run_plan, run_ik, run_collisions)


def reason(answer) -> str:
    """The reason of any answer."""
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
