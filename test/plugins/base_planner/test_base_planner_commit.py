"""The base planner's paths as the onboard follower cuts them into pieces, Commit sending them to it, and runs."""

import asyncio
import json
import math
from collections import deque
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import pytest
from crl_husky.follower_path import FollowerPath
from robot_fakes import Node

from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugin_api.trace import Signal, Trace
from husky_assembly_teleop.robot_interface.base import BaseInterface, FollowerState
from husky_assembly_teleop.plugins.base_planner.path import MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, steer, timed_path
from husky_assembly_teleop.plugins.base_planner.planner_plugin import BasePlannerPlugin
from husky_assembly_teleop.plugins.base_planner.tasks import BY_NAME
from husky_assembly_teleop.shared.base_planner_contract import BasePlanRequest
from husky_assembly_teleop.shared.planner.plan import Plan, PlannerContract

SERIAL = "alice"


def planned(*corners):
    """The path the RRT makes of these corners: steer between each pair, timed."""
    poses = [corners[0]]
    for q1, q2 in zip(corners[:-1], corners[1:]):
        poses += steer(q1, q2)
    return timed_path(poses)


@pytest.mark.safety
@pytest.mark.parametrize("corners", [
    [(0.0, 0.0, 0.0), (2.0, 1.0, math.pi / 2)],
    [(0.0, 0.0, 0.0), (-1.5, 0.2, 0.1)],
    [(0.0, 0.0, 0.3), (1.0, 1.0, 2.0), (2.0, 0.5, -2.8), (3.0, 3.0, 1.0)],
    [(0.0, 0.0, 0.0), (0.0, 0.0, 2.5)],
])
def test_follower_drives_what_the_preview_shows(corners):
    """Turns on the spot and straight drives (backwards too), the same length, ending at the goal."""
    path = planned(*corners)
    follower = FollowerPath.from_poses(*path.points.T)
    drives = [piece for piece in follower.pieces if not piece.turning]
    assert sum(piece.length for piece in drives) == pytest.approx(path.length)
    for piece in drives:
        straight = piece.points[-1] - piece.points[0]
        assert piece.length == pytest.approx(np.linalg.norm(straight))
    np.testing.assert_allclose(follower.polyline()[-1], path.goal, atol=1e-9)


class FakeBase:
    """A base that keeps the paths sent to it."""

    def __init__(self, pose_age=0.02, estopped=False, follower_age=0.1, refuses=False):
        """A base at ROS time 10 s.

        Its last valid mocap sample arrived `pose_age` seconds ago (None: never), its follower report is
        `follower_age` seconds old (None: no follower), and `refuses` makes `send_path` reject the path.
        """
        follower = None if follower_age is None else SimpleNamespace(update_time=10.0 - follower_age)
        errors = deque() if pose_age is None else deque([(10.0 - pose_age, 0.0)])
        self.mocap_id = 1
        seen = pose_age is not None
        self.state = SimpleNamespace(tracked=seen, tracking_valid=seen, marker_errors=errors,
                                     last_update_time=10.0 - pose_age if seen else None,
                                     estopped=estopped, follower=follower)
        self._node = Node(10.0)
        self.refuses = refuses
        self.sent = []

    #: The base's own gate, as the real base has it.
    follow_problem = BaseInterface.follow_problem

    def send_path(self, *args):
        """Keep the path; its id is always 7."""
        if self.refuses:
            raise ValueError("the path holds a value that is not finite")
        self.sent.append(args)
        return 7


def test_the_planner_offers_the_planner_api():
    """Other plugins can drive the base planner as a PlannerContract."""
    assert isinstance(BasePlannerPlugin(), PlannerContract)


def commit(base: FakeBase):
    """Commit a ready plan for SERIAL on `base` through the planner's API; returns (sent, message) and the planner.

    The run is not started.
    """
    planner = BasePlannerPlugin()
    ctx = SimpleNamespace(world=SimpleNamespace(robots={SERIAL: SimpleNamespace(base=base)}),
                          config=SimpleNamespace(drive_root=None), now=lambda: 10.0, log_info=lambda _message: None,
                          log_warn=lambda _message: None, spawn=lambda _label, work: work.close(),
                          submit=lambda _label, run: run())
    planner.setup(ctx)
    path = planned((0.0, 0.0, 0.0), (1.0, 1.0, 0.0))
    planner._plan = Plan(1, BasePlanRequest(SERIAL, tuple(path.goal)), "ready", path)

    async def main():
        return await planner.commit(1)

    return asyncio.run(main()), planner


@pytest.mark.safety
def test_commit_sends_a_geometric_path_capped_at_the_planning_speeds():
    """The waypoints go out untimed, with the planning speeds as caps; the planner keeps the path id."""
    base = FakeBase()
    (sent, message), planner = commit(base)
    assert sent and "path 7" in message and planner.plan.state == "sent"
    (x, y, yaw, t, linear, angular), = base.sent
    np.testing.assert_array_equal(np.column_stack([x, y, yaw]), planner.plan.path.points)
    assert t is None and (linear, angular) == (MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)
    assert planner._sent_id == 7


@pytest.mark.safety
@pytest.mark.parametrize("base, why", [
    (FakeBase(pose_age=None), "no mocap pose"),
    (FakeBase(pose_age=0.5), "no mocap pose"),
    (FakeBase(refuses=True), "not finite"),
    (FakeBase(estopped=True), "e-stopped"),
    (FakeBase(follower_age=None), "no follower"),
    (FakeBase(follower_age=5.0), "no follower"),
])
def test_commit_refused(base, why):
    """Nothing goes out without a live mocap pose, while e-stopped, with no follower, or when the base refuses it.

    A relay silent for 0.5 s counts as no pose, though the last sample was valid.
    """
    (sent, message), planner = commit(base)
    assert not sent and why in message and not base.sent and planner.plan.state == "ready"


@pytest.mark.safety
@pytest.mark.parametrize("x, stale", [(0.02, False), (0.1, True)])
def test_plan_goes_stale_once_the_robot_moves_from_its_start(x, stale):
    """A ready plan stays ready while the base is near the plan's start, and goes stale beyond STALE_POSITION."""
    _, planner = commit(FakeBase(follower_age=None))
    pose = SimpleNamespace(position=np.array([x, 0.0, 0.0]), orientation=np.array([0.0, 0.0, 0.0, 1.0]))
    planner.update(SimpleNamespace(kinematics=SimpleNamespace(base_pose=lambda _serial: pose)))
    assert planner.plan.state == ("stale" if stale else "ready")
    assert ("moved" in planner.plan.message) == stale


def test_request_settings_become_the_algorithms_settings():
    """The request's (name, value) pairs replace the defaults; an unknown algorithm or setting is refused."""
    planner = BasePlannerPlugin()
    algorithm, settings = planner._algorithm(BasePlanRequest(SERIAL, (0.0, 0.0, 0.0), "RRT + arcs",
                                                             (("lead", 0.3), ("arcs", False))))
    assert algorithm.label == "RRT + arcs" and settings.lead == 0.3 and not settings.arcs
    assert planner._algorithm(BasePlanRequest(SERIAL, (0.0, 0.0, 0.0)))[0] is planner._algorithms[0]
    for request in (BasePlanRequest(SERIAL, (0.0, 0.0, 0.0), "nope"),
                    BasePlanRequest(SERIAL, (0.0, 0.0, 0.0), "RRT + arcs", (("speed", 1.0),))):
        with pytest.raises(ValueError):
            planner._algorithm(request)


#: A recorded request of experiment "trial".
REQUEST = BasePlanRequest(SERIAL, (1.0, 1.0, 0.0), record=True, experiment="trial")


def _saving(tmp_path):
    """A plugin recording to `tmp_path`, a trace of two poses, and a context with a simulated base."""
    plugin = BasePlannerPlugin()
    plugin._recordings = tmp_path
    rec = Trace([Signal(f"{SERIAL}_floor_pose", lambda: None, ("x", "y", "yaw"), "m, rad")])
    rec.add(1.0, (0.0, 0.0, 0.0))
    rec.add(2.0, (0.5, 0.4, 0.7))
    base = SimpleNamespace(state=SimpleNamespace(follower=None), simulation=SimpleNamespace(node="husky_sim"),
                           follower_history=lambda _since: np.zeros((3, 5)),
                           odometry_history=lambda _since: np.zeros((2, 4)))
    ctx = SimpleNamespace(world=SimpleNamespace(robots={SERIAL: SimpleNamespace(base=base)}),
                          log_info=lambda _message: None, log_error=pytest.fail)
    return plugin, rec, ctx


def test_unrecorded_plans_run_gets_an_attempt_of_its_own(tmp_path):
    """Planned with Record off: the run gets a folder of its own, with what is known of the plan.

    It holds recording.npz (samples, path, follower log) and experiment.json.
    """
    plugin, rec, ctx = _saving(tmp_path)
    path = planned((0.0, 0.0, 0.0), (1.0, 1.0, 0.0))
    plugin._save(ctx, REQUEST, path, rec, "done", "2026-10-14T19:00:00+0200", 0.0, {})

    folder, = (tmp_path / "trial").iterdir()
    assert folder.name.endswith(f"_{SERIAL}_free_sim")
    data = np.load(folder / "recording.npz")
    np.testing.assert_array_equal(data["path_poses"], path.points)
    assert data[f"{SERIAL}_floor_pose"].shape == (2, 3) and data["commands"].shape == (3, 5)
    description = json.loads((folder / "experiment.json").read_text())
    assert description["run"]["outcome"] == "done" and description["plan"]["length"] == pytest.approx(path.length)


def test_run_joins_its_recorded_attempt_and_a_second_run_gets_its_own(tmp_path):
    """A recorded plan's run lands in its attempt; committing it again saves apart, naming the first attempt."""
    plugin, rec, ctx = _saving(tmp_path)
    folder = tmp_path / "trial" / "20261015_150000_alice_passage_sim"
    folder.mkdir(parents=True)
    setup = {"experiment": "trial", "task": "passage", "robot": SERIAL, "environment": "sim",
             "algorithm": "Lattice (hybrid A*)", "settings": {"margin": 0.05}}
    (folder / "experiment.json").write_text(json.dumps({**setup, "plan": {"found": True, "seconds": 1.5},
                                                        "run": None}))
    for _ in range(2):
        plugin._save(ctx, REQUEST, planned((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), rec, "done", "", 0.0,
                     {"folder": folder, "setup": setup, "seconds": 1.5})

    descriptions = {path.parent.name: json.loads(path.read_text()) for path in (tmp_path / "trial").glob("*/*.json")}
    assert len(descriptions) == 2 and descriptions[folder.name]["run"]["outcome"] == "done"
    second, = (description for name, description in descriptions.items() if name != folder.name)
    assert second["plan_folder"] == folder.name and second["task"] == "passage" and second["plan"]["seconds"] == 1.5


class FakeClock:
    """The part of a plugin context a run uses, ticking 10 Hz ROS time; `step` changes the world every tick."""

    def __init__(self, robot, step):
        """Start at ROS time 10 s, with SERIAL being `robot`."""
        self.t, self.robot, self.step, self.traces, self.logs = 10.0, robot, step, [], []
        self.world = SimpleNamespace(robots={SERIAL: robot})

    def now(self) -> float:
        """ROS time, seconds."""
        return self.t

    @contextmanager
    def record(self, *signals):
        """Sample `signals` every tick while the block runs."""
        trace = Trace(signals)
        self.traces.append(trace)
        try:
            yield trace
        finally:
            self.traces.remove(trace)

    async def wait_until(self, predicate, timeout_s=None, description=""):
        """Tick until `predicate` holds."""
        deadline = None if timeout_s is None else self.t + timeout_s
        while not predicate():
            if deadline is not None and self.t >= deadline:
                raise WaitTimeout(description)
            self.t += 0.1
            self.step(self.t)
            for trace in self.traces:
                trace.sample(self.t)
            await asyncio.sleep(0)

    async def sleep(self, seconds):
        """Tick for `seconds`."""
        until = self.t + seconds
        await self.wait_until(lambda: self.t >= until)

    def log_info(self, message):
        """Keep the message."""
        self.logs.append(message)

    log_error = log_info


def test_run_is_recorded_until_the_follower_is_done(tmp_path):
    """The robot's poses from Commit until done (plus settling) are saved, with the outcome."""
    path = planned((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    follower = FollowerState(controller="pure_pursuit", state="following", reason="", path_id=7, timed=False,
                             piece=0, piece_count=1, turning=False, progress=0.0, reference=np.zeros(3),
                             lookahead=np.full(2, np.nan), cross_track_error=0.0, along_track_error=np.nan,
                             yaw_error=0.0, command=np.zeros(2), update_time=10.0, stamp=10.0)
    base = SimpleNamespace(state=SimpleNamespace(position=np.zeros(3), orientation=np.array([0.0, 0.0, 0.0, 1.0]),
                                                 follower=follower),
                           simulation=SimpleNamespace(node="husky_sim"),
                           follower_history=lambda _since: np.zeros((0, 5)),
                           odometry_history=lambda _since: np.zeros((0, 4)))
    robot = SimpleNamespace(base=base, config=SimpleNamespace(serial=SERIAL))

    def step(t):
        """Drive along x at 0.2 m/s; the follower reports done at the end."""
        x = min(0.2 * (t - 10.0), 1.0)
        base.state.position = np.array([x, 0.0, 0.0])
        follower.progress, follower.update_time = x, t
        follower.state = "done" if x >= 1.0 else "following"

    ctx = FakeClock(robot, step)
    plugin = BasePlannerPlugin()
    plugin._recordings = tmp_path
    plugin._sent_id, plugin._sent_at = 7, ctx.now()
    asyncio.run(plugin._follow(ctx, REQUEST, path, None))

    folder, = (tmp_path / "trial").iterdir()
    data = np.load(folder / "recording.npz")
    poses = data[f"{SERIAL}_floor_pose"]
    assert poses[-1, 0] == pytest.approx(1.0) and data["t"][-1] - data["t"][0] >= 5.0
    assert str(data["outcome"]) == "done"
    assert json.loads((folder / "experiment.json").read_text())["environment"] == "sim"


def test_attempt_counts_for_its_task_only_from_its_start_to_its_target():
    """From the task's start to its target counts; from 20 cm off, or to elsewhere, it is a free attempt."""
    plugin = BasePlannerPlugin()
    ctx = SimpleNamespace(require=lambda _name: SimpleNamespace(loaded=None, index=0))
    start, target = BY_NAME["passage"].start_in_lab(), BY_NAME["passage"].target_in_lab()
    assert plugin._task_of(ctx, "passage", start, target) == "passage"
    assert plugin._task_of(ctx, "passage", (start[0] + 0.2, start[1], start[2]), target) is None
    assert plugin._task_of(ctx, "passage", start, start) is None
    assert plugin._task_of(ctx, None, start, target) is None


def test_process_task_counts_only_at_its_step():
    """step_110 counts only with step 110 selected in the process plugin."""
    plugin = BasePlannerPlugin()
    task = BY_NAME["step_110"]
    for index, counts in ((110, True), (109, False)):
        selection = SimpleNamespace(index=index)
        ctx = SimpleNamespace(require=lambda _name, selection=selection: SimpleNamespace(selection=selection))
        assert (plugin._task_of(ctx, "step_110", task.start, task.target) == "step_110") == counts
