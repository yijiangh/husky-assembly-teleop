"""The base planner's paths as the onboard follower cuts them into pieces, and Commit sending them to it."""

import asyncio
import json
import math
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import pytest
from crl_husky.follower_path import FollowerPath

from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugin_api.trace import Signal, Trace
from husky_assembly_teleop.robot_interface.base import FollowerState
from husky_assembly_teleop.plugins.base_planner.path import MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, steer, timed_path
from husky_assembly_teleop.plugins.base_planner.plugin import BasePlannerPlugin
from husky_assembly_teleop.plugins.base_planner.tasks import BY_NAME

SERIAL = "alice"


def planned(*corners):
    """The path the RRT makes of these corners: steer between each pair, timed."""
    poses = [corners[0]]
    for q1, q2 in zip(corners[:-1], corners[1:]):
        poses += steer(q1, q2)
    return timed_path(poses)


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

    def __init__(self, tracked=True, estopped=False, follower_age=0.1):
        """A base at ROS time 10 s, with a follower report `follower_age` seconds old (None: no follower)."""
        follower = None if follower_age is None else SimpleNamespace(update_time=10.0 - follower_age)
        self.state = SimpleNamespace(tracked=tracked, estopped=estopped, follower=follower)
        self.sent = []

    def send_path(self, *args):
        """Keep the path; its id is always 7."""
        self.sent.append(args)
        return 7


def commit(base: FakeBase):
    """Commit a ready plan for SERIAL on `base`; returns (sent, message) and the plugin. The run is not started."""
    plugin = BasePlannerPlugin()
    plugin.path, plugin._path_serial = planned((0.0, 0.0, 0.0), (1.0, 1.0, 0.0)), SERIAL
    ctx = SimpleNamespace(world=SimpleNamespace(robots={SERIAL: SimpleNamespace(base=base)}),
                          now=lambda: 10.0, log_info=lambda _message: None, spawn=lambda _label, work: work.close())
    return plugin.send_plan(ctx), plugin


def test_commit_sends_a_geometric_path_capped_at_the_planning_speeds():
    """The waypoints go out untimed, with the planning speeds as caps; the plugin keeps the path id."""
    base = FakeBase()
    (sent, message), plugin = commit(base)
    assert sent and "path 7" in message
    (x, y, yaw, t, linear, angular), = base.sent
    np.testing.assert_array_equal(np.column_stack([x, y, yaw]), plugin.path.points)
    assert t is None and (linear, angular) == (MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)
    assert plugin._sent_id == 7


@pytest.mark.parametrize("base, why", [
    (FakeBase(tracked=False), "not tracked"),
    (FakeBase(estopped=True), "e-stopped"),
    (FakeBase(follower_age=None), "no follower"),
    (FakeBase(follower_age=5.0), "no follower"),
])
def test_commit_refused(base, why):
    """Nothing goes out while the base is untracked or e-stopped, or no follower reports."""
    (sent, message), _ = commit(base)
    assert not sent and why in message and not base.sent


def _saving(tmp_path):
    """A plugin recording to `tmp_path`, a trace of two poses, and a context with a simulated base."""
    plugin = BasePlannerPlugin()
    plugin._recordings = tmp_path
    plugin._experiment = SimpleNamespace(value="trial")
    rec = Trace([Signal(f"{SERIAL}_floor_pose", lambda: None, ("x", "y", "yaw"), "m, rad")])
    rec.add(1.0, (0.0, 0.0, 0.0))
    rec.add(2.0, (0.5, 0.4, 0.7))
    base = SimpleNamespace(state=SimpleNamespace(follower=None), node_names=lambda: ["husky_sim"],
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
    plugin._save(ctx, SERIAL, path, rec, "done", "2026-10-14T19:00:00+0200", 0.0, {})

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
        plugin._save(ctx, SERIAL, planned((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), rec, "done", "", 0.0,
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
                           node_names=lambda: ["husky_sim", "pure_pursuit"],
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
    plugin._recordings, plugin._recording = tmp_path, SimpleNamespace(value=True)
    plugin._experiment = SimpleNamespace(value="trial")
    plugin._sent_id, plugin._sent_at = 7, ctx.now()
    asyncio.run(plugin._follow(ctx, SERIAL, path, None))

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
    plugin._task = SimpleNamespace(value="passage")
    start, target = BY_NAME["passage"].start_in_lab(), BY_NAME["passage"].target_in_lab()
    assert plugin._task_of(ctx, start, target) == "passage"
    assert plugin._task_of(ctx, (start[0] + 0.2, start[1], start[2]), target) is None
    assert plugin._task_of(ctx, start, start) is None


def test_process_task_counts_only_at_its_step():
    """step_110 counts only with step 110 selected in the process plugin."""
    plugin = BasePlannerPlugin()
    plugin._task = SimpleNamespace(value="step_110")
    task = BY_NAME["step_110"]
    for index, counts in ((110, True), (109, False)):
        ctx = SimpleNamespace(require=lambda _name, index=index: SimpleNamespace(loaded=object(), index=index))
        assert (plugin._task_of(ctx, task.start, task.target) == "step_110") == counts


@pytest.mark.parametrize("park", [(1.0, -2.0, 0.5), None])
def test_park_loads_the_robots_parked_pose(park):
    """Load → Park targets the robot's parked pose, or says it has none."""
    plugin, targets, said = BasePlannerPlugin(), [], []
    plugin.serial, plugin._used = SERIAL, SimpleNamespace(touch=lambda: None)
    plugin._target = SimpleNamespace(set=lambda *pose: targets.append(pose))
    plugin._say = lambda message, failed=False: said.append(failed)
    ctx = SimpleNamespace(config=SimpleNamespace(robots=[SimpleNamespace(serial=SERIAL, park_pose=park)]))
    plugin._load_target(ctx, "Park")
    assert targets == ([park] if park else []) and said == [park is None]
