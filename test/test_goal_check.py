"""The goal_check plugin: its log line, its record, and the request it sends, with the other plugins faked."""

from __future__ import annotations

import asyncio
import json
import math
from types import SimpleNamespace

from plugin_fakes import FakeContext

from husky_assembly_teleop.plugins.goal_check import GoalCheckPlugin, goal_line
from husky_assembly_teleop.shared.planning.ik import IkResult
from husky_assembly_teleop.shared.planning.movement import Routine
from husky_assembly_teleop.shared.planning.route_result import GoalCheck
from husky_assembly_teleop.shared.planning.scene_for import RobotBelief
from husky_assembly_teleop.shared.runs import RunSettings
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

NAMES = [f"{arm}_{joint}" for arm in ("left_ur_arm", "right_ur_arm") for joint in UR_JOINT_NAMES]
RAW = dict(zip(NAMES, [0.1 * index for index in range(12)]))
AUTHORED = {**RAW, "right_ur_arm_elbow_joint": RAW["right_ur_arm_elbow_joint"] + 0.0015}
BASE = Pose((1.0, -2.5, 0.0), (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))


def check(ok: bool = True) -> GoalCheck:
    """A check of Cindy's transfer, IK 0.0015 rad off the authored goal at one joint."""
    pair = ("robots/a200-0806/base_link", "process/bars/B20")
    ik = IkResult(dict(RAW), 0.0015, 1, 64, "ssik", ("left", "right"), pair)
    return GoalCheck("B20_J_transfer", Routine.TRANSFER_FREE, ok, "" if ok else "no IK", "robots/a200-0806", BASE,
                     dict(RAW), ik, dict(RAW), AUTHORED, {"robots/a200-0804": "live"}, ("a warning",), 1.5)


def test_the_record_is_json():
    """A check's record goes through JSON as it is, the raw gap included."""
    record = json.loads(json.dumps(check().as_record(), allow_nan=False))
    assert record["routine"] == "transfer_free" and record["base"][:3] == [1.0, -2.5, 0.0]
    assert record["ik"]["colliding"] == ["robots/a200-0806/base_link", "process/bars/B20"]
    assert abs(record["max_from_authored"] - 0.0015) < 1e-12


def test_the_log_line():
    """The line names the movement, the gap, each arm's goal and the base; a failure its reason."""
    line = goal_line(check())
    assert line.startswith("B20_J_transfer at the live base: max |goal - authored goal| = 0.0015 rad; goal (rad) left "
                           "[+0.0000, +0.1000, +0.2000, +0.3000, +0.4000, +0.5000] right [+0.6000, ")
    assert line.endswith("; base 1.000 -2.500 m 90.0 deg")
    assert goal_line(check(ok=False)) == "B20_J_transfer: no goal at the live base: no IK"


class Planner:
    """The motion planner: answers every goal request with `check()`, keeping the request."""

    def __init__(self):
        """No request yet."""
        self.requests = []

    def request_ik(self, request) -> asyncio.Future:
        """Answer at once."""
        self.requests.append(request)
        answer = asyncio.get_running_loop().create_future()
        answer.set_result(check())
        return answer


def test_check_goal_asks_with_the_run_and_records_the_answer():
    """The request carries the run's beliefs and workaround and the tick's snapshot; the answer is recorded."""
    step = SimpleNamespace(label="B20_J_transfer", action=SimpleNamespace(id="B20_J"),
                           movement=SimpleNamespace(id="B20_J_transfer"))
    selection = SimpleNamespace(loaded="loaded", index=4, step=step)
    beliefs = {"robots/a200-0805": RobotBelief(Pose())}
    records = []
    run = SimpleNamespace(view=SimpleNamespace(beliefs=beliefs, settings=RunSettings(transfer_as_free=True)),
                          request_record=lambda event, **fields: records.append((event, fields)))
    planner = Planner()
    ctx = FakeContext(plugins={"process": SimpleNamespace(selection=selection), "assembly_run": run,
                               "motion_planner": planner})
    plugin = GoalCheckPlugin()

    answer = asyncio.run(plugin.check(ctx))
    request, = planner.requests
    assert (request.loaded, request.step, request.snapshot) == ("loaded", step, ctx.scene.snapshot)
    assert request.beliefs == beliefs and request.transfer_as_free and request.label == "check goal B20_J_transfer"
    (event, fields), = records
    assert event == "check_goal" and (fields["action"], fields["movement"]) == ("B20_J", "B20_J_transfer")
    assert fields["data"] == answer.as_record()
    assert ctx.lines("info") == [goal_line(answer)] and plugin._checked_at == ("loaded", 4)


class Ghost:
    """A goal ghost: what it shows, or None while hidden."""

    def __init__(self):
        """Hidden."""
        self.shown = None

    def show(self, base, joints, parts=None):
        """Keep what is shown."""
        self.shown = (base, dict(joints))

    def hide(self):
        """Hide."""
        self.shown = None


def test_the_ghost_shows_only_while_its_step_is_selected():
    """The acting robot's ghost shows IK's joints at the base while the checked step is selected; else it hides."""
    plugin = GoalCheckPlugin()
    cindy, alice = Ghost(), Ghost()
    plugin._ghosts = {"a200-0806": cindy, "a200-0804": alice}
    plugin._serials = {"robots/a200-0806": "a200-0806", "robots/a200-0804": "a200-0804"}
    plugin._status, plugin._details = SimpleNamespace(content=""), SimpleNamespace(content="")
    loaded = object()
    selection = SimpleNamespace(loaded=loaded, index=4)
    ctx = FakeContext(plugins={"process": SimpleNamespace(selection=selection)})
    plugin._check, plugin._checked_at = check(), (loaded, 4)
    plugin.draw(ctx)
    assert cindy.shown == (BASE, RAW) and alice.shown is None
    selection.index = 5
    plugin.draw(ctx)
    assert cindy.shown is None
    selection.index, plugin._check = 4, check(ok=False)
    plugin.draw(ctx)
    assert cindy.shown is None and "no goal" in plugin._status.content
