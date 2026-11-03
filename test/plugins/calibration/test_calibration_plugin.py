"""The calibration plugin on a real viser server: a run through its steps, what goes out of date, and a resume.

Belle with a still base, an arm that arrives at once with the tool body on tool0, a relay applying her default
calibration, and an arm planner that finds everything clear. Most tests write takes from her 20261013 session
instead of sweeping; one runs every procedure. The D-pad and the hand moves never run (`test_calibration_driving.py`,
`test_calibration_hand_moves.py`).
"""

from __future__ import annotations

import asyncio
import json
from contextlib import contextmanager
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
import viser

from body_definition_fixtures import MARKERS, body, moved
from calibration_fixtures import record_take
from plugin_fakes import WATCHES
from repo_paths import DATA
from husky_assembly_teleop.colors import OK, rgb
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugin_api.trace import Trace
from husky_assembly_teleop.ui import second_press
from husky_assembly_teleop.ui.ghost import RecentUse
from husky_assembly_teleop.plugins.calibration import plugin as calibration_plugin
from husky_assembly_teleop.plugins.calibration.analysis.checks import CHECKLIST_BOXES, PRECONDITION_HOLDS, pose_change
from husky_assembly_teleop.plugins.calibration.plugin import CalibrationPlugin
from husky_assembly_teleop.plugins.calibration.steps import STEPS
from husky_assembly_teleop.plugins.calibration.steps.base import CalibrationStep
from husky_assembly_teleop.plugins.calibration.steps import s4_record as record_step
from husky_assembly_teleop.plugins.calibration.steps.s1_setup import NEW_RUN
from husky_assembly_teleop.plugins.calibration.storage.progress import Progress
from husky_assembly_teleop.plugins.calibration.robot.sweeps import named
from husky_assembly_teleop.shared.planner.timed_path import TimedPath
from husky_assembly_teleop.shared.guide.step import fails
from husky_assembly_teleop.world.checks import BAD, GOOD, INFO, PENDING, WARN
from husky_assembly_teleop.robot_interface.tool_step import tool_step_target
from husky_assembly_teleop.robot_interface.mocap_relay import (FRAME_HELD, FRAME_REFIT_PENDING, FRAME_UNTRUSTED,
                                                               AppliedCalibration, FloorCalibrationState,
                                                               MocapRelayState, applied_calibrations)
from husky_assembly_teleop.world.calibration_sessions import (SESSIONS_FOLDER, Calibration, default_calibration,
                                                              read_session_file)
from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
from scipy.spatial.transform import Rotation

SERIAL, ARM, NOW = "a200-0805", "ur_arm", 100.0
#: The Unix time preconditions are stamped with at the start (`Calibration.wall_now`); tests move `ctx.wall`.
WALL = 1_800_000_000.0
#: Where the relay publishes Belle's base_footprint.
FOOTPRINT = Pose((1.0, 2.0, 0.0))
#: The punch tip in tool0, metres.
TIP = (0.001, -0.002, 0.11)
#: The floor calibration the relay holds the lab frame by, and its lab_from_raw.
FLOOR, LAB = "20261020_101500_mocap_floor_calibration.json", ((0.1, -0.2, 0.0), (0.0, 0.0, 0.0, 1.0))


def announced(calibration: Calibration) -> AppliedCalibration:
    """A calibration as the monitor gets it from the relay's announcement."""
    return applied_calibrations(json.dumps({"calibrations": [calibration.as_dict()]}))[calibration.mocap_id]


@pytest.fixture(scope="module")
def server():
    """One viser server on a free port for the whole module, with a second as its phone page (`server.phone`)."""
    server = viser.ViserServer(port=0, verbose=False)
    server.phone = viser.ViserServer(port=0, verbose=False)
    yield server
    server.phone.stop()
    server.stop()


def tracked(mocap_id: int, pose: Pose = FOOTPRINT) -> SimpleNamespace:
    """A rigid body at `pose`, tracked well just now."""
    return SimpleNamespace(mocap_id=mocap_id, position=pose.position, orientation=pose.orientation, marker_error=1e-4,
                           marker_errors=[(NOW - 0.1, 1e-4)], last_update_time=NOW, tracking_valid=True, tracked=True)


class Ghost:
    """Keeps the joints it was last shown at; None while hidden."""

    joints = None

    def show(self, _base, joints, parts=None):
        """Keep the joints."""
        self.joints = joints

    def hide(self):
        """Forget them."""
        self.joints = None


class ArmState:
    """The driver's state: still, and reporting tool0 by the URDF as its TCP, as with the pendant's TCP zero."""

    def __init__(self, arm: "Arm", urdf):
        """Following `arm` on `urdf`, its TCP heard just now."""
        self._arm, self._urdf = arm, urdf
        self.is_executing, self.last_motion_time, self.last_slight_motion_time = False, None, None
        self.last_update_time, self.tcp_update_time = None, NOW

    def _tool0(self) -> Pose:
        joints = named(ARM, self._arm.joints)
        return compose(invert(link_pose(self._urdf, Pose(), joints, f"{ARM}_base_link")),
                       link_pose(self._urdf, Pose(), joints, f"{ARM}_tool0"))

    @property
    def tcp_position(self):
        """tool0 in the arm's base_link at the current joints."""
        return self._tool0().position

    @property
    def tcp_orientation(self):
        """tool0's orientation in the arm's base_link."""
        return self._tool0().orientation


class Arm:
    """An arm that is where it was last sent, at once."""

    def __init__(self, joints, urdf):
        """Standing at `joints`."""
        self.joints, self.simulation, self.last_refusal = np.array(joints, dtype=float), None, None
        self.state = ArmState(self, urdf)

    def joint_vector(self) -> np.ndarray:
        """Where it stands."""
        return self.joints.copy()

    def send_joint_move(self, joints, min_duration: float) -> float:
        """Arrive at once."""
        self.joints = np.array(joints, dtype=float)
        return min_duration

    def hold(self) -> None:
        """Nothing to stop."""


class FlangeBody:
    """The tool body: tool0 of the arm on the published base pose, wherever the arm stands; no message until `hear`."""

    def __init__(self, mocap_id: int, arm: Arm, urdf):
        """Following `arm`, as a body just tracked: no message yet."""
        self.arm, self.urdf, self._at = arm, urdf, None
        self.mocap_id, self.marker_error, self.marker_errors = mocap_id, None, []
        self.last_update_time, self.tracking_valid, self.tracked = None, None, False

    def hear(self) -> None:
        """A message: tracked well just now."""
        self.marker_error, self.marker_errors = 1e-4, [(NOW - 0.1, 1e-4)]
        self.last_update_time, self.tracking_valid, self.tracked = NOW, True, True

    def _pose(self) -> Pose:
        key = tuple(self.arm.joints)
        if self._at is None or self._at[0] != key:
            self._at = (key, link_pose(self.urdf, FOOTPRINT, named(ARM, self.arm.joints), f"{ARM}_tool0"))
        return self._at[1]

    @property
    def position(self):
        """Tool0's position."""
        return self._pose().position

    @property
    def orientation(self):
        """Tool0's orientation."""
        return self._pose().orientation


class Planner:
    """An arm planner that finds every path clear and every goal reachable, and moves the arm there on commit."""

    def __init__(self, arm: Arm):
        """Planning for `arm`."""
        self.arm, self.goal = arm, None
        #: The robots each commit let be unknown, in order.
        self.unknown: list[frozenset[str]] = []

    async def check_path(self, *_args):
        """No hit."""
        return None

    async def request_plan(self, request):
        """A straight move to the goal."""
        self.goal = np.array(request.goal)
        return SimpleNamespace(state="ready", message="clear", id=1,
                               path=TimedPath(np.array([0.0, 1.0]), np.array([self.arm.joints, self.goal])))

    async def commit(self, _plan_id, unknown_robots=(), clearance=True):
        """Move there at once."""
        self.unknown.append(frozenset(unknown_robots))
        self.arm.joints = self.goal.copy()
        return True, ""


def make_ctx(server: viser.ViserServer, root) -> SimpleNamespace:
    """Belle and her relay; intents and tasks run at once, except the D-pad's and the hand moves', which never end."""
    config = robot_config_from_serial("0805", DATA)
    config = replace(config, arms=tuple(replace(arm, end_effector="calibration_marker") for arm in config.arms))
    arm = Arm(config.arms[0].stow_joints, config.model.urdf)
    base = SimpleNamespace(mocap_id=config.mocap_id, soft_stopped=False, simulation=None,
                           state=SimpleNamespace(**vars(tracked(config.mocap_id)), follower=None,
                                                 joint_states_update_time=None, wheel_velocities={}))
    robot = SimpleNamespace(config=config, base=base, arms={ARM: arm}, lease_slot=SimpleNamespace(lease=None))
    default = announced(default_calibration(config.mocap_id))
    relay = SimpleNamespace(state=MocapRelayState(calibrations={config.mocap_id: default}, calibrations_time=1.0,
                                                  drive_root=str(root),
                                                  frame=FloorCalibrationState(FRAME_HELD, FLOOR, lab_from_raw=LAB)),
                            reload_base_calibration=lambda mocap_id: "future" if mocap_id == config.mocap_id else None)
    hits = SimpleNamespace(replace=lambda *_args: None, clear=lambda *_args: None)

    @contextmanager
    def ui():
        yield server.gui

    @contextmanager
    def tab(*_path, **_options):
        yield server.gui

    @contextmanager
    def phone(_label=None, _icon=None):
        yield server.phone.gui

    @contextmanager
    def held(*_args, **_kwargs):
        yield None

    def defer(_label, run, operator_only=False):
        async def callback(*_args):
            run()
        return callback

    def defer_value(_label, run, operator_only=False):
        async def callback(event):
            if event.client is not None or not operator_only:
                run(event.target.value)
        return callback

    def spawn(label, work, survives_soft_stop=False):
        if label.startswith(("drive", "hand moves")) or "outputs" in label:
            work.close()
        else:
            asyncio.run(work)
        return SimpleNamespace(done=lambda: True, cancel=lambda: None)

    async def run_in_thread(function, *args):
        return function(*args)

    async def wait_until(predicate, timeout_s=None, description=""):
        for _ in range(100):
            if predicate():
                return
        raise WaitTimeout(description)

    async def ros(_future):
        """The relay reloads and applies the session's approved calibration."""
        run = ctx.plugin.cal.run
        output = run.output_file(approved=True)
        relay.state.calibrations[config.mocap_id] = announced(read_session_file(output, output.parent,
                                                                                config.mocap_id, SERIAL))
        return SimpleNamespace(success=True)

    async def sleep(_seconds):
        return None

    view = SimpleNamespace(ui=ui, tab=tab, phone=phone, gui=server.gui, scene=server.scene,
                           scene_root="/plugins/calibration")
    ctx = SimpleNamespace(
        name="calibration", view=view, world=SimpleNamespace(robots={SERIAL: robot}, mocap_relay=relay),
        scene=SimpleNamespace(snapshot=SimpleNamespace(robots={})),
        config=SimpleNamespace(data_directory=DATA, drive_root=root, tick_period=0.05, ghost_timeout=20.0),
        kinematics=SimpleNamespace(base_pose=lambda _serial: FOOTPRINT, joints=lambda _serial: {},
                                   link_pose=lambda _serial, link: link_pose(config.model.urdf, FOOTPRINT,
                                                                             named(ARM, arm.joints), link)),
        require={"collisions": hits, "arm_planner": Planner(arm)}.__getitem__, now=lambda: NOW, defer=defer,
        defer_value=defer_value, spawn=spawn, run_in_thread=run_in_thread, wait_until=wait_until, ros=ros,
        sleep=sleep, lease_robots=held, clearance=held, freeze=held, log=[],
        trace=lambda *signals, max_samples: Trace(signals, max_samples),
        track_object=lambda _name, mocap_id, label: FlangeBody(mocap_id, arm, config.model.urdf),
        untrack_object=lambda _name: None, **WATCHES)
    ctx.log_info = ctx.log_warn = ctx.log.append
    return ctx


def make_plugin(server, tmp_path, monkeypatch) -> tuple[CalibrationPlugin, SimpleNamespace, Ghost]:
    """The plugin set up with Belle's arm chosen; the Drive at `tmp_path`."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    ghost = Ghost()
    monkeypatch.setattr(calibration_plugin, "robot_ghosts", lambda *_args: {SERIAL: ghost})
    monkeypatch.setattr(record_step, "take_basis", lambda *_args, **_kwargs: None)
    # * The test's clock stands still: two presses at one instant would count as a double click (`test_second_press`).
    monkeypatch.setattr(second_press, "DOUBLE_CLICK_SECONDS", 0.0)
    ctx = make_ctx(server, tmp_path)
    plugin = ctx.plugin = CalibrationPlugin()
    plugin.setup(ctx)
    ctx.wall = WALL
    plugin.cal.wall_now = lambda: ctx.wall
    plugin.cal.tool.hear()
    plugin.draw(ctx)
    return plugin, ctx, ghost


def tick_setup(plugin: CalibrationPlugin, labels=None, ticked: bool = True) -> None:
    """Tick (or untick) Setup's checklist boxes as the operator does: all, or those named."""
    setup = plugin.steps[0]
    for box in setup._checklist.boxes:
        if labels is None or box.label in labels:
            box.value = ticked
            setup._on_tick(box.label, ticked)


def next_step(plugin: CalibrationPlugin, ctx) -> None:
    """Press Next, and draw."""
    plugin.panel.navigate(ctx, forward=True)
    plugin.draw(ctx)


def done(plugin: CalibrationPlugin, ctx) -> int:
    """Press Done, and draw; the step it went to."""
    plugin.panel.done(ctx)
    plugin.draw(ctx)
    return plugin.panel.index


def check_collisions(plugin: CalibrationPlugin, ctx) -> None:
    """Press Check collisions in Sweeps."""
    sweeps = plugin.steps[2]
    sweeps.start(ctx, "check", sweeps._check_collisions(ctx))


def measure_markers(plugin: CalibrationPlugin, ctx) -> None:
    """Small marker errors, driven long enough, just now."""
    run = plugin.cal.run
    run.drive_seconds, run.marker_max, run.marker_time = 6.0, [1e-4, 1e-4], ctx.wall


def walk_to_validate(plugin: CalibrationPlugin, ctx) -> None:
    """Tick the checklist, drive enough, check the collisions, record takes and solve; Done goes on to Validate."""
    tick_setup(plugin)
    measure_markers(plugin, ctx)
    check_collisions(plugin, ctx)
    run = plugin.cal.run
    for joint in (0, 1):
        for take in range(3):
            record_take(run, joint, take)
    plugin.steps[4]._solve(ctx)
    assert done(plugin, ctx) == 5


def states(plugin: CalibrationPlugin, ctx) -> list[str]:
    """Each step's state in the overview."""
    return [step.state(ctx).label for step in plugin.steps]


def test_done_goes_to_the_work_left_through_the_preconditions_it_needs(server, tmp_path, monkeypatch):
    """Done from Setup goes to the expired preconditions Record needs, one after another, then to Record.

    Then on to Solve and Validate as each is done; nothing was accepted on the way.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    panel, run = plugin.panel, plugin.cal.run
    assert states(plugin, ctx) == ["none yet"] * 9
    tick_setup(plugin)
    assert done(plugin, ctx) == 1  # * Record is the work left; Marker error and Sweeps, which it needs, come first
    measure_markers(plugin, ctx)
    assert done(plugin, ctx) == 2
    assert done(plugin, ctx) == 2  # * Sweeps unchecked: Done asks once more
    assert "Press Done again within 5 s to skip: it goes on as it stands" in panel._notes_html.content
    check_collisions(plugin, ctx)
    assert run.usable() == list(range(12)) and done(plugin, ctx) == 3
    for joint in (0, 1):
        for take in range(3):
            record_take(run, joint, take)
    assert done(plugin, ctx) == 4
    plugin.steps[4]._solve(ctx)
    assert not fails(plugin.steps[4].checks(ctx)) and run.current_solved().output.is_file()
    assert done(plugin, ctx) == 5
    assert states(plugin, ctx) == ["holds"] * 3 + ["passes"] * 2 + ["none yet"] * 4
    assert run.acceptances == {}


def test_a_precondition_expired_after_the_target_is_gathered_on_the_way_back(server, tmp_path, monkeypatch):
    """Done at an expired precondition step goes on to the next one the target needs, even if this one still expired."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    ctx.wall += PRECONDITION_HOLDS + 60
    assert plugin.cal.show_work_left(ctx) == "Setup"
    # * The checklist still expired: Done asks once more, then on to Marker error, which Validate needs too.
    assert done(plugin, ctx) == 0 and done(plugin, ctx) == 1
    measure_markers(plugin, ctx)
    assert done(plugin, ctx) == 5


def test_a_target_left_by_navigating_does_not_skip_its_expired_preconditions(server, tmp_path, monkeypatch):
    """Sent to Setup for Record, then Next to Validate: Done there still goes to Setup, then on towards Record."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    run = plugin.cal.run
    plugin.cal.track_tool(ctx, run.tool_body + 1)
    ctx.wall += PRECONDITION_HOLDS + 60
    assert plugin.cal.show_work_left(ctx) == "Setup"
    for _ in range(5):
        plugin.panel.navigate(ctx, forward=True)
    assert plugin.panel.current.name == "Validate" and plugin.panel.current.state(ctx).label == "none yet"
    assert done(plugin, ctx) == 5 and done(plugin, ctx) == 0
    # * The checklist still expired: Done asks once more, then on to Marker error, which Record needs too.
    assert done(plugin, ctx) == 0 and done(plugin, ctx) == 1


def test_done_never_accepts_a_missing_or_out_of_date_result(server, tmp_path, monkeypatch):
    """Done twice on Record with no take goes on unstamped: the takes recorded later and then out of date show so."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    tick_setup(plugin)
    measure_markers(plugin, ctx)
    check_collisions(plugin, ctx)
    run = plugin.cal.run
    plugin.panel.show(ctx, 3)
    assert done(plugin, ctx) == 3 and done(plugin, ctx) == 4 and run.acceptances == {}
    for joint in (0, 1):
        for take in range(3):
            record_take(run, joint, take)
    plugin.cal.track_tool(ctx, run.tool_body + 1)
    assert states(plugin, ctx)[3] == "out of date" and plugin.cal.show_work_left(ctx) == "Record"


def test_work_not_done_yet_is_grey_and_a_half_done_collision_check_none_yet(server, tmp_path, monkeypatch):
    """Sweeps, Solve and Apply not done yet show grey chips; a collision check cancelled half-way has no result yet."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    sweeps, solve, apply = plugin.steps[2], plugin.steps[4], plugin.steps[6]
    assert [sweeps.checks(ctx)[0].level, solve.checks(ctx)[0].level, apply.checks(ctx)[0].level] == [PENDING] * 3
    run = plugin.cal.run
    run.collisions = {0: ""}
    assert sweeps.state(ctx).label == "none yet" and sweeps.checks(ctx)[0].level == PENDING


def test_take_chips_say_what_to_do_and_marker_chips_name_the_worst_sample(server, tmp_path, monkeypatch):
    """A left-out take's chip says why and to record it again; the marker chip's hover names its worst sample."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    tick_setup(plugin)
    measure_markers(plugin, ctx)
    check_collisions(plugin, ctx)
    run = plugin.cal.run
    index = record_take(run, 0, 0)
    recorded = run.takes[index]
    run.takes[index] = replace(recorded, circle=replace(recorded.circle, errors=recorded.circle.errors + 2e-3),
                               marker_errors=({"base": 1e-4}, {"base": 4e-4, "tool": 1.6e-3}, {"base": 2e-4}))
    takes = plugin.cal.takes
    chips = takes.chips(run, range(len(run.sweeps.sweeps)), by_collisions=False)
    assert "left out of Solve: fit " in chips and "record it again" in chips
    assert "not checked for collisions" not in chips and 'title="to record: pick it in Take"' in chips
    assert 'title="worst of 3: sample 2, base 0.40 mm, tool 1.60 mm"' in takes.marker_chips(run, [index])


def test_apply_shows_what_this_run_applied_and_the_relay_live(server, tmp_path, monkeypatch):
    """Applied stays passing when the relay restarts or applies another calibration: a live amber chip says so.

    An earlier solution applied is red and out of date: Apply again, after Solve once the takes changed.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    run, procedure, validate, apply = plugin.cal.run, plugin.cal.procedure, plugin.steps[5], plugin.steps[6]
    procedure.auto = True
    validate.start(ctx, "validation", validate._validate(ctx))
    procedure.auto = False
    run.accept("Validate")
    press_go_when_asked(ctx, procedure)
    apply.start(ctx, "reload", apply._reload(ctx))
    assert run.finished and apply.state(ctx).label == "passes"
    assert {c.label: c.level for c in apply.checks(ctx)}["relay now"] == GOOD
    relay = ctx.world.mocap_relay
    relay.state.calibrations[run.robot.config.mocap_id] = announced(default_calibration(run.robot.config.mocap_id))
    assert {c.label: c.level for c in apply.checks(ctx)}["relay now"] == WARN and apply.state(ctx).label == "passes"
    relay.state.calibrations_time = None
    assert {c.label: c.level for c in apply.checks(ctx)}["relay now"] == PENDING
    assert apply.state(ctx).label == "passes" and plugin.cal.show_work_left(ctx) == "Tip offset"
    run.applied_sha = "0" * 64  # * an earlier solution's
    applied = {c.label: c for c in apply.checks(ctx)}["applied"]
    assert applied.level == BAD and applied.detail.endswith(": Apply again")
    assert apply.state(ctx).label == "out of date" and apply.state(ctx).detail == applied.detail
    monkeypatch.setattr(run, "current_solved", lambda: None)  # * the takes changed since
    assert apply.state(ctx).detail.endswith(": Solve, then Apply again")


def test_done_at_an_expired_precondition_goes_on_to_the_work_it_is_needed_by(server, tmp_path, monkeypatch):
    """Back at Setup by hand, a box too old: Done goes to Tip offset, the work left, not round the steps."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    run, validate, apply = plugin.cal.run, plugin.steps[5], plugin.steps[6]
    plugin.cal.procedure.auto = True
    validate.start(ctx, "validation", validate._validate(ctx))
    run.accept("Validate")
    press_go_when_asked(ctx, plugin.cal.procedure)
    apply.start(ctx, "reload", apply._reload(ctx))
    assert states(plugin, ctx)[5:] == ["accepted", "passes", "none yet", "none yet"]
    run.ticks["Cameras calibrated"] -= PRECONDITION_HOLDS + 60
    plugin.panel.show(ctx, 0)
    assert plugin.steps[0].state(ctx).label == "expired"
    # * Expired: Done asks once more; then Tip offset, which has no pose yet: Done asks once more there too.
    assert [done(plugin, ctx) for _ in range(3)] == [0, 7, 7]


def test_the_relay_chip_is_neutral_until_this_run_applied_something(server, tmp_path, monkeypatch):
    """A fresh run: grey, saying what the relay applies; amber only once this run applied and the relay differs."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    live = {c.label: c for c in plugin.steps[6].checks(ctx)}["relay now"]
    assert live.level == PENDING and "applied nothing yet" in live.detail and "restarted" not in live.detail


def test_apply_refuses_while_the_relay_reads_another_drive(server, tmp_path, monkeypatch):
    """The relay would never find the approved calibration: Apply is red at once, not a timeout after it."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    apply, relay = plugin.steps[6], ctx.world.mocap_relay
    assert {c.label: c.level for c in apply.ready(ctx)}["relay's Drive"] == GOOD
    relay.state.drive_root = "/home/someone/Insync/2025-03 Husky Assembly"
    assert "the relay reads /home/someone/Insync" in apply.refusal(ctx)
    relay.state.drive_root = None
    assert "the relay reads no Drive" in apply.refusal(ctx)


def test_setup_apply_and_punch_need_the_relay_to_have_announced_its_calibrations(server, tmp_path, monkeypatch):
    """The relay interface always exists: only its announcement says the relay is up; until then Apply refuses."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    setup, apply, punch = plugin.steps[0], plugin.steps[6], plugin.steps[8]

    def up() -> list:
        return [{c.label: c.level for c in checks}[label] for checks, label in
                ((setup.checks(ctx), "relay up"), (apply.ready(ctx), "relay up"), (punch.ready(ctx), "relay applies"))]

    assert up() == [GOOD] * 3
    ctx.world.mocap_relay.state.calibrations_time = None
    assert up() == [BAD] * 3 and "relay up: the mocap relay announced no calibrations yet" in apply.refusal(ctx)


def test_a_solution_stays_current_while_its_takes_move_to_archive_and_back(server, tmp_path, monkeypatch):
    """Another tool body, tidied (as a definition change does), then set back: Solve and its verdict stay current."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    run = plugin.cal.run
    solved, body = run.current_solved(), run.tool_body
    plugin.cal.track_tool(ctx, body + 1)
    run.tidy()
    assert all(take.file.parent.name == "archive" for take in run.takes.values()) and run.current_solved() is None
    plugin.cal.track_tool(ctx, body)
    assert run.current_solved() is solved and states(plugin, ctx)[3:5] == ["passes", "passes"]
    run.tidy()
    assert run.current_solved() is solved and all(take.file.parent.name != "archive" for take in run.takes.values())


def test_back_and_next_never_write_refuse_or_start_work(server, tmp_path, monkeypatch):
    """Through every step and back: no run file, no collision check, nothing accepted; the panel goes everywhere."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    panel, run = plugin.panel, plugin.cal.run
    for index in range(1, 9):
        next_step(plugin, ctx)
        assert panel.index == index
    for _ in range(8):
        plugin.panel.navigate(ctx, forward=False)
    assert panel.index == 0 and run.progress is None and run.collisions == {} and run.acceptances == {}
    assert plugin.cal.procedure.outcome is None and not list((tmp_path / SESSIONS_FOLDER).glob("*/run_*.json"))


def test_the_run_file_starts_with_the_first_thing_recorded_and_navigation_never_saves(server, tmp_path, monkeypatch):
    """A tick starts it; moving between steps leaves the file as it is."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    run = plugin.cal.run
    run.notes = "kept"
    tick_setup(plugin, ["Area clear"])
    plugin.draw(ctx)
    file = run.progress.file
    assert Progress.load(file).notes == "kept" and plugin.steps[0]._run_choice.value != NEW_RUN
    run.notes = "not saved by moving"
    written = file.read_text()
    for _ in range(3):
        next_step(plugin, ctx)
    plugin.panel.navigate(ctx, forward=False)
    assert file.read_text() == written


def test_another_tool_body_puts_the_steps_it_made_out_of_date(server, tmp_path, monkeypatch):
    """Another tool body puts Record and Solve out of date, and Record has the work left; changing back restores."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    body_id = plugin.cal.run.tool_body
    plugin.cal.track_tool(ctx, body_id + 1)
    plugin.draw(ctx)
    assert states(plugin, ctx)[3:5] == ["out of date", "out of date"]
    assert ">out of date<" in plugin.panel._list_html.content
    assert plugin.cal.show_work_left(ctx) == "Record"
    plugin.cal.track_tool(ctx, body_id)
    plugin.cal.tool.hear()
    assert states(plugin, ctx)[3:5] == ["passes", "passes"] and plugin.cal.show_work_left(ctx) == "Validate"


def test_the_overview_draws_results_solid_and_preconditions_dashed(server, tmp_path, monkeypatch):
    """Setup, Marker error and Sweeps dashed; the current step outlines the expired preconditions it needs."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    rows = plugin.panel._list_html.content.split('<div style="display:flex')[1:]
    assert len(rows) == 9 and [("dashed" in row) for row in rows] == [True] * 3 + [False] * 6
    assert "dotted" not in plugin.panel._list_html.content
    ctx.wall += PRECONDITION_HOLDS + 60
    plugin.draw(ctx)
    rows = plugin.panel._list_html.content.split('<div style="display:flex')[1:]
    assert [("dotted" in row) for row in rows] == [True, True] + [False] * 7  # * Validate needs Setup, Marker error
    assert ">expired<" in rows[0] and "min ago" in rows[0]


def test_resuming_a_run_of_the_same_arm_keeps_the_tool_body_tracked(server, tmp_path, monkeypatch):
    """The same tool body stays tracked across runs: a body tracked afresh has no pose yet, which would open Setup."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    tool, file, setup = plugin.cal.tool, plugin.cal.run.progress.file, plugin.steps[0]
    setup.refresh_runs()
    setup._on_run_choice(ctx, next(label for label, progress in setup._runs.items() if progress.file == file))
    assert plugin.cal.tool is tool and plugin.panel.index == 5


def test_choosing_a_run_resumes_it_from_its_file_not_from_the_list(server, tmp_path, monkeypatch):
    """The Run list was read when the run started, with no takes: choosing it later still brings back every take."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    tick_setup(plugin)
    plugin.draw(ctx)
    setup, file = plugin.steps[0], plugin.cal.run.progress.file
    label = next(label for label, progress in setup._runs.items() if progress.file == file)
    assert setup._runs[label].takes == {} and label.endswith("in progress")
    walk_to_validate(plugin, ctx)
    setup._on_run_choice(ctx, label)
    assert len(plugin.cal.run.good_takes()) == 6


def test_a_resumed_run_comes_back_with_its_results_at_the_work_left(server, tmp_path, monkeypatch):
    """Choosing the run in Setup rebuilds it from its files and opens it at the step with work left.

    The checklist stays ticked: it is about the arm's physical setup, not the run. An expired precondition a step needs
    sends the run back to where it is met.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    file = plugin.cal.run.progress.file
    for _ in range(5):
        plugin.panel.navigate(ctx, forward=False)
    setup = plugin.steps[0]
    assert all(box.value for box in setup._checklist.boxes)
    setup._on_run_choice(ctx, NEW_RUN)
    assert plugin.cal.run.progress is None
    label = next(label for label, progress in setup._runs.items() if progress.file == file)
    setup._on_run_choice(ctx, label)
    plugin.draw(ctx)
    run = plugin.cal.run
    assert run.progress.file == file and plugin.panel.index == 5 and len(run.good_takes()) == 6
    assert run.current_solved() is not None and "at Validate" in plugin.cal.procedure.outcome.detail
    # * The checklist is about the arm, not the run: another run of the same arm keeps the ticks.
    assert all(box.value for box in setup._checklist.boxes)

    def resumed_at(**progress) -> int:
        """Resume the run with these run file fields, and say which step it opens at."""
        data = json.loads(file.read_text())
        file.write_text(json.dumps({**data, **progress}))
        setup.refresh_runs()
        setup._on_run_choice(ctx, next(label for label, p in setup._runs.items() if p.file == file))
        return plugin.panel.index

    # * Validate needs fresh marker errors: an hour later (the checklist ticked again), it opens at Marker error.
    ctx.wall += PRECONDITION_HOLDS + 60
    tick_setup(plugin)
    assert resumed_at() == 1
    # * Tool on flange ticked for another tool (a relaunch with the punch): back to Setup, that box unticked.
    assert resumed_at(marker_time=ctx.wall, tool_ticked_for="scaffolding_v3") == 0
    assert [box.label for box in setup._checklist.boxes if not box.value] == ["Tool on flange"]
    checklist = {c.label: c for c in setup.checks(ctx)}["checklist"]
    assert checklist.level == WARN and checklist.detail.startswith("Tool on flange ticked for scaffolding_v3")


def test_after_a_restart_an_expired_checklist_is_amber_and_one_never_ticked_red(server, tmp_path, monkeypatch):
    """The run file keeps the ticks: past PRECONDITION_HOLDS the boxes untick; Setup says when each was ticked."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    file, setup = plugin.cal.run.progress.file, plugin.steps[0]

    def restarted(**progress):
        """Resume the run with these run file fields, as a fresh monitor (no ticks to adopt) would."""
        data = json.loads(file.read_text())
        file.write_text(json.dumps({**data, **progress}))
        plugin.cal.run.ticks = {}
        setup.refresh_runs()
        setup._on_run_choice(ctx, next(label for label, p in setup._runs.items() if p.file == file))
        plugin.draw(ctx)
        return {c.label: c for c in setup.checks(ctx)}["checklist"]

    ctx.wall += PRECONDITION_HOLDS + 60
    expired = restarted()
    assert expired.level == WARN and "Area clear ticked" in expired.detail and "61 min ago" in expired.detail
    assert plugin.panel.index == 0 and not any(box.value for box in setup._checklist.boxes)
    assert "Area clear ticked" in setup.expired(ctx) and setup.state(ctx).label == "expired"
    assert restarted(ticks={}).level == BAD


def test_a_checklist_ticked_in_part_is_none_yet_and_done_asks_once_more(server, tmp_path, monkeypatch):
    """One box ticked is not expired but none yet, and Done asks once more; expired only once every box was ticked."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    setup = plugin.steps[0]
    tick_setup(plugin, CHECKLIST_BOXES[:1])
    plugin.draw(ctx)
    state = setup.state(ctx)
    assert state.label == "none yet" and state.level == PENDING and CHECKLIST_BOXES[1] in state.detail
    assert done(plugin, ctx) == 0 and plugin.panel._confirming(ctx)
    plugin.panel._press.forget()
    tick_setup(plugin)
    plugin.draw(ctx)
    assert setup.state(ctx).label == "holds"
    ctx.wall += PRECONDITION_HOLDS + 60
    plugin.draw(ctx)
    assert setup.state(ctx).label == "expired"


def answer_go(ctx, procedure, prompts: list[str]) -> None:
    """Make every wait the procedure starts keep its question and press Go, as the operator would when asked."""
    wait_until = ctx.wait_until

    async def go_then_wait(predicate, timeout_s=None, description=""):
        if procedure.prompt:
            prompts.append(procedure.prompt)
        procedure.go()
        return await wait_until(predicate, timeout_s, description)

    ctx.wait_until = go_then_wait


def test_an_expired_precondition_warns_and_asks_before_acting(server, tmp_path, monkeypatch):
    """Old marker errors: Record and Validate ask at Go, then act; without Go nothing is recorded.

    A moved base refuses Record (no collision check where it stands), and is amber in Sweeps itself.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    procedure, sweeps, record, validate = plugin.cal.procedure, *plugin.steps[2:4], plugin.steps[5]
    run = plugin.cal.run
    ctx.wall += PRECONDITION_HOLDS + 60
    tick_setup(plugin)
    assert {c.label: c.level for c in record.ready(ctx)}["Marker error"] == WARN
    take = run.pending()[0]
    record.start(ctx, "take", record._record_takes(ctx, take), moves=True)
    assert procedure.outcome.label == "take failed" and take not in run.takes  # * nobody pressed Go
    prompts = []
    answer_go(ctx, procedure, prompts)
    record.start(ctx, "take", record._record_takes(ctx, take), moves=True)
    assert prompts[0].startswith("Expired: Marker error") and take in run.current_takes()
    prompts.clear()
    validate.start(ctx, "validation", validate._validate(ctx), moves=True)
    assert prompts[0].startswith("Expired: Marker error") and len(run.validation().spots) == 1
    measure_markers(plugin, ctx)
    ctx.kinematics.base_pose = lambda _serial: Pose((1.1, 2.0, 0.0))
    record.start(ctx, "take", record._record_takes(ctx, run.pending()[0]), moves=True)
    assert procedure.outcome.label == "take not started"
    assert procedure.outcome.detail.startswith("Sweeps: expired: the base moved over 2 cm or 2°")
    assert [c.level for c in sweeps.ready(ctx)] == [WARN] and sweeps.state(ctx).label == "expired"
    assert "Sweeps" not in [c.label for c in validate.ready(ctx) if c.level == BAD]


def test_a_base_driven_while_go_is_waited_for_stops_the_take_before_its_sweep(server, tmp_path, monkeypatch):
    """The take started with the base where Sweeps checked it; driven 10 cm before Go: no sweep, nothing recorded."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    run, procedure, record = plugin.cal.run, plugin.cal.procedure, plugin.steps[3]
    take, wait_until = run.pending()[0], ctx.wait_until

    async def driven_then_go(predicate, timeout_s=None, description=""):
        ctx.kinematics.base_pose = lambda _serial: Pose((1.1, 2.0, 0.0))
        procedure.go()
        return await wait_until(predicate, timeout_s, description)

    ctx.wait_until = driven_then_go
    record.start(ctx, "take", record._record_takes(ctx, take), moves=True)
    assert procedure.outcome.label == "take failed" and "the base moved over 2 cm" in procedure.outcome.detail
    assert take not in run.takes and run.collision_base_moved(Pose((1.1, 2.0, 0.0))) == "100 mm, 0.0°"


def test_robot_moves_need_area_clear_and_e_stop_confirmed_within_the_hour(server, tmp_path, monkeypatch):
    """The D-pad and free drive refuse until they are ticked again; a procedure's Go confirms them, also on Auto.

    Other expired boxes only warn; non-moving actions never refuse on preconditions.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    run, procedure, marker, record, tip = plugin.cal.run, plugin.cal.procedure, plugin.steps[1], *plugin.steps[3::4]
    ctx.wall += PRECONDITION_HOLDS + 60
    measure_markers(plugin, ctx)
    tick_setup(plugin, ["Tool on flange", "Cameras calibrated"])
    assert "Area clear ticked" in marker.move_refusal(ctx) and "E-stop at hand ticked" in tip.move_refusal(ctx)
    assert {c.label: c.level for c in record.ready(ctx)}["area, e-stop"] == BAD
    prompts = []
    answer_go(ctx, procedure, prompts)
    procedure.auto = True
    record.start(ctx, "take", record._record_takes(ctx, run.pending()[0]), moves=True)
    # * Confirmed, the checklist holds again: nothing else to ask; Auto records every take left.
    assert len(prompts) == 1 and "Go confirms both" in prompts[0] and run.pending() == []
    assert run.safety_expired(ctx.wall) is None and marker.move_refusal(ctx) is None
    tick_setup(plugin, ["Cameras calibrated"], ticked=False)
    assert marker.move_refusal(ctx) is None and {c.label: c.level for c in marker.ready(ctx)}["Setup"] == WARN
    plugin.steps[4]._solve(ctx)
    assert procedure.outcome is None


def test_a_step_needs_only_earlier_precondition_steps():
    """Every name in a step's `needs` is an earlier precondition step that says when it expired (`expired`)."""
    names = [step.name for step in STEPS]
    for index, step in enumerate(STEPS):
        for name in step.needs:
            need = STEPS[names.index(name)]
            assert name in names[:index] and need.precondition and need.expired is not CalibrationStep.expired, \
                step.name


def tip_poses(urdf, joints: np.ndarray) -> list[np.ndarray]:
    """Arm poses that put TIP on one point: tilted about tool0's x and y both ways, 5° at a time up to 20°."""
    poses = [joints]
    for axis in "xy":
        for sign in (1.0, -1.0):
            here = joints
            for _ in range(4):
                turn = Pose(orientation=tuple(Rotation.from_euler(axis, sign * 5.0, degrees=True).as_quat()))
                here = tool_step_target(urdf, ARM, here, turn, TIP)
            poses.append(here)
    return poses


def press_go_when_asked(ctx, procedure) -> None:
    """Make every wait the procedure starts press Go first, as the operator would when asked."""
    wait_until = ctx.wait_until

    async def go_then_wait(predicate, timeout_s=None, description=""):
        procedure.go()
        return await wait_until(predicate, timeout_s, description)

    ctx.wait_until = go_then_wait


def labels(plugin: CalibrationPlugin, ctx) -> list[tuple[list[str], list[str]]]:
    """Each step's chip labels: its checks and its ready checks."""
    return [([check.label for check in step.checks(ctx)], [check.label for check in step.ready(ctx)])
            for step in plugin.steps]


def test_the_procedures_record_solve_validate_apply_tip_and_punch(server, tmp_path, monkeypatch):
    """On Auto, Record sweeps every take; Solve, a spot validates, Apply; the tip offset fits; Punch takes a take.

    Every step shows the same chips before anything ran and after all of it: they change colour, never appear.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    procedure, steps = plugin.cal.procedure, plugin.steps
    before = labels(plugin, ctx)
    tick_setup(plugin)
    run = plugin.cal.run
    measure_markers(plugin, ctx)
    check_collisions(plugin, ctx)
    plugin.panel.show(ctx, 3)
    plugin.draw(ctx)
    procedure.auto = True
    steps[3]._on_record(ctx)
    plugin.draw(ctx)
    assert procedure.outcome is None and len(run.good_takes()) == 12 and run.pending() == []
    # * Each take's samples stored their marker errors: 0.1 mm for both bodies here.
    assert "markers" in steps[3]._html.content and ">1 · 0.10 mm<" in steps[3]._html.content
    assert done(plugin, ctx) == 4
    steps[4]._solve(ctx)
    plugin.draw(ctx)
    solved = run.current_solved()
    metres, degrees = pose_change(solved.applied, solved.solution.footprint)
    # * As the manual's noise-free simulation of Belle: the solver's own error on her calibrated URDF.
    assert not fails(steps[4].checks(ctx)) and metres < 1e-3 and degrees < 0.05
    # * The change vs applied is against what the relay applies now; with none, there is nothing to compare. It and
    # * the axes are facts: blue.
    assert steps[4].checks(ctx)[1].value == f"{metres * 1e3:.1f} mm"
    assert [c.level for c in steps[4].checks(ctx)[1:4]] == [INFO] * 3
    calibrations = ctx.world.mocap_relay.state.calibrations
    applied = calibrations.pop(run.robot.config.mocap_id)
    assert steps[4].checks(ctx)[1].level == PENDING
    calibrations[run.robot.config.mocap_id] = applied
    assert done(plugin, ctx) == 5
    steps[5].start(ctx, "validation", steps[5]._validate(ctx), moves=True)
    plugin.draw(ctx)
    validation = run.validation()
    assert procedure.outcome is None and len(run.validation().spots) == 1
    assert len(validation.bases) == len(run.validation_poses) and validation.spread[0].max() < 1e-3
    assert set(steps[5].scene.targets_drawn[1]) == {(40, 160, 70)}  # * every target sampled, within 2 mm: green
    poses = len(run.validation_poses)
    assert f"spot 1 · {poses}" in steps[5]._html.content and "spot 2 · 0" in steps[5]._html.content
    assert ">markers</span>" in steps[5]._html.content and ">1 · 0.10 mm<" in steps[5]._html.content
    assert steps[5].checks(ctx)[0].value == f"{poses}/15"
    spot = server.scene._handle_from_node_name["/plugins/calibration/validation_spots/0"]
    assert np.allclose(spot.position, FOOTPRINT.position, atol=2e-3)  # * where the base stood for the spot
    assert done(plugin, ctx) == 5 and done(plugin, ctx) == 6  # * asked once more: one spot is not enough
    assert run.accepted("Validate") and "Validate" in run.progress.acceptances
    assert {c.label: (c.level, c.detail) for c in steps[6].ready(ctx)}["validation"] == (
        WARN, "the verdict fails, accepted in Validate")
    press_go_when_asked(ctx, procedure)
    steps[6].start(ctx, "reload", steps[6]._reload(ctx))
    plugin.draw(ctx)
    assert procedure.outcome is None and run.finished and not fails(steps[6].checks(ctx))
    tip, punch, arm = steps[7], steps[8], run.robot.arms[ARM]
    assert done(plugin, ctx) == 7
    # * Red until the monitor runs the arm bare, as with the punch on the flange; Setup's checklist holds here too.
    tip.start(ctx, "tip pose", tip._record(ctx))
    assert procedure.outcome.label == "tip pose not started" and "tool model" in procedure.outcome.detail
    robot = run.robot
    robot.config = replace(robot.config, arms=tuple(replace(a, end_effector=None) for a in robot.config.arms))
    # * Tool on flange was ticked for the marker: it does not hold for the bare arm until ticked again; it only warns.
    assert {c.label: c.level for c in tip.ready(ctx)}["Setup"] == WARN and tip.refusal(ctx) is None
    tick_setup(plugin, ["Tool on flange"])
    ready = {c.label: c.level for c in tip.ready(ctx)}
    assert ready == {"Setup": GOOD, "area, e-stop": GOOD, "tool model": GOOD, "tcp": GOOD}
    arm.joints = np.radians([90.0, -100.0, 90.0, -80.0, -90.0, 0.0])
    for joints in tip_poses(run.urdf, arm.joints):
        arm.joints = joints
        tip.start(ctx, "tip pose", tip._record(ctx))
    plugin.draw(ctx)
    assert procedure.outcome is None and not fails(tip.checks(ctx))
    assert {c.label: c.level for c in tip.checks(ctx)}["in use"] == WARN and not np.allclose(run.punch_offset, TIP)
    assert tip.state(ctx).label == "not in use"
    assert done(plugin, ctx) == 8  # * Done makes the fit Punch's TCP offset
    np.testing.assert_allclose(run.punch_offset, TIP, atol=1e-6)
    assert {c.label: c.level for c in tip.checks(ctx)}["in use"] == GOOD
    assert np.allclose(punch._offset.value, np.array(TIP) * 1e3, atol=1e-3)
    punch.start(ctx, "punch take", punch._record(ctx))
    plugin.draw(ctx)
    assert procedure.outcome is None and len(run.current_punch_takes()) == 1
    assert labels(plugin, ctx) == before
    # * Resumed, the run opens at Punch, the step with work left; Setup asks for the bare arm, not the marker.
    file, setup = run.progress.file, steps[0]
    setup.refresh_runs()
    setup._on_run_choice(ctx, next(label for label, progress in setup._runs.items() if progress.file == file))
    plugin.cal.tool = None
    plugin.draw(ctx)
    assert plugin.panel.index == 8 and not fails(setup.checks(ctx)) and len(plugin.cal.run.tip_joints) == 5
    levels = {c.label: c.level for c in setup.checks(ctx)}
    assert levels["tool mocap"] == PENDING and levels["tool model"] == GOOD
    # * The same arm: Setup's checklist keeps its ticks, which Punch relies on.
    assert [c.label for c in punch.ready(ctx) if c.level == BAD] == []
    # * An offset typed in Punch stays in use: Tip offset only warns, it leaves no work.
    plugin.cal.run.punch_offset = np.array(TIP) + 1e-3
    assert tip.state(ctx).label == "not in use" and plugin.cal.show_work_left(ctx) == "Punch"
    punch._target_moved()
    assert plugin.cal.run.current_punch_takes() == [] and plugin.cal.run.punch_target == 1


def test_apply_needs_the_validation_verdict_and_the_robot_standing_still(server, tmp_path, monkeypatch):
    """Apply refuses until the verdict passes or is accepted, and nothing is approved unless the robot stands still.

    Done pressed twice on Validate's failing verdict accepts it, until the solution changes.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    run, procedure, apply = plugin.cal.run, plugin.cal.procedure, plugin.steps[6]
    assert {c.label: c.level for c in apply.ready(ctx)}["validation"] == BAD
    apply.start(ctx, "reload", apply._reload(ctx))
    assert procedure.outcome.label == "reload not started" and "validation" in procedure.outcome.detail
    # * No samples: the second Done goes on, but a missing verdict is never accepted.
    assert done(plugin, ctx) == 5 and done(plugin, ctx) == 6 and "Validate" not in run.acceptances
    assert {c.label: c.detail for c in apply.ready(ctx)}["validation"].startswith("no verdict yet")
    plugin.panel.show(ctx, 5)
    procedure.auto = True
    plugin.steps[5].start(ctx, "validation", plugin.steps[5]._validate(ctx))
    procedure.auto = False
    assert plugin.steps[5].state(ctx).label == "fails"
    assert done(plugin, ctx) == 5 and done(plugin, ctx) == 6  # * the second Done accepts Validate's failing verdict
    assert {c.label: c.level for c in apply.ready(ctx)}["validation"] == WARN
    arm = ctx.world.robots[SERIAL].arms[ARM]
    arm.state.last_update_time, arm.state.is_executing = NOW, True
    apply.start(ctx, "reload", apply._reload(ctx))
    assert procedure.outcome.label == "reload failed" and "stands still" in procedure.outcome.detail
    assert run.current_solved().output.name.endswith("_in_progress.json") and not run.finished
    arm.state.is_executing = False
    apply.start(ctx, "reload", apply._reload(ctx))
    assert procedure.outcome is None and run.finished
    # * A new solution: the verdict accepted was for the one before.
    record_take(run, 0, 3)
    plugin.steps[4]._solve(ctx)
    assert {c.label: c.level for c in apply.ready(ctx)}["validation"] == BAD


def test_validate_samples_before_solve_and_judges_once_solved(server, tmp_path, monkeypatch):
    """Validate needs no solution: a spot sampled first is pending, and judged once Solve wrote one."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    tick_setup(plugin)
    run, procedure, validate = plugin.cal.run, plugin.cal.procedure, plugin.steps[5]
    measure_markers(plugin, ctx)
    plugin.panel.show(ctx, 5)
    procedure.auto = True
    assert "solution" not in [c.label for c in validate.ready(ctx)] and not fails(validate.ready(ctx))
    validate.start(ctx, "validation", validate._validate(ctx))
    assert procedure.outcome is None and {c.label: c.level for c in validate.checks(ctx)}["position"] == PENDING
    with monkeypatch.context() as patch:
        # * Even with enough spots, the verdict waits for a solution: Done asks once more.
        patch.setattr(validate, "checks", lambda _ctx: [])
        assert validate.state(ctx).label == "pending" and validate.state(ctx).level == PENDING
    check_collisions(plugin, ctx)
    for joint in (0, 1):
        for take in range(3):
            record_take(run, joint, take)
    plugin.steps[4]._solve(ctx)
    assert {c.label: c.level for c in validate.checks(ctx)}["position"] == GOOD
    assert {c.label: c.level for c in validate.checks(ctx)}["out of date"] == GOOD
    # * Sampled with another tool body: its chip counts them, and the line below the buttons says why the state is.
    run.validation_files = {path: replace(made, tool_body=made.tool_body + 1)
                            for path, made in run.validation_files.items()}
    old = {c.label: c for c in validate.checks(ctx)}["out of date"]
    assert (old.level, old.value) == (WARN, str(len(run.validation_files)))
    assert validate.state(ctx).label == "out of date"
    plugin.draw(ctx)
    notes = plugin.panel._notes_html.content
    assert "Validate out of date: sampled with another tool body or before a refine." in notes


def test_a_validation_ended_after_a_few_poses_keeps_their_samples(server, tmp_path, monkeypatch):
    """The arm fails at the third pose: the two sampled are kept, a file each, and count as one spot."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    tick_setup(plugin)
    run, procedure, validate, motion = plugin.cal.run, plugin.cal.procedure, plugin.steps[5], plugin.cal.motion
    measure_markers(plugin, ctx)
    plugin.panel.show(ctx, 5)
    procedure.auto = True
    sample, taken = motion.sample, []

    async def two_then_fail(*args):
        if len(taken) == 2:
            raise RuntimeError("the arm refused the move")
        taken.append(await sample(*args))
        return taken[-1]

    monkeypatch.setattr(motion, "sample", two_then_fail)
    validate.start(ctx, "validation", validate._validate(ctx))
    assert procedure.outcome.label == "validation failed"
    assert len(run.validation_files) == 2 and run.validation().spot_sizes == [2]


def test_tip_offsets_done_commits_the_fit_once_it_passes_or_is_accepted(server, tmp_path, monkeypatch):
    """A red tip fit is committed only by the second Done, which accepts it; the acceptance drops with the next pose.

    Shown again with poses, Tip offset asks whether the punch was remounted, below the controls: one Done answers no
    (committing the fit, stamping nothing); Punch remounted puts the poses out of date.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    run, tip, panel = plugin.cal.run, plugin.steps[7], plugin.panel
    poses = tip_poses(run.urdf, np.radians([90.0, -100.0, 90.0, -80.0, -90.0, 0.0]))
    for joints in poses[:3]:
        run.add_tip_pose(joints, FOOTPRINT)
    panel.show(ctx, 7)
    assert tip.state(ctx).label == "fails" and done(plugin, ctx) == 7 and not run.tip_offset_used()
    done(plugin, ctx)
    assert run.accepted("Tip offset") and run.tip_offset_used()
    run.add_tip_pose(poses[3], FOOTPRINT)
    assert not run.accepted("Tip offset") and {c.label: c.level for c in tip.checks(ctx)}["in use"] == WARN
    for joints in poses[4:]:
        run.add_tip_pose(joints, FOOTPRINT)
    panel.show(ctx, 7)
    plugin.draw(ctx)
    assert "Punch remounted since these 5 poses?" in panel._notes_html.content
    assert "Punch remounted since" not in tip._html.content and tip.state(ctx).label == "not in use"
    acceptances = dict(run.acceptances)
    assert done(plugin, ctx) != 7 and run.acceptances == acceptances and run.tip_offset_used()
    panel.show(ctx, 7)
    plugin.draw(ctx)
    assert "Punch remounted since these 5 poses?" in panel._notes_html.content
    tip._remounted()
    plugin.draw(ctx)
    assert run.current_tip() == ([], []) and tip.state(ctx).label == "out of date"
    assert "Punch remounted since" not in panel._notes_html.content


def test_punch_shows_the_base_marker_error_now_and_flags_takes_made_while_red(server, tmp_path, monkeypatch):
    """Punch has no marker precondition: a red base marker error shows live and flags the take, never refuses it.

    A take from an older punch file, without marker errors, shows its error as unknown.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    run, punch, procedure = plugin.cal.run, plugin.steps[8], plugin.cal.procedure
    robot = run.robot
    robot.config = replace(robot.config, arms=tuple(replace(a, end_effector=None) for a in robot.config.arms))
    tick_setup(plugin)
    run.punch_offset = np.array(TIP)
    plugin.panel.show(ctx, 8)
    robot.base.state.marker_error = 2.5e-3
    plugin.draw(ctx)
    # * The number in a handle of its own, so the chips' hover holds while it changes.
    assert punch.live_markers().level == BAD and ">base markers now<" in punch._html.content
    assert "2.50" not in punch.live_markers().detail and "now: 2.50 mm" in on_phone(punch._live_html).content
    assert "Marker error" not in punch.needs and punch.refusal(ctx) is None
    punch.start(ctx, "punch take", punch._record(ctx))
    assert procedure.outcome is None and run.punch_takes[0]["marker_errors"] == {"base": 2.5e-3}
    assert "1 taken with the base marker error over 2 mm" in punch.checks(ctx)[0].detail
    older = {key: value for key, value in run.punch_takes[0].items() if key != "marker_errors"}
    run.add_punch_take(older, "single")
    robot.base.state.marker_error = 1e-4
    plugin.draw(ctx)
    assert ">take 1 · 2.50 mm<" in punch._html.content and ">take 2 · ?<" in punch._html.content
    assert punch.live_markers().level == GOOD and "1 taken with" in punch.checks(ctx)[0].detail


def test_punch_takes_go_out_of_date_once_the_lab_frame_moves(server, tmp_path, monkeypatch):
    """Each take stores the lab frame; a refit used moves it, and the takes go out of date until it is back.

    The frame chip shows in Setup and Punch: amber while a refit is pending, red while not trusted, which refuses Punch.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    run, setup, punch = plugin.cal.run, plugin.steps[0], plugin.steps[8]
    robot = run.robot
    robot.config = replace(robot.config, arms=tuple(replace(a, end_effector=None) for a in robot.config.arms))
    tick_setup(plugin)
    run.punch_offset = np.array(TIP)
    plugin.panel.show(ctx, 8)
    punch.start(ctx, "punch take", punch._record(ctx))
    assert run.punch_takes[0]["lab_from_raw"] == [list(LAB[0]), list(LAB[1])] and len(run.current_punch_takes()) == 1
    state = ctx.world.mocap_relay.state
    # * No frame announced: grey in Setup, red in Punch, as a take would store none and stay current once it moves.
    announced_frame, state.frame = state.frame, None
    assert [{c.label: c.level for c in checks}["frame"] for checks in (setup.checks(ctx), punch.ready(ctx))] == \
        [PENDING, BAD]
    assert "frame: the relay announced no floor calibration yet" in punch.refusal(ctx)
    state.frame = announced_frame

    def frame(status: str, shift: float = 0.0) -> None:
        state.frame = FloorCalibrationState(status, FLOOR, pending="5.0 mm" if status != FRAME_HELD else "",
                                            lab_from_raw=((0.1 + shift, -0.2, 0.0), (0.0, 0.0, 0.0, 1.0)))
        state.anchors_missing = ("A3",) if status == FRAME_UNTRUSTED else ()

    def chips() -> list:
        return [{c.label: c.level for c in checks}["frame"] for checks in (setup.checks(ctx), punch.ready(ctx))]

    frame(FRAME_REFIT_PENDING)
    assert chips() == [WARN, WARN] and len(run.current_punch_takes()) == 1
    frame(FRAME_UNTRUSTED)
    assert chips() == [BAD, BAD] and "frame: not trusted" in punch.refusal(ctx) and "anchors missing: A3" in \
        punch.refusal(ctx)
    frame(FRAME_HELD, shift=0.0005)
    assert chips() == [GOOD, GOOD] and len(run.current_punch_takes()) == 1  # * within VOLUME_MOVED
    frame(FRAME_HELD, shift=0.005)  # * a refit used
    assert run.current_punch_takes() == [] and punch.state(ctx).label == "out of date"
    assert "lab frame move" in punch.state(ctx).detail
    frame(FRAME_HELD)
    assert len(run.current_punch_takes()) == 1


def test_validate_shows_where_the_arm_will_sample(server, tmp_path, monkeypatch):
    """Each validation pose's tool0 drawn at the robot, grey until sampled; hidden once Validate is left."""
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    walk_to_validate(plugin, ctx)
    run, validate = plugin.cal.run, plugin.steps[5]
    root = "/plugins/calibration/validation_targets"
    handles = server.scene._handle_from_node_name
    frame = handles[root]
    assert frame.visible and np.allclose(frame.position, FOOTPRINT.position)
    for index, pose in enumerate(run.validation_poses):
        expected = link_pose(run.urdf, Pose(), named(ARM, pose), f"{ARM}_tool0")
        assert np.allclose(handles[f"{root}/{index}"].position, expected.position)
    assert validate.scene.targets_drawn[1] == ((134, 142, 150),) * len(run.validation_poses)
    plugin.panel.navigate(ctx, forward=False)
    plugin.draw(ctx)
    assert not handles[root].visible


# --- --- --- --- --- the phone tab --- --- --- --- ---

def on_phone(both):
    """The phone page's handle of a widget on both pages."""
    return both.handles[1]


def in_folder(server, folder) -> dict[str, object]:
    """The phone widgets right inside `folder`, by label."""
    handles = server.phone.gui._gui_input_handle_from_uuid.values()
    return {handle.label: handle for handle in handles if handle._impl.parent_container_id == folder._impl.uuid}


def press(handle, value=None) -> None:
    """An operator's click (or, for an input, a change to `value`) on a phone widget: its callbacks run."""
    if value is not None:
        handle._impl.value = value
    callbacks = handle._impl.update_cb or [cb for cbs in handle._impl.hold_cbs_from_freq.values() for cb in cbs]
    for callback in callbacks:
        asyncio.run(callback(SimpleNamespace(client=object(), target=handle)))


def test_the_phone_tab_follows_the_laptop_and_moves_the_guide(server, tmp_path, monkeypatch):
    """The same chips, overview and step folder as the laptop; the phone's Done, Back and Next move the guide.

    A tick on the phone ticks the laptop.
    """
    plugin, ctx, _ = make_plugin(server, tmp_path, monkeypatch)
    panel, setup = plugin.panel, plugin.steps[0]
    assert on_phone(panel._status_html).content == f'<div class="phone-text">{panel._status_html.content}</div>'
    assert [on_phone(folder).visible for folder in panel._folders] == [True] + [False] * 8
    overview = panel._phone_list_html.content
    assert overview.count("<span title=") == 9 and overview.count("dashed") == 3 and "box-shadow" in overview
    press(on_phone(panel._done_button))
    assert panel.index == 0 and "Done asks again: Setup none yet" in ctx.log[-1]
    boxes = in_folder(server, on_phone(panel._folders[0]))
    for box in setup._checklist.boxes:
        press(boxes[box.label], True)
    assert all(box.value for box in setup._checklist.boxes) and set(plugin.cal.run.ticks) == set(CHECKLIST_BOXES)
    plugin.draw(ctx)
    assert on_phone(panel._done_button).color == panel._done_button.color == rgb(OK)
    press(on_phone(panel._done_button))
    assert panel.index == 1 and [on_phone(folder).visible for folder in panel._folders] == [False, True] + [False] * 7
    press(on_phone(panel._next_button))
    press(on_phone(panel._back_button))
    press(on_phone(panel._back_button))
    assert panel.index == 0


def test_the_phone_has_cancel_and_the_controls_used_at_the_robot_but_no_go(server, tmp_path, monkeypatch):
    """Go, Auto and Preview stay on the laptop; laptop-only steps say so; Tip offset frees the arm while held."""
    plugin, _, _ = make_plugin(server, tmp_path, monkeypatch)
    panel = plugin.panel
    labels = {handle.label for handle in server.phone.gui._gui_input_handle_from_uuid.values()}
    assert "Cancel" in labels and labels.isdisjoint({"Go", "Auto", "Preview"})
    assert "Check collisions" in in_folder(server, on_phone(panel._folders[2]))
    tip = in_folder(server, on_phone(panel._folders[7]))
    assert {"Free drive (hold)", "Keep free drive on", "Step size", "Record tip pose"} <= set(tip)
    assert {"Record take"}.isdisjoint(in_folder(server, on_phone(panel._folders[3])))
    held = []
    monkeypatch.setattr(plugin.cal.hand, "hold_free_drive", held.append)
    press(tip["Free drive (hold)"])
    assert held == [NOW]


def test_the_plugin_follows_motives_definitions_and_a_refine_ends_by_itself(server, tmp_path, monkeypatch):
    """Each new announcement reaches the run; refining the base ends once its definition changed, its takes archived."""
    plugin, ctx, _ghost = make_plugin(server, tmp_path, monkeypatch)
    run, state = plugin.cal.run, ctx.world.mocap_relay.state
    first = {run.robot.config.mocap_id: body(MARKERS * 1.6, run.robot.config.mocap_id, "belle")}
    state.body_definitions = first
    plugin.update(ctx)
    assert run.definitions is first
    record_take(run, 0, 0)
    marker = plugin.steps[1]
    marker._start_refine("base")
    plugin.update(ctx)
    assert run.refining == "base"
    state.body_definitions = {run.robot.config.mocap_id: body(moved(MARKERS * 1.6, noise=0.3e-3),
                                                              run.robot.config.mocap_id, "belle")}
    plugin.update(ctx)
    assert run.refining is None and run.current_takes() == {}
    assert all(take.file.parent.name == "archive" for take in run.takes.values())


def test_a_refine_in_motive_after_measuring_turns_the_drive_chip_amber_and_says_why(server, tmp_path, monkeypatch):
    """Measured, then the base body changed in Motive without Refine here: expired, and the step's chip says why."""
    plugin, ctx, _ghost = make_plugin(server, tmp_path, monkeypatch)
    run, state = plugin.cal.run, ctx.world.mocap_relay.state
    mocap_id = run.robot.config.mocap_id
    state.body_definitions = {mocap_id: body(MARKERS * 1.6, mocap_id, "belle")}
    plugin.update(ctx)
    measure_markers(plugin, ctx)
    run.marker_made = run.made  # * measured with the definitions of now, as driving keeps it
    marker = plugin.steps[1]
    assert marker.expired(ctx) is None
    state.body_definitions = {mocap_id: body(moved(MARKERS * 1.6, noise=0.3e-3), mocap_id, "belle")}
    plugin.update(ctx)
    why = marker.expired(ctx)
    drive = marker.checks(ctx)[0]
    assert why is not None and "since measured" in why and (drive.level, why in drive.detail) == (WARN, True)
    assert marker.state(ctx).label == "expired"


def test_recheck_ends_a_refine_without_definitions(server, tmp_path, monkeypatch):
    """A simulation announces no definitions: Recheck ends the refine and measures again."""
    plugin, ctx, _ghost = make_plugin(server, tmp_path, monkeypatch)
    run, marker = plugin.cal.run, plugin.steps[1]
    measure_markers(plugin, ctx)
    marker._start_refine("tool")
    marker._recheck()
    assert run.refining is None and run.marker_max is None


def test_files_move_to_archive_only_once_no_procedure_runs(server, tmp_path, monkeypatch):
    """A refine during a take: the run takes the new definition at once, its files move once the procedure ended."""
    plugin, ctx, _ghost = make_plugin(server, tmp_path, monkeypatch)
    run, state, procedure = plugin.cal.run, ctx.world.mocap_relay.state, plugin.cal.procedure
    base_id = run.robot.config.mocap_id
    state.body_definitions = {base_id: body(MARKERS * 1.6, base_id, "belle")}
    plugin.update(ctx)
    record_take(run, 0, 0)
    procedure._task = SimpleNamespace(done=lambda: False)
    state.body_definitions = {base_id: body(moved(MARKERS * 1.6, noise=0.3e-3), base_id, "belle")}
    plugin.update(ctx)
    assert run.current_takes() == {} and all(take.file.parent.name != "archive" for take in run.takes.values())
    procedure._task = None
    plugin.update(ctx)
    assert all(take.file.parent.name == "archive" for take in run.takes.values())


def test_while_a_procedure_runs_other_actions_are_refused_on_the_status_line(server, tmp_path, monkeypatch):
    """Another arm, run or tool body, Solve, another procedure: a chip says what runs; the widgets keep what holds."""
    plugin, ctx, _ghost = make_plugin(server, tmp_path, monkeypatch)
    run, procedure, setup = plugin.cal.run, plugin.cal.procedure, plugin.steps[0]
    body = run.tool_body
    procedure._task, procedure.doing = SimpleNamespace(done=lambda: False), "take 1: waypoint 2 of 12"
    for act, refused in ((lambda: setup.choose(ctx, f"{SERIAL} {ARM}"), "arm not changed"),
                         (lambda: setup._on_run_choice(ctx, NEW_RUN), "run not changed"),
                         (lambda: setup._on_tool_body(ctx, body + 1), "tool body not changed"),
                         (lambda: plugin.steps[4]._solve(ctx), "solve not started"),
                         (lambda: check_collisions(plugin, ctx), "check not started")):
        procedure.outcome = None
        act()
        assert (procedure.outcome.label, procedure.outcome.level) == (refused, WARN)
        assert procedure.outcome.detail == "take 1: waypoint 2 of 12 runs: Cancel it first"
    assert plugin.cal.run is run and run.tool_body == body and setup._tool_id.value == body and run.solved is None


def test_without_a_relay_nothing_is_taken_again_on_each_tick(server, tmp_path, monkeypatch):
    """No relay: the same empty definitions every tick, so the run file is not written again and again."""
    plugin, ctx, _ghost = make_plugin(server, tmp_path, monkeypatch)
    ctx.world.mocap_relay = None
    saves = []
    plugin.cal.run.save = lambda *args, **kwargs: saves.append(True)
    for _ in range(5):
        plugin.update(ctx)
    assert len(saves) == 2  # * the first tick takes the definitions, then tidies once


def test_the_ghost_shows_only_while_calibration_was_used_last_of_the_plugins_with_ghosts(server, tmp_path, monkeypatch):
    """Another plugin's ghost used later hides calibration's; a new preview or Preview time shows it again."""
    plugin, ctx, ghost = make_plugin(server, tmp_path, monkeypatch)
    procedure = plugin.cal.procedure
    procedure.preview_path = TimedPath(np.array([0.0, 1.0]), np.zeros((2, 6)))
    plugin.draw(ctx)
    assert ghost.joints is not None
    RecentUse(0.0).touch()  # * e.g. the arm planner panel's Plan pressed
    plugin.draw(ctx)
    assert ghost.joints is None
    procedure.preview = 0.5
    plugin.draw(ctx)
    assert ghost.joints is not None
