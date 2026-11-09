"""The observer: a session's chunks and events on this PC, copies to a mock Drive, and the events it tells."""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.logging import LoggingSeverity
from robot_lab import RobotLab

from husky_assembly_teleop.plugin_api.console_log import ConsoleLog
from husky_assembly_teleop.plugins.assembly_observer import plugin as observer_module
from husky_assembly_teleop.plugins.assembly_observer.plugin import RECORDING_FOLDER, AssemblyObserverPlugin
from husky_assembly_teleop.plugins.assembly_observer.watch import Watcher
from husky_assembly_teleop.robot_interface.arm import SentTrajectory, WrenchReading
from husky_assembly_teleop.robot_interface.base import SentPath
from husky_assembly_teleop.robot_interface.mocap_relay import FrameDrift, frame_drift
from husky_assembly_teleop.robot_interface.robot_lease import take_lease
from husky_assembly_teleop.shared.recordings.runs import load_description, load_recording
from scaffolding_assembly_core.geometry import Pose

CINDY = "a200-0806"


@pytest.fixture(autouse=True)
def local_root(tmp_path, monkeypatch):
    """Recordings go to a scratch folder, never ~/.ros."""
    monkeypatch.setattr(observer_module, "LOCAL_ROOT", tmp_path / "local")
    return tmp_path / "local"


def lab_on_drive(tmp_path) -> RobotLab:
    """The three robots, a mock Drive with the observer's copies folder, and a console log."""
    (tmp_path / RECORDING_FOLDER).mkdir(parents=True)
    lab = RobotLab()
    lab.ctx.config.drive_root = tmp_path
    lab.ctx.console_log = ConsoleLog()
    return lab


def run(lab: RobotLab, plugin: AssemblyObserverPlugin, seconds: float, during=lambda t: None) -> None:
    """Tick the lab and the plugin for `seconds`, calling `during(t)` before each tick."""
    async def ticking():
        until = lab.ctx.t + seconds
        while lab.ctx.t < until:
            during(lab.ctx.t)
            await lab.ctx.next_tick()
            plugin.update(lab.ctx)
    asyncio.run(ticking())


def button(lab: RobotLab, label: str):
    """The panel's button of that label."""
    return next(widget for widget in lab.ctx.gui.of_kind("button") if widget.label == label)


def clicked(lab: RobotLab, plugin: AssemblyObserverPlugin, label: str = "Copy to Drive") -> None:
    """Click Copy to Drive (or Open in Rerun) and wait for the helper script's own process to end."""
    async def click() -> None:
        button(lab, label).callback()
        task = plugin._copying if label == "Copy to Drive" else plugin._opening
        if task is not None:
            await task
    asyncio.run(click())


def events_of(folder) -> list[dict]:
    """A session's events, in order."""
    return [json.loads(line) for line in (folder / "events.jsonl").read_text().splitlines()]


def test_a_session_saves_every_robot_each_tick_and_the_log_lines(tmp_path):
    """Every robot's columns each tick in the chunk, compact; the session's events and its log lines from INFO up."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    lab.ctx.console_log.add(LoggingSeverity.INFO, "assembly_guide", "guide: B20_J insert done")
    lab.ctx.console_log.add(LoggingSeverity.DEBUG, "robots", "a debug line, not kept")
    run(lab, plugin, 2.0)
    plugin.teardown(lab.ctx)

    folder = plugin._folder
    description = load_description(folder)
    assert description["kind"] == "observer session" and set(description["robots"]) == set(lab.robots)
    assert description["robots"][CINDY]["arms"]["left_ur_arm"] == "ScaffoldingV3"
    recording = load_recording(folder / "chunk_001.npz")
    ticks = len(recording["t"])
    assert 19 <= ticks <= 21  # 2 s at the fakes' 10 Hz
    assert recording["0806_left_ur_arm_joints"].shape == (ticks, 6)
    assert recording["0806_left_ur_arm_joints"].dtype == np.float32
    assert recording["t"].dtype == np.float64
    assert list(recording["0806_streams.labels"])[:3] == ["base_rate", "base_max_gap", "base_delay"]
    events = events_of(folder)
    names = [event["event"] for event in events]
    assert names[0] == "session start" and names[-1] == "session end"
    logs = [event for event in events if event["event"] == "log"]
    assert [log["text"] for log in logs] == ["guide: B20_J insert done"]
    assert logs[0]["source"] == "assembly_guide" and "T" in logs[0]["time"]


def test_chunks_follow_each_other_without_a_gap(tmp_path, monkeypatch):
    """Each new chunk starts on the tick the last one ended: no tick is lost between files."""
    monkeypatch.setattr(observer_module, "CHUNK_SECONDS", 1.0)
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    run(lab, plugin, 3.05)
    plugin.teardown(lab.ctx)

    chunks = sorted(plugin._folder.glob("chunk_*.npz"))
    assert len(chunks) >= 3
    times = np.concatenate([load_recording(chunk)["t"] for chunk in chunks])
    assert np.all(np.diff(times) > 0) and np.max(np.diff(times)) < 0.15


def test_it_records_without_a_drive_and_refuses_a_copy_there(tmp_path, local_root):
    """No Drive folder: it records on this PC all the same; Copy to Drive says why it can't, in the panel and log."""
    lab, plugin = RobotLab(), AssemblyObserverPlugin()
    lab.ctx.config.drive_root = tmp_path / "no drive"
    plugin.setup(lab.ctx)
    run(lab, plugin, 0.3)
    button(lab, "Copy to Drive").callback()
    plugin.draw(lab.ctx)
    plugin.teardown(lab.ctx)

    assert plugin._folder.is_relative_to(local_root) and (plugin._folder / "events.jsonl").is_file()
    assert "not copied" in plugin._copied.content and "HUSKY_DRIVE_ROOT" in plugin._copied.content
    assert any("copy to the Drive refused" in line for line in lab.ctx.lines("warn"))


def test_days_older_than_a_week_are_deleted_at_start(tmp_path, local_root):
    """Day folders past KEPT_DAYS go at the next start; younger ones and anything not named as a day stay."""
    from datetime import date, timedelta

    old = (date.today() - timedelta(days=7)).strftime("%Y%m%d")
    young = (date.today() - timedelta(days=6)).strftime("%Y%m%d")
    for name in (old, young, "notes"):
        (local_root / name).mkdir(parents=True)
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    plugin.teardown(lab.ctx)

    assert not (local_root / old).exists() and (local_root / young).is_dir() and (local_root / "notes").is_dir()
    assert f"recordings of {old} deleted: kept 7 days" in lab.ctx.lines("info")


def test_copy_to_drive_takes_only_the_range_chosen(tmp_path):
    """From/To picks the ticks and events in that range; the session's description says what was copied."""
    import time as clock

    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    run(lab, plugin, 2.0)
    plugin._writer.submit(lambda: None).result()
    # * The fakes' ROS time is not the wall clock: the session's description says how they line up.
    description = json.loads((plugin._folder / "experiment.json").read_text())
    offset = description["wall_time"] - description["ros_time"]
    lab.ctx.console_log.add(LoggingSeverity.INFO, "assembly_guide", "guide: B20_J insert done")
    run(lab, plugin, 0.2)
    marked = lab.ctx.t
    run(lab, plugin, 2.0)
    start, end = marked - 0.5 + offset, marked + 0.5 + offset
    plugin._day.value = clock.strftime("%Y%m%d", clock.localtime(start))
    plugin._from.value = clock.strftime("%H:%M:%S", clock.localtime(start))
    plugin._to.value = clock.strftime("%H:%M:%S", clock.localtime(end + 1))
    plugin._name.value = "B20 insert"
    clicked(lab, plugin)
    plugin.teardown(lab.ctx)

    copies = list((tmp_path / RECORDING_FOLDER / "B20_insert").iterdir())
    assert len(copies) == 1, plugin._copy_line
    session, = list(copies[0].iterdir())
    copied = load_description(session)["copied"]
    assert copied["name"] == "B20_insert" and copied["to"] > copied["from"]
    texts = [event.get("text") for event in events_of(session)]
    assert "guide: B20_J insert done" in texts
    recording = load_recording(next(session.glob("chunk_*.npz")))
    low, high = copied["from"] - offset, copied["to"] - offset  # * From and To as typed, to the second
    assert len(recording["t"]) and ((recording["t"] >= low) & (recording["t"] <= high)).all()
    assert recording["0806_left_ur_arm_joints"].shape == (len(recording["t"]), 6)
    copied_events = events_of(session)
    state = next(event for event in copied_events if event["event"] == "state" and event["robot"] == CINDY)
    assert state["ros_time"] < low and state["before_range"]  # * where things stood at From, marked as before it
    assert not any(event.get("before_range") for event in copied_events if event["ros_time"] >= low)
    assert state["status"]["left_ur_arm controller"] == "scaled_joint_trajectory_controller"
    assert any(line.startswith("copied 1 session(s)") for line in lab.ctx.lines("info"))


def test_the_range_buttons_fill_from_and_to():
    """Last 15 min: from a quarter hour ago to now, today; Whole day: the full day chosen."""
    import time as clock

    lab, plugin = RobotLab(), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    plugin._pick_range("Last 15 min")
    assert plugin._to.value == "now" and plugin._day.value == clock.strftime("%Y%m%d")
    plugin._pick_range("Whole day")
    assert (plugin._from.value, plugin._to.value) == ("00:00", "23:59:59")
    plugin.teardown(lab.ctx)


# --- --- --- --- --- EVENTS --- --- --- --- ---

def watched(lab: RobotLab) -> Watcher:
    """A watcher on the lab's robots, listening, after a first look."""
    watcher = Watcher(lab.robots, None, lab.ctx.now)
    watcher.attach()
    watcher.look()[0]
    return watcher


def test_an_arm_moving_with_no_command_and_no_lease_is_told_as_from_outside(tmp_path):
    """A move nothing in the monitor sent, with no lease held, is told as coming from outside."""
    lab = RobotLab()
    watcher = watched(lab)
    arm = lab.cindy.arms["left_ur_arm"]
    arm.state.last_slight_motion_time = lab.ctx.t
    moving = [event for event in watcher.look()[0] if event["event"] == "arm moving"]
    lab.ctx.t += 1.0
    still = [event for event in watcher.look()[0] if event["event"] == "arm still"]

    assert moving[0]["robot"] == CINDY and moving[0]["by"] is None and not moving[0]["commanded"]
    assert "free drive" in moving[0]["cause"]
    assert still[0]["seconds"] == pytest.approx(1.0)


def test_a_commanded_motion_names_the_lease_holder_and_the_trajectory(tmp_path):
    """The trajectory, then the motion naming who held the lease; the lease taken and given back."""
    lab = RobotLab()
    watcher = watched(lab)
    lease = take_lease([lab.cindy.lease_slot], "robot_control: jog left", owner="robot_control")
    arm = lab.cindy.arms["left_ur_arm"]
    arm.trajectories_sent.fire(SentTrajectory(np.zeros(6), 2.0, 3, 0, lab.ctx.t, lab.ctx.t))
    arm.state.last_slight_motion_time = lab.ctx.t
    events = watcher.look()[0]
    lease.release()
    released = watcher.look()[0]

    names = [event["event"] for event in events]
    assert names.index("trajectory sent") < names.index("arm moving")
    moving = next(event for event in events if event["event"] == "arm moving")
    assert moving["by"] == "robot_control: jog left" and moving["commanded"]
    assert next(e for e in events if e["event"] == "lease taken")["holder"] == "robot_control: jog left"
    assert [e["event"] for e in released] == ["lease released"]


def test_a_base_pushed_by_hand_moves_and_stops(tmp_path):
    """A base moved with no command: one start and one stop, with the distance it went."""
    lab = RobotLab()
    lab.track(CINDY)
    watcher = watched(lab)
    events = []
    for step in range(20):
        lab.bases[CINDY] = Pose((3.0 + 0.01 * min(step, 10), 1.0, 0.0))
        lab.ctx.t += 0.1
        lab.tick(lab.ctx.t)
        events += watcher.look()[0]

    moving = next(event for event in events if event["event"] == "base moving")
    still = next(event for event in events if event["event"] == "base still")
    assert moving["robot"] == CINDY and not moving["commanded"]
    assert still["distance"] == pytest.approx(0.1, abs=0.02)


def test_a_tool_command_is_told_with_its_values(tmp_path):
    """A tool command is told with what was sent."""
    lab = RobotLab()
    watcher = watched(lab)
    lease = take_lease([lab.cindy.lease_slot], "movement_exec: B20_J", owner="movement_exec")
    with lease.use():
        lab.tools["left_ur_arm"].close()
    lease.release()

    sent = [event for event in watcher.look()[0] if event["event"] == "tool command sent"]
    assert sent == [{"event": "tool command sent", "ros_time": lab.ctx.t, "robot": CINDY, "arm": "left_ur_arm",
                     "by": "movement_exec: B20_J", "command": "tighten gripper", "motor": 1.0, "direction": 1.0}]


def test_estop_soft_stop_and_a_lost_stream_are_told_once():
    """An e-stop, a soft stop and an arm's joint_states going quiet are each told once."""
    lab = RobotLab()
    watcher = watched(lab)
    lab.cindy.base.state.estopped = True
    lab.cindy.soft_stop_latch.stopped, lab.cindy.soft_stop_latch.reason = True, "Stop all"
    lab.alice.arms["ur_arm"].state.last_update_time = lab.ctx.t - 2.0
    first = watcher.look()[0]
    again = [event["event"] for event in watcher.look()[0]]

    told = {event["event"]: event for event in first}
    assert told["e-stop"]["value"] is True and told["soft stop"]["value"] == "Stop all"
    assert told["ur_arm stream"] == {"event": "ur_arm stream", "ros_time": lab.ctx.t, "robot": "a200-0804",
                                     "value": "lost", "since": lab.ctx.t, "flickers": 0}
    assert not {"e-stop", "soft stop", "ur_arm stream"} & set(again)


def test_the_relays_drift_is_told_when_it_grows():
    """The relay's drift numbers are read from its metrics, and told again only when they change by a step."""
    sample = {"status": "held", "decides": "anchors", "off": 0.0004, "off_limit": 0.001,
              "anchors": {"moves_volume": 0.0021, "used": ["A1", "A2", "A3", "A4"], "residual": 0.0003},
              "truss_cameras": {"moves_volume": 0.0025}, "apart": 0.0006,
              "anchor_scale": {"ppm": -40.0}, "scale_changed": False}
    drift = frame_drift(sample)
    assert drift == FrameDrift("held", "anchors", 0.0004, 0.001, 0.0021, 0.0025, 0.0006, -40.0, False, 4, 0.0003)

    relay = SimpleNamespace(state=SimpleNamespace(frame=None, calibrations={}, drift=drift))
    lab = RobotLab()
    watcher = Watcher({}, relay, lab.ctx.now)
    first = watcher.look()[0]
    relay.state.drift = frame_drift({**sample, "off": 0.0005})  # within the step: not told
    same = watcher.look()[0]
    relay.state.drift = frame_drift({**sample, "off": 0.0012, "status": "refit pending"})
    grown = watcher.look()[0]

    assert [event["event"] for event in first] == ["relay deciding fit", "relay scale changed", "relay anchors used",
                                                   "relay drift"]
    assert same == []
    assert [event["event"] for event in grown] == ["relay drift"] and grown[0]["value"]["off"] == 0.0012


def test_full_commands_go_to_files_named_by_their_events(tmp_path):
    """A trajectory and a path sent: each event names a file holding the message exactly as sent."""
    from builtin_interfaces.msg import Duration
    from crl_husky_msgs.msg import BasePath
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    trajectory = JointTrajectory(joint_names=[f"j{i}" for i in range(6)], points=[
        JointTrajectoryPoint(positions=[0.1 * i] * 6, time_from_start=Duration(sec=i)) for i in (1, 2)])
    lab.cindy.arms["left_ur_arm"].trajectories_sent.fire(
        SentTrajectory(np.full(6, 0.2), 2.0, 2, 0, lab.ctx.t, lab.ctx.t, trajectory))
    path = BasePath(id=7, x=[0.0, 1.0], y=[0.0, 0.5], yaw=[0.0, 0.3])
    lab.cindy.base.paths_sent.fire(SentPath(7, 2, 0, np.zeros(2), lab.ctx.t, lab.ctx.t, path))
    run(lab, plugin, 0.2)
    plugin.teardown(lab.ctx)

    events = {event["event"]: event for event in events_of(plugin._folder)}
    sent = json.loads((plugin._folder / events["trajectory sent"]["file"]).read_text())
    assert events["trajectory sent"]["file"] == "commands/00001_trajectory_sent_a200-0806_left_ur_arm.json"
    assert [point["positions"] for point in sent["points"]] == [[0.1] * 6, [0.2] * 6]
    assert sent["points"][1]["time_from_start"]["sec"] == 2
    assert json.loads((plugin._folder / events["path sent"]["file"]).read_text())["x"] == [0.0, 1.0]
    assert "_message" not in events["trajectory sent"]


def test_peaks_keep_the_largest_reading_between_ticks_and_zeroing_is_told(tmp_path):
    """Three readings in one tick: the peak column keeps the largest; a zero is told with the reading it tares."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    arm = lab.cindy.arms["left_ur_arm"]
    for force in (3.0, 40.0, 5.0):
        arm.wrench_messages.fire(WrenchReading(np.array([0.0, 0.0, force, 0.0, 0.0, 0.5]), lab.ctx.t, lab.ctx.t))
    arm.state.wrench = np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.5])
    arm.zero_ft_sensor()
    run(lab, plugin, 0.3)
    plugin.teardown(lab.ctx)

    recording = load_recording(plugin._folder / "chunk_001.npz")
    peaks = recording["0806_left_ur_arm_wrench_peak"]
    assert peaks[0].tolist() == [40.0, 0.5] and np.isnan(peaks[1]).all()
    zeroed = next(event for event in events_of(plugin._folder) if event["event"] == "force sensor zeroed")
    assert zeroed["arm"] == "left_ur_arm" and zeroed["wrench_before"] == [0.0, 0.0, 5.0, 0.0, 0.0, 0.5]


def test_a_new_tracked_object_starts_a_chunk_with_its_columns(tmp_path):
    """Tracking a bar mid-session: a new chunk carries its pose and noise; its tracking switches are events."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    run(lab, plugin, 0.3)
    bar = lab.ctx.track_object("B20", 21)
    bar.position, bar.orientation = np.array([1.0, 2.0, 0.5]), np.array([0.0, 0.0, 0.0, 1.0])
    bar.tracked, bar.marker_error, bar.last_update_time = True, 0.0004, lab.ctx.t
    run(lab, plugin, 0.3)
    bar.tracked = False
    run(lab, plugin, 0.7)
    bar.tracked = True
    run(lab, plugin, 0.4)
    plugin.teardown(lab.ctx)

    assert "object_B20_pose" not in load_recording(plugin._folder / "chunk_001.npz")
    second = load_recording(plugin._folder / "chunk_002.npz")
    assert second["object_B20_pose"][0].tolist() == pytest.approx([1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0])
    assert second["object_B20_mocap"][0, 0] == pytest.approx(0.0004)
    names = [(event["event"], event.get("value")) for event in events_of(plugin._folder)
             if event["event"].startswith("object ")]
    assert names == [("object tracked from now on", None), ("object mocap tracking", "lost"),
                     ("object mocap tracking", "back")]


def test_switching_states_are_events_not_columns(tmp_path):
    """A controller switch and the follower's state are told as events; no column holds a flag or a mode."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    run(lab, plugin, 0.2)
    lab.cindy.arms["left_ur_arm"].state.controllers.active = "cartesian_compliance_controller"
    run(lab, plugin, 0.2)
    plugin.teardown(lab.ctx)

    switch = next(event for event in events_of(plugin._folder) if event["event"] == "left_ur_arm controller")
    assert switch["value"] == "cartesian_compliance_controller" and switch["robot"] == CINDY
    keys = load_recording(plugin._folder / "chunk_001.npz").keys()
    assert not [key for key in keys if "status" in key or key.endswith("_state")]


def test_a_short_tracking_flicker_is_counted_not_told():
    """Two short losses tell nothing; a long one is told with them counted, and its return once it held."""
    lab = RobotLab()
    lab.track(CINDY)
    watcher = watched(lab)
    state, events = lab.cindy.base.state, []
    for tracked in "FTFT" + "F" * 7 + "TF" + "TTTT":
        lab.ctx.t += 0.1
        state.tracked, state.last_update_time = tracked == "T", lab.ctx.t
        events += [event for event in watcher.look()[0] if event["event"] == "mocap tracking"]
    assert [(event["value"], event["flickers"]) for event in events] == [("lost", 2), ("back", 1)]
    assert events[1]["lost_for"] == pytest.approx(0.9)  # from the 5th sample to the 14th


def test_a_controller_switch_is_one_event_and_commands_name_their_sender():
    """A switch is told once, from and to; a trajectory names who held the lease when it went out."""
    lab = RobotLab()
    watcher = watched(lab)
    arm = lab.cindy.arms["left_ur_arm"]
    arm.state.controllers.switch_in_flight = "cartesian_compliance_controller"
    asking = [event["event"] for event in watcher.look()[0]]
    arm.state.controllers.switch_in_flight, arm.state.controllers.active = None, "cartesian_compliance_controller"
    lease = take_lease([lab.cindy.lease_slot], "robot_control: jog", owner="robot_control")
    arm.trajectories_sent.fire(SentTrajectory(np.zeros(6), 1.0, 2, 0, lab.ctx.t, lab.ctx.t))
    lease.release()
    events = watcher.look()[0]

    assert asking == []
    switches = [event for event in events if event["event"] == "left_ur_arm controller"]
    assert [(event["was"], event["value"]) for event in switches] == [
        ("scaled_joint_trajectory_controller", "cartesian_compliance_controller")]
    assert next(event for event in events if event["event"] == "trajectory sent")["by"] == "robot_control: jog"


def test_a_clearance_trip_keeps_what_came_too_close():
    """The trip event carries every pair with its contact."""
    from husky_assembly_teleop.world.clearance import ClearanceTrip
    from husky_assembly_teleop.world.collision_hit import CollisionHit, Contact
    from scaffolding_assembly_core.scene import Scene

    lab = RobotLab()
    watcher = watched(lab)
    hit = CollisionHit(Scene(), "robots/a200-0806", "exec B20_J", (("tools/a200-0806/left_ur_arm", "obstacles/rack"),),
                       (Contact(("tools/a200-0806/left_ur_arm", "obstacles/rack"), (1.0, 0.5, 0.8), (1.0, 0.5, 0.81),
                                -0.003),), margin=0.005)
    trip = ClearanceTrip(lab.ctx.t, CINDY, "movement_exec: exec B20_J", "came within 5 mm of rack", hit)
    event = next(event for event in watcher.look([trip])[0] if event["event"] == "clearance trip")
    assert "details" not in event and event["problem"] == "came within 5 mm of rack"
    assert event["hit"]["pairs"][0]["contact"]["depth"] == -0.003


def test_stale_commands_and_targets_are_unknown(tmp_path):
    """A twist past its timeout and a Cartesian target with the compliance controller off read as unknown."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    arm = lab.cindy.arms["left_ur_arm"]
    arm.state.sent_target_position, arm.state.sent_target_orientation = np.ones(3), np.array([0.0, 0.0, 0.0, 1.0])
    plugin._watcher.twists[CINDY] = (0.2, 0.1, lab.ctx.t + 0.05)
    run(lab, plugin, 0.5)
    plugin.teardown(lab.ctx)

    recording = load_recording(plugin._folder / "chunk_001.npz")
    twist = recording["0806_twist_sent"]
    assert twist[0].tolist() == pytest.approx([0.2, 0.1]) and np.isnan(twist[-1]).all()
    assert np.isnan(recording["0806_left_ur_arm_target"]).all()


def test_a_copy_leaves_damaged_lines_and_chunks_out_and_says_so(tmp_path):
    """A line cut by a crash and a truncated chunk are skipped and counted; the rest is copied."""
    from husky_assembly_teleop.plugins.assembly_observer.export import copy_range

    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    run(lab, plugin, 0.5)
    plugin.teardown(lab.ctx)
    folder = plugin._folder
    with open(folder / "events.jsonl", "a") as events:
        events.write('{"event": "log", "ros_time": 1\n')  # * cut, then a line after it: damaged
        events.write(json.dumps({"event": "log", "ros_time": load_description(folder)["ros_time"] + 0.5}) + "\n")
        events.write('{"event": "log", "ros_t')  # * the last line, perhaps being written: not counted
    (folder / "chunk_009.npz").write_bytes(b"PK\x03\x04 cut short")
    description = load_description(folder)
    wall = description["wall_time"]
    destination = tmp_path / "copy"
    destination.mkdir()
    copied = copy_range(observer_module.LOCAL_ROOT, wall - 10, wall + 10, destination, {"name": "test"})

    counts, = copied.values()
    assert counts["skipped"] == 2 and counts["events"] > 0 and counts["rows"] > 0


def test_teardown_saves_the_last_chunk_even_when_the_watcher_fails(tmp_path):
    """A watcher that raises at the end still leaves the chunk saved and the writer stopped."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    run(lab, plugin, 0.3)
    plugin._watcher.look = lambda trips=(): (_ for _ in ()).throw(RuntimeError("broken"))
    plugin.teardown(lab.ctx)
    assert (plugin._folder / "chunk_001.npz").is_file()


def test_a_repeating_log_line_is_written_with_its_final_count(tmp_path):
    """Five repeats between two ticks are one line with its count; one more repeat later is written at the end."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    for _ in range(5):
        lab.ctx.console_log.add(LoggingSeverity.WARN, "robot_control", "jog refused: not leased")
    run(lab, plugin, 0.1)
    lab.ctx.console_log.add(LoggingSeverity.WARN, "robot_control", "jog refused: not leased")
    run(lab, plugin, 0.1)
    lab.ctx.console_log.add(LoggingSeverity.INFO, "robot_control", "jog ended")
    run(lab, plugin, 0.1)
    plugin.teardown(lab.ctx)
    logs = [(event["text"], event["count"]) for event in events_of(plugin._folder) if event["event"] == "log"
            and event["source"] == "robot_control"]
    # * The console folds the five into one line by then: written once, with its count.
    assert logs == [("jog refused: not leased", 5), ("jog refused: not leased", 6), ("jog ended", 1)]


def test_log_lines_pushed_out_before_the_observer_read_them_are_told_lost(tmp_path):
    """More lines between two ticks than the console keeps: a "log lines lost" event, then the lines still kept."""
    from husky_assembly_teleop.plugin_api.console_log import KEPT

    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    run(lab, plugin, 0.1)
    for index in range(KEPT + 20):
        lab.ctx.console_log.add(LoggingSeverity.INFO, "robot_control", f"line {index}")
    run(lab, plugin, 0.1)
    for index in range(3):
        lab.ctx.console_log.add(LoggingSeverity.INFO, "robot_control", f"after {index}")
    run(lab, plugin, 0.1)
    plugin.teardown(lab.ctx)
    events = events_of(plugin._folder)
    lost = [event for event in events if event["event"] == "log lines lost"]
    assert len(lost) == 1 and lost[0]["at_most"] >= 20
    texts = [event["text"] for event in events if event["event"] == "log" and event["source"] == "robot_control"]
    assert texts[-3:] == ["after 0", "after 1", "after 2"] and f"line {KEPT + 19}" in texts


def test_a_one_point_jog_step_gets_no_file_of_its_own():
    """A tool0 jog sends a one-point trajectory every tick: its event holds it all, no commands/ file."""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    lab = RobotLab()
    watcher = watched(lab)
    step = JointTrajectory(points=[JointTrajectoryPoint(positions=[0.0] * 6)])
    lab.cindy.arms["left_ur_arm"].trajectories_sent.fire(SentTrajectory(np.zeros(6), 0.1, 1, 0, lab.ctx.t, lab.ctx.t,
                                                                        step))
    sent, = [event for event in watcher.look()[0] if event["event"] == "trajectory sent"]
    assert "_message" not in sent and sent["target"] == [0.0] * 6


def test_the_same_line_after_a_skipped_one_is_written_again(tmp_path):
    """A click, a health chip line (not kept), the same click: both clicks are in the events, and the end counts."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    for source, text in (("assembly_steps", "operator: Exec"), ("health/chips", "0806 mocap: lost"),
                         ("assembly_steps", "operator: Exec")):
        lab.ctx.console_log.add(LoggingSeverity.INFO, source, text)
        run(lab, plugin, 0.1)
    lab.ctx.console_log.add(LoggingSeverity.INFO, "assembly_steps", "operator: Exec")
    plugin.teardown(lab.ctx)
    texts = [(event["text"], event["count"]) for event in events_of(plugin._folder) if event["event"] == "log"
             and event["source"] == "assembly_steps"]
    assert texts == [("operator: Exec", 1), ("operator: Exec", 1), ("operator: Exec", 2)]


def test_a_copy_knows_a_change_between_the_last_snapshot_and_from(tmp_path):
    """A lease taken after the chunk's snapshot but before From is in the copy, so its start is right."""
    from husky_assembly_teleop.plugins.assembly_observer.export import copy_range

    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    run(lab, plugin, 0.5)
    lease = take_lease([lab.cindy.lease_slot], "robot_control: drive", owner="robot_control")
    run(lab, plugin, 0.5)
    after_lease = lab.ctx.t
    run(lab, plugin, 1.0)
    lease.release()
    plugin.teardown(lab.ctx)
    description = load_description(plugin._folder)
    offset = description["wall_time"] - description["ros_time"]
    destination = tmp_path / "copy"
    destination.mkdir()
    copy_range(observer_module.LOCAL_ROOT, after_lease + 0.3 + offset, after_lease + 0.8 + offset, destination, {})

    session, = destination.iterdir()
    names = [(event["event"], event.get("holder")) for event in events_of(session)]
    assert ("lease taken", "robot_control: drive") in names
    assert ("lease released", "robot_control: drive") not in names  # * after To


def test_a_snapshot_that_fails_is_an_event_not_a_failed_plugin(tmp_path):
    """A robot whose state can't be read: the others' states are written, and one observer error."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    lab.alice.lease_slot = None  # * reading its lease raises
    plugin.setup(lab.ctx)
    plugin.teardown(lab.ctx)
    events = events_of(plugin._folder)
    assert {event["robot"] for event in events if event["event"] == "state"} == {"a200-0805", CINDY}
    assert any(event["event"] == "observer error" for event in events)


def test_pruning_never_deletes_the_running_sessions_day(tmp_path, local_root):
    """A monitor running past a week keeps writing: its own day stays."""
    from datetime import date, timedelta

    from husky_assembly_teleop.plugins.assembly_observer.export import prune

    old = (date.today() - timedelta(days=9)).strftime("%Y%m%d")
    (local_root / old).mkdir(parents=True)
    (local_root / "20200101").mkdir()
    assert prune(local_root, date.today(), keep=old) == ["20200101"] and (local_root / old).is_dir()


def test_a_fresh_run_of_the_same_line_within_one_tick_is_its_own(tmp_path):
    """A line repeated, a skipped chip line, the same text again past the first run's count: both runs are written."""
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    for _ in range(5):
        lab.ctx.console_log.add(LoggingSeverity.WARN, "robot_control", "jog refused")
    run(lab, plugin, 0.1)
    lab.ctx.console_log.add(LoggingSeverity.INFO, "health/chips", "0806 lease: free")
    for _ in range(6):
        lab.ctx.console_log.add(LoggingSeverity.WARN, "robot_control", "jog refused")
    run(lab, plugin, 0.1)
    plugin.teardown(lab.ctx)
    counts = [event["count"] for event in events_of(plugin._folder) if event.get("text") == "jog refused"]
    assert counts == [5, 6]


def test_the_helper_script_lists_prints_and_copies(tmp_path):
    """observer_logs.py: list names the session, events prints the range's lines filtered, copy writes the folder."""
    import subprocess
    import sys
    import time as clock

    from repo_paths import REPO

    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    lab.ctx.console_log.add(LoggingSeverity.INFO, "assembly_guide", "guide: B20_J insert done")
    run(lab, plugin, 1.0)
    plugin.teardown(lab.ctx)
    script, root = str(REPO / "scripts" / "observer_logs.py"), str(observer_module.LOCAL_ROOT)
    day, start = clock.strftime("%Y%m%d"), clock.strftime("%H:%M", clock.localtime(clock.time() - 120))

    def call(*args: str) -> str:
        return subprocess.run([sys.executable, script, "--root", root, *args], capture_output=True, text=True,
                              check=True).stdout

    assert plugin._folder.name in call("list")
    printed = call("events", start, "now", "--day", day, "--grep", "insert done")
    assert printed.count("guide: B20_J insert done") == 1 and "log  [assembly_guide]" in printed
    outcome = json.loads(call("copy", start, "now", "--day", day, "--name", "script test",
                              "--to", str(tmp_path / "out")).splitlines()[-1])
    assert outcome["destination"].startswith(str(tmp_path / "out" / "script_test"))
    assert list(outcome["copied"]) == [plugin._folder.name]


def test_open_in_rerun_runs_the_helper_script_on_the_range(tmp_path, monkeypatch, local_root):
    """Open in Rerun runs `observer_logs.py rerun` on From to To in its own process; the line says what it shows."""
    script = tmp_path / "stand_in.py"
    script.write_text("import json, sys\nopen(sys.argv[0] + '.args', 'w').write(json.dumps(sys.argv[1:]))\n"
                      "print(json.dumps({'sessions': ['s'], 'rows': 7, 'events': 3, 'notes': [], 'saved': None}))\n")
    monkeypatch.setattr(observer_module, "SCRIPT", script)
    monkeypatch.setattr(observer_module, "find_spec", lambda name: object())
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    plugin._from.value = "00:00"
    clicked(lab, plugin, "Open in Rerun")
    plugin.draw(lab.ctx)
    plugin.teardown(lab.ctx)
    args = json.loads(script.with_name("stand_in.py.args").read_text())
    assert args == ["--root", str(local_root), "rerun", "00:00", "now", "--day", plugin._day.value]
    assert "opened 1 session(s), 3 events, 7 ticks in Rerun" in plugin._copied.content


def test_a_viewer_left_running_by_the_helper_script_does_not_hold_the_panel(tmp_path, monkeypatch, local_root):
    """The script ends but leaves a child (the viewer `rr.spawn` started) holding its output: the line shows at once."""
    import os
    import signal
    import time as clock

    script = tmp_path / "spawns.py"
    script.write_text("import json, subprocess, sys\nchild = subprocess.Popen(['sleep', '30'])\n"
                      "open(sys.argv[0] + '.pid', 'w').write(str(child.pid))\n"
                      "print(json.dumps({'sessions': ['s'], 'rows': 7, 'events': 3, 'notes': [], 'saved': None}))\n")
    monkeypatch.setattr(observer_module, "SCRIPT", script)
    monkeypatch.setattr(observer_module, "find_spec", lambda name: object())
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    plugin._from.value = "00:00"
    started = clock.monotonic()
    try:
        clicked(lab, plugin, "Open in Rerun")
        took = clock.monotonic() - started
        plugin.draw(lab.ctx)
        plugin.teardown(lab.ctx)
    finally:
        os.kill(int(script.with_name("spawns.py.pid").read_text()), signal.SIGKILL)
    assert took < 10.0 and "opened 1 session(s), 3 events, 7 ticks in Rerun" in plugin._copied.content


def test_open_in_rerun_without_rerun_installed_says_how_to_install_it(tmp_path, monkeypatch):
    """Without the rerun package nothing runs: the line below the buttons says how to install it."""
    monkeypatch.setattr(observer_module, "find_spec", lambda name: None)
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    plugin._from.value = "00:00"
    clicked(lab, plugin, "Open in Rerun")
    plugin.draw(lab.ctx)
    plugin.teardown(lab.ctx)
    assert plugin._opening is None and "pip install rerun-sdk==0.39.0" in plugin._copied.content


def _hanging_script(tmp_path):
    """A stand-in for the helper script that writes its pid, then never ends."""
    script = tmp_path / "hang.py"
    script.write_text("import os, sys, time\nopen(sys.argv[2] + '.pid', 'w').write(str(os.getpid()))\n"
                      "time.sleep(600)\n")
    return script


def _gone(pid: int) -> bool:
    """Whether no process has this pid (any more)."""
    import os
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return True
    return False


def test_a_hanging_copy_is_stopped_after_its_time_and_says_so(tmp_path, monkeypatch, local_root):
    """Past COPY_TIMEOUT the copy's process is killed and the panel says why; the next copy may run."""
    monkeypatch.setattr(observer_module, "SCRIPT", _hanging_script(tmp_path))
    monkeypatch.setattr(observer_module, "COPY_TIMEOUT", 1.0)
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    plugin._to.value = "now"
    plugin._from.value = "00:00"
    clicked(lab, plugin)
    pid = int((local_root.parent / "local.pid").read_text())
    plugin.draw(lab.ctx)
    plugin.teardown(lab.ctx)
    assert _gone(pid) and "no end after" in plugin._copied.content
    assert plugin._copying.done()


def test_cancelling_a_copy_stops_its_process(tmp_path, monkeypatch, local_root):
    """The plugin stopping cancels the copy's task: its process is stopped, the monitor never waits on it."""
    import time as clock

    monkeypatch.setattr(observer_module, "SCRIPT", _hanging_script(tmp_path))
    lab, plugin = lab_on_drive(tmp_path), AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    plugin._from.value = "00:00"

    async def click_then_cancel() -> None:
        button(lab, "Copy to Drive").callback()
        pid_file = local_root.parent / "local.pid"
        while not pid_file.is_file():
            await lab.ctx.sleep(0.1)
        plugin._copying.cancel()
        await asyncio.gather(plugin._copying, return_exceptions=True)
        return int(pid_file.read_text())

    started = clock.monotonic()
    pid = asyncio.run(click_then_cancel())
    plugin.teardown(lab.ctx)
    assert _gone(pid) and clock.monotonic() - started < 30
