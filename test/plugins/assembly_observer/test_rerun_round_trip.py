"""The Rerun viewer on a session the real observer recorded: every name it special-cases still reaches it."""

from __future__ import annotations

import asyncio
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.logging import LoggingSeverity
from robot_lab import RobotLab

from husky_assembly_teleop.plugin_api.console_log import KEPT, ConsoleLog
from husky_assembly_teleop.plugins.assembly_observer import plugin as observer_module
from husky_assembly_teleop.plugins.assembly_observer import rerun_view
from husky_assembly_teleop.plugins.assembly_observer.export import copy_range
from husky_assembly_teleop.plugins.assembly_observer.names import FORMAT
from husky_assembly_teleop.plugins.assembly_observer.plugin import AssemblyObserverPlugin
from husky_assembly_teleop.robot_interface.arm import SentTrajectory
from husky_assembly_teleop.robot_interface.base import SentPath
from husky_assembly_teleop.robot_interface.robot_lease import take_lease
from husky_assembly_teleop.shared.recordings.runs import load_description, load_recording
from husky_assembly_teleop.shared.runs.runs import record_event
from scaffolding_assembly_core.geometry import Pose

from repo_paths import DATA

pytest.importorskip("rerun")

CINDY = "a200-0806"
#: Format 1's names the viewer special-cases, as a recording holds them. ! A rename fails here: it needs a FORMAT
#: bump and the old name in `rerun_view.UPGRADES`, since recordings with the old name exist.
SIGNALS = {"0806_base_pose", "0806_base_mocap", "0806_follower", "0806_streams", "0806_twist_sent", "0806_odometry",
           "0806_battery", "0806_left_ur_arm_joints", "0806_left_ur_arm_tcp", "0806_left_ur_arm_target",
           "0806_left_ur_arm_wrench", "0806_left_ur_arm_wrench_peak", "0806_left_ur_arm_tool", "object_B20_pose",
           "object_B20_mocap", "monitor_late_share", "relay_drift"}
EVENTS = {"session start", "session end", "state", "relay state", "objects state", "lease taken", "lease released",
          "soft stop", "arm moving", "base moving", "base still", "path sent", "path stop sent", "trajectory sent",
          "hold sent", "clearance trip", "relay frame status", "log", "log lines lost"}


@pytest.fixture
def recorded(tmp_path, monkeypatch):
    """A session the observer recorded of the fake lab, with a tracked object, a relay and a process with a run.

    Cindy is tracked and pushed 10 cm; her lease is taken, her left arm sent a trajectory, moves and gets a hold, a
    path and its stop go to her base, a clearance trip and a soft stop hit her; the relay's frame status changes; the
    console has a guide line, the process plugin's "loaded" line and a flood it cannot keep. Two chunks.
    """
    from builtin_interfaces.msg import Duration
    from crl_husky_msgs.msg import BasePath
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    from husky_assembly_teleop.world.clearance import ClearanceTrip
    from husky_assembly_teleop.world.collision_hit import CollisionHit, Contact
    from scaffolding_assembly_core.scene import Scene

    root = tmp_path / "local"
    monkeypatch.setattr(observer_module, "LOCAL_ROOT", root)
    monkeypatch.setattr(observer_module, "CHUNK_SECONDS", 1.0)
    lab = RobotLab()
    lab.ctx.console_log = ConsoleLog()
    lab.track(CINDY)
    bar = lab.ctx.track_object("B20", 21)
    bar.position, bar.orientation = np.array([1.0, 2.0, 0.5]), np.array([0.0, 0.0, 0.0, 1.0])
    bar.tracked, bar.marker_error, bar.last_update_time = True, 0.0004, lab.ctx.t
    frame = SimpleNamespace(status="held", floor_calibration=None, refit=None, newer=None, scale=None)
    lab.ctx.world.mocap_relay = SimpleNamespace(state=SimpleNamespace(frame=frame, calibrations={}, drift=None,
                                                                      simulated=False))
    process = tmp_path / "process"
    plugin = AssemblyObserverPlugin()
    plugin.setup(lab.ctx)
    arm, base = lab.cindy.arms["left_ur_arm"], lab.cindy.base
    lab.ctx.console_log.add(LoggingSeverity.INFO, "process", f"loaded {process} in 1.0 s; 1 actions")
    (process / "runs" / "20261109_100000").mkdir(parents=True)
    record_event(process / "runs" / "20261109_100000", "exec", lab.ctx.t + 0.5, action="B20_J",
                 movement="B20_J_transfer", data={"status": "SUCCEEDED", "result": "succeeded"})
    lease = take_lease([lab.cindy.lease_slot], "robot_control: test", owner="robot_control")
    trajectory = JointTrajectory(joint_names=["shoulder_pan_joint"], points=[
        JointTrajectoryPoint(positions=[0.1 * i], time_from_start=Duration(sec=i)) for i in (1, 2)])

    def during(t: float) -> None:
        step = round((t - start) * 10)
        if step == 1:
            arm.trajectories_sent.fire(SentTrajectory(np.zeros(6), 2.0, 2, 0, t, t, trajectory))
            arm.state.last_slight_motion_time = t
            path = BasePath(id=7, x=[0.0, 1.0], y=[0.0, 0.5], yaw=[0.0, 0.0])
            base.paths_sent.fire(SentPath(7, 2, 0, np.zeros(2), t, t, path))
            lab.ctx.console_log.add(LoggingSeverity.WARN, "guide", "slow plan")
        if 2 <= step <= 6:
            lab.bases[CINDY] = Pose((3.0 + 0.02 * (step - 1), 1.0, 0.0))
        if step == 4:
            arm.trajectories_sent.fire(SentTrajectory(np.zeros(6), 0.2, 1, 1, t, t))
            base.paths_sent.fire(SentPath(7, 0, 0, np.zeros(2), t, t))
            pair = ("tools/a200-0806/left_ur_arm", "obstacles/rack")
            contact = Contact(pair, (1.0, 0.5, 0.8), (1.0, 0.5, 0.81), -0.003)
            hit = CollisionHit(Scene(), "robots/a200-0806", "test", (pair,), (contact,), margin=0.005)
            lab.ctx.clearance_trips = [ClearanceTrip(t, CINDY, "test", "came within 5 mm of rack", hit)]
            lab.cindy.soft_stop_latch.stopped, lab.cindy.soft_stop_latch.reason = True, "Stop all"
            frame.status = "refit pending"
        if step == 5:
            for index in range(KEPT + 20):
                lab.ctx.console_log.add(LoggingSeverity.INFO, "robot_control", f"line {index}")
        if step == 8:
            lease.release()

    async def ticking():
        while lab.ctx.t < start + 1.5:
            during(lab.ctx.t)
            await lab.ctx.next_tick()
            plugin.update(lab.ctx)
    start = lab.ctx.t
    asyncio.run(ticking())
    plugin.teardown(lab.ctx)
    return plugin._folder


def test_the_viewer_reads_every_name_of_a_recorded_session(recorded, tmp_path, monkeypatch):
    """Every name the viewer special-cases takes its own path there, none the plain fallback; all are format 1's.

    Base and object poses, arm joints, tool0's error, the follower, streams, mocap quality and every known event.
    """
    from rerun.chunk import RrdReader

    description = load_description(recorded)
    keys = {key for chunk in recorded.glob("chunk_*.npz") for key in load_recording(chunk)}
    events = [json.loads(line) for line in (recorded / "events.jsonl").read_text().splitlines()]
    assert description["format"] == FORMAT == 1
    assert SIGNALS <= keys, "a signal renamed: bump FORMAT, add the old name to rerun_view.UPGRADES"
    assert EVENTS <= {event["event"] for event in events}, "an event renamed: bump FORMAT, add it to UPGRADES"

    lines, every, plain, plots = {}, [], [], {}
    text_line, fields, plotted = rerun_view._text_line, rerun_view._fields, rerun_view._plots

    def spied_line(event):
        line = text_line(event)
        if line is not None:
            lines.setdefault(event["event"], line)
            every.append(line)
        return line

    def spied_plots(*args):
        found = plotted(*args)
        plots.update({path: names for path, names, _ in found})
        return found
    monkeypatch.setattr(rerun_view, "_text_line", spied_line)
    monkeypatch.setattr(rerun_view, "_fields", lambda name, event: plain.append(name) or fields(name, event))
    monkeypatch.setattr(rerun_view, "_plots", spied_plots)
    saved = tmp_path / "round_trip.rrd"
    wall = description["wall_time"]
    shown = rerun_view.log_range(recorded.parent.parent, wall - 10.0, time.time() + 10.0, save=saved, data=DATA)

    assert shown["sessions"] == [recorded.name] and not [note for note in shown["notes"] if "process" in note]
    assert plain == []
    text = {name: line[2] for name, line in lines.items()}
    assert text["lease taken"] == "0806 lease → robot_control: test"
    assert text["arm moving"].startswith("0806 left_ur_arm moving by robot_control: test")
    assert lines["soft stop"] == ("events/0806", "WARN", "0806 soft stop: Stop all")
    assert text["path sent"] == "0806 base path, 2 points by robot_control: test"
    assert text["path stop sent"] == "0806 base path stop by robot_control: test"
    assert text["trajectory sent"] == "0806 left_ur_arm trajectory, 2 points, 2.0 s by robot_control: test"
    assert text["hold sent"].startswith("0806 left_ur_arm hold, 0.2 s")
    assert text["clearance trip"] == "0806 clearance trip: came within 5 mm of rack"
    assert text["base still"].startswith("0806 base still after") and text["base moving"].startswith("0806 base moving")
    assert text["relay frame status"] == "relay frame status → refit pending"
    assert lines["log lines lost"][1] == "WARN" and text["log lines lost"].startswith("up to ")
    assert ("log/guide", "WARN", "[guide] slow plan") in every
    assert any(path == "log/process" and "loaded" in line for path, _, line in every)
    assert text["run exec"] == "B20_J_transfer succeeded"
    assert {"session start", "session end"} <= set(text)
    assert plots["0806/base_mocap"][0] == "marker_error [mm]" and "objects/B20/mocap" in plots
    assert plots["0806/follower/errors"] == ["cross_track [m]", "along_track [m]", "yaw_error [rad]"]
    assert plots["0806/streams/rate"][:2] == ["base [1/s]", "mocap [1/s]"]

    entities = {chunk.entity_path for chunk in RrdReader(saved).stream()}
    assert {"/world/0806", "/world/objects/B20", "/0806/base_pose/yaw", "/objects/B20/pose/yaw",
            "/model_0806/joints/left_ur_arm_shoulder_pan_joint", "/0806/left_ur_arm/target_error",
            "/world/0806/left_ur_arm/tcp", "/world/plans/0806", "/world/trips/0806",
            "/ghost_0806/joints/left_ur_arm_shoulder_pan_joint", "/states/0806/lease", "/states/0806/soft_stop",
            "/states/0806/left_ur_arm_motion", "/states/0806/base_motion", "/states/lab/relay_frame_status",
            "/states/run/step", "/states/lab/environment"} <= entities
    translations = [chunk for chunk in RrdReader(saved).stream() if chunk.entity_path == "/world/0806"
                    and "Transform3D:translation" in chunk.to_record_batch().schema.names]
    assert translations, "the base pose moves Cindy's frame"


def test_a_copy_keeps_the_format_and_an_old_or_newer_one_still_shows(recorded, tmp_path):
    """The Drive copy keeps "format"; without one a recording is format 1, a newer one is warned about and shown."""
    description = load_description(recorded)
    destination = tmp_path / "copy"
    destination.mkdir()
    copy_range(recorded.parent.parent, description["wall_time"] - 10.0, time.time() + 10.0, destination, {})
    assert load_description(destination / recorded.name)["format"] == FORMAT

    (recorded / "experiment.json").write_text(json.dumps({**description, "format": FORMAT + 1}))
    newer = rerun_view.log_range(recorded.parent.parent, description["wall_time"] - 10.0, time.time() + 10.0,
                                 save=tmp_path / "newer.rrd")
    assert newer["rows"] > 0 and any(f"format {FORMAT + 1} is newer" in note for note in newer["notes"])
    simulated = rerun_view.log_range(recorded.parent.parent, description["wall_time"] - 10.0, time.time() + 10.0,
                                     save=tmp_path / "sim.rrd", environment="sim")
    assert simulated["sessions"] == [] and "(sim only)" in simulated["notes"][0]
    del description["format"]
    (recorded / "experiment.json").write_text(json.dumps(description))
    found = rerun_view._sessions(recorded.parent.parent, description["wall_time"] - 10.0, time.time() + 10.0)
    assert found[0].format == 1


def test_every_older_format_has_its_upgrade():
    """A FORMAT bump comes with how the format before it reads (`UPGRADES`)."""
    assert set(rerun_view.UPGRADES) == set(range(1, FORMAT))
