"""The probe's floor calibration is raw: floor_from_raw with the raw cameras and samples; Use refit asks the relay."""

from __future__ import annotations

import asyncio
import json
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from test_floor_calibration_plugin import floor_exports, make_ctx, server  # noqa: F401 (the viser server fixture)
from husky_assembly_teleop.plugins.mocap_probe import EXPORT_FOLDER, MocapProbePlugin
from husky_assembly_teleop.robot_interface.mocap_definitions import read_cameras
from husky_assembly_teleop.robot_interface.mocap_relay import FloorCalibrationState
from husky_assembly_teleop.world.calibration_sessions import SESSIONS_FOLDER, sessions_folder
from husky_assembly_teleop.world.floor_calibration import FLOOR_LABEL, ORIGIN_LABEL
from husky_assembly_teleop.world.mocap_frame import FLOOR_SUFFIX, newest_floor_calibration

LAB = read_cameras((DATA / "sim" / "motive_cameras.json").read_text())
#: The floor calibration the relay applies then (floor_from_raw).
CORRECTION = ((0.02, -0.01, 0.0), (0.0, 0.0, 0.0043633, 0.9999905))
#: The robots' relay's floor calibrations folder, as it announces it.
ROBOTS = SESSIONS_FOLDER.as_posix()


def probed(server, tmp_path):  # noqa: F811
    """The probe with eight floor points and a base point, the relay announcing the lab's cameras and a correction."""
    ctx = make_ctx(server, tmp_path)
    ctx.world.mocap_relay.state.cameras = LAB
    ctx.world.mocap_relay.state.frame = FloorCalibrationState("held", "old.json", lab_from_raw=CORRECTION,
                                                              folder=ROBOTS)
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    rng = np.random.default_rng(0)
    for x, y in rng.uniform(-3, 3, (8, 2)):
        ctx.probe.position = np.array([x, y, 0.02])
        plugin._record(ctx, FLOOR_LABEL)
    ctx.probe.position = np.array([1.0, 2.0, 0.02])
    plugin._record(ctx, ORIGIN_LABEL)
    return plugin, ctx


def raw(point) -> np.ndarray:
    """A point the relay published under CORRECTION, taken back to raw."""
    return Rotation.from_quat(CORRECTION[1]).inv().apply(np.subtract(point, CORRECTION[0]))


@pytest.mark.parametrize("simulated", [False, True])
def test_export_stores_floor_from_raw_and_the_raw_cameras_where_the_relay_reads_them(
        server, tmp_path, monkeypatch, simulated):  # noqa: F811
    """Today's session in the folder the relay announces, the robots' or a simulation's; its reader takes it, raw."""
    monkeypatch.setenv("ROS_DOMAIN_ID", "70")
    folder = sessions_folder(simulated)
    # * Made by the relay: the robots' on the Drive, a simulation's at its start.
    (tmp_path / folder).mkdir(parents=True)
    plugin, ctx = probed(server, tmp_path)
    state = ctx.world.mocap_relay.state
    state.frame, state.simulated = replace(state.frame, folder=folder.as_posix()), simulated
    plugin._export_floor(ctx)
    [path] = (tmp_path / folder).glob(f"*/*{FLOOR_SUFFIX}")
    data = json.loads(path.read_text())
    assert path.parent.name == path.name[:8] and data["environment"] == ("sim" if simulated else "real")
    assert len(data["cameras"]) == 10 and "relay_correction" not in data and data["ros_domain"] == 70
    frame, _skipped = newest_floor_calibration(tmp_path / folder)
    name = next(iter(LAB))
    np.testing.assert_allclose(frame.cameras[name].position, raw(LAB[name].position), atol=1e-9)
    # * The lab frame's origin is the origin point, dropped onto the floor: raw, as the relay applies it.
    origin = raw((1.0, 2.0, 0.02))
    floor = Rotation.from_quat(frame.floor_from_raw.orientation).apply(origin) + frame.floor_from_raw.position
    assert floor == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert simulated or not (tmp_path / "data_experiment" / "sim").exists()


@pytest.mark.parametrize("status, label, hover, enabled", [
    ("none", "no floor calibration", "Export floor calibration", False),
    ("unchecked", "frame unchecked", "no anchor or truss-camera fit yet", False),
    ("held", "frame held", "f.json", False),
    ("refit_pending", "refit pending", "poses are off until Use refit", True),
    ("untrusted", "frame untrusted", "Check in Motive that the anchors are seen (A3 missing now)", False),
    ("error", "frame: Drive not read", "f.json: cannot read it", False)])
def test_every_frame_state_shows_in_the_relays_words_and_use_refit_only_for_a_pending_refit(
        server, tmp_path, status, label, hover, enabled):  # noqa: F811
    """A chip on its fixed line, with what to do on hover; the button always there."""
    ctx = make_ctx(server, tmp_path)
    state = ctx.world.mocap_relay.state
    state.frame = FloorCalibrationState(status, "f.json", pending="poses off now 20.0 mm",
                                        error="f.json: cannot read it" if status == "error" else "", folder=ROBOTS)
    state.anchors_missing = ("A3",)
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin.draw(ctx)
    assert f">{label}</span>" in plugin._frame_chips.content and hover in plugin._frame_chips.content
    assert plugin._use_refit.disabled is not enabled


def test_use_refit_is_refused_while_anything_moves_and_otherwise_asks_the_relay(server, tmp_path):  # noqa: F811
    """A moving arm refuses it; nothing moving, the relay's answer is shown."""
    ctx = make_ctx(server, tmp_path)
    relay = ctx.world.mocap_relay
    relay.state.frame = FloorCalibrationState("refit_pending", "f.json", pending="frame moved 20.0 mm", folder=ROBOTS)
    asked = []
    relay.use_refit = lambda: asked.append(True) or "future"
    arm = SimpleNamespace(state=SimpleNamespace(last_update_time=99.9, is_executing=True, last_slight_motion_time=None))
    ctx.world.robots = {"a200-0805": SimpleNamespace(arms={"ur_arm": arm}, base=SimpleNamespace(state=None))}
    tasks = []
    ctx.spawn = lambda _label, work: tasks.append(work)

    async def answer(_future):
        return SimpleNamespace(success=True, message="refit adopted (f_frame_1.json): frame moved 20.0 mm")

    ctx.ros = answer
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin._use_refit_now(ctx)
    assert asked == [] and "is running a trajectory" in plugin._floor_message
    ctx.world.robots = {}
    plugin._use_refit_now(ctx)
    asyncio.run(tasks[0])
    assert asked == [True] and plugin._floor_message.startswith("refit adopted")


@pytest.mark.parametrize("status", ["refit_pending", "untrusted", "error"])
def test_export_goes_through_while_the_poses_are_off(server, tmp_path, status):  # noqa: F811
    """An out-of-date floor calibration never blocks a new one: written with the relay's state, naming what it ends."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    plugin, ctx = probed(server, tmp_path)
    ctx.world.mocap_relay.state.frame = FloorCalibrationState(status, "f.json", pending="frame moved 20.0 mm",
                                                              lab_from_raw=CORRECTION, folder=ROBOTS)
    warned = []
    ctx.log_warn = warned.append
    plugin._export_floor(ctx)
    assert "it replaces f.json, whose poses were off" in warned[-1]
    [path] = floor_exports(tmp_path)
    assert plugin._floor_message == f"exported {path.name}; press Apply floor calibration to use it"
    data = json.loads(path.read_text())
    assert data["relay_frame"]["status"] == status and data["relay_frame"]["pending"] == "frame moved 20.0 mm"
    # * Each sample keeps its raw position: the correction it was recorded under, taken off.
    sample = data["samples"][0]
    assert sample["raw_position"] == pytest.approx(raw(sample["position"]).tolist(), abs=1e-12)


def test_apply_floor_calibration_is_offered_for_a_newer_one_and_asks_the_relay(server, tmp_path):  # noqa: F811
    """A newer export announced: a chip and the button; pressed with nothing moving, the relay's answer is shown."""
    ctx = make_ctx(server, tmp_path)
    relay = ctx.world.mocap_relay
    relay.state.frame = FloorCalibrationState("held", "f.json", newer="g.json", newer_moves=0.004, folder=ROBOTS)
    asked, tasks = [], []
    relay.reload_floor_calibration = lambda: asked.append(True) or "future"
    ctx.spawn = lambda _label, work: tasks.append(work)

    async def answer(_future):
        return SimpleNamespace(success=True, message="floor calibration g.json applied: points moved up to 4.0 mm")

    ctx.ros = answer
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin.draw(ctx)
    assert "newer floor calibration" in plugin._frame_chips.content and not plugin._apply_floor.disabled
    plugin._apply_floor_now(ctx)
    asyncio.run(tasks[0])
    assert asked == [True] and plugin._floor_message.startswith("floor calibration g.json applied")


def test_without_the_relays_folder_no_floor_calibration_is_written(server, tmp_path):  # noqa: F811
    """An older relay, or none, names no folder: nothing is guessed."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    plugin, ctx = probed(server, tmp_path)
    ctx.world.mocap_relay.state.frame = replace(ctx.world.mocap_relay.state.frame, folder="")
    plugin._export_floor(ctx)
    assert plugin._floor_message.startswith("not exported: the mocap relay announced no floor calibration folder")
    assert not list(tmp_path.rglob(f"*{FLOOR_SUFFIX}"))


@pytest.mark.parametrize("simulated, environment", [(False, "real"), (True, "sim")])
def test_probe_points_stay_with_the_measurements_marked_by_the_relays_mode(
        server, tmp_path, simulated, environment):  # noqa: F811
    """One folder for real and simulated points; the file says which."""
    (tmp_path / EXPORT_FOLDER).mkdir(parents=True)
    plugin, ctx = probed(server, tmp_path)
    ctx.world.mocap_relay.state.simulated = simulated
    plugin._export(ctx)
    [path] = (tmp_path / EXPORT_FOLDER).glob("*_probe_points.json")
    assert json.loads(path.read_text())["environment"] == environment
