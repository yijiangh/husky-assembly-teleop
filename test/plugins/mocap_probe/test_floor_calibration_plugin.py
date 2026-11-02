"""The mocap probe's floor calibration on a real viser server: probe the floor and the base point, then export."""

from __future__ import annotations

import json
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import viser
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from husky_assembly_teleop.colors import NONE, WARN, rgb
from husky_assembly_teleop.plugins.mocap_probe import MocapProbePlugin
from husky_assembly_teleop.ui.second_press import SECOND_PRESS_SECONDS
from husky_assembly_teleop.robot_interface.mocap_definitions import read_cameras
from husky_assembly_teleop.robot_interface.mocap_relay import FloorCalibrationState, MocapRelayState
from husky_assembly_teleop.world.calibration_sessions import SESSIONS_FOLDER
from husky_assembly_teleop.world.floor_calibration import FLOOR_LABEL, ORIGIN_LABEL, fit_plane
from husky_assembly_teleop.world.mocap_frame import FLOOR_PREFIX

NOW = 100.0
#: The relay of the robots, as it announces its floor calibration: none yet, in the robots' sessions folder.
NO_FLOOR = FloorCalibrationState("none", folder=SESSIONS_FOLDER.as_posix())
#: The lab's ten cameras, as a simulation's fake Motive announces them.
LAB_CAMERAS = read_cameras((DATA / "sim" / "motive_cameras.json").read_text())


def floor_exports(drive: Path) -> list[Path]:
    """The floor calibrations exported into the robots' dated sessions on the Drive, oldest first."""
    return sorted((drive / SESSIONS_FOLDER).glob(f"*/*{FLOOR_PREFIX}.json"))


@pytest.fixture(scope="module")
def server():
    """One viser server on a free port for the whole module, with a second as its phone page (`server.phone`)."""
    server = viser.ViserServer(port=0, verbose=False)
    server.phone = viser.ViserServer(port=0, verbose=False)
    yield server
    server.phone.stop()
    server.stop()


def make_ctx(server: viser.ViserServer, drive: Path) -> SimpleNamespace:
    """A context whose probe is tracked well, at whatever `ctx.probe.position` is set to; clicks run at once.

    The relay announces no floor calibration and the lab's cameras.
    """
    probe = SimpleNamespace(mocap_id=1863, position=np.zeros(3), orientation=np.array([0.0, 0.0, 0.0, 1.0]),
                            marker_error=1e-4, marker_errors=[(NOW - 0.1, 1e-4)], last_update_time=NOW,
                            tracking_valid=True, tracked=True)

    @contextmanager
    def gui(*_args, **_options):
        yield server.gui

    @contextmanager
    def phone(*_args):
        yield server.phone.gui

    view = SimpleNamespace(ui=gui, tab=gui, phone=phone, gui=server.gui, scene=server.scene,
                           scene_root="/mocap_probe")
    return SimpleNamespace(view=view, probe=probe, now=lambda: NOW,
                           config=SimpleNamespace(drive_root=drive, data_directory=drive / "data", lab=""),
                           world=SimpleNamespace(robots={}, mocap_relay=SimpleNamespace(
                               state=MocapRelayState(frame=NO_FLOOR, cameras=dict(LAB_CAMERAS)),
                               use_refit=lambda: None)),
                           defer=lambda _name, action: lambda _event: action(),
                           defer_value=lambda _name, action, **_kwargs: lambda value: action(value),
                           track_object=lambda *_args, **_kwargs: probe, untrack_object=lambda _name: None,
                           log_info=lambda _text: None, log_warn=lambda _text: None)


def test_the_chips_stay_the_same_while_the_probe_moves(server, tmp_path):
    """Pose, height and sample count change on their own lines: each chip line, and so its hover, stays the same."""
    ctx = make_ctx(server, tmp_path)
    ctx.spawn = lambda _name, coroutine, **_options: coroutine.close()
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    for x, y in np.random.default_rng(0).uniform(-3, 3, (8, 2)):
        ctx.probe.position = np.array([x, y, 0.02])
        plugin._record(ctx, FLOOR_LABEL)
    plugin._set_mapping(ctx, True)
    chips = (plugin._status_chip, plugin._floor_chips, plugin._phone_floor_chips, plugin._coverage_chip)
    readouts = (plugin._status, plugin._floor_status, plugin._phone_floor_status, plugin._coverage_status)
    shown = []
    for z in (0.10, 0.25):
        ctx.probe.position = np.array([0.0, 0.0, z])
        ctx.probe.last_update_time += 0.01
        plugin.update(ctx)
        plugin.draw(ctx)
        shown.append(([handle.content for handle in chips], [handle.content for handle in readouts]))
    assert shown[0][0] == shown[1][0] and all(a != b for a, b in zip(shown[0][1], shown[1][1]))
    # * Mapping is in progress (amber); no floor calibration applied keeps the poses raw (amber too).
    assert f"background:{WARN};" in plugin._coverage_chip.content
    assert f"background:{WARN};" in plugin._frame_chips.content
    plugin._set_mapping(ctx, False)
    plugin.teardown(ctx)


def test_probe_floor_and_base_point_then_export(server, tmp_path):
    """Each floor point refits the plane; with the origin point the axes show and the export holds floor_from_raw."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    ctx = make_ctx(server, tmp_path)
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin.draw(ctx)
    assert "no fit" in plugin._floor_chips.content
    assert "no fit" in plugin._phone_floor_chips.content

    plugin._export_floor(ctx)
    assert plugin._floor_message.startswith("not exported")
    rng = np.random.default_rng(0)
    for x, y in rng.uniform(-3, 3, (8, 2)):
        ctx.probe.position = np.array([x, y, 0.02 + rng.normal(0.0, 3e-4)])
        plugin._record(ctx, FLOOR_LABEL)
        plugin.draw(ctx)
    assert "fit good" in plugin._floor_chips.content and "floor: 8 pts" in plugin._floor_status.content
    assert not plugin._floor_axes.visible

    ctx.probe.position = np.array([1.0, 2.0, 0.025])
    plugin._record(ctx, ORIGIN_LABEL)
    plugin.draw(ctx)
    assert "frame not fixed" in plugin._floor_chips.content and "origin new" in plugin._floor_status.content
    assert plugin._floor_axes.visible
    assert plugin._floor_axes.position == pytest.approx([1.0, 2.0, 0.02], abs=2e-3)
    w, x, y, z = plugin._floor_axes.wxyz
    plane = fit_plane([sample.position for sample in plugin.samples])
    assert Rotation.from_quat([x, y, z, w]).apply([0.0, 0.0, 1.0]) == pytest.approx(plane.normal)

    plugin._export_floor(ctx)
    [path] = floor_exports(tmp_path)
    data = json.loads(path.read_text())
    assert data["plane"]["points"] == 9 and not list(tmp_path.rglob("*.tmp"))
    assert np.array(data["floor_from_raw"]["matrix"])[:3, 3] == pytest.approx([-1.0, -2.0, -0.02], abs=2e-3)

    # * Deleting one floor point refits with the others.
    plugin._list._set_all(False)
    plugin._list._boxes[plugin.samples[0].id].value = True
    plugin._delete(ctx)
    plugin.draw(ctx)
    assert "floor: 8 pts" in plugin._floor_status.content
    plugin.teardown(ctx)


def test_points_in_one_spot_are_not_exported(server, tmp_path):
    """Floor points probed in one spot fit a plane at any angle: no axes, a red chip, and Export refuses."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    ctx = make_ctx(server, tmp_path)
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    rng = np.random.default_rng(1)
    for _ in range(5):
        ctx.probe.position = np.array([1.0, 1.0, 0.02]) + rng.normal(0.0, 3e-4, 3)
        plugin._record(ctx, FLOOR_LABEL)
    plugin._record(ctx, ORIGIN_LABEL)
    plugin.draw(ctx)
    assert "fit unusable" in plugin._floor_chips.content
    assert not plugin._floor_axes.visible
    plugin._export_floor(ctx)
    assert plugin._floor_message.startswith("not exported: the floor fit is unusable")
    assert not floor_exports(tmp_path)
    plugin.teardown(ctx)


def test_without_cameras_from_the_relay_nothing_is_exported(server, tmp_path):
    """The relay passes over a floor calibration without cameras: refused, saying why, nothing written."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    ctx = make_ctx(server, tmp_path)
    ctx.world.mocap_relay.state.cameras = {}
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    for x, y in np.random.default_rng(0).uniform(-3, 3, (8, 2)):
        ctx.probe.position = np.array([x, y, 0.0])
        plugin._record(ctx, FLOOR_LABEL)
    plugin._record(ctx, ORIGIN_LABEL)
    plugin._export_floor(ctx)
    assert plugin._floor_message.startswith("not exported: no cameras from the mocap relay")
    assert not floor_exports(tmp_path)
    plugin.teardown(ctx)


def test_phone_delete_all_needs_a_second_tap(server, tmp_path):
    """The first tap only asks (amber, with the count); a second within SECOND_PRESS_SECONDS deletes, a double not."""
    ctx = make_ctx(server, tmp_path)
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    for x, label in enumerate(["table", "table", FLOOR_LABEL, FLOOR_LABEL, ORIGIN_LABEL]):
        ctx.probe.position = np.array([float(x), 0.0, 0.0])
        plugin._record(ctx, label)
    floor_button, all_button = plugin._confirms["floor"][0][0], plugin._confirms["all"][0][0]

    plugin._delete_all(ctx, "floor")
    plugin.draw(ctx)
    assert len(plugin.samples) == 5
    assert (floor_button.label, floor_button.color) == ("Delete floor points?", rgb(WARN))
    plugin._delete_all(ctx, "floor")
    assert len(plugin.samples) == 5, "a double click"
    ctx.now = lambda: NOW + 1.0
    plugin._delete_all(ctx, "floor")
    plugin.draw(ctx)
    assert [sample.name for sample in plugin.samples] == ["table_1", "table_2"]
    assert (floor_button.label, floor_button.color) == ("Delete floor points", rgb(NONE))

    # * A second tap too late only arms the button again.
    plugin._delete_all(ctx, "all")
    ctx.now = lambda: NOW + SECOND_PRESS_SECONDS + 2.0
    plugin._delete_all(ctx, "all")
    assert len(plugin.samples) == 2
    ctx.now = lambda: NOW + SECOND_PRESS_SECONDS + 3.0
    plugin._delete_all(ctx, "all")
    plugin.draw(ctx)
    assert not plugin.samples and all_button.label == "Delete all points"
    plugin.teardown(ctx)


def test_phone_export_needs_a_second_tap(server, tmp_path):
    """The phone's Export floor calibration writes on a second tap only; a refusal shows at once, without one."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    ctx = make_ctx(server, tmp_path)
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin._export_floor(ctx, phone=True)
    assert plugin._floor_message.startswith("not exported")
    for x, y in np.random.default_rng(0).uniform(-3, 3, (8, 2)):
        ctx.probe.position = np.array([x, y, 0.0])
        plugin._record(ctx, FLOOR_LABEL)
    plugin._record(ctx, ORIGIN_LABEL)
    button = plugin._confirms["export"][1][0]
    plugin._export_floor(ctx, phone=True)
    plugin.draw(ctx)
    assert not floor_exports(tmp_path) and (button.label, button.color) == ("Export floor calibration?", rgb(WARN))
    assert "again within 5 s" in plugin._floor_result.content
    ctx.now = lambda: NOW + 1.0
    plugin._export_floor(ctx, phone=True)
    plugin.draw(ctx)
    assert len(floor_exports(tmp_path)) == 1 and button.label == "Export floor calibration"
    plugin.teardown(ctx)
