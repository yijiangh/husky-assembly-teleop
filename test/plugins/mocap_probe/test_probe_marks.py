"""The probe's marks on the floor: stored by hand from the first export, then every calibration finds that frame."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_floor_calibration_plugin import NOW, floor_exports, make_ctx, server  # noqa: F401 (the viser server fixture)
from husky_assembly_teleop.plugins.mocap_probe import FORCE_SECONDS, MARK_BUTTONS, MocapProbePlugin
from husky_assembly_teleop.robot_interface.mocap_relay import FloorCalibrationState
from husky_assembly_teleop.world.calibration_sessions import SESSIONS_FOLDER
from husky_assembly_teleop.world.floor_calibration import FLOOR_LABEL
from husky_assembly_teleop.world.lab_frame import FORMAT, floor_xy, read_lab_frame

#: Where the marks are, in the first calibration's mocap frame (the floor at z 2 cm), metres.
MARKS = {"origin": (1.0, 2.0), "mark1": (4.5, 1.6), "mark2": (-2.0, 4.0), "mark3": (1.4, -2.5)}
#: Motive's frame after a recalibration: turned 3° and shifted 4 cm.
MOVED = Rotation.from_euler("z", 3.0, degrees=True)
SHIFT = np.array([0.04, -0.03, 0.0])


def calibrate(server, tmp_path, moved: bool = False,  # noqa: F811
              marks: dict = MARKS) -> tuple[MocapProbePlugin, object]:
    """Probe eight floor points and the four `marks`, in the first frame or the moved one."""
    ctx = make_ctx(server, tmp_path)
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    rng = np.random.default_rng(0)
    points = [(FLOOR_LABEL, (x, y)) for x, y in rng.uniform(-3, 3, (8, 2))] + list(
        zip(MARK_BUTTONS.values(), marks.values()))
    for label, (x, y) in points:
        position = np.array([x, y, 0.02])
        ctx.probe.position = MOVED.apply(position) + SHIFT if moved else position
        plugin._record(ctx, label)
    plugin.draw(ctx)
    return plugin, ctx


def store_marks(export: Path, points_file: Path) -> dict:
    """Write the lab frame file by hand, as the lab does once: the export's marks at their floor (x, y)."""
    marks = json.loads(export.read_text())["marks"]
    points_file.write_text(json.dumps({"format": FORMAT, "marks": {name: entry["floor_xy"]
                                                                   for name, entry in marks.items()}}))
    return read_lab_frame(points_file).marks


def first_calibration(server, tmp_path) -> dict:  # noqa: F811
    """Calibrate without stored marks, export, and store the marks from that export."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True, exist_ok=True)
    (tmp_path / "data" / "lab").mkdir(parents=True, exist_ok=True)
    plugin, ctx = calibrate(server, tmp_path)
    assert "frame not fixed" in plugin._floor_status.content
    assert "no marks or anchors stored in main_frame.json" in plugin._marks_html()
    plugin._export_floor(ctx)
    plugin.teardown(ctx)
    [export] = floor_exports(tmp_path)
    stored = store_marks(export, tmp_path / "data" / "lab" / "main_frame.json")
    export.unlink()
    return stored


def test_the_marks_from_the_first_export_give_every_later_calibration_its_frame(server, tmp_path):  # noqa: F811
    """The first export puts the marks in the origin-and-mocap-x frame; after Motive moved they map back there."""
    stored = first_calibration(server, tmp_path)
    assert stored["origin"] == pytest.approx([0.0, 0.0], abs=1e-9)
    assert stored["mark1"] == pytest.approx([3.5, -0.4], abs=1e-6)

    plugin, ctx = calibrate(server, tmp_path, moved=True)
    assert "fixed 4/4" in plugin._floor_status.content
    assert "mark1  x +3.500 y -0.400 m" in plugin._marks_html()
    plugin._export_floor(ctx)
    [path] = floor_exports(tmp_path)
    data = json.loads(path.read_text())
    transform = np.array(data["floor_from_raw"]["matrix"])
    for name, xy in stored.items():
        assert floor_xy(transform, MOVED.apply([*MARKS[name], 0.02]) + SHIFT) == pytest.approx(xy, abs=1e-6)
    assert data["lab_frame_file"] == "main_frame.json"
    assert data["marks"]["mark2"]["residual"] == pytest.approx(0.0, abs=1e-6)
    assert data["marks"]["mark2"]["stored_xy"] == pytest.approx(stored["mark2"])
    plugin.teardown(ctx)


def test_a_moved_mark_exports_only_on_a_second_press_and_too_few_marks_not_at_all(server, tmp_path):  # noqa: F811
    """A mark 5 mm off: red chip, exported on a second press within 2 s (orange meanwhile); one mark: no frame.

    A second press too late only arms the button again.
    """
    first_calibration(server, tmp_path)
    plugin, ctx = calibrate(server, tmp_path, marks={**MARKS, "mark2": (-2.005, 4.0)})
    assert "mark2 off" in plugin._floor_status.content
    plugin._export_floor(ctx)
    assert plugin._floor_message.startswith("not exported: mark2 lands")
    assert plugin._floor_message.endswith("press again within 2 s to export anyway")
    button = plugin._confirms["export"][0][0]
    plugin.draw(ctx)
    assert not floor_exports(tmp_path) and (button.label, button.color) == ("Press again: export anyway", "orange")
    ctx.now = lambda: NOW + FORCE_SECONDS + 1.0
    plugin._export_floor(ctx)
    assert not floor_exports(tmp_path)
    plugin._export_floor(ctx)
    plugin.draw(ctx)
    [export] = floor_exports(tmp_path)
    assert plugin._floor_message.startswith(f"exported {export.name} with a warning")
    assert (button.label, button.color) == ("Export floor calibration", None)
    export.unlink()
    plugin._remove_samples(ctx, [sample for sample in plugin.samples if sample.label in ("mark1", "mark2", "mark3")])
    plugin.draw(ctx)
    assert "no frame" in plugin._floor_status.content
    plugin._export_floor(ctx)
    assert "at least two of the marks" in plugin._floor_message
    assert not floor_exports(tmp_path)
    plugin.teardown(ctx)


def test_the_marks_check_the_frame_the_relay_applies_and_its_estimate(server, tmp_path):  # noqa: F811
    """The relay applies a frame 2 mm off, its fit says 2 mm back: the marks say 2 mm off, the relay too, 0 apart."""
    first_calibration(server, tmp_path)
    plugin, ctx = calibrate(server, tmp_path)
    assert "poses off mm: marks – · relay –" in plugin._floor_status.content
    assert "marks vs relay: – mm apart" in plugin._floor_status.content
    state = ctx.world.mocap_relay.state
    state.frame = FloorCalibrationState("held", lab_from_raw=((-0.998, -2.0, -0.02), (0.0, 0.0, 0.0, 1.0)))
    state.fit_now, state.metrics_time = ((-0.002, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), ctx.now()
    plugin.draw(ctx)
    assert "poses off mm: marks 2.0 · relay 2.0" in plugin._floor_status.content
    assert "marks vs relay: 0.0 mm apart" in plugin._floor_status.content
    plugin.teardown(ctx)
