"""The probe's Anchor button records the tip, a hint: the anchor is the marker nearest it, read with the probe away."""

from __future__ import annotations

import json
from dataclasses import asdict, replace
from types import SimpleNamespace

import numpy as np
import pytest

from test_floor_calibration_plugin import NOW, floor_exports, make_ctx, server  # noqa: F401 (the server fixture)
from husky_assembly_teleop.plugins.mocap_probe import (BEYOND_COLOR, MARKERS_AWAY, NOT_FOUND_COLOR,
                                                       STORED_ANCHOR_COLOR, WITHIN_COLOR, MocapProbePlugin)
from husky_assembly_teleop.world.calibration_sessions import SESSIONS_FOLDER
from husky_assembly_teleop.world.floor_calibration import ANCHOR_LABEL
from husky_assembly_teleop.world.lab_frame import FORMAT

#: Unlabeled markers the relay announces: two anchors on the floor and one on a table.
MARKERS = ((-2.8, -3.4, 0.03), (3.4, -1.7, 0.02), (1.5, 1.1, 1.0))


def pointing(server, tmp_path, anchors: dict | None = None):  # noqa: F811
    """The probe plugin with the relay's markers just announced and the lab frame's anchors stored."""
    (tmp_path / "data" / "lab").mkdir(parents=True)
    frame = {"format": FORMAT, "anchors": anchors or {}}
    (tmp_path / "data" / "lab" / "main_frame.json").write_text(json.dumps(frame))
    ctx = make_ctx(server, tmp_path)
    ctx.config.lab = "main"
    ctx.world.mocap_relay.state.unlabeled_markers, ctx.world.mocap_relay.state.markers_time = MARKERS, NOW - 1.0
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    return plugin, ctx


def anchors(plugin) -> dict:
    """The anchor samples by name, with their positions."""
    return {sample.name: tuple(sample.position) for sample in plugin.samples if sample.label == ANCHOR_LABEL}


def away(ctx, markers=MARKERS) -> None:
    """The probe put down far from every anchor, and a reading of `markers` come MARKERS_AWAY s later."""
    later = NOW + MARKERS_AWAY + 1.0
    ctx.now = lambda: later
    ctx.probe.position = np.array([0.0, 0.0, 3.0])
    ctx.world.mocap_relay.state.unlabeled_markers, ctx.world.mocap_relay.state.markers_time = markers, later


def test_the_tip_is_recorded_named_by_the_lab_frame(server, tmp_path):  # noqa: F811
    """Tip 6 cm off a stored anchor's marker: the tip, under the stored name; again, the same anchor."""
    plugin, ctx = pointing(server, tmp_path, {"A4": [3.41, -1.69, 0.02]})
    ctx.probe.position = np.array([3.4, -1.64, 0.02])
    plugin._record_anchor(ctx)
    ctx.probe.position = np.array([3.4, -1.66, 0.02])
    plugin._record_anchor(ctx)
    assert anchors(plugin) == {"A4_1": (3.4, -1.64, 0.02), "A4_2": (3.4, -1.66, 0.02)}
    plugin.teardown(ctx)


def test_a_tip_the_lab_frame_has_no_anchor_near_is_named_the_next_free_anchor(server, tmp_path):  # noqa: F811
    """No stored anchor near it: A1 is stored elsewhere, so it is A2."""
    plugin, ctx = pointing(server, tmp_path, {"A1": [0.0, 0.0, 0.0]})
    ctx.probe.position = np.array([-2.75, -3.4, 0.05])
    plugin._record_anchor(ctx)
    assert list(anchors(plugin)) == ["A2_1"]
    plugin.teardown(ctx)


def test_the_tip_is_recorded_while_the_probe_hides_the_marker(server, tmp_path):  # noqa: F811
    """No marker near the tip, or the relay's markers 10 s old: the tip is recorded all the same."""
    plugin, ctx = pointing(server, tmp_path)
    ctx.world.mocap_relay.state.unlabeled_markers = ()
    ctx.probe.position = np.array(MARKERS[2])
    plugin._record_anchor(ctx)
    ctx.world.mocap_relay.state.markers_time = NOW - 10.0
    ctx.probe.position = np.array(MARKERS[1])
    plugin._record_anchor(ctx)
    assert list(anchors(plugin)) == ["A1_1", "A2_1"] and "read once the probe is 0.5 m away" in plugin._floor_message
    plugin.teardown(ctx)


def test_markers_taken_before_the_frame_changed_are_not_used(server, tmp_path):  # noqa: F811
    """Use refit or Apply moves lab_from_raw: markers the relay took before it are refused until newer ones come."""
    plugin, ctx = pointing(server, tmp_path)
    ctx.probe.position = np.array(MARKERS[1])
    plugin._record_anchor(ctx)
    away(ctx)
    state = ctx.world.mocap_relay.state
    state.frame = replace(state.frame, status="held", lab_from_raw=((0.02, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), time=50.0)
    state.markers_stamp = 49.0
    found, missing = plugin._anchors_now(ctx)
    assert found == {} and "from before its floor calibration changed" in missing["A1"]
    state.markers_stamp = 51.0
    assert list(plugin._anchors_now(ctx)[0]) == ["A1"]
    plugin.teardown(ctx)


def floor_and_marks(plugin, ctx) -> None:
    """Eight floor points and the origin, so the floor calibration can be exported."""
    for x, y in np.random.default_rng(0).uniform(-3, 3, (8, 2)):
        ctx.probe.position = np.array([x, y, 0.0])
        plugin._record(ctx, "floor")
    ctx.probe.position = np.array([0.5, 0.5, 0.0])
    plugin._record(ctx, "origin")


def test_the_export_takes_every_anchor_marker_from_one_reading_with_the_probe_away(server, tmp_path):  # noqa: F811
    """Tips 1 cm off the markers, which then move 1 mm: the anchors are the markers of the reading, the samples tips."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    plugin, ctx = pointing(server, tmp_path)
    for marker in MARKERS[:2]:
        ctx.probe.position = np.add(marker, (0.0, 0.0, 0.01))
        plugin._record_anchor(ctx)
    floor_and_marks(plugin, ctx)
    later = tuple(tuple(np.add(marker, (0.001, 0.0, 0.0))) for marker in MARKERS)
    away(ctx, later)
    plugin._export_floor(ctx)
    [path] = floor_exports(tmp_path)
    data = json.loads(path.read_text())
    assert data["anchors"] == {"A1": list(later[0]), "A2": list(later[1])}
    tips = {sample["name"]: sample["position"] for sample in data["samples"] if sample["label"] == ANCHOR_LABEL}
    assert tips == {"A1_1": list(np.add(MARKERS[0], (0.0, 0.0, 0.01))), "A2_1": list(np.add(MARKERS[1], (0, 0, 0.01)))}
    plugin.teardown(ctx)


def test_the_probe_near_an_anchor_stops_the_export(server, tmp_path):  # noqa: F811
    """Just pointed at: not exported; put down, then back within 0.5 m of A2: A2 not read until it is away again."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    plugin, ctx = pointing(server, tmp_path)
    for marker in MARKERS[:2]:
        ctx.probe.position = np.array(marker)
        plugin._record_anchor(ctx)
    floor_and_marks(plugin, ctx)
    ctx.world.mocap_relay.state.markers_time = NOW + 1.0
    plugin._export_floor(ctx)
    assert plugin._floor_message.startswith("not exported: anchors not read (A1, A2: probe near)")
    away(ctx)
    ctx.probe.position = np.add(MARKERS[1], (0.3, 0.0, 0.0))
    plugin.update(ctx)
    assert list(plugin._anchors_now(ctx)[1]) == ["A2"]
    plugin.teardown(ctx)


def test_an_anchor_not_in_the_relays_reading_stops_the_export(server, tmp_path):  # noqa: F811
    """A2's marker gone from the relay's markers: not exported, saying which."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    plugin, ctx = pointing(server, tmp_path)
    for marker in MARKERS[:2]:
        ctx.probe.position = np.array(marker)
        plugin._record_anchor(ctx)
    floor_and_marks(plugin, ctx)
    away(ctx, (MARKERS[0], MARKERS[2]))
    plugin._export_floor(ctx)
    assert plugin._floor_message.startswith("not exported: anchors not read (A2: no unlabeled marker within 10 cm)")
    plugin.teardown(ctx)


def test_loaded_anchors_keep_their_anchor_and_numbering(server, tmp_path):  # noqa: F811
    """A1_1 loaded twice: the second is A1_2, still anchor A1; the next pointed at A1 is A1_3, and the export goes."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    plugin, ctx = pointing(server, tmp_path)
    ctx.probe.position = np.array(MARKERS[1])
    plugin._record_anchor(ctx)
    exported = json.dumps({"samples": [asdict(sample) for sample in plugin.samples]})
    plugin._remove_samples(ctx, list(plugin.samples))
    for _ in range(2):
        plugin._load_points(ctx, SimpleNamespace(name="points.json", content=exported.encode()))
    plugin._record_anchor(ctx)
    assert list(anchors(plugin)) == ["A1_1", "A1_2", "A1_3"]
    floor_and_marks(plugin, ctx)
    away(ctx)
    plugin._export_floor(ctx)
    [path] = floor_exports(tmp_path)
    assert list(json.loads(path.read_text())["anchors"]) == ["A1"]
    plugin.teardown(ctx)


def test_an_anchor_moved_from_its_stored_place_is_left_out_and_still_exported(server, tmp_path):  # noqa: F811
    """A2 6 mm from its stored place: amber chip naming it, export goes, A2 kept for the relay with its new place."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    stored = {f"A{index + 1}": list(marker) for index, marker in enumerate(MARKERS)}
    stored["A2"] = list(np.add(MARKERS[1], (0.006, 0.0, 0.0)))
    plugin, ctx = pointing(server, tmp_path, stored)
    for marker in MARKERS:
        ctx.probe.position = np.array(marker)
        plugin._record_anchor(ctx)
    floor_and_marks(plugin, ctx)
    away(ctx)
    plugin._export_floor(ctx)
    [path] = floor_exports(tmp_path)
    data = json.loads(path.read_text())
    assert sorted(data["anchors"]) == ["A1", "A2", "A3"]
    assert data["anchors_in_floor"]["A2"]["moved"] == pytest.approx(0.006)
    assert "A2" not in data["fits"]["both"]["residuals"]
    plugin.draw(ctx)
    assert "left out as moved: A2" in plugin._floor_status.content and "A2 " in plugin._floor_status.content
    plugin.teardown(ctx)


def calibrated(ctx) -> None:
    """The relay applies a floor calibration, the identity: its markers are in the lab frame."""
    state = ctx.world.mocap_relay.state
    state.frame = replace(state.frame, status="held", lab_from_raw=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))


def test_stored_anchors_are_found_by_themselves_and_one_not_found_is_asked_for(server, tmp_path):  # noqa: F811
    """A1, A2 stored 1 mm off their markers, A3 where none is: A1, A2 read unpointed, A3 "point"; export on press 2."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    stored = {"A1": list(np.add(MARKERS[0], (0.001, 0.0, 0.0))), "A2": list(np.add(MARKERS[1], (0.001, 0.0, 0.0))),
              "A3": [0.0, 3.0, 0.0]}
    plugin, ctx = pointing(server, tmp_path, stored)
    calibrated(ctx)
    floor_and_marks(plugin, ctx)
    away(ctx)
    found, missing = plugin._anchors_now(ctx)
    assert sorted(found) == ["A1", "A2"] and list(missing) == ["A3"] and missing["A3"].endswith("point at it")
    plugin.draw(ctx)
    assert "anchors mm:</div><div>  A1 0.0 · A2 0.0 · A3 point</div>" in plugin._floor_status.content
    assert "A3 not found" in plugin._floor_status.content and "A3 not found" in plugin._phone_floor_status.content
    plugin._show_stored.value = True
    plugin.draw(ctx)
    assert [tuple(sphere.color) for sphere in plugin._anchor_spheres["A3"]] == [NOT_FOUND_COLOR] * 2
    assert plugin._anchor_labels["A3"][1] == "A3 not found: point at it" and plugin._anchor_labels["A1"][1] == "A1"
    plugin._export_floor(ctx)
    assert plugin._floor_message.startswith("not exported: anchors not read (A3: no unlabeled marker within 10 cm of "
                                            "its stored place: point at it)")
    plugin._export_floor(ctx)
    [path] = floor_exports(tmp_path)
    assert sorted(json.loads(path.read_text())["anchors"]) == ["A1", "A2"]
    plugin.teardown(ctx)


def test_stored_anchors_are_not_looked_for_without_a_floor_calibration(server, tmp_path):  # noqa: F811
    """No floor calibration applied: the relay's markers are raw, so a stored place says nothing; only pointed ones."""
    plugin, ctx = pointing(server, tmp_path, {"A1": list(MARKERS[0])})
    away(ctx)
    assert plugin._anchors_now(ctx) == ({}, {})
    plugin.teardown(ctx)


def test_the_probe_near_a_stored_anchor_keeps_it_unread(server, tmp_path):  # noqa: F811
    """Never pointed at, but the probe passes 0.3 m from A1: its marker is read only MARKERS_AWAY s later."""
    plugin, ctx = pointing(server, tmp_path, {"A1": list(MARKERS[0])})
    calibrated(ctx)
    away(ctx)
    ctx.probe.position = np.add(MARKERS[0], (0.3, 0.0, 0.0))
    plugin.update(ctx)
    assert plugin._anchors_now(ctx)[1] == {"A1": "probe near"}
    plugin.teardown(ctx)


def test_stored_marks_and_anchors_show_when_ticked_where_the_frame_puts_them(server, tmp_path):  # noqa: F811
    """Hidden unticked or without a frame; ticked, at the relay's frame, and once fitted, with the axes."""
    plugin, ctx = pointing(server, tmp_path, {"A1": list(MARKERS[0])})
    calibrated(ctx)
    plugin.draw(ctx)
    assert not plugin._stored.visible
    plugin._show_stored.value = True
    plugin.draw(ctx)
    assert plugin._stored.visible and list(plugin._stored.position) == pytest.approx([0.0, 0.0, 0.0])
    floor_and_marks(plugin, ctx)
    plugin.draw(ctx)
    assert plugin._stored.position == pytest.approx(plugin._floor_axes.position)
    state = ctx.world.mocap_relay.state
    state.frame = replace(state.frame, lab_from_raw=None)
    plugin._remove_samples(ctx, list(plugin.samples))
    plugin.draw(ctx)
    assert not plugin._stored.visible
    plugin.teardown(ctx)


def test_each_anchor_read_shows_as_its_marker_with_distance_and_noise(server, tmp_path):  # noqa: F811
    """A1 stored 1 mm off its marker, the relay's look giving it 0.04 mm noise: its marker labelled with both."""
    plugin, ctx = pointing(server, tmp_path, {"A1": list(np.add(MARKERS[0], (0.001, 0.0, 0.0)))})
    calibrated(ctx)
    away(ctx)
    ctx.world.mocap_relay.state.marker_noise = ((MARKERS[0], 0.04), (MARKERS[2], 0.5))
    plugin._show_stored.value = True
    plugin.draw(ctx)
    sphere, label, text = plugin._read["A1"]
    assert list(sphere.position) == pytest.approx(MARKERS[0]) and text == "A1 1.0 mm · noise 0.04 mm"
    assert [tuple(sphere.color) for sphere in plugin._anchor_spheres["A1"]] == [WITHIN_COLOR] * 2
    plugin._show_stored.value = False
    plugin.draw(ctx)
    assert not plugin._read
    assert [tuple(sphere.color) for sphere in plugin._anchor_spheres["A1"]] == [STORED_ANCHOR_COLOR] * 2


def test_an_anchor_beyond_its_tolerance_turns_its_spheres_red(server, tmp_path):  # noqa: F811
    """A1 stored 3 mm off its marker: over the 2 mm, red."""
    plugin, ctx = pointing(server, tmp_path, {"A1": list(np.add(MARKERS[0], (0.003, 0.0, 0.0)))})
    calibrated(ctx)
    away(ctx)
    plugin._show_stored.value = True
    plugin.draw(ctx)
    assert [tuple(sphere.color) for sphere in plugin._anchor_spheres["A1"]] == [BEYOND_COLOR] * 2
    plugin.teardown(ctx)


def test_a_stored_anchor_far_off_its_marker_is_asked_for_and_pointing_names_it(server, tmp_path):  # noqa: F811
    """A1 stored 15 cm off its marker: "point" even with the probe there; the tip is named A1, which is then read."""
    (tmp_path / SESSIONS_FOLDER).mkdir(parents=True)
    plugin, ctx = pointing(server, tmp_path, {"A1": list(np.add(MARKERS[0], (0.15, 0.0, 0.0)))})
    calibrated(ctx)
    ctx.probe.position = np.add(MARKERS[0], (0.0, 0.0, 0.01))
    plugin.update(ctx)
    assert plugin._anchors_now(ctx)[1]["A1"].endswith("point at it")
    plugin._record_anchor(ctx)
    assert list(anchors(plugin)) == ["A1_1"]
    away(ctx)
    assert list(plugin._anchors_now(ctx)[0]) == ["A1"]
    plugin.teardown(ctx)
