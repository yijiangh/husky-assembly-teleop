"""The mocap_bars plugin: every bar with markers tracked, fitted ten times a second at most, one full search per tick.

The ghost bars, the per-marker and plate stats, the `[mocap bar]` report and the contract for the as-built record.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from planning_fixtures import purged_process
from plugin_fakes import FakeContext, FakeScene

from husky_assembly_teleop.plugins.mocap_bars import FIT_PERIOD, LOG_PERIOD, SEARCH_RETRY, MocapBarsPlugin
from husky_assembly_teleop.robot_interface.mocap_state import RawMarkers
from husky_assembly_teleop.world.bar_pose import bar_slide_roll, transform_points
from husky_assembly_teleop.world.marker_layouts import bar_layouts, fake_sensed_markers
from husky_assembly_teleop.world.measured import TrackedObject
from scaffolding_assembly_core.geometry import Pose

BAR = "bars/B20"
BARS = ("bars/B20", "bars/B21", "bars/B34", "bars/B35")


def bench() -> tuple[MocapBarsPlugin, FakeContext, dict[str, TrackedObject]]:
    """The plugin set up on the purged process after one update: its four bars tracked, no message yet."""
    loaded = purged_process()
    selection = SimpleNamespace(process=loaded.process, ids=None, movement=None)
    ctx = FakeContext(plugins={"process": SimpleNamespace(selection=selection)}, name="mocap_bars")
    ctx.view.scene = FakeScene()
    plugin = MocapBarsPlugin()
    plugin.setup(ctx)
    plugin.update(ctx)
    return plugin, ctx, {bar: state.obj for bar, state in plugin._bars.items()}


def stream(obj: TrackedObject, T: np.ndarray, points: np.ndarray, t: float, seed: int = 0) -> None:
    """A message of a bar's asset at pose T (slid 30 mm, rolled 2°, one stray marker), as the relay stores it."""
    sensed = fake_sensed_markers(T, points, seed=seed)
    count = len(sensed.points)
    obj.markers = RawMarkers(np.array(sensed.ids), sensed.points, np.ones(count, bool), sensed.residuals,
                             np.full(count, 0.019), capture_time=t)
    obj.last_update_time = t


def seat(bar: str) -> np.ndarray:
    """A bar's assembly pose, 4x4."""
    return purged_process().process.bodies[bar].assembly_pose.matrix()


def test_every_bar_with_a_layout_is_tracked_on_load_and_the_id_override_retracks_the_chosen_one():
    """B20, B21, B34 and B35 as assets 1020..1035; the Mocap ID box and Track re-track only the chosen bar."""
    plugin, ctx, objs = bench()
    assert list(objs) == list(BARS)
    assert [obj.mocap_id for obj in objs.values()] == [1020, 1021, 1034, 1035]
    assert sorted(ctx.world.tracked_objects) == ["fitted_bar/B20", "fitted_bar/B21", "fitted_bar/B34", "fitted_bar/B35"]
    assert plugin._bar.options == list(BARS) and plugin._bar_id == BAR and plugin._mocap_id.value == 1020
    assert plugin._stats_rows == 3 + 8 + 7
    plugin._mocap_id.value = 1840
    plugin._track(ctx)
    assert ctx.world.tracked_objects["fitted_bar/B20"].mocap_id == 1840
    assert ctx.world.tracked_objects["fitted_bar/B21"].mocap_id == 1021
    assert any("tracking B20 as asset 1020, B21 as asset 1021" in line for line in ctx.lines("info"))
    plugin._choose("bars/B21")
    assert plugin._mocap_id.value == 1021


def test_fits_a_streaming_bar_at_most_every_tenth_second_then_around_the_last_fit():
    """The first message gets the full search; the next within 0.1 s waits; then the match around the last fit."""
    plugin, ctx, objs = bench()
    layout = bar_layouts(purged_process().process)[BAR]
    T, obj = seat(BAR), objs[BAR]
    start = ctx.t
    stream(obj, T, layout.points, start)
    plugin.update(ctx)
    plugin.draw(ctx)
    state = plugin._bars[BAR]
    first = state.fit
    assert first.ok and not first.tracked
    assert "fit 8/8" in plugin._readout()[1] and "fit 8/8" in plugin._status.content
    # * The fitted pose is the slid, rolled one; each CAD marker got its fake marker, the stray none.
    expected = T @ bar_slide_roll(0.03, 2.0)
    np.testing.assert_allclose(first.T_world_from_bar[:3, 3], expected[:3, 3], atol=2e-3)
    rows = plugin._matched(state).reshape(-1, 3)
    np.testing.assert_allclose(rows, transform_points(expected, layout.points), atol=3e-3)
    assert plugin._fit_lines.points.shape == (9, 2, 3) and plugin._measured_cloud.points.shape == (9, 3)
    assert plugin.fits[BAR].fit is first and plugin.fits[BAR].pose == state.pose and list(plugin.fits) == [BAR]

    ctx.t = start + FIT_PERIOD / 2
    stream(obj, T, layout.points, ctx.t, seed=1)
    plugin.update(ctx)
    assert state.fit is first
    ctx.t = start + FIT_PERIOD
    stream(obj, T, layout.points, ctx.t, seed=2)
    plugin.update(ctx)
    assert state.fit is not first and state.fit.ok and state.fit.tracked


def test_one_full_search_per_tick_round_robin():
    """Two bars streaming at once with no planned pose: one is searched this tick, the other the next."""
    plugin, ctx, objs = bench()
    layouts = bar_layouts(purged_process().process)
    for bar in ("bars/B20", "bars/B21"):
        stream(objs[bar], seat(bar), layouts[bar].points, ctx.t)
    plugin.update(ctx)
    fitted = [bar for bar in BARS if plugin._bars[bar].fit is not None and plugin._bars[bar].fit.ok]
    assert fitted == ["bars/B20"]
    assert plugin._bars["bars/B21"].fit is None, "the other bar keeps its last fit (none yet) while it waits"
    ctx.t += 0.02
    plugin.update(ctx)
    assert plugin._bars["bars/B21"].fit.ok and not plugin._bars["bars/B21"].fit.tracked
    assert plugin._bars["bars/B20"].fit.ok
    assert sorted(plugin.fits) == ["bars/B20", "bars/B21"]


def test_a_failed_search_waits_before_the_next_and_shows_why():
    """Two markers in view: no fit, its reason on the status line and in the stats; no search for SEARCH_RETRY."""
    plugin, ctx, objs = bench()
    obj = objs[BAR]
    obj.markers = RawMarkers(np.array([1, 2]), np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]]), np.ones(2, bool),
                             np.full(2, 0.0005), np.full(2, 0.019), capture_time=ctx.t)
    obj.last_update_time = ctx.t
    plugin.update(ctx)
    plugin.draw(ctx)
    state = plugin._bars[BAR]
    assert state.fit is not None and not state.fit.ok and state.pose is None
    assert plugin._readout()[1] == "B20: markers 2, measured 2; only 2 markers in view (need at least 3)"
    assert "FAILED: only 2 markers in view" in plugin._stats.content
    assert not plugin._fit_cloud.visible and BAR not in plugin._ghosts
    searched = state.fitted_update
    ctx.t += SEARCH_RETRY / 2
    obj.last_update_time = ctx.t
    plugin.update(ctx)
    assert state.fitted_update == searched, "no new search within SEARCH_RETRY"
    ctx.t += SEARCH_RETRY
    obj.last_update_time = ctx.t
    plugin.update(ctx)
    assert state.fitted_update == ctx.t


def test_ghost_bars_draw_every_fitted_bar_and_hide_when_unticked():
    """A fitted B20 and B21 each get a see-through frame at the fitted pose; unticking hides them, not the markers."""
    plugin, ctx, objs = bench()
    layouts = bar_layouts(purged_process().process)
    for bar in ("bars/B20", "bars/B21"):
        stream(objs[bar], seat(bar), layouts[bar].points, ctx.t)
        plugin.update(ctx)
        ctx.t += 0.2
    plugin.draw(ctx)
    assert sorted(plugin._ghosts) == ["bars/B20", "bars/B21"]
    for bar in ("bars/B20", "bars/B21"):
        frame = plugin._ghosts[bar]
        assert frame.visible and frame.name.endswith(f"/ghosts/{bar[5:]}")
        np.testing.assert_allclose(frame.position, plugin._bars[bar].pose.position)
    meshes = [node for node in ctx.view.scene.nodes if node.name.startswith(plugin._ghosts[BAR].name + "/")]
    assert meshes, "the bar's shapes and its halves' hang under the frame"
    plugin._ghost_bars.value = False
    plugin.draw(ctx)
    assert not any(frame.visible for frame in plugin._ghosts.values()) and plugin._fit_cloud.visible


def test_the_stats_block_has_fixed_rows_and_lists_markers_and_plates():
    """A fitted B20: a summary, eight marker rows, the strays, four plates, three spacings, the deviation."""
    plugin, ctx, objs = bench()
    layout = bar_layouts(purged_process().process)[BAR]
    T = seat(BAR)
    ids = {BAR: "process/bars/B20"}
    ctx.plugins["process"].selection.ids = ids
    ctx.scene.snapshot.world_poses["process/bars/B20"] = purged_process().process.bodies[BAR].assembly_pose
    stream(objs[BAR], T, layout.points, ctx.t)
    plugin.update(ctx)
    plugin.draw(ctx)
    rows = [row for row in plugin._stats.content.split("<div>")[1:]]
    assert len(rows) == plugin._stats_rows == 18
    text = plugin._stats.content
    assert "source asset  age 0 ms  fit 8/8  rms " in text and "ambiguous no  dropped 0  occluded 0" in text
    assert text.count("matched") == 8 and "stray sensed 1" in text
    assert text.count("pair 48.") == 4 and text.count(" → ") == 3
    assert "vs planned  dpos 30." in text and "dz(along) +30." in text
    assert plugin._stats.content.count("—") == 0


def test_measure_and_log_prints_three_mocap_bar_lines_and_log_live_every_five_seconds():
    """The button prints the report at once; "Log live" repeats it every LOG_PERIOD, not in between."""
    plugin, ctx, objs = bench()
    layout = bar_layouts(purged_process().process)[BAR]
    stream(objs[BAR], seat(BAR), layout.points, ctx.t)
    plugin._measure_and_log(ctx)
    lines = [line for line in ctx.lines("info") if line.startswith("[mocap bar] ")]
    assert len(lines) == 3
    assert lines[0].startswith("[mocap bar] bars/B20 | source asset (age 0 ms) | 8/8 matched | rms ")
    assert lines[1].startswith("[mocap bar] residuals (mm): J20-") and lines[1].endswith("| stray sensed: 1")
    assert lines[2].startswith("[mocap bar] bar ends (world, mm): Rhino - | MoCap start [")
    plugin._log_live.value = True
    plugin.update(ctx)
    assert sum(line.startswith("[mocap bar] bars/B20") for line in ctx.lines("info")) == 2
    ctx.t += LOG_PERIOD / 2
    plugin.update(ctx)
    assert sum(line.startswith("[mocap bar] bars/B20") for line in ctx.lines("info")) == 2
    ctx.t += LOG_PERIOD
    plugin.update(ctx)
    assert sum(line.startswith("[mocap bar] bars/B20") for line in ctx.lines("info")) == 3


def test_measure_now_fits_every_streaming_bar_afresh_for_the_contract():
    """`measure_now` fits B20 around its hint and B21 by the full search; both come back, neither tracked."""
    plugin, ctx, objs = bench()
    layouts = bar_layouts(purged_process().process)
    for bar in ("bars/B20", "bars/B21"):
        stream(objs[bar], seat(bar), layouts[bar].points, ctx.t)
    hint = Pose.from_matrix(seat(BAR) @ bar_slide_roll(0.03, 2.0))  # where the fake puts the bar
    found = plugin.measure_now({BAR: hint})
    assert sorted(found) == ["bars/B20", "bars/B21"]
    assert found[BAR].fit.ok and found[BAR].tracked and found[BAR].mocap_id == 1020 and found[BAR].deviation is None
    assert found["bars/B21"].fit.ok and not found["bars/B21"].tracked and found["bars/B21"].source == "asset"
    record = found[BAR].to_json()
    assert record["fit"]["ok"] and len(record["pose"]) == 7 and record["layout"]["length"] == layouts[BAR].length
    assert plugin.fits[BAR].fit is found[BAR].fit
