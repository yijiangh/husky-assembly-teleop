"""The mocap_bars plugin: every bar fitted to the raw markers near where it should be, at most every 0.1 s.

One full search per tick. The bars as see-through scene bodies, the per-marker and plate stats, the `[mocap bar]`
report and the contract for the as-built record.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from planning_fixtures import purged_process
from plugin_fakes import FakeContext, FakeScene

from husky_assembly_teleop.plugins.mocap_bars import (ARROW_MODES, FIT_PERIOD, GAP_COLOR, LOG_PERIOD, MAX_MARKER_AGE,
                                                      SEARCH_RETRY, TURN_COLOR, MocapBarsPlugin, deviation_arrows)
from husky_assembly_teleop.robot_interface.mocap_state import MarkerCloud
from husky_assembly_teleop.shared.bars.layouts import bar_layouts, fake_sensed_markers
from husky_assembly_teleop.shared.bars.pose import bar_slide_roll
from husky_assembly_teleop.world.marker_match import transform_points
from husky_assembly_teleop.world.scene import PluginScene
from scaffolding_assembly_core.geometry import Geometry, Pose
from scaffolding_assembly_core.scene import Body, Scene

BAR = "bars/B20"
BARS = ("bars/B20", "bars/B21", "bars/B34", "bars/B35")


def bench() -> tuple[MocapBarsPlugin, FakeContext]:
    """The plugin set up on the purged process after one update, a relay without a cloud yet."""
    loaded = purged_process()
    selection = SimpleNamespace(process=loaded.process, ids=None, movement=None)
    ctx = FakeContext(plugins={"process": SimpleNamespace(selection=selection)}, name="mocap_bars")
    ctx.view.scene = FakeScene()
    live, ctx.tick_scene = Scene(), Scene()
    ctx.scene = PluginScene(live, "mocap_bars", ctx.log_warn, lambda: ctx.tick_scene)
    ctx.world.mocap_relay = SimpleNamespace(state=SimpleNamespace(marker_cloud=None))
    plugin = MocapBarsPlugin()
    plugin.setup(ctx)
    plugin.update(ctx)
    return plugin, ctx


def stream(ctx: FakeContext, *points: np.ndarray) -> MarkerCloud:
    """One relay frame captured now holding these markers (19 mm balls, 0.5 mm residuals), as the monitor keeps it."""
    positions = np.concatenate([np.reshape(each, (-1, 3)) for each in points]) if points else np.zeros((0, 3))
    count = len(positions)
    cloud = MarkerCloud(positions, np.full(count, 0.019), np.full(count, 0.0005), np.arange(count), ctx.t, ctx.t)
    ctx.world.mocap_relay.state.marker_cloud = cloud
    return cloud


def bar_markers(bar: str, seed: int = 0) -> np.ndarray:
    """A bar's markers slid 30 mm and rolled 2° from its assembly pose, with noise; the fake's stray left out."""
    layout = bar_layouts(purged_process().process)[bar]
    sensed = fake_sensed_markers(seat(bar), layout.points, seed=seed)
    expected = transform_points(seat(bar) @ bar_slide_roll(0.03, 2.0), layout.points)
    near = np.linalg.norm(sensed.points[:, None] - expected[None], axis=2).min(axis=1) < 0.005
    return sensed.points[near]


def seat(bar: str) -> np.ndarray:
    """A bar's assembly pose, 4x4."""
    return purged_process().process.bodies[bar].assembly_pose.matrix()


def test_every_bar_with_a_layout_is_a_hidden_scene_body_that_never_collides():
    """B20, B21, B34 and B35 as "mocap_bars/<B>": visual shapes only, hidden before a fit, no warning; no asset."""
    plugin, ctx = bench()
    assert list(plugin._bars) == list(BARS)
    bodies = {body_id: body for body_id, body in ctx.scene.bodies.items()}
    assert sorted(bodies) == ["mocap_bars/B20", "mocap_bars/B21", "mocap_bars/B34", "mocap_bars/B35"]
    assert all(body.geometry.visual and not body.geometry.collision and not body.enabled for body in bodies.values())
    assert not ctx.world.tracked_objects and not any("collide" in line for line in ctx.lines("warn"))
    assert plugin._bar.options == list(BARS) and plugin._bar_id == BAR
    assert plugin._stats_rows == 3 + 8 + 7
    assert any("fitting B20, B21, B34, B35 to the raw markers within 50 mm" in line for line in ctx.lines("info"))


def test_fits_a_streaming_bar_at_most_every_tenth_second_then_around_the_last_fit():
    """The first cloud gets the full search (30 mm off), within 0.1 s none, then the match around the last fit.

    The body follows the fit.
    """
    plugin, ctx = bench()
    placed(ctx, BAR)
    layout = bar_layouts(purged_process().process)[BAR]
    T = seat(BAR)
    start = ctx.t
    stream(ctx, bar_markers(BAR))
    plugin.update(ctx)
    plugin.draw(ctx)
    state = plugin._bars[BAR]
    first = state.fit
    assert first.ok and not first.tracked
    assert "fit 8/8" in plugin._readout()[1] and "fit 8/8" in plugin._status.content
    assert plugin._readout()[0] == "fitted 1 of 4: B20"
    # * The chip has its own handle, so the refits leave its hover alone.
    assert ">markers<" in plugin._status_chip.content and "fit 8/8" not in plugin._status_chip.content
    # * The fitted pose is the slid, rolled one; each CAD marker got its marker.
    expected = T @ bar_slide_roll(0.03, 2.0)
    np.testing.assert_allclose(first.T_world_from_bar[:3, 3], expected[:3, 3], atol=2e-3)
    rows = plugin._matched(state).reshape(-1, 3)
    np.testing.assert_allclose(rows, transform_points(expected, layout.points), atol=3e-3)
    assert plugin._fit_lines.points.shape == (9, 2, 3) and plugin._measured_cloud.points.shape == (8, 3)
    assert plugin.fits[BAR].fit is first and plugin.fits[BAR].pose == state.pose and ok_fits(plugin) == [BAR]
    body = ctx.scene.bodies["mocap_bars/B20"]
    assert body.enabled and body.placement == state.pose and not ctx.scene.bodies["mocap_bars/B21"].enabled

    ctx.t = start + FIT_PERIOD / 2
    stream(ctx, bar_markers(BAR, seed=1))
    plugin.update(ctx)
    assert state.fit is first
    ctx.t = start + FIT_PERIOD
    stream(ctx, bar_markers(BAR, seed=2))
    plugin.update(ctx)
    assert state.fit is not first and state.fit.ok and state.fit.tracked


def test_only_markers_near_where_the_bar_should_be_count():
    """Markers far from B20's seat (the rack, other robots) are never fitted; B20 moved away is not found at all."""
    plugin, ctx = bench()
    placed(ctx, BAR)
    far = seat(BAR)[:3, 3] + np.random.default_rng(0).uniform(0.3, 2.0, (40, 3))
    stream(ctx, bar_markers(BAR), far)
    plugin.update(ctx)
    state = plugin._bars[BAR]
    assert state.fit.ok and len(state.sensed.points) == 8 and state.sensed.source == "cloud"

    ctx.t += SEARCH_RETRY + FIT_PERIOD
    stream(ctx, bar_markers(BAR) + np.array([0.0, 0.5, 0.0]))
    plugin.update(ctx)
    assert not state.fit.ok and len(state.sensed.points) == 0 and state.pose is None
    assert "within 50 mm of where the bar should be" in state.fit.reason
    assert not ctx.scene.bodies["mocap_bars/B20"].enabled


def test_a_held_bar_is_looked_for_where_the_live_scene_has_it():
    """While the process places B20 elsewhere (on a flange), its markers are looked for there, not at its seat."""
    plugin, ctx = bench()
    held = seat(BAR).copy()
    held[:3, 3] += [0.5, 0.0, 0.3]
    ctx.plugins["process"].selection.ids = {BAR: "process/bars/B20"}
    ctx.tick_scene.bodies["process/bars/B20"] = Body("process/bars/B20", Geometry((), ()), Pose())
    ctx.tick_scene.world_poses["process/bars/B20"] = Pose.from_matrix(held)
    layout = bar_layouts(purged_process().process)[BAR]
    stream(ctx, transform_points(held, layout.points), bar_markers(BAR))
    plugin.update(ctx)
    state = plugin._bars[BAR]
    assert state.fit.ok and state.fit.tracked and len(state.sensed.points) == 8
    np.testing.assert_allclose(state.pose.matrix(), held, atol=1e-6)
    assert state.deviation.dpos_mm < 1e-3


def placed(ctx: FakeContext, *bars: str) -> None:
    """The live scene shows these bars at their assembly poses (built)."""
    ctx.plugins["process"].selection.ids = {bar: f"process/{bar}" for bar in bars}
    for bar in bars:
        ctx.tick_scene.bodies[f"process/{bar}"] = Body(f"process/{bar}", Geometry((), ()), Pose())
        ctx.tick_scene.world_poses[f"process/{bar}"] = Pose.from_matrix(seat(bar))


def test_a_bar_with_a_hidden_marker_is_followed_by_the_quick_match_and_stays_shown():
    """Once found, a bar with one ball hidden is matched around its last fit every 0.1 s (7 of 8), no full search."""
    plugin, ctx = bench()
    placed(ctx, *BARS)
    stream(ctx, *(bar_markers(bar) for bar in BARS))
    for _ in range(len(BARS)):
        ctx.t += 0.05
        stream(ctx, *(bar_markers(bar) for bar in BARS))
        plugin.update(ctx)
    assert all(plugin._bars[bar].fit.ok for bar in BARS)
    searches = []
    original = plugin._search_one
    plugin._search_one = lambda *args: searches.append(args) or original(*args)
    for step in range(20):
        ctx.t += 0.05
        stream(ctx, *(bar_markers(bar, seed=step)[1:] for bar in BARS))
        plugin.update(ctx)
        assert all(ctx.scene.bodies[f"mocap_bars/{bar[5:]}"].enabled for bar in BARS)
    assert not searches
    assert all(plugin._bars[bar].fit.tracked and plugin._bars[bar].fit.n_matched == 7 for bar in BARS)


def test_a_bar_the_live_scene_lacks_never_gets_the_full_search():
    """B20 not built yet, its markers 30 mm off its seat: matched only around the seat (every marker), not searched.

    The live scene has it, disabled at its seat, as the process plugin's `scene_bodies` puts every bar there.
    """
    plugin, ctx = bench()
    placed(ctx, BAR)
    ctx.tick_scene.bodies[f"process/{BAR}"] = Body(f"process/{BAR}", Geometry((), ()), Pose(), enabled=False)
    searches = []
    original = plugin._search_one
    plugin._search_one = lambda *args: searches.append(args) or original(*args)
    stream(ctx, bar_markers(BAR))
    plugin.update(ctx)
    state = plugin._bars[BAR]
    assert not searches and not state.fit.ok and state.fit.reason.startswith("not shown in the step's scene")
    stream(ctx, transform_points(seat(BAR), bar_layouts(purged_process().process)[BAR].points))
    ctx.t += SEARCH_RETRY
    stream(ctx, transform_points(seat(BAR), bar_layouts(purged_process().process)[BAR].points))
    plugin.update(ctx)
    assert state.fit.ok and state.fit.tracked, "on its seat it is found by the quick match"


def test_one_full_search_per_tick_round_robin():
    """Two bars of the live scene off their seats at once: one is searched this tick, the other the next."""
    plugin, ctx = bench()
    placed(ctx, *BARS)
    stream(ctx, bar_markers("bars/B20"), bar_markers("bars/B21"))
    plugin.update(ctx)
    fitted = [bar for bar in BARS if plugin._bars[bar].fit is not None and plugin._bars[bar].fit.ok]
    assert fitted == ["bars/B20"]
    assert plugin._bars["bars/B21"].fit is None, "the other bar keeps its last fit (none yet) while it waits"
    ctx.t += 0.02
    stream(ctx, bar_markers("bars/B20"), bar_markers("bars/B21"))
    plugin.update(ctx)
    assert plugin._bars["bars/B21"].fit.ok and not plugin._bars["bars/B21"].fit.tracked
    assert plugin._bars["bars/B20"].fit.ok
    assert ok_fits(plugin) == ["bars/B20", "bars/B21"]
    assert sorted(plugin.fits) == list(BARS), "the bars not there have their failed fit"


def test_a_failed_search_waits_before_the_next_and_shows_why():
    """Two markers near B20: no fit, its reason on the status line and in the stats; no search for SEARCH_RETRY."""
    plugin, ctx = bench()
    placed(ctx, BAR)
    first = stream(ctx, bar_markers(BAR)[:2])
    plugin.update(ctx)
    plugin.draw(ctx)
    state = plugin._bars[BAR]
    assert state.fit is not None and not state.fit.ok and state.pose is None
    assert plugin._readout()[1].startswith("B20: 2 markers near; only 2 markers in view (need at least 3)")
    assert "FAILED: only 2 markers in view" in plugin._stats.content
    assert not plugin._fit_cloud.visible and not ctx.scene.bodies["mocap_bars/B20"].enabled
    ctx.t += SEARCH_RETRY / 2
    stream(ctx, bar_markers(BAR)[:2])
    plugin.update(ctx)
    assert state.fitted_cloud is first, "no new search within SEARCH_RETRY"
    ctx.t += SEARCH_RETRY
    latest = stream(ctx, bar_markers(BAR)[:2])
    plugin.update(ctx)
    assert state.fitted_cloud is latest


def test_a_bar_hides_once_its_markers_are_old_and_the_chip_says_so():
    """No new cloud for over MAX_MARKER_AGE: the body hides, the chip turns "markers stale"; none: "no markers"."""
    plugin, ctx = bench()
    placed(ctx, BAR)
    plugin.draw(ctx)
    assert ">no markers<" in plugin._status_chip.content
    stream(ctx, bar_markers(BAR))
    plugin.update(ctx)
    assert ctx.scene.bodies["mocap_bars/B20"].enabled
    ctx.t += MAX_MARKER_AGE + 0.1
    plugin.update(ctx)
    plugin.draw(ctx)
    assert not ctx.scene.bodies["mocap_bars/B20"].enabled and ">markers stale<" in plugin._status_chip.content


def test_raw_markers_draws_the_whole_cloud():
    """The "Raw markers" box shows every marker of the newest cloud; off, none."""
    plugin, ctx = bench()
    far = seat(BAR)[:3, 3] + np.random.default_rng(0).uniform(0.3, 2.0, (40, 3))
    stream(ctx, bar_markers(BAR), far)
    plugin.update(ctx)
    plugin.draw(ctx)
    assert not plugin._cloud_points.visible
    plugin._raw_markers.value = True
    plugin.draw(ctx)
    assert plugin._cloud_points.visible and plugin._cloud_points.points.shape == (48, 3)


def test_the_stats_block_has_fixed_rows_and_lists_markers_and_plates():
    """A fitted B20: a summary, eight marker rows, the strays, four plates, three spacings, the deviation."""
    plugin, ctx = bench()
    placed(ctx, BAR)
    stream(ctx, bar_markers(BAR))
    plugin.update(ctx)
    plugin.draw(ctx)
    rows = [row for row in plugin._stats.content.split("<div>")[1:]]
    assert len(rows) == plugin._stats_rows == 18
    text = plugin._stats.content
    assert "source cloud  age 0 ms  fit 8/8  rms " in text and "ambiguous no  dropped 0  occluded 0" in text
    assert text.count("matched") == 8 and "stray sensed 0" in text
    assert text.count("pair 48.") == 4 and text.count(" → ") == 3
    assert "vs planned  dpos 30." in text and "dz(along) +30." in text
    assert plugin._stats.content.count("—") == 0


def test_measure_and_log_prints_three_mocap_bar_lines_and_log_live_every_five_seconds():
    """The button prints the report at once; "Log live" repeats it every LOG_PERIOD, not in between."""
    plugin, ctx = bench()
    placed(ctx, BAR)
    plugin.update(ctx)
    stream(ctx, bar_markers(BAR))
    plugin._measure_and_log(ctx)
    lines = [line for line in ctx.lines("info") if line.startswith("[mocap bar] ")]
    assert len(lines) == 3
    assert lines[0].startswith("[mocap bar] bars/B20 | source cloud (age 0 ms) | 8/8 matched | rms ")
    assert lines[1].startswith("[mocap bar] residuals (mm): J20-") and lines[1].endswith("| stray sensed: 0")
    assert lines[2].startswith("[mocap bar] bar ends (world, mm): Rhino start [") and "| MoCap start [" in lines[2]
    plugin._log_live.value = True
    plugin.update(ctx)
    assert sum(line.startswith("[mocap bar] bars/B20") for line in ctx.lines("info")) == 2
    ctx.t += LOG_PERIOD / 2
    plugin.update(ctx)
    assert sum(line.startswith("[mocap bar] bars/B20") for line in ctx.lines("info")) == 2
    ctx.t += LOG_PERIOD
    plugin.update(ctx)
    assert sum(line.startswith("[mocap bar] bars/B20") for line in ctx.lines("info")) == 3


def test_measure_now_fits_every_bar_afresh_for_the_contract():
    """`measure_now` fits B20 around its hint and B21 by the full search; every bar comes back, B34 unfitted."""
    plugin, ctx = bench()
    assert plugin.measure_now() == {}, "no cloud yet"
    placed(ctx, BAR, "bars/B21")
    plugin.update(ctx)
    stream(ctx, bar_markers("bars/B20"), bar_markers("bars/B21"))
    hint = Pose.from_matrix(seat(BAR) @ bar_slide_roll(0.03, 2.0))  # where the fake puts the bar
    found = plugin.measure_now({BAR: hint})
    assert sorted(found) == list(BARS)
    assert found[BAR].fit.ok and found[BAR].tracked and found[BAR].deviation is None
    assert found["bars/B21"].fit.ok and not found["bars/B21"].tracked and found["bars/B21"].source == "cloud"
    assert not found["bars/B34"].fit.ok and found["bars/B34"].pose is None
    record = found[BAR].to_json()
    assert record["fit"]["ok"] and len(record["pose"]) == 7 and record["layout"]["length"] == layouts_length()
    assert "mocap_id" not in record
    assert plugin.fits[BAR].fit is found[BAR].fit


def ok_fits(plugin: MocapBarsPlugin) -> list[str]:
    """The bars whose latest fit is ok."""
    return [bar for bar, found in plugin.fits.items() if found.fit.ok]


def layouts_length() -> float:
    """B20's length."""
    return bar_layouts(purged_process().process)[BAR].length


def test_deviation_arrows_point_from_planned_to_fitted_magnified_and_label_true_values():
    """2 mm across and 1° about the bar: three gap arrows 40 mm long, true labels, a purple arc about z only."""
    planned = seat(BAR)
    c, s = np.cos(np.radians(1.0)), np.sin(np.radians(1.0))
    nudge = np.array([[c, -s, 0.0, 0.002], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    fitted = planned @ nudge
    arrows = deviation_arrows(planned, fitted, 1.0, magnify=20, name="B20")
    texts = [text for text, _ in arrows.labels]
    assert texts[:3] == ["2.00 mm", "B20 2.00 mm", "2.00 mm"]
    assert texts[3:] == ["about X: +0.00°", "about Y: +0.00°", "about Z: +1.00°"]
    gaps = [segment for segment, color in zip(arrows.segments, arrows.colors) if color == GAP_COLOR]
    # * A shaft and two head strokes per gap; the shaft is 20 times the 2 mm.
    assert len(gaps) == 9 and np.isclose(np.linalg.norm(gaps[0][1] - gaps[0][0]), 0.04)
    assert np.allclose((gaps[0][1] - gaps[0][0]) / 0.04, planned[:3, 0])
    turns = [segment for segment, color in zip(arrows.segments, arrows.colors) if color == TURN_COLOR]
    assert len(turns) > 2
    # * The arc sweeps 20° about the planned z axis, from the planned y axis.
    centre = planned[:3, 3] + planned[:3, 2] * (-0.5 + 0.15 * 0.8)
    end = turns[-3][1] - centre
    assert np.isclose(np.degrees(np.arctan2(end @ -planned[:3, 0], end @ planned[:3, 1])), 20.0, atol=0.5)
    still = deviation_arrows(planned, planned, 1.0)
    assert [text for text, _ in still.labels][:3] == ["0.00 mm"] * 3 and TURN_COLOR not in still.colors


def test_arrows_show_the_chosen_bar_or_every_bar_or_none():
    """Fitted, placed bars get arrows and labels; "all bars" names each middle gap; "off" hides them."""
    plugin, ctx = bench()
    placed(ctx, *BARS)
    for _ in range(len(BARS) + 1):
        ctx.t += 0.05
        stream(ctx, *(bar_markers(bar) for bar in BARS))
        plugin.update(ctx)
    plugin.draw(ctx)
    assert plugin._arrows.value == ARROW_MODES[1] and plugin._arrow_lines.visible
    shown = [label.text for label in plugin._arrow_labels if label.visible]
    # * Slid 30 mm along the bar: every gap reads 30 mm, give or take the noise.
    assert all(text.startswith("30.0") for text in shown[:3]) and len(shown) == 6
    plugin._arrows.value = ARROW_MODES[2]
    plugin.draw(ctx)
    shown = [label.text for label in plugin._arrow_labels if label.visible]
    named = [text.split()[0] for text in shown if text.startswith("B")]
    assert len(shown) == 6 * len(BARS) and named == ["B20", "B21", "B34", "B35"]
    plugin._arrows.value = ARROW_MODES[0]
    plugin.draw(ctx)
    assert not plugin._arrow_lines.visible and not any(label.visible for label in plugin._arrow_labels)


def test_the_fit_settings_change_the_search_radius_and_timings():
    """A 10 mm search misses a bar slid 30 mm; the marker age slider hides a bar sooner."""
    plugin, ctx = bench()
    placed(ctx, BAR)
    plugin._radius_mm.value = 10
    stream(ctx, bar_markers(BAR))
    plugin.update(ctx)
    state = plugin._bars[BAR]
    assert not state.fit.ok and "within 10 mm" in state.fit.reason
    plugin._radius_mm.value = 50
    ctx.t += SEARCH_RETRY
    stream(ctx, bar_markers(BAR))
    plugin.update(ctx)
    assert state.fit.ok and ctx.scene.bodies["mocap_bars/B20"].enabled
    plugin._marker_age.value = 0.1
    ctx.t += 0.2
    plugin.update(ctx)
    assert not ctx.scene.bodies["mocap_bars/B20"].enabled
