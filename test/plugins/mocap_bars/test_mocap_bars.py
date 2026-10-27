"""The mocap_bars plugin fits B20's streaming markers with the bar fit: ten times a second at most, hint-guided."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from husky_assembly_teleop.plugins.mocap_bars import FIT_PERIOD, MocapBarsPlugin
from husky_assembly_teleop.robot_interface.mocap_state import RawMarkers
from husky_assembly_teleop.world.bar_pose import bar_slide_roll, transform_points
from husky_assembly_teleop.world.marker_layouts import bar_layouts, fake_sensed_markers
from husky_assembly_teleop.world.measured import TrackedObject
from planning_fixtures import purged_process
from plugin_fakes import FakeContext, FakeScene

BAR = "bars/B20"


def bench() -> tuple[MocapBarsPlugin, FakeContext, TrackedObject]:
    """The plugin set up on the purged process, tracking B20's asset (no message yet)."""
    loaded = purged_process()
    selection = SimpleNamespace(process=loaded.process, ids=None, movement=None)
    ctx = FakeContext(plugins={"process": SimpleNamespace(selection=selection)}, name="mocap_bars")
    ctx.view.scene = FakeScene()
    obj = TrackedObject("fitted_bar", 1020)
    # ? FakeContext tracks nothing until task F adds it: the two calls the plugin makes.
    ctx.track_object = lambda name, mocap_id, label="": obj
    ctx.untrack_object = lambda name: None
    plugin = MocapBarsPlugin()
    plugin.setup(ctx)
    plugin._mocap_id.value = 1020
    plugin._track(ctx)
    return plugin, ctx, obj


def stream(obj: TrackedObject, T: np.ndarray, points: np.ndarray, t: float, seed: int = 0) -> None:
    """A message of B20's asset at pose T (slid 30 mm, rolled 2°, one stray marker), as the relay stores it."""
    sensed = fake_sensed_markers(T, points, seed=seed)
    count = len(sensed.points)
    obj.markers = RawMarkers(np.array(sensed.ids), sensed.points, np.ones(count, bool), sensed.residuals,
                             np.full(count, 0.019), capture_time=t)
    obj.last_update_time = t


def test_fits_a_streaming_bar_at_most_every_tenth_second_then_around_the_last_fit():
    """The first message gets the full search; the next within 0.1 s waits; then the match around the last fit."""
    plugin, ctx, obj = bench()
    layout = bar_layouts(purged_process().process)[BAR]
    T = purged_process().process.bodies[BAR].assembly_pose.matrix()
    start = ctx.t
    stream(obj, T, layout.points, start)
    plugin.update(ctx)
    plugin.draw(ctx)
    first = plugin._fit
    assert first.ok and not first.tracked
    assert plugin._readout()[1].startswith("fit 8/8") and "fit 8/8" in plugin._status.content
    # * The fitted pose is the slid, rolled one; each CAD marker got its fake marker, the stray none.
    expected = T @ bar_slide_roll(0.03, 2.0)
    np.testing.assert_allclose(first.T_world_from_bar[:3, 3], expected[:3, 3], atol=2e-3)
    rows = plugin._matched(len(layout.points)).reshape(-1, 3)
    np.testing.assert_allclose(rows, transform_points(expected, layout.points), atol=3e-3)
    assert plugin._bar_frame.visible and plugin._fit_lines.points.shape == (9, 2, 3)
    assert plugin._measured_cloud.points.shape == (9, 3)

    ctx.t = start + FIT_PERIOD / 2
    stream(obj, T, layout.points, ctx.t, seed=1)
    plugin.update(ctx)
    assert plugin._fit is first
    ctx.t = start + FIT_PERIOD
    stream(obj, T, layout.points, ctx.t, seed=2)
    plugin.update(ctx)
    assert plugin._fit is not first and plugin._fit.ok and plugin._fit.tracked


def test_a_failed_fit_shows_why_and_hides_the_bar():
    """Two markers in view: no fit, the fit's reason on the status line, nothing fitted drawn."""
    plugin, ctx, obj = bench()
    obj.markers = RawMarkers(np.array([1, 2]), np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]]), np.ones(2, bool),
                             np.full(2, 0.0005), np.full(2, 0.019), capture_time=ctx.t)
    obj.last_update_time = ctx.t
    plugin.update(ctx)
    plugin.draw(ctx)
    assert plugin._fit is None and plugin._pose is None
    assert plugin._readout()[1] == f"{BAR}: only 2 markers in view (need at least 3)"
    assert not plugin._bar_frame.visible and not plugin._fit_cloud.visible
