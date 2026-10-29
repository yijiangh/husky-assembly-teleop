"""The mocap probe's coverage map on a real viser server: walk the probe, see the map on the floor, export and load."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np

from test_floor_calibration_plugin import NOW, make_ctx, server  # noqa: F401 (the module's viser server fixture)
from husky_assembly_teleop.plugins.mocap_probe import EXPORT_FOLDER, MAP_PERIOD, MocapProbePlugin
from husky_assembly_teleop.ui.top_down import render_top_down
from husky_assembly_teleop.world.coverage_map import COVERAGE_BIN, COVERAGE_ORIGIN, COVERAGE_SIZE


def walk(plugin: MocapProbePlugin, ctx: SimpleNamespace, points: np.ndarray, start: float) -> float:
    """Tick once per point with the probe there, 20 ticks a second from `start`; returns the time after."""
    time = start
    for point in points:
        ctx.probe.position, ctx.probe.last_update_time = np.array([*point, 1.0]), time
        ctx.now = lambda now=time: now
        plugin.update(ctx)
        plugin.draw(ctx)
        time += 0.05
    return time


def test_walk_maps_the_floor_then_export_and_load(server, tmp_path):  # noqa: F811
    """A walk along y = 1 m shows in the image's row for that y; the export loads back with every sample."""
    (tmp_path / EXPORT_FOLDER).mkdir(parents=True)
    ctx = make_ctx(server, tmp_path)
    ctx.spawn = lambda _name, coroutine, **_options: coroutine.close()
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin._set_mapping(ctx, True)
    time = walk(plugin, ctx, np.column_stack([np.linspace(-2.0, 2.0, 80), np.full(80, 1.0)]), NOW)
    ctx.now = lambda: time + MAP_PERIOD
    plugin.draw(ctx)

    image = plugin._coverage_image.image
    row = int((1.0 - COVERAGE_ORIGIN[1]) / COVERAGE_BIN)
    column = int((0.0 - COVERAGE_ORIGIN[0]) / COVERAGE_BIN)
    assert image.shape == (round(COVERAGE_SIZE[1] / COVERAGE_BIN), round(COVERAGE_SIZE[0] / COVERAGE_BIN), 4)
    assert image[row, column, 3] > 0 and image[column, row, 3] == 0
    assert "80 samples" in plugin._coverage_status.content and "here tracked 100%" in plugin._coverage_status.content
    assert render_top_down(*plugin._map_inputs(None)).shape == (480, 480, 3)

    plugin._export_coverage(ctx)
    exported = next((tmp_path / EXPORT_FOLDER).glob("*mocap_coverage.json"))
    plugin._clear_coverage(ctx)
    assert plugin.coverage.count == 0
    plugin._load_coverage(ctx, SimpleNamespace(name=exported.name, content=Path(exported).read_bytes()))
    assert plugin.coverage.count == 80
    plugin.draw(ctx)
    assert plugin._coverage_image.image[row, column, 3] > 0
