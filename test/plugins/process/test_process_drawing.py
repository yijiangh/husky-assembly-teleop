"""The process drawing's Rhino bar lines: every bar's centre line and name at its assembly pose, pink, on a switch."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from plugin_fakes import FakeScene
from process_fixtures import build_process

from husky_assembly_teleop.plugins.process.drawing import ProcessDrawing, bar_lines


def test_every_bar_gets_a_line_of_its_length_through_its_assembly_pose_and_a_label(tmp_path):
    """Two bars: two segments, each the bar's length, centred on its assembly pose; two labels with the bar names."""
    process = build_process(tmp_path)
    segments, labels = bar_lines(process)
    bars = [(body_id, body) for body_id, body in process.bodies.items() if body_id.startswith("bars/")]
    assert len(segments) == len(labels) == len(bars) == 2
    for (body_id, body), segment, (name, middle) in zip(bars, segments, labels):
        assert name == body_id[len("bars/"):]
        np.testing.assert_allclose(segment.mean(axis=0), body.assembly_pose.position, atol=1e-9)
        np.testing.assert_allclose(middle, body.assembly_pose.position, atol=1e-9)
        assert np.linalg.norm(segment[1] - segment[0]) == pytest_approx(body.geometry.collision[0].height)


def pytest_approx(value: float, rel: float = 1e-9) -> object:
    """pytest.approx, without importing pytest at module level in a helper."""
    import pytest
    return pytest.approx(value, rel=rel)


def test_the_drawing_builds_the_lines_hidden_and_the_switch_shows_them(tmp_path):
    """After `build`, one line node and a label per bar, hidden; `set_bar_lines(True)` shows all of them."""
    process = build_process(tmp_path)
    scene = FakeScene()
    drawing = ProcessDrawing(SimpleNamespace(scene=scene), "/process", process, models={})
    list(drawing.build())
    lines, = [node for node in scene.of_kind("line_segments") if node.name.endswith("/bar_lines/lines")]
    labels = [node for node in scene.of_kind("label") if "/bar_lines/" in node.name]
    assert lines.points.shape == (2, 2, 3) and len(labels) == 2 and not lines.visible
    assert all(not label.visible for label in labels)
    drawing.set_bar_lines(True)
    assert lines.visible and all(label.visible for label in labels)
    drawing.set_bar_lines(False)
    assert not lines.visible
