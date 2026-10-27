"""The runtime's see-through robots: a fade hides the textured robot and shows a single-colour twin at its opacity."""

from __future__ import annotations

import pytest
from repo_paths import DATA

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.ui.visualization import Visualization

SERIAL = "a200-0804"


@pytest.fixture
def viz():
    """Both servers on free ports."""
    viz = Visualization(port=0, phone_port=0)
    yield viz
    viz.stop()


def test_a_faded_robot_shows_its_twin_and_comes_back_solid(viz):
    """0.3 hides the GLB visual meshes and shows a twin at 0.3; None hides the twin and shows the GLB again."""
    viz.load_robot(robot_config_from_serial("0804", DATA))
    drawn = viz._robots[SERIAL]
    assert drawn.twin is None and drawn.urdf.show_visual
    viz.set_robot_opacity(SERIAL, 0.3)
    assert drawn.twin is not None and drawn.opacity == 0.3 and not drawn.urdf.show_visual
    assert drawn.twin.show_visual and all(mesh.opacity == 0.3 for mesh in drawn.twin._meshes)
    viz.set_robot_opacity(SERIAL, None)
    assert drawn.opacity is None and drawn.urdf.show_visual and not drawn.twin.show_visual
    viz.set_robot_opacity("a200-9999", 0.3)  # unknown: ignored


def test_a_plugin_view_puts_back_what_it_faded_when_cleared(viz):
    """A plugin's view fades a robot; `clear` (the plugin stopping) draws it solid again."""
    viz.load_robot(robot_config_from_serial("0804", DATA))
    view = viz.view_for("a_plugin")
    view.set_robot_opacity(SERIAL, 0.3)
    assert viz._robots[SERIAL].opacity == 0.3
    view.clear()
    assert viz._robots[SERIAL].opacity is None
