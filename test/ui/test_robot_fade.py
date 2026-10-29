"""The runtime's see-through robots: a plugin fades a whole robot, over the parts the checks only assume."""

from __future__ import annotations

import pytest
from repo_paths import DATA

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.ui.visualization import GUESSED_OPACITY, Visualization
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

SERIAL = "a200-0804"


@pytest.fixture
def viz():
    """Both servers on free ports, with Alice loaded."""
    viz = Visualization(port=0, phone_port=0)
    viz.load_robot(robot_config_from_serial("0804", DATA))
    yield viz
    viz.stop()


def opacities(viz: Visualization) -> dict[str | None, set[float | None]]:
    """The opacities Alice's meshes are at, by part."""
    return {part: {handle.opacity for handle in handles} for part, handles in viz._robots[SERIAL].parts.items()}


def test_a_faded_robot_is_all_at_its_opacity_and_comes_back_to_its_guessed_parts(viz):
    """The arm not measured: 0.3 fades every part; None puts the arm back at GUESSED_OPACITY and the base solid."""
    model = robot_config_from_serial("0804", DATA).model
    robot = RobotObject(f"robots/{SERIAL}", model, Pose(), {}, unmeasured=frozenset({"ur_arm_elbow_joint"}))
    viz.draw(Scene(robots={robot.id: robot}))
    assert opacities(viz) == {None: {None}, "ur_arm": {GUESSED_OPACITY}}
    viz.set_robot_opacity(SERIAL, 0.3)
    assert opacities(viz) == {None: {0.3}, "ur_arm": {0.3}}
    viz.set_robot_opacity(SERIAL, None)
    assert opacities(viz) == {None: {None}, "ur_arm": {GUESSED_OPACITY}}
    viz.set_robot_opacity("a200-9999", 0.3)  # unknown: ignored


def test_a_plugin_view_puts_back_what_it_faded_when_cleared(viz):
    """A plugin's view fades a robot; `clear` (the plugin stopping) draws it as measured again."""
    view = viz.view_for("a_plugin")
    view.set_robot_opacity(SERIAL, 0.3)
    assert viz._robots[SERIAL].opacity == 0.3
    view.clear()
    assert viz._robots[SERIAL].opacity is None
