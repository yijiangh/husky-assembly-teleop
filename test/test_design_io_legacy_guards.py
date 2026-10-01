"""Tests for the legacy converter's refusals: data a design cannot carry stops the conversion instead of being dropped."""

from types import SimpleNamespace

import pytest
from compas.geometry import Frame
from compas_robots import ToolModel
from compas_robots.model import Joint

from husky_assembly_teleop.design_io.legacy import _model_geometry, _refuse_unsupported


def _movement(trajectory=None, **tool) -> SimpleNamespace:
    """A movement with one AT3L tool state; `tool` overrides its fields."""
    state = dict(is_hidden=False, configuration=None, attachment_frame=Frame.worldXY())
    state.update(tool)
    tools = {"AT3L": SimpleNamespace(**state), "ObstacleRobotAlice": SimpleNamespace(is_hidden=True)}
    return SimpleNamespace(movement_id="B1__J_M2", trajectory=trajectory,
                           start_state=SimpleNamespace(tool_states=tools))


def test_supported_movement_passes():
    """Identity or missing attachment frames, no configuration, no trajectory: accepted. Other robots are ignored."""
    _refuse_unsupported(_movement())
    _refuse_unsupported(_movement(attachment_frame=None))


@pytest.mark.parametrize("change, message", [
    (dict(trajectory=object()), "trajectory"),
    (dict(is_hidden=True), "hidden"),
    (dict(configuration=object()), "configuration"),
    (dict(attachment_frame=Frame([0.0, 0.0, 0.01], [1, 0, 0], [0, 1, 0])), "off its flange"),
])
def test_unsupported_movement_refused(change, message):
    """Each kind of data a design has no place for raises, naming the movement."""
    with pytest.raises(ValueError, match=f"B1__J_M2.*{message}"):
        _refuse_unsupported(_movement(**change))


def test_tool_with_moving_joint_refused():
    """A tool with a revolute joint is not flattened into one shape."""
    tool = ToolModel(None, Frame.worldXY(), name="Gripper")
    base = tool.add_link("base")
    finger = tool.add_link("finger")
    tool.add_joint("finger_joint", Joint.REVOLUTE, base, finger, limit=(-1.0, 1.0))
    with pytest.raises(ValueError, match="Gripper has moving joints"):
        _model_geometry(tool)
