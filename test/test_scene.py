"""Tests for scene.PluginScene: owner checks, and the warning for bodies that never collide."""

import pytest

from bar_assembly_core.geometry import Geometry, box_geometry
from bar_assembly_core.scene import Body, Scene
from husky_assembly_teleop.world.scene import PluginScene
from bar_assembly_core.geometry import Pose


def test_only_own_ids():
    """A plugin may put and remove only ids under its own name."""
    scene = PluginScene(Scene(), "cell", lambda _message: None, Scene)
    scene.put(Body("cell/bars/B1", box_geometry((1.0, 0.1, 0.1)), Pose()))
    with pytest.raises(ValueError):
        scene.put(Body("obstacles/B1", box_geometry((1.0, 0.1, 0.1)), Pose()))
    with pytest.raises(ValueError):
        scene.remove("obstacles/B1")


def test_body_without_collision_meshes_warns_once():
    """A body that is drawn but never collides is reported, once per id."""
    warnings: list[str] = []
    scene = PluginScene(Scene(), "cell", warnings.append, Scene)
    visual_only = Geometry(box_geometry((1.0, 1.0, 1.0)).visual, ())
    for _ in range(2):
        scene.put(Body("cell/ghost", visual_only, Pose()))
    scene.put(Body("cell/solid", box_geometry((1.0, 1.0, 1.0)), Pose()))
    assert len(warnings) == 1 and "cell/ghost" in warnings[0]
