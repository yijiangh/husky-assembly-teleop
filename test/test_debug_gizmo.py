"""The debug gizmo keeps its temporary obstacles in a file, and puts them back at the next start."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.scene import Scene
from husky_assembly_teleop.plugins import debug_gizmo
from husky_assembly_teleop.plugins.debug_gizmo import DebugGizmoPlugin
from husky_assembly_teleop.world.scene import PluginScene


@pytest.fixture
def save_file(tmp_path, monkeypatch):
    """A save file in a temporary folder, so the tests never touch the real one."""
    path = tmp_path / "gizmo.json"
    monkeypatch.setattr(debug_gizmo, "SAVE_FILE", path)
    return path


def _context() -> SimpleNamespace:
    """The part of a plugin context that saving and loading use, with an empty scene."""
    warnings: list[str] = []
    scene = PluginScene(Scene(), DebugGizmoPlugin.name, log_warn=warnings.append, snapshot=Scene)
    return SimpleNamespace(scene=scene, log_warn=warnings.append, warnings=warnings)


def test_obstacles_are_back_after_a_restart(save_file):
    """Kind, number, size and pose survive; a new obstacle does not take a saved one's number."""
    first, ctx = DebugGizmoPlugin(), _context()
    first._put_tmp(ctx, "box", 1, (0.5, 0.4, 1.0), Pose((1.0, 2.0, 0.5)))
    moved = Pose((3.0, -1.0, 0.6), (0.0, 0.0, 0.7071068, 0.7071068))
    first._put_tmp(ctx, "cylinder", 4, (0.3, 0.3, 1.2), moved)
    first._save(ctx)

    second, restarted = DebugGizmoPlugin(), _context()
    second._load(restarted)
    assert second._tmp == first._tmp
    assert restarted.scene.bodies["debug_gizmo/tmp/cylinder_4"].placement == moved
    assert restarted.scene.bodies["debug_gizmo/tmp/box_1"].label == "tmp box 1"
    assert second._tmp_count == 4 and not restarted.warnings


def test_saved_only_when_changed(save_file):
    """Nothing is written until an obstacle exists, and a removed one leaves the file."""
    plugin, ctx = DebugGizmoPlugin(), _context()
    plugin._save(ctx)
    assert not save_file.exists()
    body_id = plugin._put_tmp(ctx, "box", 1, (0.5, 0.5, 1.0), Pose())
    plugin._save(ctx)
    written = save_file.stat().st_mtime_ns
    plugin._save(ctx)
    assert save_file.stat().st_mtime_ns == written
    del plugin._tmp[body_id]
    plugin._save(ctx)
    restarted = DebugGizmoPlugin()
    restarted._load(_context())
    assert restarted._tmp == {}


def test_broken_file_is_logged_and_kept(save_file):
    """A file that cannot be read starts the plugin empty with a warning, and is not overwritten."""
    save_file.write_text("{not json")
    plugin, ctx = DebugGizmoPlugin(), _context()
    plugin._load(ctx)
    plugin._save(ctx)
    assert plugin._tmp == {} and len(ctx.warnings) == 1
    assert save_file.read_text() == "{not json"
