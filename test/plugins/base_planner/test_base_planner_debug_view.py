"""Tests for the base planner's debug view: a search's record drawn on the floor and listed in the panel."""

from types import SimpleNamespace

import pytest
import viser

from husky_assembly_teleop.plugins.base_planner.algorithms import Debug
from husky_assembly_teleop.plugins.base_planner.debug_view import DebugView


@pytest.fixture
def server():
    """A viser server on a free port."""
    server = viser.ViserServer(port=0, verbose=False)
    yield server
    server.stop()


def _debug_nodes(server: viser.ViserServer) -> set[str]:
    """Names of the debug view's scene nodes."""
    return {name for name in server.scene._handle_from_node_name if name.startswith("/plugin/debug/")}


def test_draws_each_layer_and_lists_it_then_clears(server):
    """Lines and points per layer, a legend row each plus the notes; None removes it all."""
    legend = server.gui.add_html("")
    view = DebugView(SimpleNamespace(view=SimpleNamespace(scene=server.scene, scene_root="/plugin")), legend)
    debug = Debug(on=True)
    debug.path("moves", [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)], (1, 2, 3))
    debug.points("hits", [(0.5, 0.5)], (4, 5, 6))
    debug.note("search 1: 3 corners")
    view.show(debug)
    assert _debug_nodes(server) == {"/plugin/debug/0_lines", "/plugin/debug/1_points"}
    assert "moves" in legend.content and "hits" in legend.content and "search 1: 3 corners" in legend.content
    view.show(None)
    assert not _debug_nodes(server) and legend.content == ""


def test_debug_off_draws_nothing(server):
    """A record made with debug off shows nothing."""
    legend = server.gui.add_html("")
    view = DebugView(SimpleNamespace(view=SimpleNamespace(scene=server.scene, scene_root="/plugin")), legend)
    debug = Debug()
    debug.points("hits", [(0.5, 0.5)], (4, 5, 6))
    view.show(debug)
    assert not _debug_nodes(server) and legend.content == ""


def _ctx(server: viser.ViserServer):
    """A plugin context with just what the view uses; deferred callbacks run at once."""
    return SimpleNamespace(view=SimpleNamespace(scene=server.scene, scene_root="/plugin", gui=server.gui),
                           defer_value=lambda label, run: lambda event: run(event.target.value))


def _record() -> Debug:
    """A record with two layers."""
    debug = Debug(on=True)
    debug.path("moves", [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], (1, 2, 3))
    debug.points("hits", [(0.5, 0.5)], (4, 5, 6))
    return debug


def test_layer_checkboxes_hide_layers_and_keep_them_hidden_for_the_next_search(server):
    """Unticking a layer hides its nodes; the next record's layer of that name starts hidden; All shows every one."""
    layers = server.gui.add_folder("Debug layers")
    notes = server.gui.add_html("")
    view = DebugView(_ctx(server), server.gui.add_html(""), layers, notes)
    assert not layers.visible
    view.show(_record())
    assert layers.visible and set(view._checkboxes) == {"moves", "hits"}
    view._checkboxes["hits"].value = False
    hits = server.scene._handle_from_node_name["/plugin/debug/1_points"]
    assert not hits.visible
    view.show(_record())
    hits = server.scene._handle_from_node_name["/plugin/debug/1_points"]
    assert not hits.visible and view._checkboxes["hits"].value is False
    view._show_all(True)
    assert hits.visible and view._checkboxes["hits"].value is True
    view.show(None)
    assert not layers.visible and not view._checkboxes and notes.content == ""
