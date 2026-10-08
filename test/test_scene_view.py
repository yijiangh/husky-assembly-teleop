"""Tests for `scene_view`: building on a budget, moving only on change, removing, tracked objects, collision mode."""

import pytest
import viser

from bar_assembly_core.geometry import BoxShape, Geometry, box_geometry
from bar_assembly_core.scene import Body, Scene
from bar_assembly_core.geometry import Pose
from husky_assembly_teleop.ui.scene_view import SceneView

BOX = box_geometry((0.1, 0.1, 0.1))


@pytest.fixture(scope="module")
def server():
    """One viser server on a free port for the whole module."""
    server = viser.ViserServer(port=0, verbose=False)
    yield server
    server.stop()


def _nodes(server: viser.ViserServer) -> set[str]:
    """Names of every scene node on the server."""
    return set(server.scene._handle_from_node_name)


def _snapshot(bodies: dict[str, tuple[Body, Pose]] | None = None) -> Scene:
    """Build a snapshot from bodies with their world poses."""
    bodies = bodies or {}
    return Scene(bodies={body_id: body for body_id, (body, _) in bodies.items()},
                 world_poses={body_id: pose for body_id, (_, pose) in bodies.items()})


def _body(body_id: str, geometry=BOX, pose: Pose = Pose()) -> tuple[Body, Pose]:
    """Make a body placed at `pose`, paired with that pose as its world pose."""
    return Body(body_id, geometry, pose), pose


def _tracked(name: str, geometry=None, pose: Pose = Pose()) -> tuple[Body, Pose]:
    """Make a tracked object "tracked/<name>" with a fix at `pose`; a frame only without geometry."""
    return _body(f"tracked/{name}", geometry or Geometry((), ()), pose)


def test_builds_on_budget(server):
    """The first sync builds up to the budget; the next builds the rest."""
    view = SceneView(server, build_budget=2)
    snapshot = _snapshot({f"budget/b{i}": _body(f"budget/b{i}") for i in range(3)})
    view.sync(snapshot)
    assert len(view._bodies) == 2
    view.sync(snapshot)
    assert set(view._bodies) == {"budget/b0", "budget/b1", "budget/b2"}
    assert "/scene/budget/b2/visual_0" in _nodes(server)


def test_moves_only_on_change(server, monkeypatch):
    """An unchanged pose is not reassigned; a changed one is."""
    view = SceneView(server)
    view.sync(_snapshot({"move/a": _body("move/a")}))
    assigned = []
    setter = viser.FrameHandle.position.fset
    monkeypatch.setattr(viser.FrameHandle, "position", property(
        viser.FrameHandle.position.fget, lambda handle, value: (assigned.append(value), setter(handle, value))))

    view.sync(_snapshot({"move/a": _body("move/a")}))
    assert assigned == []
    view.sync(_snapshot({"move/a": _body("move/a", pose=Pose((1.0, 2.0, 3.0)))}))
    assert assigned == [(1.0, 2.0, 3.0)]
    assert tuple(view._bodies["move/a"].frame.position) == (1.0, 2.0, 3.0)


def test_rebuilds_on_new_geometry_or_color(server):
    """A different Geometry object or colour rebuilds the body's nodes."""
    view = SceneView(server)
    view.sync(_snapshot({"rebuild/a": _body("rebuild/a")}))
    first = view._bodies["rebuild/a"].frame

    view.sync(_snapshot({"rebuild/a": _body("rebuild/a", geometry=box_geometry((0.2, 0.2, 0.2)))}))
    second = view._bodies["rebuild/a"].frame
    assert second is not first

    body, pose = _body("rebuild/a", geometry=view._bodies["rebuild/a"].geometry)
    body.color = (1.0, 0.0, 0.0, 0.5)
    view.sync(_snapshot({"rebuild/a": (body, pose)}))
    assert view._bodies["rebuild/a"].frame is not second


def test_removes_bodies(server):
    """A body gone from the snapshot loses its nodes."""
    view = SceneView(server)
    view.sync(_snapshot({"rm/tables/t1": _body("rm/tables/t1"), "rm/chairs/c1": _body("rm/chairs/c1")}))

    view.sync(_snapshot({"rm/chairs/c1": _body("rm/chairs/c1")}))
    assert set(view._bodies) == {"rm/chairs/c1"}
    assert "/scene/rm/tables/t1" not in _nodes(server) and "/scene/rm/tables/t1/visual_0" not in _nodes(server)

    view.sync(_snapshot())
    assert view._bodies == {}
    assert "/scene/rm/chairs/c1" not in _nodes(server)


def test_tracked_objects(server):
    """A tracked object is a body with axes: meshes only with geometry; it follows its pose and goes when missing."""
    view = SceneView(server)
    view.sync(_snapshot({"tracked/probe": _tracked("probe"), "tracked/bar": _tracked("bar", geometry=BOX)}))
    assert "/scene/tracked/probe" in _nodes(server) and not any(
        name.startswith("/scene/tracked/probe/") for name in _nodes(server))
    assert view._bodies["tracked/probe"].frame.show_axes
    assert "/scene/tracked/bar/visual_0" in _nodes(server)

    view.sync(_snapshot({"tracked/bar": _tracked("bar", geometry=BOX, pose=Pose((0.0, 0.0, 1.0)))}))
    assert "tracked/probe" not in view._bodies
    assert tuple(view._bodies["tracked/bar"].frame.position) == (0.0, 0.0, 1.0)


def test_disabled_bodies_are_hidden_not_removed(server):
    """A disabled body is built hidden, and switching it keeps its nodes."""
    view = SceneView(server)
    off = Body("en/box", BOX, Pose(), enabled=False)
    view.sync(_snapshot({"en/box": (off, Pose())}))
    frame = view._bodies["en/box"].frame
    assert not frame.visible
    view.sync(_snapshot({"en/box": _body("en/box")}))
    assert view._bodies["en/box"].frame is frame and frame.visible
    view.sync(_snapshot({"en/box": (off, Pose())}))
    assert view._bodies["en/box"].frame is frame and not frame.visible


def test_collision_mode_rebuilds_with_collision_shapes(server):
    """Switching to collision shapes rebuilds bodies and tracked objects from `geometry.collision`, and back.

    The visual is offset, as a floor mark is: the collision shape must not inherit its position.
    """
    floor_mark = BoxShape((1.0, 1.0, 0.01), Pose((0.0, 0.0, -0.995)))
    split = Geometry(visual=(floor_mark,), collision=(BoxShape((1.0, 1.0, 2.0)),))
    view = SceneView(server)
    snapshot = _snapshot({"col/wall": _body("col/wall", geometry=split),
                          "tracked/col_bar": _tracked("col_bar", geometry=split)})
    view.sync(snapshot)
    frame = view._bodies["col/wall"].frame
    assert tuple(server.scene._handle_from_node_name["/scene/col/wall/visual_0"].dimensions) == (1.0, 1.0, 0.01)

    view.sync(snapshot, collision=True)
    assert view._bodies["col/wall"].frame is not frame
    collision = server.scene._handle_from_node_name["/scene/col/wall/collision_0"]
    assert tuple(collision.dimensions) == (1.0, 1.0, 2.0) and tuple(collision.position) == (0.0, 0.0, 0.0)
    assert "/scene/col/wall/visual_0" not in _nodes(server)
    bar = server.scene._handle_from_node_name["/scene/tracked/col_bar/collision_0"]
    assert tuple(bar.dimensions) == (1.0, 1.0, 2.0)

    view.sync(snapshot)
    assert tuple(server.scene._handle_from_node_name["/scene/col/wall/visual_0"].dimensions) == (1.0, 1.0, 0.01)
    bar = server.scene._handle_from_node_name["/scene/tracked/col_bar/visual_0"]
    assert tuple(bar.dimensions) == (1.0, 1.0, 0.01)
