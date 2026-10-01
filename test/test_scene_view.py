"""Tests for `scene_view`: building on a budget, moving only on change, removing, tracked objects."""

import pytest
import viser

from husky_assembly_teleop.world.geometry import box_geometry
from husky_assembly_teleop.world.scene import Body, Pose, SceneSnapshot, TrackedDescription, TrackedEntry
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


def _snapshot(bodies: dict[str, tuple[Body, Pose]] | None = None,
              tracked: dict[str, TrackedEntry] | None = None) -> SceneSnapshot:
    """Build a snapshot from bodies with their world poses, and tracked entries."""
    bodies = bodies or {}
    return SceneSnapshot(bodies={body_id: body for body_id, (body, _) in bodies.items()},
                         world_poses={body_id: pose for body_id, (_, pose) in bodies.items()},
                         tracked=tracked or {})


def _body(body_id: str, geometry=BOX, pose: Pose = Pose()) -> tuple[Body, Pose]:
    """Make a body placed at `pose`, paired with that pose as its world pose."""
    return Body(body_id, geometry, pose), pose


def _tracked(name: str, geometry=None, pose: Pose = Pose()) -> TrackedEntry:
    """Make a tracked object with a fix at `pose`."""
    return TrackedEntry(name, TrackedDescription(geometry), pose, True, None)


def test_builds_on_budget(server):
    """The first sync builds up to the budget; the next builds the rest."""
    view = SceneView(server, build_budget=2)
    snapshot = _snapshot({f"budget/b{i}": _body(f"budget/b{i}") for i in range(3)})
    view.sync(snapshot)
    assert len(view._bodies) == 2
    view.sync(snapshot)
    assert set(view._bodies) == {"budget/b0", "budget/b1", "budget/b2"}
    assert "/scene/budget/b2/mesh_0" in _nodes(server)


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
    assert "/scene/rm/tables/t1" not in _nodes(server) and "/scene/rm/tables/t1/mesh_0" not in _nodes(server)

    view.sync(_snapshot())
    assert view._bodies == {}
    assert "/scene/rm/chairs/c1" not in _nodes(server)


def test_tracked_objects(server):
    """A tracked object gets a frame, meshes only with geometry, follows its pose and hides when missing."""
    view = SceneView(server)
    view.sync(_snapshot(tracked={"probe": _tracked("probe"), "bar": _tracked("bar", geometry=BOX)}))
    assert view._tracked["probe"].meshes == []
    assert len(view._tracked["bar"].meshes) == 1 and "/tracked/bar/mesh_0" in _nodes(server)

    view.sync(_snapshot(tracked={"bar": _tracked("bar", geometry=BOX, pose=Pose((0.0, 0.0, 1.0)))}))
    assert not view._tracked["probe"].frame.visible
    assert view._tracked["bar"].frame.visible
    assert tuple(view._tracked["bar"].frame.position) == (0.0, 0.0, 1.0)

    view.sync(_snapshot(tracked={"probe": _tracked("probe")}))
    assert view._tracked["probe"].frame.visible and not view._tracked["bar"].frame.visible
