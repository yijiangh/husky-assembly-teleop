"""The collisions panel on a real viser server: listing per source, ticking draws, focus, clearing, hulls."""

from __future__ import annotations

import asyncio
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import pytest
import viser

from husky_assembly_teleop.plugins import collisions
from husky_assembly_teleop.plugins.collisions import CollisionsPlugin, _item, _summary
from husky_assembly_teleop.world.clearance import ClearanceTrip
from husky_assembly_teleop.world.collision_hit import CollisionHit, Contact
from scaffolding_assembly_core.geometry import Geometry, Pose, TriMesh, box_geometry
from scaffolding_assembly_core.scene import Body, Scene

ROOT = "/plugins/collisions"
#: An L-shaped prism: concave, so its hull has fewer faces and more volume.
L_SHAPE = TriMesh.from_arrays(
    [(0, 0, 0), (2, 0, 0), (2, 1, 0), (1, 1, 0), (1, 2, 0), (0, 2, 0),
     (0, 0, 1), (2, 0, 1), (2, 1, 1), (1, 1, 1), (1, 2, 1), (0, 2, 1)],
    [(0, 2, 1), (0, 3, 2), (0, 5, 3), (3, 5, 4), (6, 7, 8), (6, 8, 9), (6, 9, 11), (9, 10, 11),
     (0, 1, 7), (0, 7, 6), (1, 2, 8), (1, 8, 7), (2, 3, 9), (2, 9, 8), (3, 4, 10), (3, 10, 9),
     (4, 5, 11), (4, 11, 10), (5, 0, 6), (5, 6, 11)])


@pytest.fixture(scope="module")
def server():
    """One viser server on a free port for the whole module."""
    server = viser.ViserServer(port=0, verbose=False)
    yield server
    server.stop()


def make_ctx(server: viser.ViserServer) -> SimpleNamespace:
    """A plugin context whose view is the server and whose intents, tasks and threads run at once."""
    @contextmanager
    def tab(*_path):
        yield server.gui

    def defer(_label, run, operator_only=False):
        async def callback(*_args):
            run()
        return callback

    def defer_value(_label, run, operator_only=False):
        async def callback(event):
            if event.client is not None or not operator_only:
                run(event.target.value)
        return callback

    def spawn(_label, work, survives_soft_stop=False):
        asyncio.run(work)

    async def run_in_thread(function, *args):
        return function(*args)

    view = SimpleNamespace(ui=tab, scene=server.scene, scene_root=ROOT)
    return SimpleNamespace(view=view, defer=defer, defer_value=defer_value, spawn=spawn, run_in_thread=run_in_thread,
                           clearance_trips=(), now=lambda: 0.0)


def hit(depth: float = -0.012, rack: Geometry = box_geometry((0.2, 0.2, 0.2))) -> CollisionHit:
    """A wrist hitting a rack, both bodies here, with one contact."""
    bodies = {"t/wrist": Body("t/wrist", box_geometry((0.1, 0.1, 0.1)), Pose(), label="wrist"),
              "t/rack": Body("t/rack", rack, Pose(), label="rack")}
    scene = Scene(bodies=bodies, world_poses={key: Pose() for key in bodies})
    contact = Contact(("t/wrist", "t/rack"), (0.0, 0.0, 0.0), (0.0, 0.0, 0.01), depth)
    return CollisionHit(scene, "robots/x", "waypoint 1", (("t/wrist", "t/rack"),), (contact,))


def nodes(server: viser.ViserServer, prefix: str) -> list[str]:
    """Names of the scene nodes below a path."""
    return [name for name in server.scene._handle_from_node_name if name.startswith(prefix)]


def test_ticked_hits_draw_and_unticked_ones_go(server):
    """A hit listed as shown draws both sides and its contact; unticking removes it; another stays undrawn."""
    plugin = CollisionsPlugin()
    plugin.setup(make_ctx(server))
    plugin.replace("calibration", {"take 1": hit(), "take 2": hit()}, shown=("take 1",))
    plugin.draw(plugin._ctx)
    assert set(plugin._drawn) == {("calibration", "take 1")}
    root = plugin._drawn["calibration", "take 1"].root.name
    drawn = nodes(server, root)
    assert f"{root}/contact_0/depth" in drawn and {f"{root}/1", f"{root}/2"} <= set(drawn)
    assert f"{root}/3" not in drawn

    plugin._sources["calibration"].list._boxes["take 1"].value = False
    plugin.draw(plugin._ctx)
    assert plugin._drawn == {} and nodes(server, root) == []


def test_a_drawing_goes_after_shown_for_its_hit_still_listed(server):
    """A ticked hit is drawn for SHOWN_FOR s, then unticked: its drawing goes, the hit stays; ticked again, it draws."""
    clock = [100.0]
    ctx = make_ctx(server)
    ctx.now = lambda: clock[0]
    plugin = CollisionsPlugin()
    plugin.setup(ctx)
    plugin.replace("calibration", {"take 1": hit()}, shown=("take 1",))
    plugin.draw(ctx)
    root = plugin._drawn["calibration", "take 1"].root.name
    clock[0] += collisions.SHOWN_FOR - 1.0
    plugin.draw(ctx)
    assert ("calibration", "take 1") in plugin._drawn
    clock[0] += 2.0
    plugin.draw(ctx)
    listed = plugin._sources["calibration"]
    assert plugin._drawn == {} and nodes(server, root) == []
    assert "take 1" in listed.hits and not listed.list.is_selected("take 1")
    listed.list._boxes["take 1"].value = True
    plugin.draw(ctx)
    assert ("calibration", "take 1") in plugin._drawn


def test_replace_drops_a_sources_old_hits_and_clear_drops_the_source(server):
    """A second report replaces the first; clearing removes the group and shows the empty note again."""
    plugin = CollisionsPlugin()
    plugin.setup(make_ctx(server))
    plugin.replace("planner", {"a": hit(), "b": hit()}, shown=("a", "b"))
    plugin.draw(plugin._ctx)
    plugin.replace("planner", {"c": hit()})
    plugin.draw(plugin._ctx)
    assert list(plugin._sources["planner"].hits) == ["c"] and plugin._drawn == {}
    assert not plugin._empty.visible
    plugin.clear("planner")
    plugin.draw(plugin._ctx)
    assert plugin._sources == {} and plugin._empty.visible


def test_a_new_report_keeps_the_sources_folder_where_it_is(server):
    """Re-reporting a source refills its folder in place: the folders below it don't move, its list is the new hits."""
    plugin = CollisionsPlugin()
    plugin.setup(make_ctx(server))
    plugin.replace("planner", {"a": hit()})
    plugin.replace("calibration", {"take 1": hit()})
    folder = plugin._sources["planner"].folder
    plugin.replace("planner", {"b": hit(), "c": hit()})
    found = plugin._sources["planner"]
    assert found.folder is folder and list(plugin._sources) == ["planner", "calibration"]
    assert list(found.list._boxes) == ["b", "c"] and "hit: " in found.summary.content


def test_focus_follows_the_ticked_hits(server):
    """Focus offers the ticked hits; a hit no longer ticked falls back to none."""
    plugin = CollisionsPlugin()
    plugin.setup(make_ctx(server))
    plugin.replace("calibration", {"take 1": hit()}, shown=("take 1",))
    plugin.draw(plugin._ctx)
    assert tuple(plugin._focus.options) == ("none", "calibration: take 1")
    plugin._focus.value = "calibration: take 1"
    plugin.draw(plugin._ctx)
    assert plugin._focused is not None
    plugin.clear("calibration")
    plugin.draw(plugin._ctx)
    assert plugin._focus.value == "none" and plugin._focused is None


def test_as_checked_draws_a_concave_mesh_as_its_hull(server, monkeypatch):
    """With "As checked", the L-shaped rack (see-through, hit side) is drawn as its convex hull; without, as itself."""
    drawn = []
    see_through = collisions._see_through
    monkeypatch.setattr(collisions, "_see_through",
                        lambda shape, color: drawn.append(shape) or see_through(shape, color))
    plugin = CollisionsPlugin()
    plugin.setup(make_ctx(server))
    plugin.replace("calibration", {"take 1": hit(rack=Geometry((L_SHAPE,), (L_SHAPE,)))}, shown=("take 1",))
    counts = []
    for as_checked in (True, False):
        plugin._as_checked.value = as_checked
        drawn.clear()
        plugin.draw(plugin._ctx)
        counts.append(next(len(shape.vertices) for shape in drawn if isinstance(shape, TriMesh)))
    assert counts[0] < len(L_SHAPE.vertices) == counts[1]


def test_shapes_are_see_through_meshes_that_write_no_depth():
    """Boxes and meshes become blended, two-sided meshes: three.js then writes no depth for them."""
    for shape in (box_geometry((0.1, 0.2, 0.3)).collision[0], L_SHAPE):
        material = collisions._see_through(shape, (1.0, 0.0, 0.0, 0.5)).visual.material
        assert (material.alphaMode, material.doubleSided, material.baseColorFactor[3]) == ("BLEND", True, 128)


def test_labels_name_the_first_pair_its_depth_and_what_is_hit_most():
    """Each item names the first pair and the deepest overlap; the source summary counts what was hit."""
    assert _item("take 1", hit()) == "take 1: wrist ↔ rack · 12.0 mm"
    no_contact = CollisionHit(hit().scene, "robots/x", "start", (("t/wrist", "t/rack"),), (None,))
    assert _item("plan", no_contact) == "plan: wrist ↔ rack · no contact found"
    assert _summary({"take 1": hit(), "take 2": hit()}) == "hit: rack ×2"
    assert _summary({}) == "nothing collides"
    assert np.isclose(hit().contacts[0].depth, -0.012)


def test_clearance_trips_are_listed_with_the_newest_ticked_and_clear_hides_them(server):
    """Trips with a hit come under "clearance", newest ticked; after Clear only later trips come back."""
    plugin = CollisionsPlugin()
    ctx = make_ctx(server)
    plugin.setup(ctx)
    ctx.clearance_trips = (ClearanceTrip(100.0, "0805", "base_exp: run", "robot 0805 came within 10 cm", hit()),
                           ClearanceTrip(101.0, "0805", "base_exp: run", "robot 0805 cannot be checked", None),
                           ClearanceTrip(102.0, "0806", "robot_control: drive", "robot 0806 came within 10 cm", hit()))
    plugin.update(ctx)
    found = plugin._sources["clearance"]
    assert [label.split()[-1] for label in found.hits] == ["0805", "0806"]
    assert found.list.selected() == [list(found.hits)[-1]]

    plugin.clear("clearance")
    plugin.update(ctx)
    assert "clearance" not in plugin._sources
    ctx.clearance_trips = (*ctx.clearance_trips, ClearanceTrip(103.0, "0804", "base_exp: run", "came within", hit()))
    plugin.update(ctx)
    assert [label.split()[-1] for label in plugin._sources["clearance"].hits] == ["0804"]
