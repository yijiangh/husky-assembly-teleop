"""
Lists what collision checks hit, per source, and draws the ticked hits as they were checked.

Sources report through `CollisionsContract` (calibration takes, a failed arm or base plan, the motion planner's
last answer, base_exp's blocked way back); clearance trips of every plugin (`ctx.clearance_trips`) come under
"clearance", the newest ticked. A ticked hit shows both sides of every blocking pair: what moved solid red, what it
moved into see-through cyan, each contact as two dots, the deepest with its depth. The focused hit also shows every
robot as checked, faint. "As checked" draws meshes as the check tested them.

? viser has no order-independent transparency: see-through shapes of different colours pop in front of each other as
  the view turns. So only one strong colour is see-through (cyan), drawn without depth writes; red is solid.

! Never moves the camera. A drawing stays until it is unticked, replaced or cleared, SHOWN_FOR s at most: then its
  hit is unticked, still listed.
"""

from __future__ import annotations

import time
from collections import Counter
from dataclasses import dataclass
from itertools import count
from typing import Collection, Mapping

import numpy as np
import trimesh
import viser

from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..world.collision_hit import CollisionHit
from ..ui.checklist import CheckList
from ..ui.quaternion import quaternion_to_wxyz
from ..ui.scene_view import add_shape
from ..ui.style import note, note_line
from scaffolding_assembly_core.geometry import BoxShape, CylinderShape, Geometry, Pose, Shape, TriMesh
from scaffolding_assembly_core.ids import link_id
from scaffolding_assembly_core.scene import Scene, scene_parts

#: Colours, (r, g, b, alpha) from 0 to 1.
MOVED_COLOR = (1.0, 0.15, 0.1, 1.0)    # solid red: what moved into something
HIT_COLOR = (0.0, 0.75, 0.9, 0.45)     # see-through cyan: what it moved into, the red visible inside it
ROBOT_COLOR = (0.6, 0.6, 0.6, 0.15)    # faint grey: the robots as checked, for the focused hit
CONTACT_COLOR = (255, 200, 0)          # the contact's two points and the line between them
CONTACT_RADIUS = 0.006                 # m
#: The Focus choice that shows no robots.
NO_FOCUS = "none"
#: Seconds a ticked hit stays drawn; then it is unticked, still listed (tick it to see it again).
SHOWN_FOR = 120.0
#: The source clearance trips are listed under.
CLEARANCE = "clearance"


@dataclass
class _Source:
    """One source's group in the panel: its summary line, its list, and its hits by label."""

    folder: viser.GuiFolderHandle
    summary: viser.GuiHtmlHandle
    list: CheckList
    hits: dict[str, CollisionHit]


@dataclass
class _Drawn:
    """What a drawing shows, to tell when it is out of date."""

    hit: CollisionHit
    as_checked: bool
    root: viser.SceneNodeHandle
    #: When it was drawn, `ctx.now()`.
    since: float


@register
class CollisionsPlugin(HuskyPlugin):
    """A panel of collision hits reported by other plugins; offers `CollisionsContract`."""

    name = "collisions"
    icon = viser.Icon.ALERT_TRIANGLE

    def __init__(self):
        """Start with nothing reported."""
        self._ctx: PluginContext | None = None
        self._sources: dict[str, _Source] = {}
        #: Drawn hits by (source, label), and the focused hit's robots.
        self._drawn: dict[tuple[str, str], _Drawn] = {}
        self._focused: _Drawn | None = None
        #: Convex hulls by mesh identity, as (mesh, hull): the mesh is kept, so its id is never reused.
        self._hulls: dict[int, tuple[TriMesh, TriMesh]] = {}
        #: Hits ready to draw by `id(hit)`, as (hit, part id -> (geometry, pose)); those being prepared.
        self._ready: dict[int, tuple[CollisionHit, dict]] = {}
        self._preparing: set[int] = set()
        self._names = count()
        #: The clearance trips listed last, and when the operator last cleared them (only later ones come back).
        self._trips: tuple = ()
        self._cleared_trips = -float("inf")

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel's fixed part; each source adds its own group below it."""
        self._ctx = ctx
        with self._ctx.view.ui() as gui:
            self._shapes = gui.add_checkbox(
                "Collision shapes", initial_value=False,
                hint="Draw what collision checks use instead of what things look like, in the whole 3D view: robots "
                     "in red, scene bodies and tracked objects in their own colour.")
            self._shapes.on_update(ctx.defer_value("collision shapes", ctx.view.show_collision))
            self._as_checked = gui.add_checkbox(
                "As checked", initial_value=True,
                hint="Draw meshes as the check tested them (compas_fab: each mesh's convex hull); untick for the "
                     "models' own meshes")
            self._focus = gui.add_dropdown("Focus", options=[NO_FOCUS], initial_value=NO_FOCUS,
                                           hint="Also show every robot as it was checked for this ticked hit, faint")
            self._empty = gui.add_html(note("no collisions reported"))

    def teardown(self, ctx: PluginContext) -> None:
        """Draw what things look like again, if the collision shapes were on."""
        ctx.view.show_collision(False)

    # --- --- --- --- --- CONTRACT --- --- --- --- ---

    def replace(self, source: str, hits: Mapping[str, CollisionHit], shown: Collection[str] = ()) -> None:
        """Make these a source's hits, dropping its earlier ones (`CollisionsContract`).

        A source already listed keeps its folder where it is, so the folders below it don't move; a new one goes last.
        """
        found = self._sources.get(source)
        if found is None:
            with self._ctx.view.ui() as gui:
                folder = gui.add_folder(source)
                with folder:
                    summary = gui.add_html("")
                    clear = gui.add_button("Clear", icon=viser.Icon.TRASH,
                                           hint=f"Remove {source}'s hits from the list")
                    checklist = CheckList(self._ctx, gui, "hits", empty_text="nothing collides")
            clear.on_click(self._ctx.defer(f"clear {source}", lambda: self.clear(source)))
            found = self._sources[source] = _Source(folder, summary, checklist, {})
        for label in found.hits:
            found.list.remove(label)
        found.summary.content = note_line(_summary(hits))
        for label, hit in hits.items():
            found.list.add(label, _item(label, hit), hint=_details(hit), selected=label in shown)
        found.hits = dict(hits)

    def clear(self, source: str) -> None:
        """Drop a source and its hits (`CollisionsContract`)."""
        found = self._sources.pop(source, None)
        if found is not None:
            found.folder.remove()
        if source == CLEARANCE and self._trips:
            self._cleared_trips = self._trips[-1].time

    # --- --- --- --- --- CLEARANCE TRIPS --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """List new clearance trips with what came too close, the newest ticked: it just stopped a robot."""
        trips = ctx.clearance_trips
        if trips is self._trips:
            return
        self._trips = trips
        hits = {}
        for trip in trips:
            if trip.hit is not None and trip.time > self._cleared_trips:
                label = f"{time.strftime('%H:%M:%S', time.localtime(trip.time))} {trip.serial}"
                hits[label if label not in hits else f"{label} ({len(hits) + 1})"] = trip.hit
        if hits:
            self.replace(CLEARANCE, hits, shown=(list(hits)[-1],))

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Draw the ticked hits and the focused hit's robots; remove what is no longer ticked or out of date."""
        as_checked, now = self._as_checked.value, ctx.now()
        # * A hit drawn for SHOWN_FOR s is unticked: its drawing goes below, the hit stays listed.
        for (source, label), drawn in self._drawn.items():
            if now - drawn.since > SHOWN_FOR and source in self._sources:
                self._sources[source].list.deselect(label)
        listed = {id(hit) for found in self._sources.values() for hit in found.hits.values()}
        self._ready = {key: ready for key, ready in self._ready.items() if key in listed}
        ticked = {(source, label): found.hits[label]
                  for source, found in self._sources.items() for label in found.list.selected()}
        for key, drawn in list(self._drawn.items()):
            if ticked.get(key) is not drawn.hit or drawn.as_checked != as_checked:
                drawn.root.remove()
                del self._drawn[key]
        for key, hit in ticked.items():
            if key not in self._drawn and self._prepared(ctx, hit):
                self._drawn[key] = _Drawn(hit, as_checked, self._draw_hit(ctx, hit, as_checked), now)

        choices = {f"{source}: {label}": hit for (source, label), hit in ticked.items()}
        options = (NO_FOCUS, *choices)
        if tuple(self._focus.options) != options:
            self._focus.options = options
        if self._focus.value not in options:
            self._focus.value = NO_FOCUS
        focus = choices.get(self._focus.value)
        if self._focused is not None and (self._focused.hit is not focus or self._focused.as_checked != as_checked):
            self._focused.root.remove()
            self._focused = None
        if focus is not None and self._focused is None and self._prepared(ctx, focus):
            self._focused = _Drawn(focus, as_checked, self._draw_robots(ctx, focus, as_checked), now)

        for found in self._sources.values():
            found.list.sync()
        self._empty.visible = not self._sources

    def _prepared(self, ctx: PluginContext, hit: CollisionHit) -> bool:
        """Whether a hit's parts and hulls are ready to draw; starts preparing them on a thread if not."""
        if id(hit) not in self._ready and id(hit) not in self._preparing:
            self._preparing.add(id(hit))
            ctx.spawn("prepare a hit", self._prepare(ctx, hit), survives_soft_stop=True)
        return id(hit) in self._ready

    async def _prepare(self, ctx: PluginContext, hit: CollisionHit) -> None:
        """Work out a hit's parts and missing hulls on a thread: a robot's first parts load its meshes (~1 s)."""
        try:
            parts, hulls = await ctx.run_in_thread(_parts_and_hulls, hit, frozenset(self._hulls))
        finally:
            self._preparing.discard(id(hit))
        self._hulls.update(hulls)
        self._ready[id(hit)] = (hit, parts)

    def _draw_hit(self, ctx: PluginContext, hit: CollisionHit, as_checked: bool) -> viser.SceneNodeHandle:
        """Both sides of every pair at their checked poses, and each contact; returns the root to remove."""
        scene = ctx.view.scene
        root = scene.add_frame(f"{ctx.view.scene_root}/hits/{next(self._names)}", show_axes=False)
        parts = self._ready[id(hit)][1]
        drawn: set[str] = set()
        depths = [contact.depth if contact is not None else np.inf for contact in hit.contacts]
        deepest = int(np.argmin(depths)) if depths else -1
        for index, pair in enumerate(hit.pairs):
            contact = hit.contacts[index] if index < len(hit.contacts) else None
            for side, color in zip(contact.pair if contact else pair, (MOVED_COLOR, HIT_COLOR)):
                for part_id in _parts_of(hit.scene, side, parts):
                    if part_id not in drawn:
                        drawn.add(part_id)
                        self._add_part(scene, f"{root.name}/{len(drawn)}", *parts[part_id], color,
                                       as_checked and part_id not in hit.concave)
            if contact is not None:
                path = f"{root.name}/contact_{index}"
                for name, point in (("a", contact.point_a), ("b", contact.point_b)):
                    scene.add_icosphere(f"{path}/{name}", radius=CONTACT_RADIUS, color=CONTACT_COLOR, position=point)
                scene.add_line_segments(f"{path}/line", np.array([[contact.point_a, contact.point_b]]), CONTACT_COLOR,
                                        thickness=2.0, thickness_units="screen")
                if index == deepest:  # ? one label: several at one spot overlap; the hover text lists them all
                    scene.add_label(f"{path}/depth", _depth(contact.depth),
                                    position=tuple((np.add(contact.point_a, contact.point_b) / 2).tolist()))
        return root

    def _draw_robots(self, ctx: PluginContext, hit: CollisionHit, as_checked: bool) -> viser.SceneNodeHandle:
        """Every robot of a hit's scene as checked, faint; returns the root to remove."""
        scene = ctx.view.scene
        root = scene.add_frame(f"{ctx.view.scene_root}/focus/{next(self._names)}", show_axes=False)
        parts = self._ready[id(hit)][1]
        robot_ids = [part_id for robot in hit.scene.robots.values() if robot.enabled
                     for part_id in _parts_of(hit.scene, robot.id, parts)]
        for index, part_id in enumerate(robot_ids):
            self._add_part(scene, f"{root.name}/{index}", *parts[part_id], ROBOT_COLOR,
                           as_checked and part_id not in hit.concave)
        return root

    def _add_part(self, scene: viser.SceneApi, name: str, geometry: Geometry, pose: Pose,
                  color: tuple[float, ...], hulls: bool) -> None:
        """One part's collision shapes (visual ones if it has none) under a frame at its pose: solid, or see-through."""
        scene.add_frame(name, show_axes=False, position=tuple(np.asarray(pose.position, dtype=float).tolist()),
                        wxyz=quaternion_to_wxyz(pose.orientation))
        for index, shape in enumerate(_shapes(geometry)):
            drawn = self._hulls[id(shape)][1] if hulls and id(shape) in self._hulls else shape
            if len(color) < 4 or color[3] >= 1.0:
                add_shape(scene, f"{name}/{index}", drawn, color)
            else:
                scene.add_mesh_trimesh(f"{name}/{index}", _see_through(drawn, color), cast_shadow=False,
                                       receive_shadow=False)


def _see_through(shape: Shape, color: tuple[float, ...]) -> trimesh.Trimesh:
    """A shape as a see-through mesh in its part's frame: blended, both sides, writing no depth.

    ? viser's own shapes with an opacity still write depth, so where two overlap, the one drawn first hides the
      other's inside; three.js loads a GLB material with alphaMode BLEND without depth writes.
    """
    if isinstance(shape, BoxShape):
        mesh = trimesh.creation.box(extents=shape.size, transform=shape.origin.matrix())
    elif isinstance(shape, CylinderShape):
        mesh = trimesh.creation.cylinder(radius=shape.radius, height=shape.height, transform=shape.origin.matrix())
    else:
        mesh = trimesh.Trimesh(shape.vertices, shape.faces, process=False)
    rgba = [int(round(255 * value)) for value in (*color, 1.0)[:4]]
    mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
        baseColorFactor=rgba, metallicFactor=0.0, roughnessFactor=0.8, alphaMode="BLEND", doubleSided=True))
    return mesh


def _shapes(geometry: Geometry) -> tuple[Shape, ...]:
    """What a part draws: its collision shapes, or its visual ones if it has none."""
    return geometry.collision or geometry.visual


def _parts_and_hulls(hit: CollisionHit, known: Collection[int]) -> tuple[dict, dict[int, tuple[TriMesh, TriMesh]]]:
    """A hit's parts by id, and the convex hull of each concave mesh among them not in `known`. On a thread.

    Returns:
        tuple: Part id -> (geometry, pose); mesh id -> (mesh, hull), the mesh kept so its id is never reused.
    """
    parts = {part_id: (geometry, pose) for part_id, geometry, pose in scene_parts(hit.scene)}
    hulls = {}
    for geometry, _pose in (part for part_id, part in parts.items() if part_id not in hit.concave):
        for shape in _shapes(geometry):
            if isinstance(shape, TriMesh) and not shape.convex and id(shape) not in known:
                hull = trimesh.Trimesh(shape.vertices, shape.faces, process=False).convex_hull
                hulls[id(shape)] = (shape, TriMesh(np.asarray(hull.vertices, dtype=np.float64),
                                                   np.asarray(hull.faces, dtype=np.int32), True))
    return parts, hulls


def _parts_of(scene: Scene, side: str, parts: Mapping[str, tuple]) -> list[str]:
    """The drawn parts of one side of a pair: itself, or every part of a whole robot."""
    if side in parts:
        return [side]
    robot = scene.robots.get(side)
    if robot is None:
        return []
    ids = [link_id(side, link) for link in robot.model.link_geometry] + [t.id for t in robot.model.tools.values()]
    return [part_id for part_id in ids if part_id in parts]


def _depth(depth: float) -> str:
    """How far two shapes overlap, e.g. "12.3 mm deep", or how far apart a near miss is, e.g. "41.0 mm apart"."""
    return f"{-depth * 1e3:.1f} mm deep" if depth < 0.0 else f"{depth * 1e3:.1f} mm apart"


def _sides(hit: CollisionHit, index: int) -> tuple[str, str]:
    """A pair's two sides, narrowed by its contact where there is one."""
    contact = hit.contacts[index] if index < len(hit.contacts) else None
    return contact.pair if contact is not None else hit.pairs[index]


def _item(label: str, hit: CollisionHit) -> str:
    """One checkbox's text: the first pair, how many more, and the deepest overlap; kept short for the panel."""
    a, b = _sides(hit, 0)
    more = f" +{len(hit.pairs) - 1}" if len(hit.pairs) > 1 else ""
    depths = [contact.depth for contact in hit.contacts if contact is not None]
    deepest = (f"{-min(depths) * 1e3:.1f} mm" if min(depths) < 0.0 else f"{min(depths) * 1e3:.1f} mm apart") \
        if depths else "no contact found"
    return f"{label}: {hit.name(a)} ↔ {hit.name(b)}{more} · {deepest}"


def _details(hit: CollisionHit) -> str:
    """Hover text: the message, then every pair with its depth."""
    lines = [f"{hit.robot}: {hit.message}"]
    for index in range(len(hit.pairs)):
        a, b = _sides(hit, index)
        contact = hit.contacts[index] if index < len(hit.contacts) else None
        lines.append(f"{hit.name(a)} ↔ {hit.name(b)}: "
                     f"{_depth(contact.depth) if contact is not None else 'no contact found'}")
    return "\n".join(lines)


def _summary(hits: Mapping[str, CollisionHit]) -> str:
    """What a source's hits run into most, e.g. "hit: ipad_rack ×2, top_plate ×1"."""
    if not hits:
        return "nothing collides"
    counts = Counter(hit.name(_sides(hit, index)[1])
                     for hit in hits.values() for index in range(len(hit.pairs)))
    return "hit: " + ", ".join(f"{name} ×{n}" for name, n in counts.most_common())
