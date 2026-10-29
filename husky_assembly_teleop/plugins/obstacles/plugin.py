"""
The lab's obstacles in the scene, and the tools to edit them.

Furniture and floor border come from the lab file; temporary obstacles are for a session's tests. The floor is a slab
under z = 0 out to the border walls, not drawn: arms, tools and what they hold collide with it, the robots' platform
links (wheels, chassis) may touch it.

- Lab obstacles come from `data/lab/<lab>.json` (`-p lab:=<name>`, default "main"), in git and the same for everyone.
  One may be drawn as meshes from `data/lab/meshes/` (furniture, screens); it still collides as its shape.
  Edits stay in memory until Save; Revert goes back to the file; an amber chip shows unsaved changes.
- Temporary obstacles (purple, "obstacles/temporary/<n>") are kept in a local file (`world/lab.TEMPORARY_FILE`),
  written at once, never in the lab file. Promote moves one into the lab, saved with the next Save.
- Select in the dropdown, or with Edit ticked by clicking in the 3D view. Edit (off at start, ticked by Add) shows the
  gizmos: drag with them (only the yaw of a turn is kept), resize with the corner one, or type position, yaw and size.

Ids: "obstacles/boxes/<name>", "obstacles/cylinders/<name>", "obstacles/trusses/<name>", and the border walls
"obstacles/boxes/border_x_min" and so on, and "obstacles/floor". Planners see them in `ctx.scene.snapshot`. Other
plugins read the lab through `shared.obstacles_contract.ObstaclesContract` (`lab`).
"""

from __future__ import annotations

import math
from dataclasses import replace
from html import escape
from pathlib import Path

import viser

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...ui.body_gizmo import BodyGizmo
from ...ui.style import BUSY, NONE, OK, SECTION_CTRL, block, chip, chip_line, note, section, values
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import link_id, robot_id
from scaffolding_assembly_core.scene import Body
from ...world.lab import (DEFAULT_LAB, GROUPS, LAB_COLOR, TEMPORARY_COLOR, Border, Lab, Shape, border_bodies,
                          floor_body, lab_file, mesh_directory_of, platform_links, read_lab, read_temporary, shape_id,
                          slug, write_lab, write_temporary)

#: Dropdown entry while there is nothing to edit.
NONE_ENTRY = "(none)"
#: Size of a new temporary obstacle, metres: box sides, or a cylinder's (diameter, diameter, height).
NEW_SIZE = (0.5, 0.5, 1.0)


@register
class ObstaclesPlugin(HuskyPlugin):
    """Puts the lab file's obstacles, border and floor, and the temporary obstacles, into the scene, and edits them."""

    name = "obstacles"
    icon = viser.Icon.BARRIER_BLOCK
    hidden = True

    def __init__(self):
        """Start with an empty lab; setup reads the files."""
        #: The lab file, the lab as saved in it, and the lab as edited (`ObstaclesContract.lab`: replace, never edit).
        self.path: Path | None = None
        self.saved = Lab()
        self.lab = Lab()
        #: Temporary obstacles by number, and as last written; numbers are never reused.
        self.temporary: dict[int, Shape] = {}
        self._written: dict[int, Shape] | None = None
        self._next_number = 1
        self._gizmo: BodyGizmo | None = None
        # The body the gizmo was last put on.
        self._gizmo_on: str | None = None
        # Result of the last action.
        self._message = ""
        # The obstacle dropdown and the Edit checkbox; None until setup builds them.
        self._select: viser.GuiDropdownHandle | None = None
        self._edit: viser.GuiCheckboxHandle | None = None

    def setup(self, ctx: PluginContext) -> None:
        """Read the lab and temporary files, put every obstacle into the scene, and build the panel.

        Args:
            ctx: This plugin's context.
        """
        self.load(ctx)
        self._put_all(ctx)
        with ctx.view.ui() as gui:
            gui.add_html(section("lab", SECTION_CTRL))
            self._lab_status: viser.GuiHtmlHandle = gui.add_html("")
            lab_actions = gui.add_button_group("Lab", ["Save", "Revert"])
            border = self.lab.border or Border((0.0, 0.0), (0.0, 0.0))
            self._border_x = gui.add_vector2("Border x m", border.x, step=0.01,
                                             hint="Lowest and highest x of the usable floor; walls stand outside it.")
            self._border_y = gui.add_vector2("Border y m", border.y, step=0.01)

            gui.add_html(section("edit", SECTION_CTRL))
            self._edit = gui.add_checkbox("Edit", False, hint="Show the gizmos and pick obstacles by clicking in 3D.")
            self._select = gui.add_dropdown("Obstacle", [NONE_ENTRY])
            self._name = gui.add_text("Name", initial_value="", hint="A lab obstacle's name; Rename applies it.")
            self._position = gui.add_vector3("Position m", (0.0, 0.0, 0.0), step=0.001, hint="Its centre.")
            self._yaw = gui.add_number("Yaw °", 0.0, step=0.1)
            self._size = gui.add_vector3("Size m", NEW_SIZE, step=0.01, min=(0.01,) * 3,
                                         hint="Box sides; a cylinder's diameter (x) and height (z); a truss column's "
                                              "side (x) and height (z).")
            selected = gui.add_button_group("Selected", ["Rename", "Remove", "Promote"])
            add = gui.add_button_group("Add", ["Box", "Cylinder"])
            self._status: viser.GuiHtmlHandle = gui.add_html("")
        self._gizmo = BodyGizmo(ctx, "gizmo", candidates=lambda: list(self.shapes()), on_pick=self._pick,
                                on_move=lambda body_id, pose: self._move(ctx, body_id, pose),
                                on_resize=lambda body_id, pose, size: self._resize(ctx, body_id, pose, size))

        # ! Callbacks run on a viser thread: hand every action to the main thread.
        lab_actions.on_click(ctx.defer_value("lab", lambda clicked: (
            self.save(ctx) if clicked == "Save" else self.revert(ctx))))
        selected.on_click(ctx.defer_value("selected obstacle", lambda clicked: {
            "Rename": lambda: self.rename(ctx, self._select.value, self._name.value),
            "Remove": lambda: self.remove(ctx, self._select.value),
            "Promote": lambda: self.promote(ctx, self._select.value)}[clicked]()))
        add.on_click(ctx.defer_value("add obstacle", lambda clicked: self.add(ctx, clicked.lower())))

        async def _typed(event: viser.GuiEvent) -> None:
            # * Skip our own writes from `draw`: viser calls back for those too, without a client.
            if event.client is None:
                return
            body_id, position, yaw, size = self._select.value, self._position.value, self._yaw.value, self._size.value
            ctx.submit("type obstacle", lambda: self.set_typed(ctx, body_id, position, yaw, size))

        for widget in (self._position, self._yaw, self._size):
            widget.on_update(_typed)

        async def _bordered(event: viser.GuiEvent) -> None:
            if event.client is None:
                return
            x, y = self._border_x.value, self._border_y.value
            ctx.submit("type border", lambda: self.set_border(ctx, x, y))

        self._border_x.on_update(_bordered)
        self._border_y.on_update(_bordered)

    def update(self, ctx: PluginContext) -> None:
        """Write the temporary obstacles when they changed.

        Args:
            ctx: This plugin's context.
        """
        self.write_temporary(ctx)

    def teardown(self, ctx: PluginContext) -> None:
        """Remove the scene click callback; the runtime removes the bodies.

        Args:
            ctx: This plugin's context.
        """
        if self._gizmo is not None:
            self._gizmo.remove()

    # --- --- --- --- --- FILES --- --- --- --- ---

    def load(self, ctx: PluginContext) -> None:
        """Read the lab file and the temporary obstacles; a broken file is logged, never overwritten."""
        self.path = lab_file(ctx.config.data_directory, ctx.config.lab or DEFAULT_LAB)
        try:
            self.saved = self.lab = read_lab(self.path)
        except (OSError, ValueError) as error:
            self._message = f"lab not loaded: {error}"
            ctx.log_error(self._message)
        try:
            self.temporary = read_temporary()
            self._written = dict(self.temporary)
        except (OSError, ValueError, KeyError, TypeError) as error:  # * json.JSONDecodeError is a ValueError
            # * `_written` stays None: the broken file is never overwritten.
            ctx.log_warn(f"temporary obstacles not loaded: {error!r}")
        self._next_number = max(self.temporary, default=0) + 1

    def save(self, ctx: PluginContext) -> None:
        """Write the edited lab to its file."""
        if self.path is None:
            return
        write_lab(self.lab, self.path)
        self.saved = self.lab
        self._message = f"saved {len(self.lab.shapes)} obstacles to {self.path}"
        ctx.log_info(self._message)

    def revert(self, ctx: PluginContext) -> None:
        """Drop the unsaved edits: back to the lab as saved."""
        self.lab = self.saved
        self._put_all(ctx)
        self._gizmo_on = None
        self._message = "reverted to the saved lab"

    def write_temporary(self, ctx: PluginContext) -> None:
        """Write the temporary obstacles if they changed since the last write, unless their file was broken."""
        if self._written is None or self.temporary == self._written:
            return
        self._written = dict(self.temporary)
        try:
            write_temporary(self.temporary)
        except OSError as error:
            ctx.log_warn(f"temporary obstacles not saved: {error!r}")

    # --- --- --- --- --- EDITING (run as intents) --- --- --- --- ---

    def shapes(self) -> dict[str, Shape]:
        """Every editable obstacle by id: the lab's, then the temporary ones. The border walls are not in it."""
        return {**{shape_id(self.name, shape): shape for shape in self.lab.shapes},
                **{self._temporary_id(number): shape for number, shape in self.temporary.items()}}

    @property
    def dirty(self) -> bool:
        """bool: Whether the lab has edits that are not saved."""
        return self.lab != self.saved

    def add(self, ctx: PluginContext, kind: str) -> str:
        """Add a temporary "box" or "cylinder" standing at the origin, select it, tick Edit, and return its id."""
        number, self._next_number = self._next_number, self._next_number + 1
        shape = Shape(kind, f"tmp {kind} {number}", (0.0, 0.0, NEW_SIZE[2] / 2), NEW_SIZE, source="temporary")
        self.temporary[number] = shape
        body_id = self._temporary_id(number)
        ctx.scene.put(self._body(body_id, shape))
        self._choose(body_id)
        if self._edit is not None:
            self._edit.value = True
        return body_id

    def remove(self, ctx: PluginContext, body_id: str) -> None:
        """Remove an obstacle, lab or temporary."""
        number = self._temporary_number(body_id)
        if number is not None:
            del self.temporary[number]
        elif body_id in self.shapes():
            self.lab = replace(self.lab, shapes=tuple(s for s in self.lab.shapes if shape_id(self.name, s) != body_id))
        else:
            return
        ctx.scene.remove(body_id)
        self._message = f"removed {body_id}"

    def promote(self, ctx: PluginContext, body_id: str) -> str | None:
        """Move a temporary obstacle into the lab (saved with the next Save); return its new id."""
        number = self._temporary_number(body_id)
        if number is None:
            self._message = "only a temporary obstacle can be promoted"
            return None
        shape = self.temporary.pop(number)
        shape = replace(shape, name=self._free_name(f"{shape.type} {number}"), source="promoted temporary")
        self.lab = replace(self.lab, shapes=self.lab.shapes + (shape,))
        ctx.scene.remove(body_id)
        new_id = shape_id(self.name, shape)
        ctx.scene.put(self._body(new_id, shape))
        self._choose(new_id)
        self._message = f"{body_id} is now {new_id}; Save to keep it"
        return new_id

    def rename(self, ctx: PluginContext, body_id: str, name: str) -> str | None:
        """Give a lab obstacle a new name, and so a new id; return it."""
        name = name.strip()
        shape = self.shapes().get(body_id)
        if shape is None or self._temporary_number(body_id) is not None or not name:
            self._message = "only a lab obstacle can be renamed, to a name that is not empty"
            return None
        if name == shape.name:
            return body_id
        new = replace(shape, name=self._free_name(name))
        new_id = self._replace(ctx, body_id, new)
        self._choose(new_id)
        return new_id

    def set_typed(self, ctx: PluginContext, body_id: str, position, yaw_degrees: float, size) -> None:
        """Set an obstacle's typed centre, yaw (degrees) and size."""
        shape = self.shapes().get(body_id)
        if shape is not None:
            new = replace(shape, center=tuple(float(v) for v in position), yaw=math.radians(float(yaw_degrees)))
            self._replace(ctx, body_id, new.resized(size))
            self._gizmo_on = None  # put the gizmos on it again in draw

    def set_border(self, ctx: PluginContext, x, y) -> None:
        """Set the usable floor's x and y range; the walls and the floor follow."""
        border = self.lab.border or Border((0.0, 0.0), (0.0, 0.0))
        x, y = tuple(sorted(float(v) for v in x)), tuple(sorted(float(v) for v in y))
        self.lab = replace(self.lab, border=replace(border, x=x, y=y))
        ctx.scene.put_many(self._border_and_floor(ctx))

    def _pick(self, body_id: str) -> None:
        """Select an obstacle picked in the 3D view."""
        self._choose(body_id)

    def _move(self, ctx: PluginContext, body_id: str, pose: Pose) -> None:
        """Move an obstacle to the gizmo's pose, upright."""
        shape = self.shapes().get(body_id)
        if shape is not None:
            moved = shape.moved(pose)
            self._replace(ctx, body_id, moved)
            if self._gizmo is not None:
                self._gizmo.follow(moved.pose, moved.size)

    def _resize(self, ctx: PluginContext, body_id: str, pose: Pose, size: tuple[float, float, float]) -> None:
        """Resize an obstacle to the corner gizmo."""
        shape = self.shapes().get(body_id)
        if shape is not None:
            self._replace(ctx, body_id, shape.moved(pose).resized(size))

    # --- --- --- --- --- HELPERS --- --- --- --- ---

    def _replace(self, ctx: PluginContext, body_id: str, new: Shape) -> str:
        """Put an edited shape in place of the one at `body_id`, in the model and the scene; return its id."""
        number = self._temporary_number(body_id)
        if number is not None:
            old, self.temporary[number] = self.temporary[number], new
            new_id = body_id
        else:
            old = self.shapes()[body_id]
            self.lab = replace(self.lab, shapes=tuple(new if shape_id(self.name, s) == body_id else s
                                                      for s in self.lab.shapes))
            new_id = shape_id(self.name, new)
        body = ctx.scene.bodies.get(body_id)
        if new_id != body_id or body is None or (new.type, new.size) != (old.type, old.size):
            # * A new Geometry only when the shape changed: mirrors rebuild a body on a new one.
            if new_id != body_id:
                ctx.scene.remove(body_id)
            ctx.scene.put(self._body(new_id, new))
        else:
            body.placement = new.pose
        return new_id

    def _put_all(self, ctx: PluginContext) -> None:
        """Put every obstacle, the border walls and the floor into the scene, replacing what this plugin had there."""
        ctx.scene.remove_prefix(f"{self.name}/")
        ctx.scene.put_many([self._body(body_id, shape) for body_id, shape in self.shapes().items()]
                           + self._border_and_floor(ctx))

    def _border_and_floor(self, ctx: PluginContext) -> list[Body]:
        """The border walls and the floor under them, which every robot's platform links may touch."""
        touches = tuple(link_id(robot_id(robot.serial), link) for robot in ctx.config.robots
                        for link in platform_links(robot.model))
        return border_bodies(self.name, self.lab.border) + [floor_body(self.name, self.lab.border, touches)]

    def _body(self, body_id: str, shape: Shape) -> Body:
        """The scene body of one obstacle."""
        color = TEMPORARY_COLOR if self._temporary_number(body_id) is not None else LAB_COLOR
        return Body(body_id, shape.geometry(mesh_directory_of(self.path)), shape.pose, label=shape.name, color=color)

    def _temporary_id(self, number: int) -> str:
        """The id of temporary obstacle `number`."""
        return f"{self.name}/temporary/{number}"

    def _temporary_number(self, body_id: str) -> int | None:
        """The number of a temporary obstacle's id, or None for any other id."""
        prefix = f"{self.name}/temporary/"
        rest = body_id[len(prefix):] if body_id.startswith(prefix) else ""
        return int(rest) if rest.isdigit() and int(rest) in self.temporary else None

    def _free_name(self, name: str) -> str:
        """`name`, or with " 2", " 3", ... added until no lab obstacle's id uses it."""
        taken = {slug(shape.name) for shape in self.lab.shapes}
        candidate, count = name, 1
        while slug(candidate) in taken:
            count += 1
            candidate = f"{name} {count}"
        return candidate

    def _choose(self, body_id: str) -> None:
        """Select an obstacle in the dropdown, adding it to the options first if it is new."""
        if self._select is None:
            return
        options = self._options()
        if tuple(self._select.options) != options:
            self._select.options = options
        if body_id in options:
            self._select.value = body_id

    def _options(self) -> tuple[str, ...]:
        """The dropdown's entries: every editable obstacle, or NONE_ENTRY."""
        return tuple(self.shapes()) or (NONE_ENTRY,)

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Show the lab's state and the selected obstacle in the fields, and keep the gizmos on it.

        Args:
            ctx: This plugin's context.
        """
        name = self.path.name if self.path else "no lab file"
        state = chip("unsaved", BUSY, "Save writes the lab file") if self.dirty else chip("saved", OK)
        self._lab_status.content = chip_line(chip(escape(name), NONE, str(self.path)) + state) + block(values(
            f"{len(self.lab.shapes)} obstacles, {len(self.temporary)} temporary"))
        if self.lab.border is not None:
            self._border_x.value = tuple(round(v, 3) for v in self.lab.border.x)
            self._border_y.value = tuple(round(v, 3) for v in self.lab.border.y)
        options = self._options()
        if tuple(self._select.options) != options:
            self._select.options = options
        body_id = self._select.value
        shape = self.shapes().get(body_id)
        self._gizmo.active = self._edit.value
        if body_id != self._gizmo_on or not self._edit.value:
            self._gizmo.place(body_id if shape else None, shape.pose if shape else None,
                              shape.size if shape else None, square=bool(shape and shape.square))
            self._gizmo_on = body_id if self._edit.value else None
            # * The name only when the selection changes, so typing in it is never overwritten.
            self._name.value = shape.name if shape else ""
        if shape is None:
            self._status.content = note("no obstacle: add one, or check the lab file")
            return
        # ? Rounded, so the fields are only rewritten when the obstacle really changes.
        self._position.value = tuple(round(v, 4) for v in shape.center)
        self._yaw.value = round(math.degrees(shape.yaw), 2)
        self._size.value = tuple(round(v, 4) for v in shape.size)
        kind = "temporary" if self._temporary_number(body_id) is not None else f"lab, {GROUPS[shape.type][:-1]}"
        lines = [f"{shape.name} ({kind})", shape.source]
        if self._message:
            lines.append(self._message)
        self._status.content = block(values(*(escape(line) for line in lines if line)))
