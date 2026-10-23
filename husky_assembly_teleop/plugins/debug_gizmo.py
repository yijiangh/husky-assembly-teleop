"""
Quick debug tool: click any scene body with a world pose, drag it with a gizmo, and log where it ends up.

Clicking in the 3D view picks the nearest body under the cursor; the dropdown does the same. Position and roll,
pitch, yaw can also be typed in. Log prints the pose; Reset puts the body back where it was first seen.
Tick "Edit" (off at start) to show the gizmo and pick by click; viser does not tell the server whether a folder is open.
To edit the lab's obstacles for good (move, resize, add, save), use the `obstacles` plugin.

! Writes other plugins' live bodies directly, which plugins normally must not do. Debug only: the moves are never
  saved, and the owner may put the body back at any time.

Run with:  -p plugins:="['obstacles', 'debug_gizmo']"
"""

from __future__ import annotations

from html import escape

import viser
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..ui.body_gizmo import BodyGizmo
from ..ui.style import SECTION_CTRL, block, note, numbers, section, values

#: Dropdown entry while no body can be moved.
NONE_ENTRY = "(none)"


def _rpy(pose: Pose) -> tuple[float, float, float]:
    """Roll, pitch, yaw of `pose`, degrees."""
    return tuple(Rotation.from_quat(pose.orientation).as_euler("xyz", degrees=True))


@register
class DebugGizmoPlugin(HuskyPlugin):
    """Moves any world-placed scene body with a gizmo and logs its pose."""

    name = "debug_gizmo"
    icon = viser.Icon.BUG
    hidden = True

    def __init__(self):
        """Start with nothing selected."""
        # Each body's pose when first seen, by id; for Reset and the shown offset.
        self._original: dict[str, Pose] = {}
        self._gizmo: BodyGizmo | None = None
        # The body the gizmo was last put on.
        self._gizmo_on: str | None = None

    def setup(self, ctx: PluginContext) -> None:
        """Build the widgets and the (hidden) gizmo.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            gui.add_html(section("move body", SECTION_CTRL))
            self._edit = gui.add_checkbox("Edit", False, hint="Show the gizmo and pick bodies by clicking in 3D.")
            self._select = gui.add_dropdown("Body", [NONE_ENTRY])
            self._position = gui.add_vector3("Position m", (0.0, 0.0, 0.0), step=0.001)
            self._rpy = gui.add_vector3("RPY °", (0.0, 0.0, 0.0), step=0.1,
                                        hint="Roll, pitch, yaw about fixed X, Y, Z, degrees.")
            actions = gui.add_button_group("Selected", ["Log", "Reset"])
            self._status: viser.GuiHtmlHandle = gui.add_html("")
        self._gizmo = BodyGizmo(ctx, "gizmo", candidates=lambda: self._movable(ctx),
                                on_pick=self._pick, on_move=lambda body_id, pose: self._move(ctx, body_id, pose))

        # ! Callbacks run on a viser thread: hand every action to the main thread.
        actions.on_click(ctx.defer_value("selected body", lambda clicked: {
            "Log": self._log, "Reset": self._reset}[clicked](ctx)))

        async def _typed(event: viser.GuiEvent) -> None:
            # * Skip our own writes from `draw`: viser calls back for those too, without a client.
            if event.client is None:
                return
            body_id, position, rpy = self._select.value, self._position.value, self._rpy.value
            ctx.submit("type pose", lambda: self._set_pose(ctx, body_id, position, rpy))

        self._position.on_update(_typed)
        self._rpy.on_update(_typed)

    def teardown(self, ctx: PluginContext) -> None:
        """Remove the scene click callback.

        Args:
            ctx: This plugin's context.
        """
        if self._gizmo is not None:
            self._gizmo.remove()

    # --- --- --- --- --- ACTIONS (run as intents) --- --- --- --- ---

    def _pick(self, body_id: str) -> None:
        """Select a body picked in the 3D view."""
        if body_id in self._select.options:
            self._select.value = body_id

    def _movable(self, ctx: PluginContext) -> list[str]:
        """Every body with a world pose of its own (not attached), sorted."""
        return sorted(body_id for body_id in ctx.scene.bodies if self._pose(ctx, body_id) is not None)

    def _pose(self, ctx: PluginContext, body_id: str) -> Pose | None:
        """The world pose of body `body_id`, or None if it is gone or attached."""
        body = ctx.scene.bodies.get(body_id)
        return body.placement if body is not None and isinstance(body.placement, Pose) else None

    def _move(self, ctx: PluginContext, body_id: str, pose: Pose) -> None:
        """Set body `body_id` to the gizmo pose."""
        if self._pose(ctx, body_id) is not None:
            ctx.scene.bodies[body_id].placement = pose

    def _set_pose(self, ctx: PluginContext, body_id: str, position: tuple[float, ...], rpy: tuple[float, ...]) -> None:
        """Set body `body_id` to a typed position and roll, pitch, yaw (degrees), and move the gizmo with it."""
        if self._pose(ctx, body_id) is not None:
            orientation = Rotation.from_euler("xyz", rpy, degrees=True).as_quat()
            ctx.scene.bodies[body_id].placement = Pose.from_arrays(position, orientation)
            self._gizmo_on = None  # put the gizmo on it again in draw

    def _log(self, ctx: PluginContext) -> None:
        """Log the selected body's pose: position, quaternion (x, y, z, w), roll, pitch, yaw."""
        body_id = self._select.value
        pose = self._pose(ctx, body_id)
        if pose is None:
            return
        ctx.log_info(f"{body_id}: position=({', '.join(f'{v:.4f}' for v in pose.position)}) "
                     f"orientation=({', '.join(f'{v:.4f}' for v in pose.orientation)}) "
                     f"rpy_deg=({', '.join(f'{v:.2f}' for v in _rpy(pose))})")

    def _reset(self, ctx: PluginContext) -> None:
        """Put the selected body back where it was when first seen."""
        body_id = self._select.value
        if self._pose(ctx, body_id) is not None and body_id in self._original:
            ctx.scene.bodies[body_id].placement = self._original[body_id]
            self._gizmo_on = None

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def _sync_options(self, ctx: PluginContext) -> None:
        """List every movable body in the dropdown, and remember the pose of new ones."""
        ids = self._movable(ctx)
        for body_id in ids:
            self._original.setdefault(body_id, self._pose(ctx, body_id))
        options = tuple(ids) or (NONE_ENTRY,)
        if tuple(self._select.options) != options:
            self._select.options = options

    def draw(self, ctx: PluginContext) -> None:
        """Keep the body list current, and show the selected body's pose in the fields and its offset.

        Args:
            ctx: This plugin's context.
        """
        self._sync_options(ctx)
        body_id = self._select.value
        pose = self._pose(ctx, body_id)
        self._gizmo.active = self._edit.value
        if body_id != self._gizmo_on or not self._edit.value:
            self._gizmo.place(body_id, pose)
            self._gizmo_on = body_id if self._edit.value else None
        if pose is None:
            self._status.content = note("no movable body: load a plugin that puts bodies, e.g. obstacles")
            return
        original = self._original[body_id]
        offset = tuple(a - b for a, b in zip(pose.position, original.position))
        # ? Rounded, so the fields are only rewritten when the pose really changes.
        self._position.value = tuple(round(v, 4) for v in pose.position)
        self._rpy.value = tuple(round(v, 2) for v in _rpy(pose))
        self._status.content = block(escape(ctx.scene.snapshot.label(body_id)) + values(
            f"Δ   {numbers(offset, 3, 8, 3)} m"))
