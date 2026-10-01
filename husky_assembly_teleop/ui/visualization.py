"""
The user interface: a viser server, the world drawn from each tick's snapshot, and one private corner
for each plugin.

- ! Build viser nodes once and keep the handles: re-adding them every tick leaks and flickers.
- ! viser callbacks run on viser's threads: they must not touch world state, the scene or ROS, only
  hand work to the plugin's queue (PluginContext.defer).
"""

from __future__ import annotations

from contextlib import AbstractContextManager, contextmanager
from dataclasses import dataclass
from html import escape
from pathlib import Path
from typing import Iterator

import numpy as np
import trimesh
import viser
import viser.extras
import yourdfpy

from ..config import RobotConfig
from ..world.scene import SceneSnapshot
from .quaternion import quaternion_to_wxyz
from .scene_view import SceneView
from ..tool_urdfs import resolve_mesh_path
from .style import FAIL


#: Mesh roughness, 0 = mirror, 1 = flat; just under 1 keeps edges readable.
MATTE_ROUGHNESS = 0.9

#: Shown in the browser tab and at the top of the control panel.
PAGE_TITLE = "Husky Monitor"

#: Fallback colour for a mesh that carries no colour of its own.
_DEFAULT_MESH_COLOR = (0.8, 0.8, 0.8, 1.0)

#: Joint change (rad or m) below which a robot is not re-posed; `ViserUrdf.update_cfg` costs 7-9 ms per robot.
JOINT_TOLERANCE = 1e-4


def joints_changed(values: np.ndarray, shown: np.ndarray | None) -> bool:
    """Whether joint values differ from those shown (None: nothing shown yet) by more than JOINT_TOLERANCE."""
    return shown is None or not np.allclose(values, shown, rtol=0.0, atol=JOINT_TOLERANCE)


def make_matte(mesh: trimesh.Trimesh) -> None:
    """Give one mesh a matte material in place, keeping its colour.

    Args:
        mesh: Mesh to restyle.
    """
    material = getattr(mesh.visual, "material", None)
    color = getattr(material, "main_color", None)
    if color is None:
        color = getattr(mesh.visual, "main_color", None)
    if color is None:
        color = _DEFAULT_MESH_COLOR

    mesh.visual = trimesh.visual.TextureVisuals(
        uv=getattr(mesh.visual, "uv", None),
        material=trimesh.visual.material.PBRMaterial(
            baseColorFactor=color,
            # Keep the original texture; only the shine changes.
            baseColorTexture=getattr(material, "image", None),
            metallicFactor=0.0,
            roughnessFactor=MATTE_ROUGHNESS,
        ),
    )


def load_urdf(urdf_file: Path) -> yourdfpy.URDF:
    """Parse a URDF for display (visual meshes only), resolving `package://` paths.

    ! Keep the filename handler: yourdfpy's default silently skips `package://` meshes.

    Args:
        urdf_file: URDF describing the robot as built.

    Returns:
        yourdfpy.URDF: The parsed model, with visual meshes loaded.
    """

    def resolve(fname: str) -> str:
        """Turn one mesh reference into an absolute path.

        ! Keep the name `fname`: yourdfpy passes it as a keyword.
        """
        return resolve_mesh_path(fname, urdf_file)

    model = yourdfpy.URDF.load(
        str(urdf_file),
        filename_handler=resolve,
        load_meshes=True,
        build_collision_scene_graph=False,
        load_collision_meshes=False,
    )
    for mesh in model.scene.geometry.values():
        make_matte(mesh)
    return model


def mesh_color(mesh: trimesh.Trimesh) -> tuple[int, int, int]:
    """A mesh's colour, RGB 0-255, from the material `make_matte` gave it."""
    return tuple(int(c) for c in mesh.visual.material.baseColorFactor[:3])


def add_simple_urdf(target, model: yourdfpy.URDF, root: str) -> viser.extras.ViserUrdf:
    """Draw a URDF below `root` as single-colour meshes, each in its own material's colour.

    ! Only the real robots (`Visualization.load_robot`) are drawn as GLB, with textures. Everything else uses
      this: single-colour meshes build faster and take `.opacity` at any time; GLB nodes have no opacity.

    Args:
        target: The viser server, or a plugin's view (PluginView).
        model: The URDF, from `load_urdf`.
        root: Scene path to draw it below.

    Returns:
        viser.extras.ViserUrdf: The drawn URDF; pose it with `update_cfg`, fade it through its meshes' `.opacity`.
    """
    # * A colour override makes ViserUrdf add single-colour meshes; each then gets its own colour back.
    urdf = viser.extras.ViserUrdf(target, model, root_node_name=root, mesh_color_override=_DEFAULT_MESH_COLOR[:3])
    # ? ViserUrdf adds one mesh node per visual mesh, in `scene.geometry` order.
    for handle, mesh in zip(urdf._meshes, model.scene.geometry.values()):
        handle.color = mesh_color(mesh)
    return urdf


@dataclass
class _DrawnRobot:
    """The handles for one robot in the viser scene.

    Attributes:
        base: Parent frame carrying the base pose; moves the whole robot.
        urdf: The mesh set, updated through `update_cfg`.
        joint_names: Actuated joint names, in `update_cfg` order.
        values: Joint values last drawn, or None before the first draw.
    """

    base: viser.FrameHandle
    urdf: viser.extras.ViserUrdf
    joint_names: tuple[str, ...]
    values: np.ndarray | None = None


class Visualization:
    """Owns the viser server and the robot and scene nodes. Plugins get a PluginView, never the server."""

    def __init__(self, port: int):
        """Start the viser server.

        Args:
            port: Port the web UI listens on.
        """
        self._server = viser.ViserServer(port=port, label=PAGE_TITLE, verbose=False)
        # * Widest panel: fixed-width number rows wrap at the default.
        self._server.gui.configure_theme(control_width="large")
        # ? Sets the browser tab title (the first <title> on the page wins). HTML, not markdown,
        #   which viser pads into an empty row.
        self._server.gui.add_html(f"<title>{PAGE_TITLE}</title>")

        # * Hidden until a plugin is stopped; see `show_broken`.
        self._broken = self._server.gui.add_html("", visible=False)

        # * Soft stop: a button and Esc both raise a flag the monitor reads next tick.
        #   ! Esc works only in the focused tab and not inside a text field. Not an e-stop.
        self._stop_requested = False
        stop = self._server.gui.add_button("Stop all (Esc)", color="red", icon=viser.Icon.HAND_STOP,
                                           hint="Soft stop: stop every arm's program, switch every base "
                                                "controller off, cancel every plugin task. Resume in the "
                                                "health panel. Not an emergency stop.")
        stop.on_click(self._request_stop)
        self._server.gui.add_command("Stop all robots", description="Soft stop of every robot",
                                     hotkey="escape", icon=viser.Icon.HAND_STOP).on_trigger(self._request_stop)

        # * Freeze: while frozen the monitor skips every plugin's `draw`, so panel text can be selected
        #   (each rewrite drops the selection). Controls, updates and tasks keep running.
        self._frozen = False
        self._freeze = self._server.gui.add_button("Live", color="green", icon=viser.Icon.ACTIVITY)
        self._freeze.on_click(self._toggle_freeze)
        self._show_freeze_state()

        # Per-robot handles by serial, built once in load_robot.
        self._robots: dict[str, _DrawnRobot] = {}

        self._scene_view = SceneView(self._server)

        self._server.scene.add_grid("/grid", width=10.0, height=10.0)
        self._server.scene.add_frame("/origin", show_axes=True)

    def load_robot(self, config: RobotConfig) -> None:
        """Add one robot's meshes to the scene at its default pose. Called once at startup: loading is slow.

        Args:
            config: Identity, URDF and default pose for this robot.
        """
        # The base pose goes on a parent frame, so `draw` writes one pose plus one joint vector.
        root = f"/robots/{config.serial}"
        base = self._server.scene.add_frame(root, show_axes=False)
        base.position = config.default_position
        base.wxyz = quaternion_to_wxyz(config.default_orientation)

        model = load_urdf(config.urdf_file)
        drawn = viser.extras.ViserUrdf(self._server, model, root_node_name=root)
        self._robots[config.serial] = _DrawnRobot(
            base=base, urdf=drawn, joint_names=tuple(drawn.get_actuated_joint_names()))

    def view_for(self, plugin_name: str) -> PluginView:
        """Carve out a private scene subtree and GUI folder for one plugin.

        Args:
            plugin_name: Unique plugin name; becomes the scene path segment.

        Returns:
            PluginView: The plugin's slice of the UI.
        """
        return PluginView(self._server, plugin_name)

    def atomic(self) -> AbstractContextManager[None]:
        """Batch every scene change made inside the block into one update.

        Returns:
            AbstractContextManager[None]: Browsers never render a half-applied update.
        """
        return self._server.atomic()

    @property
    def frozen(self) -> bool:
        """bool: Whether the operator froze the panels. Read by the monitor each tick."""
        return self._frozen

    def _request_stop(self, _event: object) -> None:
        """Ask for a soft stop. A viser callback; only sets a flag the tick reads."""
        self._stop_requested = True

    def take_stop_request(self) -> bool:
        """Whether a soft stop was asked for since the last call. Read by the monitor each tick.

        Returns:
            bool: True once per request; several presses within one tick count as one.
        """
        if not self._stop_requested:
            return False
        self._stop_requested = False
        return True

    def show_broken(self, stopped: dict[str, str]) -> None:
        """Show a red banner at the top of the panel: which plugins are stopped, and to restart.

        Args:
            stopped: Why each stopped plugin stopped, by name, in the order they stopped.
        """
        rows = "".join(f"<li><b>{escape(name)}</b>: {escape(reason)}</li>" for name, reason in stopped.items())
        self._broken.content = (
            f'<div style="background:{FAIL};color:#fff;border-radius:4px;padding:6px 10px;'
            f'font-size:12px"><b>⚠ Monitor broken: restart it.</b>'
            f'<ul style="margin:4px 0 0;padding-left:18px">{rows}</ul>'
            f'<div style="margin-top:4px">Stopped plugins no longer act on their buttons. '
            f'Details are in the log.</div></div>')
        self._broken.visible = True

    def _toggle_freeze(self, _event: viser.GuiEvent) -> None:
        """Freeze or unfreeze the panels. A viser callback; only flips a flag the tick reads."""
        self._frozen = not self._frozen
        self._show_freeze_state()

    def _show_freeze_state(self) -> None:
        """Make the freeze button show the current state."""
        if self._frozen:
            self._freeze.label, self._freeze.color, self._freeze.icon = "Frozen", "orange", viser.Icon.SNOWFLAKE
            self._freeze.hint = "Panels are paused, so their text can be selected. Click to go live again."
        else:
            self._freeze.label, self._freeze.color, self._freeze.icon = "Live", "green", viser.Icon.ACTIVITY
            self._freeze.hint = ("Click to pause every panel, so its text can be selected and copied. "
                                 "Buttons still act; the robots in the 3D view stay live.")

    def draw(self, snapshot: SceneSnapshot) -> None:
        """Draw this tick's copy of the world: robots, tracked objects and scene bodies.

        Called once per tick inside `atomic()`, also while frozen.

        Args:
            snapshot: The tick's copy of the world.
        """
        for serial, entry in snapshot.robots.items():
            drawn = self._robots.get(serial)
            if drawn is None:
                continue
            drawn.base.position = entry.base.position
            drawn.base.wxyz = quaternion_to_wxyz(entry.base.orientation)
            values = np.array([entry.joints.get(name, 0.0) for name in drawn.joint_names])
            if joints_changed(values, drawn.values):
                drawn.urdf.update_cfg(values)
                drawn.values = values
        self._scene_view.sync(snapshot)

    def stop(self) -> None:
        """Shut the server down and join its thread."""
        self._server.stop()


class PluginView:
    """One plugin's private scene subtree and GUI folder in viser, handed to it as `ctx.view`."""

    def __init__(self, server: viser.ViserServer, plugin_name: str):
        """Create the plugin's scene root and GUI folder.

        Args:
            server: The shared viser server.
            plugin_name: Unique plugin name.
        """
        self._server = server
        self._name = plugin_name

        #: Scene path owned by this plugin, e.g. "/plugins/calibration". Everything it adds hangs below it.
        self.scene_root = f"/plugins/{plugin_name}"
        self._root = server.scene.add_frame(self.scene_root, show_axes=False)
        # ? Created on first use of `ui()`, so a panel-only plugin leaves no empty folder.
        self._folder: viser.GuiFolderHandle | None = None
        #: Separate panels made through `panel()`, removed with everything else.
        self._panels: list[viser.PanelHandle] = []

    @property
    def scene(self) -> viser.SceneApi:
        """viser.SceneApi: Scene api. Paths passed to it must start with scene_root."""
        return self._server.scene

    @property
    def gui(self) -> viser.GuiApi:
        """viser.GuiApi: GUI api. Prefer `ui()`, which also parents to the folder."""
        return self._server.gui

    @contextmanager
    def ui(self) -> Iterator[viser.GuiApi]:
        """Add GUI widgets inside this plugin's folder; only widgets created inside the block land there.

        Yields:
            viser.GuiApi: The GUI api, with this plugin's folder as parent.
        """
        if self._folder is None:
            self._folder = self._server.gui.add_folder(self._name)
        with self._folder:
            yield self._server.gui

    def panel(self) -> viser.PanelHandle:
        """Create a panel owned by this plugin: a window beside the main one.

        ! Always use this instead of `gui.add_panel()`: only panels made here are removed on teardown.

        Returns:
            viser.PanelHandle: The new panel, not yet placed.
        """
        panel = self._server.gui.add_panel()
        self._panels.append(panel)
        return panel

    def clear(self) -> None:
        """Remove everything this plugin added: scene nodes, its folder, its panels."""
        self._root.remove()
        if self._folder is not None:
            self._folder.remove()
        for panel in self._panels:
            panel.remove()
        self._panels.clear()
