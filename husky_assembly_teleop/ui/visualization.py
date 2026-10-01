"""
The user interface: a viser server, the world drawn from each tick's snapshot,
and one private corner of both for each plugin.

! Build viser nodes once and keep the handles; later assign `position` / `wxyz` /
  `visible`. Re-adding nodes every tick leaks and flickers.

! viser callbacks run on its worker threads, not the main thread. They must not
  touch world state, the scene or ROS, only hand work to the plugin's queue
  (PluginContext.defer).
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


#: Mesh roughness, 0 = mirror, 1 = flat. The shipped meshes look like wet
#: plastic; just under 1 keeps edges readable.
MATTE_ROUGHNESS = 0.9

#: Shown in the browser tab and at the top of the control panel.
PAGE_TITLE = "Husky Monitor"

#: Fallback colour for a mesh that carries no colour of its own.
_DEFAULT_MESH_COLOR = (0.8, 0.8, 0.8, 1.0)


def make_matte(mesh: trimesh.Trimesh) -> None:
    """Give one mesh a matte material in place, keeping its colour.

    ! Must be done on the mesh: `ViserUrdf` takes no material arguments.

    Args:
        mesh: Mesh to restyle. Modified in place.
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

    ! Keep the explicit filename handler: yourdfpy's default leaves `package://`
      unresolved and silently skips the mesh, giving an empty robot with no error.

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


@dataclass
class _DrawnRobot:
    """The handles for one robot in the viser scene.

    Attributes:
        base: Parent frame carrying the base pose; moves the whole robot.
        urdf: The mesh set, updated through `update_cfg`.
        joint_names: Actuated joint names, in `update_cfg` order.
    """

    base: viser.FrameHandle
    urdf: viser.extras.ViserUrdf
    joint_names: tuple[str, ...]


class Visualization:
    """Owns the viser server and the scene nodes mirroring world and cell state.

    ! The server handle stays in here. Plugins get a PluginView, the monitor
      gets `atomic()`, `draw()` and `stop()`. Nothing above holds a ViserServer.
    """

    def __init__(self, port: int):
        """Start the viser server.

        Args:
            port: Port the web UI listens on.
        """
        self._server = viser.ViserServer(port=port, label=PAGE_TITLE, verbose=False)
        # * The widest panel viser offers. The status rows are fixed-width
        #   monospace numbers, which wrap and become unreadable at the default.
        self._server.gui.configure_theme(control_width="large")
        # ? viser has no setting for the browser tab title; its page says
        #   "Viser". But the browser takes the first <title> found anywhere on
        #   the page, so an otherwise invisible widget sets it.
        #   ! An HTML widget, not markdown: viser wraps markdown in a padded
        #     box, which left an empty row at the top of the panel.
        self._server.gui.add_html(f"<title>{PAGE_TITLE}</title>")

        # * Top of the panel, hidden until a plugin is stopped for failing. The
        #   monitor is then broken and must be restarted; see `show_broken`.
        self._broken = self._server.gui.add_html("", visible=False)

        # * Soft stop of every robot, first in the panel: a button, and Esc from
        #   anywhere on the page. Both only raise a flag; the monitor reads it
        #   at the start of the next tick and does the stopping on its thread.
        #   ! Esc reaches only the browser tab that has focus, and does nothing
        #     while the cursor is in a text or number field. Not an e-stop.
        self._stop_requested = False
        stop = self._server.gui.add_button("Stop all (Esc)", color="red", icon=viser.Icon.HAND_STOP,
                                           hint="Soft stop: stop every arm's program, switch every base "
                                                "controller off, cancel every plugin task. Resume in the "
                                                "health panel. Not an emergency stop.")
        stop.on_click(self._request_stop)
        self._server.gui.add_command("Stop all robots", description="Soft stop of every robot",
                                     hotkey="escape", icon=viser.Icon.HAND_STOP).on_trigger(self._request_stop)

        # * Global freeze, first in the panel so it sits above every plugin.
        #   ? Why. Panels are rewritten every tick, and each rewrite drops the
        #     browser's text selection, so live numbers cannot be copied. viser
        #     has no clipboard call and HTML cannot call back, so the way to copy
        #     is to stop the rewriting: while frozen, the monitor skips every
        #     plugin's `draw`. Controls, updates and tasks all keep running.
        # * One button shows the state and toggles it: green "Live", or orange
        #   "Frozen". A single full-width row, and it never changes height.
        self._frozen = False
        self._freeze = self._server.gui.add_button("Live", color="green", icon=viser.Icon.ACTIVITY)
        self._freeze.on_click(self._toggle_freeze)
        self._show_freeze_state()

        # Persistent per-robot handles, keyed by serial. Built once in
        # load_robot, mutated in draw, never rebuilt per tick.
        self._robots: dict[str, _DrawnRobot] = {}

        # * The scene's bodies and the tracked objects, drawn generically: the
        #   core has no idea what a bar or a rack is. Plugins draw only their
        #   own extras (targets, paths, markers) in their PluginView.
        self._scene_view = SceneView(self._server)

        self._server.scene.add_grid("/grid", width=10.0, height=10.0)
        self._server.scene.add_frame("/origin", show_axes=True)

    def load_robot(self, config: RobotConfig) -> None:
        """Add one robot's meshes to the scene, at its default pose. Called once.

        ! Built here rather than on first sight in draw, because viser is
          retained mode: the meshes go in once and `draw` only assigns
          transforms afterwards. Loading 50-odd meshes takes a moment, which is
          fine at startup and would not be inside a 20 Hz tick.

        Args:
            config: Identity, URDF and default pose for this robot.
        """
        # The base pose goes on a parent frame rather than on every mesh: moving
        # the frame carries the whole robot, so draw only ever writes one pose
        # plus one joint vector. It starts at the default pose, which `draw`
        # leaves alone until mocap has a fix -- so a fleet is spread out from
        # the first frame rather than piled up at the origin.
        root = f"/robots/{config.serial}"
        base = self._server.scene.add_frame(root, show_axes=False)
        base.position = config.default_position
        base.wxyz = quaternion_to_wxyz(config.default_orientation)

        model = load_urdf(config.urdf_file)
        drawn = viser.extras.ViserUrdf(self._server, model, root_node_name=root)
        self._robots[config.serial] = _DrawnRobot(
            base=base, urdf=drawn, joint_names=tuple(drawn.get_actuated_joint_names()))

    def view_for(self, plugin_name: str) -> "PluginViewImpl":
        """Carve out a private scene subtree and GUI folder for one plugin.

        Args:
            plugin_name: Unique plugin name; becomes the scene path segment.

        Returns:
            PluginViewImpl: The plugin's slice of the UI.
        """
        return PluginViewImpl(self._server, plugin_name)

    def atomic(self) -> AbstractContextManager[None]:
        """Batch every scene change made inside the block into one update.

        Returns:
            AbstractContextManager[None]: Inside it, browsers cannot render a
                frame where the robot has moved but the bar it holds has not.
        """
        return self._server.atomic()

    @property
    def frozen(self) -> bool:
        """bool: Whether the operator froze the panels. Read by the monitor each tick."""
        return self._frozen

    def _request_stop(self, _event: object) -> None:
        """Ask for a soft stop. A viser callback (button or Esc), on a viser thread.

        ? Not an intent, like the freeze: it belongs to the core, and only sets
          one bool that the tick reads.
        """
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
        """Freeze or unfreeze the panels. A viser callback, on a viser thread.

        ? Not an intent: this belongs to the core, not a plugin, and all it does
          is flip one bool that the tick reads. A single assignment is safe
          across threads; the worst case is a tick drawn one step late.
        """
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

        Called once per tick inside `atomic()`, also while panels are frozen.
        Robots only get new transforms; the scene view builds what is new.

        ? Robots show what planners get: the last mocap fix (the default pose
          before any) and the last value of each joint, as Kinematics keeps them.

        Args:
            snapshot: The tick's copy of the world.
        """
        for serial, entry in snapshot.robots.items():
            drawn = self._robots.get(serial)
            if drawn is None:
                continue
            drawn.base.position = entry.base.position
            drawn.base.wxyz = quaternion_to_wxyz(entry.base.orientation)
            drawn.urdf.update_cfg(np.array([entry.joints.get(name, 0.0) for name in drawn.joint_names]))
        self._scene_view.sync(snapshot)

    def stop(self) -> None:
        """Shut the server down and join its thread."""
        self._server.stop()


class PluginViewImpl:
    """One plugin's private scene subtree and GUI folder.

    Satisfies the PluginView Protocol in plugin_api/context.py structurally, so this module
    need not import it and plugins need not import this one.
    """

    def __init__(self, server: viser.ViserServer, plugin_name: str):
        """Create the plugin's scene root and GUI folder.

        Args:
            server: The shared viser server.
            plugin_name: Unique plugin name.
        """
        self._server = server
        self._name = plugin_name

        # Everything the plugin adds hangs below this frame, so two plugins
        # cannot collide on a path and cleanup is a single remove().
        self.scene_root = f"/plugins/{plugin_name}"
        self._root = server.scene.add_frame(self.scene_root, show_axes=False)
        # ? Created on first use of `ui()`, so a plugin that only uses separate
        #   panels leaves no empty folder in the main panel. Plugins set up in
        #   dependency order, so folders still appear in that order.
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
        """Add GUI widgets inside this plugin's folder.

        viser picks a widget's parent from a thread-local "current container"
        that a folder handle sets while entered, so widgets only land in the
        right folder when created inside this block.

        Yields:
            viser.GuiApi: The GUI api, with this plugin's folder as parent.
        """
        if self._folder is None:
            self._folder = self._server.gui.add_folder(self._name)
        with self._folder:
            yield self._server.gui

    def panel(self) -> viser.PanelHandle:
        """Create a separate panel owned by this plugin. See PluginView.panel.

        Returns:
            viser.PanelHandle: The new panel, not yet placed.
        """
        panel = self._server.gui.add_panel()
        self._panels.append(panel)
        return panel

    def clear(self) -> None:
        """Remove everything this plugin added: scene nodes, its folder, its panels.

        Removing the root frame removes the whole subtree below it, so plugins
        need not track their own scene handles just to clean up.
        """
        self._root.remove()
        if self._folder is not None:
            self._folder.remove()
        for panel in self._panels:
            panel.remove()
        self._panels.clear()
