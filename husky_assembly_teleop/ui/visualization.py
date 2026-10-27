"""
The user interface: the desktop and phone pages, the world drawn each tick, and one private corner per plugin.

- ! Build viser nodes once and keep the handles: re-adding them every tick leaks and flickers.
- ! viser callbacks run on viser's threads: they must not touch world state, the scene or ROS, only
  hand work to the plugin's queue (PluginContext.defer).
"""

from __future__ import annotations

from contextlib import AbstractContextManager, contextmanager
from dataclasses import dataclass, field
from functools import cache
from html import escape
from pathlib import Path
from typing import Callable, Iterator, Mapping

import numpy as np
import trimesh
import viser
import viser.extras
import viser.transforms
import yourdfpy

from ..config import RobotConfig
from scaffolding_assembly_core.robot import Tool
from scaffolding_assembly_core.scene import Scene
from .phone import PHONE_TITLE, PhonePage
from .quaternion import quaternion_to_wxyz
from .scene_view import SceneView, add_shape
from ..tool_urdfs import resolve_mesh_path
from .style import FAIL, NESTED_TABS_STYLE, PATH_TABS_MARKER


#: Mesh roughness, 0 = mirror, 1 = flat; just under 1 keeps edges readable.
MATTE_ROUGHNESS = 0.9

#: Shown in the browser tab and at the top of the control panel.
PAGE_TITLE = "Husky Monitor"

#: Fallback colour for a mesh that carries no colour of its own.
_DEFAULT_MESH_COLOR = (0.8, 0.8, 0.8, 1.0)

#: Colour of the robots' collision meshes, RGB 0-1, when the view shows collision shapes.
COLLISION_COLOR = (0.9, 0.35, 0.35)

#: Colour of the mounted tools, RGB 0-1; their models carry none.
TOOL_COLOR = (0.27, 0.27, 0.31)

#: Joint change (rad or m) below which a robot is not re-posed.
JOINT_TOLERANCE = 1e-4

#: Starting width of the side panel, pixels. Fits an arm's joint row, the widest line.
SIDE_PANEL_WIDTH = 420

#: Where a tab goes in a side panel: (label, icon) per level, outermost first.
TabPath = tuple[tuple[str, viser.IconName | None], ...]


def joints_changed(values: np.ndarray, shown: np.ndarray | None) -> bool:
    """Whether joint values differ from those shown (None: nothing shown yet) by more than JOINT_TOLERANCE."""
    return shown is None or not np.allclose(values, shown, rtol=0.0, atol=JOINT_TOLERANCE)


@dataclass(eq=False)
class _MovingJoint:
    """One non-fixed joint's frame and what poses it.

    Attributes:
        frame: The child link's frame, posed relative to the parent link's.
        prismatic: True if it slides, False if it turns.
        index: Index of the value driving it in `update_cfg`'s vector.
        multiplier: Mimic multiplier, 1 if not a mimic joint.
        offset: Mimic offset, 0 if not a mimic joint.
        wxyz: Rotation of the joint origin.
        position: Position of the joint origin, scaled.
        axis: Unit joint axis in the parent frame (origin rotation applied); scaled if prismatic.
    """

    frame: viser.FrameHandle
    prismatic: bool
    index: int
    multiplier: float
    offset: float
    wxyz: tuple[float, float, float, float]
    position: np.ndarray
    axis: np.ndarray


class FastViserUrdf(viser.extras.ViserUrdf):
    """A `ViserUrdf` whose `update_cfg` re-poses only the frames of moving joints, from each joint's origin and axis.

    ? viser's own reads every joint's transform back from the model's scene graph and converts it to a quaternion:
      7 ms for a 38-joint robot, of which the kinematics is 0.4 ms.
    ! Does not pose the yourdfpy model: pose it yourself before reading `get_transform` from it.
    """

    def __init__(self, *args, **kwargs):
        """As `ViserUrdf`; poses the model's meshes at zero joints.

        Raises:
            NotImplementedError: If a joint is floating or planar.
        """
        super().__init__(*args, **kwargs)
        index = {name: i for i, name in enumerate(self._urdf.actuated_joint_names)}
        self._moving: list[_MovingJoint] = []
        # ? Visual and collision trees repeat the joints; `_joint_map_values` lines up with `_joint_frames`.
        for joint, frame in zip(self._joint_map_values, self._joint_frames):
            origin = np.eye(4) if joint.origin is None else joint.origin
            wxyz = tuple(viser.transforms.SO3.from_matrix(origin[:3, :3]).wxyz)
            position = origin[:3, 3] * self._scale
            frame.wxyz, frame.position = wxyz, position
            if joint.type == "fixed":
                continue
            if joint.type not in ("revolute", "continuous", "prismatic"):
                raise NotImplementedError(f"joint {joint.name!r} is {joint.type}")
            mimic = joint.mimic
            axis = np.asarray(joint.axis, dtype=float) / np.linalg.norm(joint.axis)
            prismatic = joint.type == "prismatic"
            self._moving.append(_MovingJoint(
                frame=frame, prismatic=prismatic,
                index=index[joint.name if mimic is None else mimic.joint],
                multiplier=1.0 if mimic is None or mimic.multiplier is None else mimic.multiplier,
                offset=0.0 if mimic is None or mimic.offset is None else mimic.offset,
                wxyz=wxyz, position=position,
                axis=origin[:3, :3] @ axis * self._scale if prismatic else axis))

    def update_cfg(self, configuration: np.ndarray) -> None:
        """Pose the moving joints' frames at these values, in `get_actuated_joint_names` order."""
        for joint in self._moving:
            value = float(configuration[joint.index]) * joint.multiplier + joint.offset
            if joint.prismatic:
                joint.frame.position = joint.position + joint.axis * value
            else:
                # Origin rotation, then the turn about the axis: q_origin * (cos(a/2), sin(a/2) axis).
                w1, x1, y1, z1 = joint.wxyz
                s = np.sin(value / 2)
                w2, x2, y2, z2 = np.cos(value / 2), s * joint.axis[0], s * joint.axis[1], s * joint.axis[2]
                joint.frame.wxyz = (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                                    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                                    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                                    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)

    def add_tools(self, tools: Mapping[str, Tool], color: tuple[float, ...] = TOOL_COLOR,
                  collision_color: tuple[float, ...] = COLLISION_COLOR) -> None:
        """Hang each mounted tool's shapes on its flange's frame, so they move with the arm in the same update.

        Visual shapes go below the visual tree's flange and collision shapes below the collision tree's, so
        `show_visual` / `show_collision` switch them with the links. Their handles join `_meshes`.

        Args:
            tools: Flange link -> mounted tool, as in `RobotModel.tools`.
            color: Colour of the visual shapes, (r, g, b) or (r, g, b, a) from 0 to 1.
            collision_color: Colour of the collision shapes, the same way.
        """
        visual_root = f"{self._root_node_name}/visual".replace("//", "/")
        flanges: dict[tuple[str, str], viser.SceneNodeHandle] = {}
        for joint, frame in zip(self._joint_map_values, self._joint_frames):
            if joint.child in tools:
                tree = "visual" if frame.name.startswith(f"{visual_root}/") else "collision"
                flanges[(tree, joint.child)] = frame
        for flange, tool in tools.items():
            for tree, shapes, look in (("visual", tool.geometry.visual, color),
                                       ("collision", tool.geometry.collision, collision_color)):
                frame = flanges.get((tree, flange))
                if frame is None:
                    continue  # that tree is not drawn
                self._meshes.extend(add_shape(self._target.scene, f"{frame.name}/tool_{index}", shape, look)
                                    for index, shape in enumerate(shapes))


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


def load_urdf(urdf_file: Path, collision: bool = False) -> yourdfpy.URDF:
    """Parse a URDF for display, resolving `package://` paths.

    ! Keep the filename handler: yourdfpy's default silently skips `package://` meshes.

    Args:
        urdf_file: URDF describing the robot as built.
        collision: Also load the collision meshes, for `ViserUrdf(load_collision_meshes=True)`.

    Returns:
        yourdfpy.URDF: The parsed model, with visual meshes (and collision meshes if asked) loaded.
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
        build_collision_scene_graph=collision,
        load_collision_meshes=collision,
    )
    for mesh in model.scene.geometry.values():
        make_matte(mesh)
    return model


@cache
def shared_urdf(urdf_file: Path) -> yourdfpy.URDF:
    """`load_urdf` with collision meshes, parsed once per file and shared.

    Each robot and ghost would otherwise re-read its meshes (about 1 s per robot). ? The collision meshes add
    about 0.1 s per robot.

    ! `FastViserUrdf` never poses it, and anyone else posing it would change it for every user: never read
      `get_transform` from it. Main thread only.

    Args:
        urdf_file: URDF describing the robot as built.

    Returns:
        yourdfpy.URDF: The parsed model, shared by every caller with this file.
    """
    return load_urdf(urdf_file, collision=True)


def mesh_color(mesh: trimesh.Trimesh) -> tuple[int, int, int]:
    """A mesh's colour, RGB 0-255, from the material `make_matte` gave it."""
    return tuple(int(c) for c in mesh.visual.material.baseColorFactor[:3])


def add_simple_urdf(target, model: yourdfpy.URDF, root: str) -> FastViserUrdf:
    """Draw a URDF below `root` as single-colour meshes, each in its own material's colour.

    ! Only the real robots (`Visualization.load_robot`) are drawn as GLB, with textures. Everything else uses
      this: single-colour meshes build faster and take `.opacity` at any time; GLB nodes have no opacity.

    Args:
        target: The viser server, or a plugin's view (PluginView).
        model: The URDF, from `load_urdf`.
        root: Scene path to draw it below.

    Returns:
        FastViserUrdf: The drawn URDF; pose it with `update_cfg`, fade it through its meshes' `.opacity`.
    """
    # * A colour override makes ViserUrdf add single-colour meshes; each then gets its own colour back.
    urdf = FastViserUrdf(target, model, root_node_name=root, mesh_color_override=_DEFAULT_MESH_COLOR[:3])
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
    urdf: FastViserUrdf
    joint_names: tuple[str, ...]
    values: np.ndarray | None = None
    #: The tools on its flanges, for the see-through twin; the twin (single-colour meshes, made on first fade) and
    #: its opacity, None while the robot is drawn solid.
    tools: Mapping[str, Tool] = field(default_factory=dict)
    urdf_file: Path | None = None
    twin: FastViserUrdf | None = None
    opacity: float | None = None


def pinned_server(port: int, parameter: str, label: str) -> viser.ViserServer:
    """Start a viser server on exactly `port`, so bookmarks keep working; 0 takes any free port.

    Args:
        port: The port.
        parameter: The ROS parameter that sets it, for the error message.
        label: The server's label.

    Raises:
        RuntimeError: The port is taken. viser would have moved to the next free port without a word.
    """
    server = viser.ViserServer(port=port, label=label, verbose=False)
    if port and server.get_port() != port:
        server.stop()
        raise RuntimeError(f"Port {port} is taken: is another monitor running? Stop it, or pick another port "
                           f"with -p {parameter}:=<port>.")
    return server


class SidePanel:
    """A panel of tabs docked on the right that plugins share: tabs found by their path of labels, made on first use.

    Users can drag its tabs out into windows of their own or into other panels. It shows only while it holds a tab.
    """

    def __init__(self, server: viser.ViserServer, minimized: bool = False):
        """Dock the panel, hidden until its first tab.

        ! Each panel docked later sits further out, so make the panel meant for the edge last.

        Args:
            server: The desktop server.
            minimized: Whether the panel starts minimized to a strip.
        """
        self._server = server
        self._panel = server.gui.add_panel(visible=False)
        self._panel.dock_right()
        self._panel.set_width(SIDE_PANEL_WIDTH)
        if minimized:
            self._panel.minimize()
        #: Tabs by their path of labels, and per tab the tab group holding the tabs nested in it.
        self._tabs: dict[tuple[str, ...], viser.GuiTabHandle] = {}
        self._groups: dict[tuple[str, ...], viser.GuiTabGroupHandle] = {}

    def tab(self, path: TabPath, shared: int = 0) -> tuple[viser.GuiTabHandle, bool]:
        """The tab at `path`, a (label, icon) per level, made with any missing parents; tabs keep the order made.

        Args:
            path: (label, icon) per level.
            shared: How many outer levels are tabs shared by several plugins, like Plan; their tab bars show as paths.

        Returns:
            tuple[viser.GuiTabHandle, bool]: The tab, and whether this call made it.
        """
        labels = tuple(label for label, _ in path)
        made = labels not in self._tabs
        for depth, (label, icon) in enumerate(path, 1):
            if labels[:depth] not in self._tabs:
                self._tabs[labels[:depth]] = self._add_tab(labels[:depth - 1], label, icon, depth - 1 <= shared)
        return self._tabs[labels], made

    def remove(self, labels: tuple[str, ...]) -> None:
        """Remove the tab at `labels`, with every tab nested in it, and then each parent left with no tabs."""
        for key in [key for key in self._tabs if key[:len(labels)] == labels]:
            self._groups.pop(key, None)
            tab = self._tabs.pop(key)
            if key == labels:
                tab.remove()
        parent = labels[:-1]
        if parent and not any(key[:len(parent)] == parent and key != parent for key in self._tabs):
            self.remove(parent)
        self._panel.visible = bool(self._tabs)

    def _add_tab(self, parent: tuple[str, ...], label: str, icon: viser.IconName | None,
                 shared: bool) -> viser.GuiTabHandle:
        """Add a tab to the panel, or to the tab group inside the parent tab; either is made when first needed.

        A group inside a parent shared by several plugins (`shared`) is marked, so its bar shows as a path.
        """
        if not parent:
            self._panel.visible = True
            return self._panel.add_tab(label, icon)
        if parent not in self._groups:
            with self._tabs[parent]:
                if shared:
                    self._server.gui.add_html(PATH_TABS_MARKER)
                self._groups[parent] = self._server.gui.add_tab_group()
        return self._groups[parent].add_tab(label, icon)


class Visualization:
    """Owns the viser servers (desktop and phone) and the robot and scene nodes. Plugins get a PluginView."""

    def __init__(self, port: int, phone_port: int):
        """Start the desktop and the phone server.

        Args:
            port: Port the desktop page listens on.
            phone_port: Port the phone page listens on.

        Raises:
            RuntimeError: A port is taken; no server is left running.
        """
        self._server = pinned_server(port, "viser_port", PAGE_TITLE)
        try:
            self._phone = PhonePage(pinned_server(phone_port, "phone_port", PHONE_TITLE), self._request_stop)
        except Exception:
            # ! The monitor only releases a Visualization that finished constructing.
            self._server.stop()
            raise
        # * Widest panel: fixed-width number rows wrap at the default.
        self._server.gui.configure_theme(control_width="large")
        # ? Sets the browser tab title (the first <title> on the page wins). HTML, not markdown,
        #   which viser pads into an empty row.
        self._server.gui.add_html(f"<title>{PAGE_TITLE}</title>{NESTED_TABS_STYLE}")

        # * Hidden until a plugin is stopped; see `show_broken`.
        self._broken = self._server.gui.add_html("", visible=False)

        # * Soft stop: a button and Esc both raise a flag the monitor reads next tick.
        #   ! Esc works only in the focused tab and not inside a text field. Not an e-stop.
        self._stop_requested = False
        stop = self._server.gui.add_button("Stop all (Esc)", color="red", icon=viser.Icon.HAND_STOP,
                                           hint="Soft stop: stop every arm's program, switch every base "
                                                "controller off, stop every tool, cancel the plugin tasks that may "
                                                "move a robot. The robots refuse motion commands until Resume in "
                                                "the health panel. Not an emergency stop.")
        stop.on_click(self._request_stop)
        self._server.gui.add_command("Stop all robots", description="Soft stop of every robot",
                                     hotkey="escape", icon=viser.Icon.HAND_STOP).on_trigger(self._request_stop)

        # * Pause: while paused the monitor skips every plugin's `draw`, so panel text can be selected
        #   (each rewrite drops the selection). Controls, updates and tasks keep running.
        self._paused = False
        self._pause = self._server.gui.add_button("Live", color="green", icon=viser.Icon.ACTIVITY)
        self._pause.on_click(self._toggle_pause)
        self._show_pause_state()

        # * Read by `draw` each tick, so the switch is applied on the main thread.
        self._show_collision = self._server.gui.add_checkbox(
            "Collision", initial_value=False,
            hint="Draw what collision checks use instead of what things look like: robots in red, "
                 "scene bodies and tracked objects in their own colour.")
        self._collision_shown = False

        #: Plugins' header folders (`PluginView.header`), under Stop all. Unlabelled: no title or border.
        self._header = self._server.gui.add_folder(None)
        # * This panel floats in the top left corner. The plugins' tabs (`PluginView.tab`) are docked on the right,
        #   and those hidden by default in a minimized strip at the right edge.
        self._server.gui.main_panel.float(x=15, y=15)
        self._side = SidePanel(self._server)
        self._hidden_side = SidePanel(self._server, minimized=True)

        # Per-robot handles by serial, built once in load_robot.
        self._robots: dict[str, _DrawnRobot] = {}

        self._scene_view = SceneView(self._server)

        self._server.scene.add_grid("/grid", width=10.0, height=10.0)
        self._server.scene.add_frame("/origin", show_axes=True)

    def load_robot(self, config: RobotConfig) -> None:
        """Add one robot's meshes, its tools on their flanges, at its default pose. Once at startup: loading is slow.

        Args:
            config: Identity, model and default pose for this robot.
        """
        # The base pose goes on a parent frame, so `draw` writes one pose plus one joint vector.
        root = f"/robots/{config.serial}"
        base = self._server.scene.add_frame(root, show_axes=False)
        base.position = config.default_position
        base.wxyz = quaternion_to_wxyz(config.default_orientation)

        model = shared_urdf(config.model.urdf)
        drawn = FastViserUrdf(self._server, model, root_node_name=root, load_collision_meshes=True,
                              collision_mesh_color_override=COLLISION_COLOR)
        drawn.add_tools(config.model.tools)
        self._show_robot_collision(drawn, self._collision_shown)
        self._robots[config.serial] = _DrawnRobot(
            base=base, urdf=drawn, joint_names=tuple(drawn.get_actuated_joint_names()), tools=config.model.tools,
            urdf_file=config.model.urdf)

    @staticmethod
    def _show_robot_collision(urdf: FastViserUrdf, collision: bool, faded: bool = False) -> None:
        """Show a robot's collision meshes or its visual ones; a faded robot's visual ones stay hidden (its twin shows).

        A URDF without collision meshes stays visual.
        """
        # ? viser warns instead of hiding when a URDF has no collision meshes.
        if urdf._collision_root_frame is not None:
            urdf.show_visual, urdf.show_collision = not collision and not faded, collision

    def set_robot_opacity(self, serial: str, opacity: float | None) -> None:
        """Draw a robot see-through at `opacity`, or solid again (None).

        The GLB meshes take no opacity, so a single-colour twin of the robot and its tools (made on the first fade,
        about a second) shows in their place under the same base frame, posed with them in `draw`.

        Args:
            serial: The robot's serial; an unknown one is ignored.
            opacity: 0 to 1, or None for the textured robot.
        """
        drawn = self._robots.get(serial)
        if drawn is None or drawn.opacity == opacity:
            return
        if opacity is not None and drawn.twin is None:
            twin = add_simple_urdf(self._server, shared_urdf(drawn.urdf_file), f"/robots/{serial}/twin")
            twin.add_tools(drawn.tools)
            if drawn.values is not None:
                twin.update_cfg(drawn.values)
            drawn.twin = twin
        drawn.opacity = opacity
        if drawn.twin is not None:
            for mesh in drawn.twin._meshes:
                mesh.opacity = opacity
            drawn.twin.show_visual = opacity is not None
        self._show_robot_collision(drawn.urdf, self._collision_shown, faded=opacity is not None)

    def view_for(self, plugin_name: str, hidden: bool = False, tab_path: tuple[str, ...] = (),
                 icon: viser.IconName | None = None) -> PluginView:
        """Carve out a private scene subtree and side panel tabs for one plugin.

        Args:
            plugin_name: Unique plugin name; becomes the scene path segment.
            hidden: Whether its tab starts in the minimized panel (`HuskyPlugin.hidden`).
            tab_path: Where its tab goes (`HuskyPlugin.tab_path`).
            icon: The icon on its own tabs (`HuskyPlugin.icon`).

        Returns:
            PluginView: The plugin's slice of the UI.
        """
        return PluginView(self._server, plugin_name, self._header, self._phone, (self._side, self._hidden_side),
                          hidden, tab_path, icon, fade=self.set_robot_opacity)

    def atomic(self) -> AbstractContextManager[None]:
        """Batch every scene change made inside the block into one update.

        Returns:
            AbstractContextManager[None]: Browsers never render a half-applied update.
        """
        return self._server.atomic()

    @property
    def paused(self) -> bool:
        """bool: Whether the operator paused the panels. Read by the monitor each tick."""
        return self._paused

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

    def _toggle_pause(self, _event: viser.GuiEvent) -> None:
        """Pause the panels or make them live again. A viser callback; only flips a flag the tick reads."""
        self._paused = not self._paused
        self._show_pause_state()

    def _show_pause_state(self) -> None:
        """Make the pause button show the current state."""
        if self._paused:
            self._pause.label, self._pause.color, self._pause.icon = "Paused", "orange", viser.Icon.PLAYER_PAUSE
            self._pause.hint = "Panels are paused, so their text can be selected. Click to go live again."
        else:
            self._pause.label, self._pause.color, self._pause.icon = "Live", "green", viser.Icon.ACTIVITY
            self._pause.hint = ("Click to pause every panel, so its text can be selected and copied. "
                                "Buttons still act; the robots in the 3D view stay live.")

    def draw(self, snapshot: Scene) -> None:
        """Draw this tick's copy of the world: robots with their tools, tracked objects and scene bodies.

        Called once per tick inside `atomic()`, also while paused.

        ! Only the snapshot taken this tick: the real robots and their tools must always show the newest
          measurement, never a copy a planner kept.

        Args:
            snapshot: The tick's copy of the world.
        """
        collision = self._show_collision.value
        if collision != self._collision_shown:
            self._collision_shown = collision
            for drawn in self._robots.values():
                self._show_robot_collision(drawn.urdf, collision, faded=drawn.opacity is not None)
        for object_id, entry in snapshot.robots.items():
            drawn = self._robots.get(object_id.split("/", 1)[1])  # ? measured robots are "robots/<serial>"
            if drawn is None:
                continue
            drawn.base.position = entry.base.position
            drawn.base.wxyz = quaternion_to_wxyz(entry.base.orientation)
            values = np.array([entry.joints.get(name, 0.0) for name in drawn.joint_names])
            if joints_changed(values, drawn.values):
                drawn.urdf.update_cfg(values)
                if drawn.twin is not None:
                    drawn.twin.update_cfg(values)
                drawn.values = values
        self._scene_view.sync(snapshot, collision)

    @property
    def ports(self) -> tuple[int, int]:
        """tuple[int, int]: The ports the desktop and the phone page listen on."""
        return self._server.get_port(), self._phone.server.get_port()

    def stop(self) -> None:
        """Shut both servers down and join their threads."""
        self._server.stop()
        self._phone.server.stop()


class PluginView:
    """One plugin's private scene subtree and side panel tabs in viser, and its phone tab, as `ctx.view`."""

    def __init__(self, server: viser.ViserServer, plugin_name: str, header: viser.GuiFolderHandle, phone: PhonePage,
                 sides: tuple[SidePanel, SidePanel], hidden: bool, tab_path: tuple[str, ...],
                 icon: viser.IconName | None, fade: Callable[[str, float | None], None] = lambda _s, _o: None):
        """Create the plugin's scene root; its header, tabs and phone tab come on first use.

        Args:
            server: The shared viser server.
            plugin_name: Unique plugin name.
            header: The desktop's header folder, under Stop all.
            phone: The phone page.
            sides: The side panel, and the minimized one for tabs hidden by default.
            hidden: Whether `ui()` goes in the minimized panel.
            tab_path: Where `ui()` goes, a label per level; empty for a tab named after the plugin.
            icon: The plugin's icon, on its own tabs.
            fade: Fades a real robot's drawing (`Visualization.set_robot_opacity`).
        """
        self._server = server
        self._fade, self._faded = fade, set()
        self._name = plugin_name
        self._header_parent = header
        self._header: viser.GuiFolderHandle | None = None
        self._sides = sides
        self._hidden = hidden
        #: The plugin's name as tabs show it, and its icon. Its own tabs carry the icon, shared parents like Plan none.
        self._label, self._icon = plugin_name.replace("_", " ").capitalize(), icon
        labels = tab_path or (self._label,)
        self._tab_path = tuple((label, icon if label == self._label or depth == len(labels) else None)
                               for depth, label in enumerate(labels, 1))
        #: How many outer levels of the tab path are shared with other plugins, up to the tab named after this one.
        self._shared = next((depth for depth, label in enumerate(labels[:-1]) if label == self._label),
                            len(labels) - 1)
        #: Side panel tabs this plugin made: their panel and path of labels.
        self._side_tabs: list[tuple[SidePanel, tuple[str, ...]]] = []
        #: This plugin's bands in its shared tab (`ui(band=)`), by (the tab's path of labels, band).
        self._bands: dict[tuple[tuple[str, ...], float], viser.GuiFolderHandle] = {}
        self._phone = phone
        self._phone_tab: viser.GuiTabHandle | None = None
        self._phone_header: viser.GuiFolderHandle | None = None

        #: Scene path owned by this plugin, e.g. "/plugins/calibration". Everything it adds hangs below it.
        self.scene_root = f"/plugins/{plugin_name}"
        self._root = server.scene.add_frame(self.scene_root, show_axes=False)
        #: The folder `folder()` made in the main panel.
        self._folder: viser.GuiFolderHandle | None = None
        #: Separate panels made through `panel()`, removed with everything else.
        self._panels: list[viser.PanelHandle] = []

    @property
    def scene(self) -> viser.SceneApi:
        """viser.SceneApi: Scene api. Paths passed to it must start with scene_root."""
        return self._server.scene

    @property
    def gui(self) -> viser.GuiApi:
        """viser.GuiApi: GUI api. Prefer `ui()` or `tab()`, which also set the parent."""
        return self._server.gui

    def ui(self, band: float | None = None) -> AbstractContextManager[viser.GuiApi]:
        """Add GUI widgets inside this plugin's tab, at its `tab_path`; only widgets created inside the block land there.

        Plugins with the same `tab_path` share one tab through bands: a band is an unlabelled folder of this plugin's,
        made on first use, and bands sort top to bottom by their number, whichever plugin made them.

        Args:
            band: The band's place in the tab, or None to add to the tab itself.

        Returns:
            AbstractContextManager[viser.GuiApi]: The GUI api, with this plugin's tab or band as parent.
        """
        return self._side_tab(self._tab_path, self._hidden, self._shared, band)

    @contextmanager
    def folder(self, expanded: bool = True) -> Iterator[viser.GuiApi]:
        """Add GUI widgets inside this plugin's folder in the main panel, under Stop all, instead of a tab.

        ! Only for what must show whichever tabs are open, like health's rows: folders cannot be moved.

        Args:
            expanded: Whether the folder starts open; only the call that makes it counts.

        Yields:
            viser.GuiApi: The GUI api, with this plugin's folder as parent.
        """
        if self._folder is None:
            self._folder = self._server.gui.add_folder(self._label, expand_by_default=expanded)
        with self._folder:
            yield self._server.gui

    def tab(self, label: str, icon: viser.IconName | None = None, hidden: bool = False, top: bool = False
            ) -> AbstractContextManager[viser.GuiApi]:
        """Add widgets inside a further tab of this plugin, inside a tab named after it, so its owner shows.

        Args:
            label: The tab's label.
            icon: The tab's icon, free to choose (nested in the plugin's tab); read when the tab is made.
            hidden: Whether the tab goes in the minimized panel at the right edge, for what is not needed every time.
            top: Whether the tab goes at the top level, beside Assembly and Control, instead of inside the plugin's
                own tab: for a tab the operator drags out into a window of its own. ! Its label must be unique.

        Returns:
            AbstractContextManager[viser.GuiApi]: The GUI api, with the tab as parent.
        """
        path = ((label, icon),) if top else ((self._label, self._icon), (label, icon))
        return self._side_tab(path, hidden, 0)

    def set_robot_opacity(self, serial: str, opacity: float | None) -> None:
        """Draw a real robot see-through at `opacity`, or solid again (None); `clear` puts back every one faded."""
        self._fade(serial, opacity)
        (self._faded.add if opacity is not None else self._faded.discard)(serial)

    @contextmanager
    def _side_tab(self, path: TabPath, hidden: bool, shared: int, band: float | None = None
                  ) -> Iterator[viser.GuiApi]:
        """Add widgets inside the tab at `path` in a side panel, or in this plugin's `band` there (`ui`).

        The tab is removed with the plugin that made it, a parent once it is empty; a band with its plugin.
        """
        side = self._sides[hidden]
        tab, made = side.tab(path, shared)
        labels = tuple(label for label, _ in path)
        if made:
            self._side_tabs.append((side, labels))
        parent = tab
        if band is not None:
            if (labels, band) not in self._bands:
                with tab:
                    self._bands[(labels, band)] = self._server.gui.add_folder(None, order=band)
            parent = self._bands[(labels, band)]
        with parent:
            yield self._server.gui

    def panel(self) -> viser.PanelHandle:
        """Create a panel owned by this plugin: a window beside the main one. Prefer `tab()`, which shares a panel.

        ! Always use this instead of `gui.add_panel()`: only panels made here are removed on teardown.

        Returns:
            viser.PanelHandle: The new panel, not yet placed.
        """
        panel = self._server.gui.add_panel()
        self._panels.append(panel)
        return panel

    @contextmanager
    def header(self) -> Iterator[viser.GuiApi]:
        """Add widgets under Stop all, so they show whichever tabs are open.

        ! For a short status only, like health's banner: everything else belongs in `ui()`.

        Yields:
            viser.GuiApi: The GUI api, with this plugin's header folder as parent.
        """
        if self._header is None:
            with self._header_parent:
                self._header = self._server.gui.add_folder(None)
        with self._header:
            yield self._server.gui

    @contextmanager
    def phone(self, label: str | None = None, icon: viser.IconName | None = None) -> Iterator[viser.GuiApi]:
        """Add widgets inside this plugin's tab on the phone page; the first call creates the tab.

        ! Keep it to what an operator needs next to the robots. Callbacks run on viser's threads, as on the desktop.

        Args:
            label: The tab's label, default the plugin name; read on the first call only.
            icon: The tab's icon, default the plugin's; read on the first call only.

        Yields:
            viser.GuiApi: The phone server's GUI api, with this plugin's tab as parent.
        """
        if self._phone_tab is None:
            self._phone_tab = self._phone.tabs.add_tab(label or self._name, icon or self._icon)
        with self._phone_tab:
            yield self._phone.server.gui

    @contextmanager
    def phone_header(self) -> Iterator[viser.GuiApi]:
        """Add widgets above the phone page's tabs, shown on every tab; for a short status only.

        Yields:
            viser.GuiApi: The phone server's GUI api, with this plugin's header folder as parent.
        """
        if self._phone_header is None:
            with self._phone.header:
                self._phone_header = self._phone.server.gui.add_folder(None)
        with self._phone_header:
            yield self._phone.server.gui

    def clear(self) -> None:
        """Remove all this plugin added: scene nodes, header, folder, panels, bands and tabs, phone tab and header."""
        for serial in list(self._faded):
            self.set_robot_opacity(serial, None)
        self._root.remove()
        if self._folder is not None:
            self._folder.remove()
        for panel in self._panels:
            panel.remove()
        self._panels.clear()
        for band in self._bands.values():
            # ? A tab another plugin made, and removed first, took this band with it; viser warns on a second remove.
            if not band._impl.removed:
                band.remove()
        self._bands.clear()
        for side, labels in self._side_tabs:
            side.remove(labels)
        self._side_tabs.clear()
        for handle in (self._header, self._phone_tab, self._phone_header):
            if handle is not None:
                handle.remove()
