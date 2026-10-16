"""
The user interface: the desktop and phone pages, the world drawn each tick, and one private corner per plugin.

- ! Build viser nodes once and keep the handles: re-adding them every tick leaks and flickers.
- ! viser callbacks run on viser's threads: they must not touch world state, the scene or ROS, only
  hand work to the plugin's queue (PluginContext.defer).
"""

from __future__ import annotations

from contextlib import AbstractContextManager, contextmanager
from dataclasses import dataclass
from functools import cache
from html import escape
from pathlib import Path
from typing import Iterator, Mapping

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
from .style import FAIL


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

        # * Read by `draw` each tick, so the switch is applied on the main thread.
        self._show_collision = self._server.gui.add_checkbox(
            "Collision", initial_value=False,
            hint="Draw what collision checks use instead of what things look like: robots in red, "
                 "scene bodies and tracked objects in their own colour.")
        self._collision_shown = False

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
            base=base, urdf=drawn, joint_names=tuple(drawn.get_actuated_joint_names()))

    @staticmethod
    def _show_robot_collision(urdf: FastViserUrdf, collision: bool) -> None:
        """Show a robot's collision meshes or its visual ones. A URDF without collision meshes stays visual."""
        # ? viser warns instead of hiding when a URDF has no collision meshes.
        if urdf._collision_root_frame is not None:
            urdf.show_visual, urdf.show_collision = not collision, collision

    def view_for(self, plugin_name: str) -> PluginView:
        """Carve out a private scene subtree and GUI folder for one plugin.

        Args:
            plugin_name: Unique plugin name; becomes the scene path segment.

        Returns:
            PluginView: The plugin's slice of the UI.
        """
        return PluginView(self._server, plugin_name, self._phone)

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

    def draw(self, snapshot: Scene) -> None:
        """Draw this tick's copy of the world: robots with their tools, tracked objects and scene bodies.

        Called once per tick inside `atomic()`, also while frozen.

        ! Only the snapshot taken this tick: the real robots and their tools must always show the newest
          measurement, never a copy a planner kept.

        Args:
            snapshot: The tick's copy of the world.
        """
        collision = self._show_collision.value
        if collision != self._collision_shown:
            self._collision_shown = collision
            for drawn in self._robots.values():
                self._show_robot_collision(drawn.urdf, collision)
        for object_id, entry in snapshot.robots.items():
            drawn = self._robots.get(object_id.split("/", 1)[1])  # ? measured robots are "robots/<serial>"
            if drawn is None:
                continue
            drawn.base.position = entry.base.position
            drawn.base.wxyz = quaternion_to_wxyz(entry.base.orientation)
            values = np.array([entry.joints.get(name, 0.0) for name in drawn.joint_names])
            if joints_changed(values, drawn.values):
                drawn.urdf.update_cfg(values)
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
    """One plugin's private scene subtree and GUI folder in viser, and its tab on the phone, as `ctx.view`."""

    def __init__(self, server: viser.ViserServer, plugin_name: str, phone: PhonePage):
        """Create the plugin's scene root; its GUI folder and phone tab come on first use.

        Args:
            server: The shared viser server.
            plugin_name: Unique plugin name.
            phone: The phone page.
        """
        self._server = server
        self._name = plugin_name
        self._phone = phone
        self._phone_tab: viser.GuiTabHandle | None = None
        self._phone_header: viser.GuiFolderHandle | None = None

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

    @contextmanager
    def phone(self, label: str | None = None, icon: viser.IconName | None = None) -> Iterator[viser.GuiApi]:
        """Add widgets inside this plugin's tab on the phone page; the first call creates the tab.

        ! Keep it to what an operator needs next to the robots. Callbacks run on viser's threads, as on the desktop.

        Args:
            label: The tab's label, default the plugin name; read on the first call only.
            icon: The tab's icon; read on the first call only.

        Yields:
            viser.GuiApi: The phone server's GUI api, with this plugin's tab as parent.
        """
        if self._phone_tab is None:
            self._phone_tab = self._phone.tabs.add_tab(label or self._name, icon)
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
        """Remove everything this plugin added: scene nodes, its folder, its panels, its phone tab and header."""
        self._root.remove()
        if self._folder is not None:
            self._folder.remove()
        for panel in self._panels:
            panel.remove()
        self._panels.clear()
        for handle in (self._phone_tab, self._phone_header):
            if handle is not None:
                handle.remove()
