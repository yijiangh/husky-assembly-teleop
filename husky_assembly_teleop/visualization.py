"""
The user interface: a viser server, a scene mirroring the world, and one private
corner of both for each plugin.

! viser is retained mode. Build handles once, then mutate them.
  There is no frame to draw and no `server.update()`. Add a node, keep the
  handle, and assign `position` / `wxyz` / `visible` when something changes;
  viser pushes the diff to browsers on its own thread. Re-adding nodes every
  tick would leak and flicker.

! Threading. The server runs on its own thread and dispatches GUI and
  scene-click callbacks on a 32-worker pool, so callbacks registered here do NOT
  run on the ROS thread. They may not touch world state, cell state, PyBullet or
  ROS -- only hand work to the owning plugin's queue, which is what
  PluginContext.defer wraps up.

Why this is not called `husky_viser`, and why the old rebuild-the-panel habit
must not come across: doc/refactor_rationale.md.
"""

from __future__ import annotations

from contextlib import AbstractContextManager, contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np
import trimesh
import viser
import viser.extras
import yourdfpy

from .config import RobotConfig
from .world_state import WorldState


#: How rough the robot meshes are drawn, 0 = mirror, 1 = completely flat. The
#: meshes ship with `specular=[128,128,128]` and `glossiness=250`, which reads
#: as wet plastic under viser's lighting. Just short of 1 so edges still catch
#: a little light and the shape stays readable.
MATTE_ROUGHNESS = 0.9

#: Fallback colour for a mesh that carries no colour of its own.
_DEFAULT_MESH_COLOR = (0.8, 0.8, 0.8, 1.0)


def make_matte(mesh: trimesh.Trimesh) -> None:
    """Give one mesh a matte material in place, keeping the colour it had.

    ! Done on the mesh rather than at the viser call, because `ViserUrdf` adds
      geometry with `add_mesh_trimesh`, which takes no material arguments -- the
      appearance comes entirely from what the mesh carries.

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
            # Keep any texture image the original material had; only the shine
            # is being changed.
            baseColorTexture=getattr(material, "image", None),
            metallicFactor=0.0,
            roughnessFactor=MATTE_ROUGHNESS,
        ),
    )


#: Where `package://<pkg>/...` mesh references in our URDFs resolve to: the
#: packages sit side by side under data/husky_urdf/, so the scheme prefix is
#: simply replaced by that directory.
_PACKAGE_PREFIX = "package://"


def load_urdf(urdf_file: Path) -> yourdfpy.URDF:
    """Parse a URDF for display, resolving its `package://` mesh references.

    ! yourdfpy's own `filename_handler_magic` does not resolve `package://`
      here: it returns the reference unchanged, and yourdfpy then skips the
      mesh silently, so the robot appears as an empty scene graph with no error
      anywhere. Hence the explicit handler.

    Collision geometry is skipped. It is PyBullet's business, and drawing both
    would double the mesh count for no benefit.

    Args:
        urdf_file: URDF describing the robot as built.

    Returns:
        yourdfpy.URDF: The parsed model, with visual meshes loaded.
    """
    package_root = urdf_file.resolve().parent.parent.parent

    def resolve(fname: str) -> str:
        """Turn one `package://` reference into a path under the package root.

        ! The parameter must be called `fname`: yourdfpy calls the handler with
          that keyword, so renaming it raises a TypeError at load time.
        """
        if fname.startswith(_PACKAGE_PREFIX):
            return str(package_root / fname[len(_PACKAGE_PREFIX):])
        return fname

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


def quaternion_to_wxyz(quaternion: np.ndarray) -> tuple[float, float, float, float]:
    """Reorder a quaternion from the xyzw we carry to the wxyz viser wants.

    ! Easy to miss and hard to see: a wrong order still renders, just rotated,
      so the robot looks plausible and points the wrong way. ROS, PyBullet and
      RobotState all use xyzw; viser scene handles use wxyz.

    Args:
        quaternion: Orientation as (x, y, z, w).

    Returns:
        tuple[float, float, float, float]: The same rotation as (w, x, y, z).
    """
    x, y, z, w = (float(value) for value in quaternion)
    return (w, x, y, z)


@dataclass
class _DrawnRobot:
    """The handles and cached configuration for one robot in the viser scene.

    Attributes:
        base: Parent frame carrying the robot's base pose. Moving it moves the
            whole robot, so draw writes one pose rather than one per link.
        urdf: The mesh set, updated through `update_cfg`.
        joint_names: Actuated joint names, in the order `update_cfg` expects.
        configuration: Last drawn joint vector, in that same order. Kept so a
            joint with no measurement this tick holds its value instead of
            snapping to zero.
    """

    base: viser.FrameHandle
    urdf: viser.extras.ViserUrdf
    joint_names: tuple[str, ...]
    configuration: np.ndarray = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        """Start every joint at zero, which is what ViserUrdf draws initially."""
        if self.configuration is None:
            self.configuration = np.zeros(len(self.joint_names))


class Visualization:
    """Owns the viser server and the scene nodes mirroring world and cell state.

    ! The server handle stays in here. Plugins get a PluginView, the monitor
      gets `atomic()`, `draw()` and `stop()`. Nothing above holds a ViserServer.
    """

    def __init__(self, port: int = 8080):
        """Start the viser server.

        Args:
            port: Port the web UI listens on.
        """
        self._server = viser.ViserServer(port=port, verbose=False)

        # Persistent per-robot handles, keyed by serial. Built once in
        # load_robot, mutated in draw, never rebuilt per tick.
        #
        # ! The core draws the real robots and nothing else. Every other piece
        #   of geometry belongs to a plugin and goes in that plugin's
        #   PluginView. The core has no idea what a bar or a rack is.
        self._robots: dict[str, _DrawnRobot] = {}

        self._server.scene.add_grid("/grid", width=10.0, height=10.0)

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

    def draw(self, world: WorldState) -> None:
        """Push the measured robot poses into the scene.

        Called once per tick inside `atomic()`. Only assigns to handles built in
        `load_robot`; adds nothing.

        ! Only tracked base poses are applied, matching RobotScene. A robot with
          no mocap fix keeps its last drawn pose rather than jumping to the
          origin, so the browser and PyBullet agree about where it is.

        Args:
            world: Measured state to show.
        """
        for serial, state in world.robot_states().items():
            drawn = self._robots.get(serial)
            if drawn is None:
                continue
            if state.base_tracked:
                drawn.base.position = tuple(state.base_position)
                drawn.base.wxyz = quaternion_to_wxyz(state.base_orientation)
            # A joint we have no measurement for holds its last drawn value, for
            # the same reason: absent is not zero.
            drawn.configuration = np.array(
                [state.joint_positions.get(name, previous) for name, previous
                 in zip(drawn.joint_names, drawn.configuration)], dtype=float)
            drawn.urdf.update_cfg(drawn.configuration)

    def stop(self) -> None:
        """Shut the server down and join its thread."""
        self._server.stop()


class PluginViewImpl:
    """One plugin's private scene subtree and GUI folder.

    Satisfies the PluginView Protocol in context.py structurally, so this module
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
        self._folder = server.gui.add_folder(plugin_name)

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
        with self._folder:
            yield self._server.gui

    def clear(self) -> None:
        """Remove everything this plugin added to the scene and the panel.

        Removing the root frame removes the whole subtree below it, so plugins
        need not track their own scene handles just to clean up.
        """
        self._root.remove()
        self._folder.remove()
