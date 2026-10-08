"""
The cell's overlay: what the core's 3D view cannot draw of one design state.

Drawn from the design alone, with forward kinematics from each robot's yourdfpy model.

- * Bodies that `stand` are scene bodies, drawn by the core. The overlay draws the robots, their tools and the
  bodies they carry, and, when asked, every other body (absent ones), always see-through.
- `load_robot_models` is slow: run it on the loading thread.
- `DesignDrawing` is main thread only; a new state only changes poses, joint values and colours.
- ? Robots are single-colour meshes (`add_simple_urdf`), so "Ghost" only changes their opacity.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Mapping

import numpy as np
import viser
import yourdfpy

from bar_assembly_core.design import Design, State
from bar_assembly_core.geometry import Geometry, Pose, compose, shape_mesh
from bar_assembly_core.ids import split_link_id
from ...ui.visualization import FastViserUrdf, add_simple_urdf, joints_changed, load_urdf, quaternion_to_wxyz
from .design import body_color, stands

#: Colours, RGB 0-255. Held bodies stand out; other bodies use `design.body_color`.
ATTACHED_COLOR = (240, 120, 30)
TOOL_COLOR = (70, 70, 80)

#: Opacity of robots, tools and held bodies when "Ghost" is on, and always of bodies that do not stand.
GHOST_OPACITY = 0.4

#: Bodies added per tick while building, so a large design never stalls one tick.
BODIES_PER_TICK = 20


def load_robot_models(design: Design) -> dict[str, yourdfpy.URDF]:
    """Read every robot's URDF with its visual meshes, by robot id. Any thread."""
    return {robot_id: load_urdf(robot.urdf) for robot_id, robot in design.robots.items()}


@dataclass
class _Robot:
    """One robot's nodes and model.

    Attributes:
        frame: Node at the robot base; the meshes hang below it.
        urdf: The meshes, posed by `update_cfg`.
        model: The yourdfpy model, posed alongside `urdf`; read for forward kinematics.
        names: Actuated joint names, in `update_cfg` order.
        values: Joint values last shown, or None before the first show.
    """

    frame: viser.FrameHandle
    urdf: FastViserUrdf
    model: yourdfpy.URDF
    names: tuple[str, ...]
    values: np.ndarray | None = None


class DesignDrawing:
    """Every robot, tool and body of a design in viser, posed from one state at a time."""

    def __init__(self, view, root: str, design: Design, models: Mapping[str, yourdfpy.URDF]):
        """Create the root node; `build` adds the rest.

        Args:
            view: The plugin's view (PluginView); ViserUrdf adds the robot meshes through it.
            root: Scene path of the drawing, below the plugin's scene root.
            design: The design.
            models: Robot id -> model, from `load_robot_models`.
        """
        self._view, self._root_path, self._design, self._models = view, root, design, models
        self._root = view.scene.add_frame(root, show_axes=False)
        self._robots: dict[str, _Robot] = {}
        #: Tool or body id -> (frame, meshes).
        self._tools: dict[str, tuple[viser.FrameHandle, list[viser.MeshHandle]]] = {}
        self._bodies: dict[str, tuple[viser.FrameHandle, list[viser.MeshHandle]]] = {}
        #: Body id -> (colour, opacity) its meshes have, so unchanged ones are not sent again.
        self._looks: dict[str, tuple[tuple[int, int, int], float | None]] = {}
        self._ghost = False

    def build(self) -> Iterator[None]:
        """Add every node: one robot per tick, then the tools, then BODIES_PER_TICK bodies per tick.

        Yields:
            None: When a tick's share is done.
        """
        for robot_id, model in self._models.items():
            path = f"{self._root_path}/{robot_id}"
            frame = self._view.scene.add_frame(path, show_axes=False, visible=False)
            urdf = add_simple_urdf(self._view, model, path)
            if self._ghost:
                for mesh in urdf._meshes:
                    mesh.opacity = GHOST_OPACITY
            self._robots[robot_id] = _Robot(frame, urdf, model, tuple(urdf.get_actuated_joint_names()))
            yield
        for tool_id, tool in self._design.tools.items():
            self._tools[tool_id] = self._add(f"{self._root_path}/{tool_id}", tool.geometry, TOOL_COLOR)
        yield
        for count, (body_id, body) in enumerate(self._design.bodies.items(), start=1):
            # ? Coloured by `show`, which knows whether the body is held.
            self._bodies[body_id] = self._add(f"{self._root_path}/bodies/{body_id}", body.geometry,
                                              body_color(body_id))
            if count % BODIES_PER_TICK == 0:
                yield

    def _add(self, path: str, geometry: Geometry, color: tuple[int, int, int]):
        """A hidden frame at `path` with a geometry's visual shapes below it, coloured RGB 0-255.

        Returns:
            tuple: (frame, meshes).
        """
        frame = self._view.scene.add_frame(path, show_axes=False, visible=False)
        meshes = []
        for index, shape in enumerate(geometry.visual):
            mesh = shape_mesh(shape)  # primitives (bar cylinders) as triangles, cached per shape
            meshes.append(self._view.scene.add_mesh_simple(
                f"{path}/{index}", mesh.vertices.astype(np.float32), mesh.faces.astype(np.uint32),
                color=color, opacity=GHOST_OPACITY if self._ghost else None))
        return frame, meshes

    # --- --- --- --- --- PER STATE --- --- --- --- ---

    @property
    def visible(self) -> bool:
        """bool: Whether the drawing is shown."""
        return self._root.visible

    @visible.setter
    def visible(self, value: bool) -> None:
        self._root.visible = value

    def show(self, state: State, joints: Mapping[str, Mapping[str, float]], everything: bool = False) -> None:
        """Pose the overlay from one state.

        Args:
            state: Which robots are there and where, which bodies are present, held or moved.
            joints: Robot id -> joint values to draw it at (`design.displayed_joints`);
                joints not given are drawn at zero.
            everything: Also draw the bodies that neither stand nor are carried (absent ones).
        """
        # * Robots first: tools and held bodies read their links from the posed models.
        for robot_id, robot in self._robots.items():
            robot_state = state.robots.get(robot_id)
            tools = self._design.robots[robot_id].tools
            if robot_state is None or robot_state.base is None:
                # ? Absent, or its base not decided by the design: there is nowhere to draw it.
                robot.frame.visible = False
                for tool_id in tools.values():
                    self._tools[tool_id][0].visible = False
                continue
            _place(robot.frame, robot_state.base)
            given = joints.get(robot_id, {})
            values = np.array([float(given.get(name, 0.0)) for name in robot.names])
            if joints_changed(values, robot.values):
                robot.urdf.update_cfg(values)
                robot.model.update_cfg(values)
                robot.values = values
            robot.frame.visible = True
            for flange, tool_id in tools.items():
                frame = self._tools[tool_id][0]
                _place(frame, self._link_pose(robot_id, flange, robot_state.base))
                frame.visible = True

        for body_id, (frame, meshes) in self._bodies.items():
            carried = state.carried.get(body_id) if body_id in state.present else None
            if carried is not None:
                holder, link = split_link_id(carried.to)
                holder_state = state.robots.get(holder)
                if holder_state is None or holder_state.base is None:
                    frame.visible = False  # carried by a robot that is not drawn
                    continue
                pose = compose(self._link_pose(holder, link, holder_state.base), carried.offset)
                look = (ATTACHED_COLOR, GHOST_OPACITY if self._ghost else None)
            elif everything and not stands(state, body_id):
                pose = state.poses.get(body_id, self._design.bodies[body_id].pose)
                look = (body_color(body_id), GHOST_OPACITY)
            else:
                frame.visible = False  # standing: the core draws it; or not asked for
                continue
            _place(frame, pose)
            if self._looks.get(body_id) != look:
                # ? Compared first: viser sends a message for every assignment.
                for mesh in meshes:
                    mesh.color, mesh.opacity = look
                self._looks[body_id] = look
            frame.visible = True

    def _link_pose(self, robot_id: str, link: str, base: Pose) -> Pose:
        """A link's world pose, from the robot's model as last posed and its base pose."""
        return compose(base, Pose.from_matrix(self._robots[robot_id].model.get_transform(frame_to=link)))

    def set_ghost(self, ghost: bool) -> None:
        """Draw robots and tools see-through, or solid again; held bodies follow on the next `show`."""
        if ghost == self._ghost:
            return
        self._ghost = ghost
        opacity = GHOST_OPACITY if ghost else None
        for robot in self._robots.values():
            for mesh in robot.urdf._meshes:
                mesh.opacity = opacity
        for _, meshes in self._tools.values():
            for mesh in meshes:
                mesh.opacity = opacity

    def remove(self) -> None:
        """Remove every node of the drawing."""
        self._root.remove()


def _place(frame: viser.FrameHandle, pose: Pose) -> None:
    """Move a frame node to a pose."""
    frame.position = tuple(float(v) for v in pose.position)
    frame.wxyz = quaternion_to_wxyz(pose.orientation)
