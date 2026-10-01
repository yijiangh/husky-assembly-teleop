"""
Draws one state of a design in viser, from the design alone; forward kinematics from each robot's yourdfpy model.

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

from ...design_io import Design, Geometry, Pose, State, compose, shape_mesh
from ...design_io.types import split_link_id
from ...ui.visualization import FastViserUrdf, add_simple_urdf, joints_changed, load_urdf, quaternion_to_wxyz

#: Colours, RGB 0-255. Held bodies stand out.
BAR_COLOR = (205, 170, 110)
JOINT_COLOR = (120, 120, 140)
GROUND_COLOR = (170, 180, 170)
ATTACHED_COLOR = (240, 120, 30)
TOOL_COLOR = (70, 70, 80)

#: Opacity of robots, tools and bodies when "Ghost" is on.
GHOST_OPACITY = 0.4

#: Bodies added per tick while building, so a large design never stalls one tick.
BODIES_PER_TICK = 20


def load_robot_models(design: Design) -> dict[str, yourdfpy.URDF]:
    """Read every robot's URDF with its visual meshes, by robot id. Any thread."""
    return {robot_id: load_urdf(robot.urdf) for robot_id, robot in design.robots.items()}


def body_color(body_id: str) -> tuple[int, int, int]:
    """The colour of a body nobody holds: bar, ground, or anything else (joint halves)."""
    if body_id.startswith("bars/"):
        return BAR_COLOR
    if body_id.startswith("ground/"):
        return GROUND_COLOR
    return JOINT_COLOR


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

    def show(self, state: State, joints: Mapping[str, Mapping[str, float]]) -> None:
        """Pose everything from one state.

        Args:
            state: Which robots are there and where, which bodies are present, held or moved.
            joints: Robot id -> joint values to draw it at (`design.displayed_joints`);
                joints not given are drawn at zero.
        """
        # * Robots first: tools and held bodies read their links from the posed models.
        for robot_id, robot in self._robots.items():
            robot_state = state.robots.get(robot_id)
            tools = self._design.robots[robot_id].tools
            if robot_state is None:
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
            if body_id not in state.present:
                frame.visible = False
                continue
            attached = state.attached.get(body_id)
            if attached is not None:
                holder, link = split_link_id(attached.to)
                holder_state = state.robots.get(holder)
                if holder_state is None:
                    frame.visible = False  # held by a robot that is not there
                    continue
                pose = compose(self._link_pose(holder, link, holder_state.base), attached.grasp)
                color = ATTACHED_COLOR
            else:
                pose, color = state.poses.get(body_id, self._design.bodies[body_id].pose), body_color(body_id)
            _place(frame, pose)
            for mesh in meshes:
                mesh.color = color
            frame.visible = True

    def _link_pose(self, robot_id: str, link: str, base: Pose) -> Pose:
        """A link's world pose, from the robot's model as last posed and its base pose."""
        return compose(base, Pose.from_matrix(self._robots[robot_id].model.get_transform(frame_to=link)))

    def set_ghost(self, ghost: bool) -> None:
        """Draw everything see-through, or solid again."""
        if ghost == self._ghost:
            return
        self._ghost = ghost
        opacity = GHOST_OPACITY if ghost else None
        for robot in self._robots.values():
            for mesh in robot.urdf._meshes:
                mesh.opacity = opacity
        for _, meshes in [*self._tools.values(), *self._bodies.values()]:
            for mesh in meshes:
                mesh.opacity = opacity

    def remove(self) -> None:
        """Remove every node of the drawing."""
        self._root.remove()


def _place(frame: viser.FrameHandle, pose: Pose) -> None:
    """Move a frame node to a pose."""
    frame.position = tuple(float(v) for v in pose.position)
    frame.wxyz = quaternion_to_wxyz(pose.orientation)
