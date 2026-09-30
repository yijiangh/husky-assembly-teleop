"""
Draws a robot cell state in viser: the robot, its tools and the rigid bodies.

- `prepare_cell_meshes` turns compas meshes into numpy triangles (pure data, slow;
  run it on the loading thread).
- `CellDrawing` owns the viser nodes (main thread); a new state only changes poses.

Scene tree per cell:

    <root>/robot/<link>/<i>      link frames under the base, meshes inside
    <root>/tool/<tool>/...       same, for each tool
    <root>/body/<body>/<i>       meshes under the rigid body's frame

Poses come from compas_robots forward kinematics, not PyBullet.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator

import numpy as np
import viser
from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation
from compas_fab.robots import RobotCell, RobotCellState
from compas_robots import Configuration, RobotModel

#: RGB 0-255 for a link whose URDF has no material.
DEFAULT_LINK_COLOR = (180, 180, 180)
#: Rigid-body colours (RGB 0-255); attached bodies stand out.
BAR_COLOR = (205, 170, 110)
JOINT_COLOR = (120, 120, 140)
ATTACHED_COLOR = (240, 120, 30)

#: The design export parks unused robots here without marking them hidden, so a
#: tool at this point is skipped.
PARKED_POSITION = (50.0, 50.0, 0.0)
PARKED_TOLERANCE = 1e-3  # metres


@dataclass(frozen=True)
class MeshPart:
    """One triangle mesh, ready for viser.

    Attributes:
        vertices: (V, 3) float32, in metres, in the owning link's or body's frame.
        faces: (F, 3) uint32 vertex indices.
        color: RGB 0-255.
    """

    vertices: np.ndarray
    faces: np.ndarray
    color: tuple[int, int, int]


@dataclass(frozen=True)
class CellMeshes:
    """Every mesh of one robot cell, prepared on the loading thread.

    Attributes:
        robot: Link name -> meshes, for the robot model.
        tools: Tool id -> link name -> meshes.
        bodies: Rigid body id -> meshes.
    """

    robot: dict[str, list[MeshPart]]
    tools: dict[str, dict[str, list[MeshPart]]]
    bodies: dict[str, list[MeshPart]]


# --- --- --- --- --- PREPARING (loading thread) --- --- --- --- ---

def _triangles(mesh: Mesh, transformation: Transformation | None = None, scale: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """Triangulate a compas mesh into numpy arrays.

    Args:
        mesh: The compas mesh.
        transformation: Applied to the vertices after scaling, or None.
        scale: Uniform scale applied first, e.g. a rigid body's native_scale.

    Returns:
        tuple[np.ndarray, np.ndarray]: (V, 3) float32 vertices and (F, 3) uint32 faces.
    """
    vertices, faces = mesh.to_vertices_and_faces(triangulated=True)
    points = np.asarray(vertices, dtype=float) * scale
    if transformation is not None:
        matrix = np.asarray(transformation.matrix, dtype=float)
        points = points @ matrix[:3, :3].T + matrix[:3, 3]
    return points.astype(np.float32), np.asarray(faces, dtype=np.uint32).reshape(-1, 3)


def _link_meshes(model: RobotModel) -> dict[str, list[MeshPart]]:
    """Collect the visual meshes of a robot or tool model, in link frames.

    Each visual's origin and scale are baked into the vertices.

    Args:
        model: A compas RobotModel or ToolModel.

    Returns:
        dict[str, list[MeshPart]]: Link name -> meshes; links without visuals are left out.
    """
    links: dict[str, list[MeshPart]] = {}
    for link in model.links:
        parts = []
        for visual in link.visual:
            shape = visual.geometry.shape
            # ? Only mesh geometry is drawn; URDF primitives (box, cylinder) are skipped.
            meshes = getattr(shape, "meshes", None) or []
            origin = Transformation.from_frame(visual.origin) if visual.origin else None
            scale = getattr(shape, "scale", None)
            if scale is not None:
                scale_matrix = Transformation.from_matrix(np.diag([*scale, 1.0]).tolist())
                origin = scale_matrix if origin is None else origin * scale_matrix
            material = visual.material
            rgba = material.color.rgba if material is not None and material.color is not None else None
            color = tuple(int(round(c * 255)) for c in rgba[:3]) if rgba else DEFAULT_LINK_COLOR
            for mesh in meshes:
                vertices, faces = _triangles(mesh, origin)
                parts.append(MeshPart(vertices, faces, color))
        if parts:
            links[link.name] = parts
    return links


def prepare_cell_meshes(cell: RobotCell) -> CellMeshes:
    """Turn every mesh of a cell into numpy triangles; safe on any thread.

    Args:
        cell: The robot cell.

    Returns:
        CellMeshes: The robot's, each tool's and each rigid body's meshes.
    """
    bodies = {}
    for body_id, body in cell.rigid_body_models.items():
        # Placeholder colour; `CellDrawing.show` sets the real one per state.
        bodies[body_id] = [MeshPart(*_triangles(mesh, scale=body.native_scale), BAR_COLOR)
                           for mesh in body.visual_meshes]
    return CellMeshes(
        robot=_link_meshes(cell.robot_model),
        tools={tool_id: _link_meshes(tool) for tool_id, tool in cell.tool_models.items()},
        bodies=bodies,
    )


# --- --- --- --- --- POSES --- --- --- --- ---

def _pose(frame: Frame) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Convert a compas frame to viser's (position, wxyz).

    Args:
        frame: The frame.

    Returns:
        tuple: Position (x, y, z) and quaternion (w, x, y, z).
    """
    return tuple(frame.point), tuple(frame.quaternion.wxyz)


def _is_parked(frame: Frame) -> bool:
    """Whether a tool sits at the parking spot, meaning "not in this step".

    Args:
        frame: The tool's base frame.

    Returns:
        bool: True if it is at PARKED_POSITION.
    """
    return bool(np.allclose(list(frame.point), PARKED_POSITION, atol=PARKED_TOLERANCE))


def link_frames(model: RobotModel, configuration: Configuration | None) -> dict[str, Frame]:
    """Compute every link's frame relative to the model's base, in one pass.

    Args:
        model: Robot or tool model.
        configuration: Joint values; missing joints are zero. None for a model
            with no joints.

    Returns:
        dict[str, Frame]: Link name -> frame. The root link is at the base.
    """
    full = model.zero_configuration()
    if configuration is not None:
        given = dict(zip(configuration.joint_names, configuration.joint_values))
        full = Configuration([given.get(name, value) for name, value in zip(full.joint_names, full.joint_values)],
                             full.joint_types, full.joint_names)
    transformations = model.compute_transformations(full)
    frames = {}
    for link in model.links:
        joint = link.parent_joint
        frames[link.name] = (joint.current_origin.transformed(transformations[joint.name])
                             if joint else Frame.worldXY())
    return frames


# --- --- --- --- --- DRAWING (main thread) --- --- --- --- ---

@dataclass
class _DrawnModel:
    """The viser nodes of one robot or tool model.

    Attributes:
        base: Frame node at the model's base.
        links: Link name -> frame node, child of `base`.
        meshes: Every mesh node below the links, for opacity changes.
    """

    base: viser.FrameHandle
    links: dict[str, viser.FrameHandle] = field(default_factory=dict)
    meshes: list[viser.MeshHandle] = field(default_factory=list)


class CellDrawing:
    """The viser nodes for one robot cell, posed from one cell state at a time.

    ! `build` is a generator: drive it from a task (`await ctx.next_tick()` per step) so the mesh upload is spread over several ticks.
    """

    def __init__(self, scene: viser.SceneApi, root: str):
        """Create the (empty, hidden) root node.

        Args:
            scene: The viser scene api.
            root: Scene path for this cell, below the plugin's scene root.
        """
        self._scene = scene
        self._root_path = root
        self._root = scene.add_frame(root, show_axes=False, visible=False)
        self._robot: _DrawnModel | None = None
        self._tools: dict[str, _DrawnModel] = {}
        self._bodies: dict[str, viser.FrameHandle] = {}
        self._body_meshes: dict[str, list[viser.MeshHandle]] = {}
        self._opacity: float | None = None

    def build(self, meshes: CellMeshes) -> Iterator[None]:
        """Add every mesh node, yielding after each model.

        Args:
            meshes: The cell's prepared meshes.

        Yields:
            None: After each robot or tool model, and after the rigid bodies.
        """
        self._robot = self._add_model(f"{self._root_path}/robot", meshes.robot)
        yield
        for tool_id, links in meshes.tools.items():
            self._tools[tool_id] = self._add_model(f"{self._root_path}/tool/{tool_id}", links)
            yield
        for body_id, parts in meshes.bodies.items():
            path = f"{self._root_path}/body/{body_id}"
            self._bodies[body_id] = self._scene.add_frame(path, show_axes=False, visible=False)
            self._body_meshes[body_id] = [self._add_mesh(f"{path}/{i}", part) for i, part in enumerate(parts)]
        yield

    def _add_model(self, path: str, links: dict[str, list[MeshPart]]) -> _DrawnModel:
        """Add one robot or tool model's nodes.

        Args:
            path: Scene path of the model's base node.
            links: Link name -> meshes.

        Returns:
            _DrawnModel: Its handles.
        """
        drawn = _DrawnModel(base=self._scene.add_frame(path, show_axes=False))
        for link_name, parts in links.items():
            link_path = f"{path}/{link_name}"
            drawn.links[link_name] = self._scene.add_frame(link_path, show_axes=False)
            drawn.meshes += [self._add_mesh(f"{link_path}/{i}", part) for i, part in enumerate(parts)]
        return drawn

    def _add_mesh(self, path: str, part: MeshPart) -> viser.MeshHandle:
        """Add one mesh node at identity below its parent.

        Args:
            path: Scene path.
            part: The mesh.

        Returns:
            viser.MeshHandle: Its handle.
        """
        return self._scene.add_mesh_simple(path, part.vertices, part.faces, color=part.color,
                                           opacity=self._opacity)

    # --- --- --- --- --- PER STATE --- --- --- --- ---

    @property
    def visible(self) -> bool:
        """bool: Whether this cell is shown."""
        return self._root.visible

    @visible.setter
    def visible(self, value: bool) -> None:
        self._root.visible = value

    def show(self, cell: RobotCell, state: RobotCellState) -> None:
        """Pose everything from one state.

        Args:
            cell: The cell this drawing was built from.
            state: A state with a full robot configuration and resolved frames
                for attached objects (design.displayed_state).
        """
        self._pose_model(self._robot, cell.robot_model, state.robot_configuration, state.robot_base_frame)

        for tool_id, drawn in self._tools.items():
            tool_state = state.tool_states.get(tool_id)
            if tool_state is None or tool_state.is_hidden or tool_state.frame is None or _is_parked(tool_state.frame):
                drawn.base.visible = False
                continue
            self._pose_model(drawn, cell.tool_models[tool_id], tool_state.configuration, tool_state.frame)

        for body_id, node in self._bodies.items():
            body_state = state.rigid_body_states.get(body_id)
            if body_state is None or body_state.is_hidden or body_state.frame is None:
                node.visible = False
                continue
            node.position, node.wxyz = _pose(body_state.frame)
            node.visible = True
            attached = bool(body_state.attached_to_link or body_state.attached_to_tool)
            color = ATTACHED_COLOR if attached else (BAR_COLOR if "bar" in body_id else JOINT_COLOR)
            for mesh in self._body_meshes[body_id]:
                mesh.color = color

    def _pose_model(self, drawn: _DrawnModel, model: RobotModel, configuration: Configuration | None,
                    base_frame: Frame | None) -> None:
        """Place one model's base and links.

        Args:
            drawn: Its nodes.
            model: The robot or tool model.
            configuration: Its joint values, or None for all zero.
            base_frame: Where its base is in the world, or None for the origin.
        """
        drawn.base.position, drawn.base.wxyz = _pose(base_frame or Frame.worldXY())
        drawn.base.visible = True
        for link_name, frame in link_frames(model, configuration).items():
            node = drawn.links.get(link_name)
            if node is not None:
                node.position, node.wxyz = _pose(frame)

    def set_opacity(self, opacity: float | None) -> None:
        """Set the opacity of every mesh.

        Args:
            opacity: 0-1, or None for opaque.
        """
        self._opacity = opacity
        models = [self._robot, *self._tools.values()] if self._robot else list(self._tools.values())
        for mesh in [m for drawn in models for m in drawn.meshes] + \
                [m for meshes in self._body_meshes.values() for m in meshes]:
            mesh.opacity = opacity

    def remove(self) -> None:
        """Remove every node of this cell from the scene."""
        self._root.remove()
