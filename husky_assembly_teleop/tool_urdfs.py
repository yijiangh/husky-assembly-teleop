"""
The mounted tools' models, read from their URDFs into the core's `Tool`, and robot URDFs with absolute mesh paths.

A tool is never joined onto a robot's URDF: it is a `Tool` on the robot's `RobotModel`, drawn on its flange and
built by the mirrors from there.

* Adding a tool model: put its URDF in data/tool_urdf/ (root link in the tool0 frame, a `tcp` link at its TCP) and
  add it to TOOL_URDFS.
"""

from __future__ import annotations

from pathlib import Path
from xml.etree.ElementTree import Element, parse

from yourdfpy import URDF

from scaffolding_assembly_core.geometry import Geometry, Pose, TriMesh
from scaffolding_assembly_core.robot import Tool

#: Where the tool URDFs live, under the data directory.
TOOL_URDF_DIRECTORY = "tool_urdf"

#: Tool URDF per end effector kind: one file, or one per arm name for left/right tools.
#: ? A single arm ("ur_arm") gets the left-hand v3 tool.
#: TODO scaffolding_v1 has no model yet, so it is neither drawn nor collision checked.
TOOL_URDFS: dict[str, str | dict[str, str]] = {
    "robotiq": "robotiq_2f_85.urdf",
    "scaffolding_v3": {"ur_arm": "scaffolding_v3_left.urdf",
                       "left_ur_arm": "scaffolding_v3_left.urdf",
                       "right_ur_arm": "scaffolding_v3_right.urdf"},
}

#: The tool URDF link at the tool centre point; it becomes `Tool.tcp`, for planners. Not set on the pendant,
#: which stays at tool0 (`ArmConfig.pendant_tcp`).
TCP_LINK = "tcp"

#: `package://<pkg>/...` resolves three levels above the URDF (<root>/<package>/urdf/<file>.urdf).
_PACKAGE_PREFIX = "package://"
_FILE_PREFIX = "file://"


def resolve_mesh_path(filename: str, urdf_file: Path) -> str:
    """Turn one mesh reference of a URDF into an absolute path.

    Args:
        filename: The mesh's `filename`: `package://...`, `file://...`, relative to the URDF, or absolute.
        urdf_file: The URDF the reference is written in.
    """
    if filename.startswith(_PACKAGE_PREFIX):
        return str(urdf_file.resolve().parent.parent.parent / filename[len(_PACKAGE_PREFIX):])
    if filename.startswith(_FILE_PREFIX):
        filename = filename[len(_FILE_PREFIX):]
    return str((urdf_file.resolve().parent / filename).resolve())


def tool_urdf(kind: str, arm_name: str, data_directory: Path) -> Path | None:
    """Find the URDF for tool `kind` on arm `arm_name`.

    Args:
        kind: The end effector kind (config.EndEffectorKind).
        arm_name: The arm it is mounted on, e.g. "left_ur_arm".
        data_directory: The data root.

    Returns:
        Path | None: The tool's URDF, or None for a tool without a model.

    Raises:
        ValueError: If `kind` has models for some arms, but not for this one.
    """
    entry = TOOL_URDFS.get(kind)
    if entry is None:
        return None
    if isinstance(entry, dict):
        if arm_name not in entry:
            raise ValueError(f"no {kind} model for arm {arm_name!r}; known arms: {', '.join(entry)}")
        entry = entry[arm_name]
    return data_directory / TOOL_URDF_DIRECTORY / entry


def tool_spec(kind: str, arm_name: str, data_directory: Path, tool_id: str) -> Tool | None:
    """A tool as the core describes it: its URDF's visual and collision meshes at zero joints, and its TCP.

    All in the tool0 frame, the tool URDF's root link. The TCP is the link TCP_LINK, or tool0 if there is none.

    Args:
        kind: The end effector kind (config.EndEffectorKind).
        arm_name: The arm it is mounted on, e.g. "left_ur_arm".
        data_directory: The data root, where the tool URDFs are.
        tool_id: Its id in scenes, e.g. "tools/a200-0806/left_ur_arm".

    Returns:
        Tool | None: The tool, or None for a tool without a model.

    Raises:
        ValueError: If `kind` has models for some arms, but not for this one.
    """
    tool_file = tool_urdf(kind, arm_name, data_directory)
    if tool_file is None:
        return None

    def resolve(fname: str) -> str:
        """Turn one mesh reference into an absolute path. ! Keep the name `fname`: yourdfpy passes it as a keyword."""
        return resolve_mesh_path(fname, tool_file)

    model = URDF.load(str(tool_file), build_scene_graph=True, load_meshes=True, build_collision_scene_graph=True,
                      load_collision_meshes=True, filename_handler=resolve)
    visual = tuple(TriMesh.from_arrays(mesh.vertices, mesh.faces) for mesh in model.scene.dump(concatenate=False))
    collision = tuple(TriMesh.from_arrays(mesh.vertices, mesh.faces)
                      for mesh in model.collision_scene.dump(concatenate=False))
    tcp = (Pose.from_matrix(model.get_transform(frame_to=TCP_LINK, frame_from=model.base_link))
           if TCP_LINK in model.link_map else Pose())
    return Tool(id=tool_id, geometry=Geometry(visual, collision), tcp=tcp, kind=kind)


def absolute_mesh_urdf(robot_urdf: Path, out_file: Path) -> Path:
    """Write a copy of `robot_urdf` whose mesh paths are absolute; nothing else changes.

    ? PyBullet and compas_robots cannot resolve `package://` paths, so mirrors load this copy.

    Args:
        robot_urdf: The robot's URDF.
        out_file: Where to write the copy. Its directory is created.

    Returns:
        Path: `out_file`.
    """
    tree = parse(robot_urdf)
    _make_meshes_absolute(tree.getroot(), robot_urdf)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out_file, encoding="utf-8", xml_declaration=True)
    return out_file


def _make_meshes_absolute(robot: Element, urdf_file: Path) -> None:
    """Rewrite every mesh filename under `robot` to an absolute path.

    ! Only <mesh> elements: gazebo plugins also have `filename` attributes, which are not meshes.
    """
    for mesh in robot.iter("mesh"):
        mesh.set("filename", resolve_mesh_path(mesh.get("filename"), urdf_file))
