"""
The mounted tools' models: joined onto a robot's URDF for drawing and live kinematics, or as `Tool`s for planning.

Stitching gives each tool's names the arm's name as a prefix, fixes its root link to <arm>_tool0, and makes all mesh
paths absolute. Planning cells keep tools separate: `tool_spec` reads a tool URDF's collision shapes.

* Adding a tool model: put its URDF in data/tool_urdf/ (root link in the tool0 frame) and add it to TOOL_URDFS.
"""

from __future__ import annotations

from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, parse

from yourdfpy import URDF

from bar_assembly_core.geometry import Geometry, Pose, TriMesh
from bar_assembly_core.robot import Tool

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
    """A tool for planning cells: its URDF's collision meshes at zero joints, in the tool0 frame; TCP at tool0.

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

    model = URDF.load(str(tool_file), build_scene_graph=False, load_meshes=False, build_collision_scene_graph=True,
                      load_collision_meshes=True, filename_handler=resolve)
    shapes = tuple(TriMesh.from_arrays(mesh.vertices, mesh.faces)
                   for mesh in model.collision_scene.dump(concatenate=False))
    return Tool(id=tool_id, geometry=Geometry(shapes, shapes), tcp=Pose(), kind=kind)


def stitch_tools(robot_urdf: Path, tools: dict[str, str | None], data_directory: Path, out_file: Path) -> Path:
    """Write `robot_urdf` with every mounted tool joined onto its arm's tool0.

    Args:
        robot_urdf: The robot's URDF, without tools.
        tools: The end effector kind per arm name; None for a bare arm.
        data_directory: The data root, where the tool URDFs are.
        out_file: Where to write the combined URDF. Its directory is created.

    Returns:
        Path: `out_file`.

    Raises:
        ValueError: If an arm has no tool0 link, or a tool URDF has no single root link.
    """
    tree = parse(robot_urdf)
    robot = tree.getroot()
    _make_meshes_absolute(robot, robot_urdf)
    robot_links = {link.get("name") for link in robot.findall("link")}

    for arm_name, kind in tools.items():
        tool_file = None if kind is None else tool_urdf(kind, arm_name, data_directory)
        if tool_file is None:
            continue
        tool0 = f"{arm_name}_tool0"
        if tool0 not in robot_links:
            raise ValueError(f"{robot_urdf.name} has no link {tool0} to mount {kind} on")
        tool = parse(tool_file).getroot()
        _make_meshes_absolute(tool, tool_file)
        root = _prefix_names(tool, f"{arm_name}_", tool_file)
        robot.extend(element for element in tool if element.tag in ("material", "link", "joint"))
        mount = SubElement(robot, "joint", name=f"{tool0}-{root}", type="fixed")
        SubElement(mount, "parent", link=tool0)
        SubElement(mount, "child", link=root)
        SubElement(mount, "origin", xyz="0 0 0", rpy="0 0 0")

    out_file.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out_file, encoding="utf-8", xml_declaration=True)
    return out_file


def _make_meshes_absolute(robot: Element, urdf_file: Path) -> None:
    """Rewrite every mesh filename under `robot` to an absolute path.

    ! Only <mesh> elements: gazebo plugins also have `filename` attributes, which are not meshes.
    """
    for mesh in robot.iter("mesh"):
        mesh.set("filename", resolve_mesh_path(mesh.get("filename"), urdf_file))


def _prefix_names(tool: Element, prefix: str, tool_file: Path) -> str:
    """Prefix every link, joint and material name of a tool URDF, in place.

    Args:
        tool: The tool URDF's <robot> element.
        prefix: E.g. "left_ur_arm_".
        tool_file: The tool URDF, for error messages.

    Returns:
        str: The prefixed root link.

    Raises:
        ValueError: If the tool does not have exactly one root link.
    """
    children = {child.get("link") for child in tool.iter("child")}
    roots = [link.get("name") for link in tool.findall("link") if link.get("name") not in children]
    if len(roots) != 1:
        raise ValueError(f"{tool_file.name}: expected one root link, found {roots}")

    for link in tool.findall("link"):
        link.set("name", prefix + link.get("name"))
    for joint in tool.findall("joint"):
        joint.set("name", prefix + joint.get("name"))
        for end in joint.findall("parent") + joint.findall("child"):
            end.set("link", prefix + end.get("link"))
        for mimic in joint.findall("mimic"):
            mimic.set("joint", prefix + mimic.get("joint"))
    # Covers both definitions and references inside visuals.
    for material in tool.iter("material"):
        if material.get("name"):
            material.set("name", prefix + material.get("name"))
    return prefix + roots[0]
