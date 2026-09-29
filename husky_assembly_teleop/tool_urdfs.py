"""
Tool geometry: stitching the mounted tools onto a robot's URDF at load time.

The robot URDFs (config._ROBOTS_BY_SERIAL) describe each robot up to its arms'
tool0 links and no further. Which tool is on which arm is configuration, and
changes from run to run (the `tools` parameter), so no tool is baked into them.
Instead each tool has a small URDF of its own under data/tool_urdf/, and
`stitch_tools` joins the mounted ones onto the robot, writing one combined URDF.
That one file is what PyBullet, viser and the frame lookups all read, so the
robot and its tools can never disagree.

? Why stitch rather than keep one full URDF per configuration.
  Robots x tool combinations x calibration multiplies files quickly, and every
  copy of a robot is one more place a calibration change can be forgotten.
  See doc/refactor_rationale.md.

How one tool is joined onto an arm:
  - Every link, joint and material of the tool gets the arm's name in front
    ("left_ur_arm_" + name), so both arms can carry the same kind of tool.
  - A fixed joint puts the tool's root link exactly on <arm>_tool0: a tool URDF
    is drawn in tool0's own frame.
  - Every mesh path, of robot and tools alike, is made absolute, so the
    combined file works from wherever it is written.

* Adding a tool model: write its URDF into data/tool_urdf/ (root link in the
  tool0 frame, mesh paths relative to the file) and add it to TOOL_URDFS.
"""

from __future__ import annotations

from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, parse

#: Where the tool URDFs live, under the data directory.
TOOL_URDF_DIRECTORY = "tool_urdf"

#: The tool URDF for each end effector kind: one file, or one per arm name for
#: tools that come in a left and a right hand version.
#: ? A single arm carrying a v3 gets the left-hand tool, as in the old code.
#: TODO scaffolding_v1 has no model yet, so it is not drawn and not collision
#:      checked. Add one here once its mesh exists.
TOOL_URDFS: dict[str, str | dict[str, str]] = {
    "robotiq": "robotiq_2f_85.urdf",
    "scaffolding_v3": {"ur_arm": "scaffolding_v3_left.urdf",
                       "left_ur_arm": "scaffolding_v3_left.urdf",
                       "right_ur_arm": "scaffolding_v3_right.urdf"},
}

#: Mesh references of this form point into a ROS package. Ours sit side by side
#: in one directory, three levels above the URDF that uses them
#: (<root>/<package>/urdf/<file>.urdf), so the prefix is replaced by that directory.
_PACKAGE_PREFIX = "package://"
_FILE_PREFIX = "file://"


def resolve_mesh_path(filename: str, urdf_file: Path) -> str:
    """Turn one mesh reference of a URDF into an absolute path.

    Args:
        filename: The mesh's `filename` attribute: `package://<pkg>/...`,
            `file://...`, a path relative to the URDF, or an absolute path.
        urdf_file: The URDF the reference is written in.

    Returns:
        str: The absolute path of the mesh.
    """
    if filename.startswith(_PACKAGE_PREFIX):
        return str(urdf_file.resolve().parent.parent.parent / filename[len(_PACKAGE_PREFIX):])
    if filename.startswith(_FILE_PREFIX):
        filename = filename[len(_FILE_PREFIX):]
    return str((urdf_file.resolve().parent / filename).resolve())


def tool_urdf(kind: str, arm_name: str, data_directory: Path) -> Path | None:
    """The URDF that draws `kind` on the arm `arm_name`.

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
        ValueError: If an arm has no tool0 link in `robot_urdf`, or a tool URDF
            has no single root link.
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
        # Materials, links and joints, in the tool file's own order.
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

    ! Only <mesh> elements: a URDF also carries `filename` attributes on gazebo
      plugins (libgazebo_ros_control.so), which are not files of ours.
    """
    for mesh in robot.iter("mesh"):
        mesh.set("filename", resolve_mesh_path(mesh.get("filename"), urdf_file))


def _prefix_names(tool: Element, prefix: str, tool_file: Path) -> str:
    """Put `prefix` in front of every link, joint and material name of a tool URDF.

    Args:
        tool: The tool URDF's <robot> element, changed in place.
        prefix: E.g. "left_ur_arm_".
        tool_file: The tool URDF, for error messages.

    Returns:
        str: The tool's root link -- the one no joint moves -- with its prefix.

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
    # Top-level definitions and the references inside visuals alike.
    for material in tool.iter("material"):
        if material.get("name"):
            material.set("name", prefix + material.get("name"))
    return prefix + roots[0]
