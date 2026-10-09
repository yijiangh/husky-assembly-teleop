"""
URDF and SRDF files, and the conventions of the UR arms every user of the core shares.

Names, groups, mesh references and copying, in plain `xml.etree` (runs in Rhino too).

! A design URDF's mesh paths are relative to the URDF: resolve them against its folder, not the working directory.
"""

from __future__ import annotations

import shutil
from hashlib import sha256
from io import BytesIO
from pathlib import Path, PurePath
from typing import Dict, List, Optional, Sequence, Tuple
from xml.etree.ElementTree import Element, ElementTree, TreeBuilder, XMLParser, parse, tostring

import numpy as np
from scipy.spatial.transform import Rotation

PACKAGE_SCHEME, FILE_SCHEME = "package://", "file://"


# --- --- --- --- --- READING --- --- --- --- ---

def _root(path: Path) -> Element:
    """The root element of an XML file, comments kept so a copied URDF keeps them."""
    parser = XMLParser(target=TreeBuilder(insert_comments=True))
    return parse(str(path), parser=parser).getroot()


def urdf_links(urdf: Path) -> set[str]:
    """Every link name of a URDF."""
    return {link.get("name") for link in _root(urdf).findall("link")}


def urdf_joints(urdf: Path) -> dict[str, str]:
    """Joint name -> type ("revolute", "fixed", ...) of a URDF, in file order."""
    return {joint.get("name"): joint.get("type") for joint in _root(urdf).findall("joint")}


def movable_joints(urdf: Path, srdf: Path) -> tuple[str, ...]:
    """The joints a design state must list, in URDF order: all but fixed, SRDF-passive (wheels) and mimic ones.

    Args:
        urdf: The URDF file.
        srdf: The SRDF file of the same robot.
    """
    passive = {joint.get("name") for joint in _root(srdf).findall("passive_joint")}
    return tuple(joint.get("name") for joint in _root(urdf).findall("joint")
                 if joint.get("type") != "fixed" and joint.get("name") not in passive
                 and joint.find("mimic") is None)


def srdf_group_tips(srdf: Path) -> dict[str, str]:
    """Group name -> tip link of every SRDF group made of a `<chain>`."""
    tips = {}
    for group in _root(srdf).findall("group"):
        chain = group.find("chain")
        if chain is not None and chain.get("tip_link"):
            tips[group.get("name")] = chain.get("tip_link")
    return tips


def mesh_references(urdf: Path) -> list[str]:
    """The `filename` of every `<mesh>` in a URDF, as written, in file order (repeats included)."""
    return [mesh.get("filename", "") for mesh in _root(urdf).iter("mesh")]


def is_relative_reference(filename: str) -> bool:
    """Whether a `<mesh filename=...>` is a plain relative path (no `package://`, `file://`, or absolute path)."""
    return not (filename.startswith(PACKAGE_SCHEME) or filename.startswith(FILE_SCHEME)
                or Path(filename).is_absolute())


def resolved_urdf_text(urdf: Path) -> str:
    """The URDF text with relative mesh paths made absolute; other references are left as they are.

    ? compas_robots resolves plain paths against the working directory, so compas callers load this instead.
    """
    root = _root(urdf)
    folder = Path(urdf).resolve().parent
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename and is_relative_reference(filename):
            mesh.set("filename", str((folder / filename).resolve()))
    return tostring(root, encoding="unicode")


# --- --- --- --- --- COPYING --- --- --- --- ---

def _find_package(name: str, package_dirs: Sequence[Path]) -> Optional[Path]:
    """The folder of a ROS package: `<dir>/<name>`, or a dir that is the package itself; None if not found."""
    for folder in package_dirs:
        folder = Path(folder)
        if folder.name == name and folder.is_dir():
            return folder
        if (folder / name).is_dir():
            return folder / name
    return None


def _without_anchor(path: Path) -> str:
    """An absolute path as a relative one, e.g. "/opt/meshes/a.stl" -> "opt/meshes/a.stl"."""
    return PurePath(*path.parts[1:]).as_posix()


def _locate(filename: str, urdf_folder: Path, package_dirs: Sequence[Path]) -> Tuple[Path, str]:
    """Find a referenced mesh and choose its path below `meshes/`.

    The path is `<pkg>/<rest>` for `package://`, the relative path for files inside the URDF's folder,
    else the absolute path without its leading `/`.

    Args:
        filename: The reference as written in the URDF.
        urdf_folder: Folder of the source URDF.
        package_dirs: Folders to find packages in.

    Returns:
        tuple[Path, str]: The source file, and its path below `meshes/` (with `/`).

    Raises:
        FileNotFoundError: If the package or the file is missing.
    """
    if filename.startswith(PACKAGE_SCHEME):
        package, _, rest = filename[len(PACKAGE_SCHEME):].partition("/")
        package_dir = _find_package(package, package_dirs)
        if package_dir is None:
            raise FileNotFoundError(f"mesh {filename!r}: package {package!r} not found in {list(package_dirs)}")
        source, below = package_dir / rest, f"{package}/{rest}"
    else:
        path = Path(filename[len(FILE_SCHEME):] if filename.startswith(FILE_SCHEME) else filename)
        source = path if path.is_absolute() else (urdf_folder / path)
        source = source.resolve()
        # ? A design's own URDF already keeps its meshes under `meshes/`: keep that path, or a rewrite nests it again.
        for base in (urdf_folder.resolve() / "meshes", urdf_folder.resolve()):
            try:
                below = source.relative_to(base).as_posix()
                break
            except ValueError:
                continue
        else:
            below = _without_anchor(source)
    if not source.is_file():
        raise FileNotFoundError(f"mesh {filename!r} not found (looked at {source})")
    return source, below


def _robot_copy(urdf: Path, srdf: Path, package_dirs: Sequence[Path]) -> Tuple[bytes, bytes, Dict[str, Path]]:
    """What `copy_robot` writes, in memory: the URDF with its mesh paths rewritten, the SRDF, and the meshes.

    Returns:
        tuple: URDF bytes, SRDF bytes, and each mesh's path below `meshes/` -> its source file.
    """
    urdf = Path(urdf)
    tree = ElementTree(_root(urdf))
    copies: Dict[str, Tuple[Path, str]] = {}
    for mesh in tree.getroot().iter("mesh"):
        filename = mesh.get("filename", "")
        if filename not in copies:
            copies[filename] = _locate(filename, urdf.parent, package_dirs)
        mesh.set("filename", f"meshes/{copies[filename][1]}")
    text = BytesIO()
    tree.write(text, encoding="utf-8", xml_declaration=True)
    return text.getvalue(), Path(srdf).read_bytes(), {below: source for source, below in copies.values()}


def copy_robot(urdf: Path, srdf: Path, dest_dir: Path, package_dirs: Sequence[Path] = ()) -> tuple[Path, Path]:
    """Copy a robot into a design: `robot.urdf`, `robot.srdf`, and every mesh under `meshes/`, named relatively.

    Args:
        urdf: Source URDF.
        srdf: Source SRDF.
        dest_dir: The robot's folder in the design, e.g. `<design>/robots/cindy`. Created if missing.
        package_dirs: Folders holding packages named by `package://`, or package folders themselves.

    Returns:
        tuple[Path, Path]: The copied URDF and SRDF.

    Raises:
        FileNotFoundError: If a referenced mesh (or its package) is missing.
    """
    dest_dir = Path(dest_dir)
    # ? Read everything before writing: the source may already be the destination.
    urdf_text, srdf_text, meshes = _robot_copy(urdf, srdf, package_dirs)
    dest_dir.mkdir(parents=True, exist_ok=True)
    for below, source in meshes.items():
        target = dest_dir / "meshes" / below
        if target.exists() and target.resolve() == source.resolve():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    urdf_out, srdf_out = dest_dir / "robot.urdf", dest_dir / "robot.srdf"
    urdf_out.write_bytes(urdf_text)
    srdf_out.write_bytes(srdf_text)
    return urdf_out, srdf_out


#: (file stats of a robot's URDF, SRDF and meshes) -> its files hash; reading 40 MB of meshes takes a while.
_FILES_HASHES: Dict[tuple, str] = {}


def robot_files_hash(urdf: Path, srdf: Path, package_dirs: Sequence[Path] = ()) -> str:
    """The SHA-256 of a robot's files as `copy_robot` writes them: URDF, SRDF and every mesh with its path.

    The same for a source robot and for its copy in a design, so a design hashes the same in memory and on disk.

    Raises:
        FileNotFoundError: If a referenced mesh (or its package) is missing.
    """
    urdf_text, srdf_text, meshes = _robot_copy(urdf, srdf, package_dirs)
    stats = tuple((below, source.stat().st_mtime_ns, source.stat().st_size) for below, source in sorted(meshes.items()))
    key = (urdf_text, srdf_text, stats)
    if key not in _FILES_HASHES:
        digest = sha256(urdf_text + b"\0" + srdf_text)
        for below, source in sorted(meshes.items()):
            digest.update(b"\0" + below.encode("utf-8") + b"\0" + source.read_bytes())
        _FILES_HASHES[key] = digest.hexdigest()
    return _FILES_HASHES[key]


# --- --- --- --- --- UR ARMS --- --- --- --- ---

#: Joint names in the UR driver's order. The URDF has the same names with the arm's prefix ("left_ur_arm_").
UR_JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

#: Links of its arm ("<arm>_<suffix>") a mounted tool may touch. The SRDFs don't list these pairs,
#: so collision checkers add them themselves.
TOOL_TOUCHES_ARM_LINKS = ("wrist_2_link", "wrist_3_link", "flange", "tool0")

#: Stock ur_description turns both joints below `<arm>_base_link` 180 deg about z.
STOCK_YAW = np.pi


def joint_origin(joint: Element) -> Tuple[List[float], Rotation]:
    """Return a URDF joint's `<origin>` as (xyz, rotation); a missing origin or attribute is zero."""
    origin = joint.find("origin")
    xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    return [float(v) for v in xyz.split()], Rotation.from_euler("xyz", [float(v) for v in rpy.split()])


def stock_frame_problem(urdf_file: Path, arm_name: str) -> Optional[str]:
    """Check the two joints below an arm's base_link are the stock ones.

    Args:
        urdf_file: The robot's URDF.
        arm_name: The arm's prefix, e.g. "ur_arm".

    Returns:
        str | None: None if stock, otherwise what is wrong and how to fix it.
    """
    wrong = []
    for joint in parse(urdf_file).getroot().findall("joint"):
        if joint.find("parent").get("link") != f"{arm_name}_base_link":
            continue
        if joint.find("child").get("link") not in (f"{arm_name}_base_link_inertia", f"{arm_name}_base"):
            continue
        xyz, rotation = joint_origin(joint)
        turn = (Rotation.from_euler("z", STOCK_YAW).inv() * rotation).magnitude()
        if np.linalg.norm(xyz) > 1e-6 or turn > 1e-6:
            wrong.append(f"{joint.get('name')} (rpy {joint.find('origin').get('rpy')})")
    if not wrong:
        return None
    return (f"URDF {Path(urdf_file).name}, arm {arm_name}: joints below {arm_name}_base_link are not the stock UR "
            f"ones (xyz 0 0 0, rpy 0 0 pi): {', '.join(wrong)}. So {arm_name}_base_link is not the robot "
            f"controller's base_link. The arm's mounting belongs in the joint above it. "
            f"Fix with scripts/fix_ur_base_frames.py; see doc/ur_frames.md.")
