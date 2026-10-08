"""
URDF and SRDF helpers in plain `xml.etree` (runs in Rhino too): names, group tips, mesh references, copying.

! A design URDF's mesh paths are relative to the URDF: resolve them against its folder, not the working directory.
"""

from __future__ import annotations

import shutil
from pathlib import Path, PurePath
from typing import Dict, Optional, Sequence, Tuple
from xml.etree.ElementTree import Element, ElementTree, TreeBuilder, XMLParser, parse, tostring

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
    urdf, srdf, dest_dir = Path(urdf), Path(srdf), Path(dest_dir)
    # ? Parse everything before writing: the source may already be the destination.
    tree = ElementTree(_root(urdf))
    srdf_text = Path(srdf).read_bytes()
    copies: Dict[str, Tuple[Path, str]] = {}
    for mesh in tree.getroot().iter("mesh"):
        filename = mesh.get("filename", "")
        if filename not in copies:
            copies[filename] = _locate(filename, urdf.parent, package_dirs)
        mesh.set("filename", f"meshes/{copies[filename][1]}")

    dest_dir.mkdir(parents=True, exist_ok=True)
    for source, below in copies.values():
        target = dest_dir / "meshes" / below
        if target.exists() and target.resolve() == source.resolve():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    urdf_out, srdf_out = dest_dir / "robot.urdf", dest_dir / "robot.srdf"
    tree.write(str(urdf_out), encoding="utf-8", xml_declaration=True)
    srdf_out.write_bytes(srdf_text)
    return urdf_out, srdf_out
