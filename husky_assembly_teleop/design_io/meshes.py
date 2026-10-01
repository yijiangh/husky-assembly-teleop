"""
Mesh files of a design (format §6): read with trimesh, written as plain OBJ text.

* One `MeshCache` per read: every shape naming the same file gets the SAME TriMesh object, so
  mirrors, which cache what they build per object, build it once.
? trimesh drops vertices no face uses; nothing else about a mesh changes on a write and read.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict

from trimesh import Trimesh, load

from .geometry import TriMesh

#: File types a design may use for meshes, chosen by extension.
MESH_SUFFIXES = (".obj", ".stl", ".glb")


def read_mesh(path: Path) -> TriMesh:
    """Read one mesh file (.obj, .stl or .glb); polygons are split into triangles.

    Args:
        path: The mesh file.

    Returns:
        TriMesh: Its triangles, as stored (no merging or cleaning).
    """
    mesh = load(str(path), force="mesh", process=False)
    return TriMesh.from_arrays(mesh.vertices, mesh.faces)


def write_mesh(mesh: TriMesh, path: Path) -> None:
    """Write one mesh file, its type chosen by the extension. Creates missing folders.

    * OBJ is written here, with every coordinate exact (`repr`), so a read gives the same numbers.
      Other types go through trimesh.

    Args:
        mesh: The mesh.
        path: Where to write; `.obj`, `.stl` or `.glb`.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() != ".obj":
        Trimesh(mesh.vertices, mesh.faces, process=False).export(str(path))
        return
    lines = [f"v {x!r} {y!r} {z!r}" for x, y, z in mesh.vertices.tolist()]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in mesh.faces.tolist()]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class MeshCache:
    """Meshes read so far, by file: one TriMesh object per file."""

    def __init__(self):
        self._meshes: Dict[Path, TriMesh] = {}

    def get(self, path: Path) -> TriMesh:
        """The mesh of a file, read on first use.

        Args:
            path: The mesh file.

        Returns:
            TriMesh: The same object for every call with the same file.
        """
        key = Path(path).resolve()
        if key not in self._meshes:
            self._meshes[key] = read_mesh(key)
        return self._meshes[key]
