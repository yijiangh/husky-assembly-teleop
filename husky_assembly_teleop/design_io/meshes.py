"""
Mesh files of a design (format §6): read with trimesh, written as plain OBJ text.

* `MeshCache` gives every shape naming the same file the same TriMesh object, so mirrors build it once.
? A write and read keeps a mesh as it is, except that unused vertices are dropped.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict

from trimesh import Trimesh, load

from .geometry import TriMesh


def read_mesh(path: Path) -> TriMesh:
    """Read one mesh file (.obj, .stl or .glb) as triangles, without merging or cleaning."""
    mesh = load(str(path), force="mesh", process=False)
    return TriMesh.from_arrays(mesh.vertices, mesh.faces)


def write_mesh(mesh: TriMesh, path: Path) -> None:
    """Write one mesh file, its type chosen by the extension; OBJ keeps every coordinate exact.

    Args:
        mesh: The mesh.
        path: Where to write; `.obj`, `.stl` or `.glb`. Missing folders are created.
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
        """Start with no meshes read."""
        self._meshes: Dict[Path, TriMesh] = {}

    def get(self, path: Path) -> TriMesh:
        """The mesh of a file, read on first use; the same object for every call with the same file."""
        key = Path(path).resolve()
        if key not in self._meshes:
            self._meshes[key] = read_mesh(key)
        return self._meshes[key]
