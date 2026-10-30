"""
The 3D page the as-built collector writes: the robots' link shapes, the room, and the .html carrying the 3D library.

The browser half is `data/bar_holding_acc_data/viewer_common.js` + `4_assembly_viewer.js`, the old monitor's, unchanged:
`viewer_script` glues them into the one module the page carries, `write_page` wraps it with the scene data and three.js
(`scripts/fetch_three_js.sh` downloads the library once per checkout). Ported from the old `viewer_common.py`; the
robots come from the process's URDFs through the core, the room from the lab file (`world/lab.py`) and, with a Rhino
file, its `env` layer and the Motive camera rig (rhino3dm, imported only then).
"""

from __future__ import annotations

import base64
import json
import re
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import trimesh

from scaffolding_assembly_core.geometry import BoxShape, CylinderShape, Pose, TriMesh
from scaffolding_assembly_core.kinematics import link_poses
from scaffolding_assembly_core.urdf import link_geometry

from ...world.lab import Lab

#: The viewer's browser files, and the 3D library (not in git: `scripts/fetch_three_js.sh`).
DATA_FOLDER = Path(__file__).resolve().parents[3] / "data" / "bar_holding_acc_data"
VENDOR_DIR = DATA_FOLDER / "vendor" / "three-r185"
THREE_CORE = VENDOR_DIR / "build" / "three.core.min.js"
THREE_MODULE = VENDOR_DIR / "build" / "three.module.min.js"
ORBIT_CONTROLS = VENDOR_DIR / "examples" / "jsm" / "controls" / "OrbitControls.js"
#: The Rhino layers holding the room's solids and the Motive camera rig.
ENV_LAYER = "env"
CAMERA_LAYER = "Mocap::mocap_cameras"
#: How far each camera's view cone is drawn, and how wide: a readable stand-in, not a measured frustum.
CAMERA_CONE_M = 0.9
CAMERA_CONE_HALF_ANGLE_DEG = 22.0
#: The chassis links whose extent is the robot's footprint on the overview.
CHASSIS_LINKS = ("base_link", "top_chassis_link", "front_bumper_link", "rear_bumper_link")


# * ================================================================ packing

def pack(array: np.ndarray) -> str:
    """A numeric array as base64 of its bytes: thousands of coordinates as json text would triple the file."""
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode("ascii")


def flatten(matrix: np.ndarray) -> list[float]:
    """A 4x4 matrix as the 16 numbers three.js reads, column by column."""
    return [round(float(v), 6) for v in np.asarray(matrix).T.reshape(-1)]


def matrix_from_pose(pose) -> np.ndarray | None:
    """A recorded pose as a 4x4: [x, y, z, qx, qy, qz, qw], or the old [[pos], [quat]]; None when there is none."""
    if pose is None:
        return None
    if len(pose) == 2:
        pose = [*pose[0], *pose[1]]
    return Pose(tuple(map(float, pose[:3])), tuple(map(float, pose[3:7]))).matrix()


def _triangles(shape) -> tuple[np.ndarray, np.ndarray] | None:
    """A core shape as (vertices, faces) in its body's frame; None for one that cannot be drawn."""
    if isinstance(shape, TriMesh):
        return np.asarray(shape.vertices, dtype=np.float32), np.asarray(shape.faces, dtype=np.int64)
    if isinstance(shape, BoxShape):
        mesh = trimesh.creation.box(extents=shape.size, transform=shape.origin.matrix())
    elif isinstance(shape, CylinderShape):
        mesh = trimesh.creation.cylinder(radius=shape.radius, height=shape.height, transform=shape.origin.matrix())
    else:
        return None
    return np.asarray(mesh.vertices, dtype=np.float32), np.asarray(mesh.faces, dtype=np.int64)


def link_shapes(urdf: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Each link's visual shapes merged into one (vertices, faces) in the link frame; links without any left out."""
    out = {}
    for link, geometry in link_geometry(urdf).items():
        pieces = [piece for piece in map(_triangles, geometry.visual) if piece is not None]
        if not pieces:
            continue
        vertices, faces, offset = [], [], 0
        for points, triangles in pieces:
            vertices.append(points)
            faces.append(triangles + offset)
            offset += len(points)
        out[link] = (np.vstack(vertices), np.vstack(faces))
    return out


def pack_link_meshes(shapes: Mapping[str, tuple[np.ndarray, np.ndarray]]) -> dict:
    """The robot's link shapes packed small: {link: {positions, indices, bits}}, each in its own frame."""
    packed = {}
    for name, (points, faces) in shapes.items():
        # ! uint16 indices only while a link stays under 65536 vertices; the heaviest here is the chassis.
        index_type = np.uint16 if len(points) <= 65535 else np.uint32
        packed[name] = {"positions": pack(np.asarray(points, dtype=np.float32)),
                        "indices": pack(np.asarray(faces, dtype=index_type)),
                        "bits": 16 if index_type is np.uint16 else 32}
    return packed


def footprint_size(shapes: Mapping[str, tuple[np.ndarray, np.ndarray]]) -> list[float]:
    """The chassis's overall [length, width, height], metres, for the overview rectangles."""
    chassis = [name for name in shapes if "wheel" in name or name in CHASSIS_LINKS]
    points = np.vstack([shapes[name][0] for name in chassis]) if chassis else np.zeros((1, 3))
    return (points.max(axis=0) - points.min(axis=0)).tolist()


class RobotModels:
    """The robots' meshes, loaded once per URDF, and each drawn pose's link matrices."""

    def __init__(self):
        """Nothing is loaded until a robot is drawn."""
        #: Model key (the process robot's name) -> {urdf, shapes, meshes, footprint}.
        self.models: dict[str, dict] = {}

    def pose(self, name: str, urdf: Path, base, joints: Mapping[str, float], *, live: bool, source: str) -> dict:
        """One robot drawn at one pose: {name, model, live, source, base, links} (16 numbers per matrix)."""
        if name not in self.models:
            shapes = link_shapes(urdf)
            self.models[name] = {"urdf": urdf, "shapes": shapes, "meshes": pack_link_meshes(shapes),
                                 "footprint": footprint_size(shapes)}
        model = self.models[name]
        base_pose = Pose.from_matrix(matrix_from_pose(base))
        placed = link_poses(model["urdf"], base_pose, {k: float(v) for k, v in joints.items() if v is not None},
                            list(model["shapes"]))
        return {"name": name, "model": name, "live": live, "source": source, "base": flatten(base_pose.matrix()),
                "links": {link: flatten(pose.matrix()) for link, pose in placed.items()}}

    def payload(self) -> dict:
        """The meshes the page needs: {model: {meshes, footprint}} for every model drawn."""
        return {key: {"meshes": model["meshes"], "footprint": model["footprint"]} for key, model in self.models.items()}


# * ================================================================ the room

def _packed_mesh(solids: Sequence[tuple[np.ndarray, Sequence[Sequence[int]]]]) -> dict | None:
    """Many solids as one drawable mesh: {positions, indices, count}; a quad face becomes two triangles."""
    if not solids:
        return None
    positions, triangles, offset = [], [], 0
    for vertices, faces in solids:
        positions.extend(np.asarray(vertices, dtype=float).tolist())
        for face in faces:
            triangles.append([offset + face[0], offset + face[1], offset + face[2]])
            if len(face) == 4:
                triangles.append([offset + face[0], offset + face[2], offset + face[3]])
        offset += len(vertices)
    return {"positions": pack(np.array(positions, dtype=np.float32)),
            "indices": pack(np.array(triangles, dtype=np.uint32)), "count": len(solids)}


def lab_environment(lab: Lab) -> dict | None:
    """The lab file's obstacles as one mesh: boxes and trusses as boxes, cylinders as 24-sided prisms."""
    solids = []
    for shape in lab.shapes:
        pose = Pose((shape.center[0], shape.center[1], shape.center[2]),
                    (0.0, 0.0, float(np.sin(shape.yaw / 2)), float(np.cos(shape.yaw / 2))))
        if shape.type == "cylinder":
            mesh = trimesh.creation.cylinder(radius=shape.size[0] / 2, height=shape.size[2], sections=24,
                                             transform=pose.matrix())
        else:
            mesh = trimesh.creation.box(extents=shape.size, transform=pose.matrix())
        solids.append((np.asarray(mesh.vertices), np.asarray(mesh.faces).tolist()))
    return _packed_mesh(solids)


def _read_3dm(path: Path):
    """The Rhino model and the module, or (None, None) without rhino3dm or a readable file (said)."""
    try:
        import rhino3dm as r3
    except ImportError:
        print("  [env] rhino3dm not installed; the .3dm is skipped")
        return None, None
    model = None
    for _ in range(3):  # Insync may touch the file mid-read
        model = r3.File3dm.Read(str(path))
        if model is not None:
            break
    if model is None:
        print(f"  [env] could not read {path}")
    return model, r3


def _meshes_of_3dm_solid(geometry, r3) -> list:
    """The drawable meshes of one Rhino solid: a mesh as is, an extrusion's, a Brep's faces'; else none."""
    kind = type(geometry).__name__
    if kind == "Mesh":
        return [geometry]
    if kind == "Extrusion":
        mesh = geometry.GetMesh(r3.MeshType.Default)
        return [mesh] if mesh else []
    if kind == "Brep":
        faces = [geometry.Faces[i].GetMesh(r3.MeshType.Default) for i in range(len(geometry.Faces))]
        return [mesh for mesh in faces if mesh]
    return []


def environment_3dm(path: Path, layer: str = ENV_LAYER) -> dict | None:
    """The room's solids on a Rhino layer (millimetres) as one mesh in metres; None without any (said)."""
    model, r3 = _read_3dm(path)
    if model is None:
        return None
    layers = {each.Index for each in model.Layers if each.Name.lower() == layer.lower()}
    if not layers:
        print(f"  [env] layer {layer!r} not found in {path.name}")
        return None
    solids, skipped = [], 0
    for obj in model.Objects:
        if obj.Attributes.LayerIndex not in layers:
            continue
        meshes = _meshes_of_3dm_solid(obj.Geometry, r3)
        if not meshes and type(obj.Geometry).__name__ in ("Mesh", "Extrusion", "Brep", "SubD"):
            skipped += 1
        for mesh in meshes:
            vertices = [[mesh.Vertices[i].X * 1e-3, mesh.Vertices[i].Y * 1e-3, mesh.Vertices[i].Z * 1e-3]
                        for i in range(len(mesh.Vertices))]
            faces = []
            for i in range(len(mesh.Faces)):
                face = tuple(mesh.Faces[i])  # (a, b, c, d); a triangle when c == d
                faces.append([face[0], face[1], face[2]] if face[2] == face[3] else list(face))
            solids.append((np.array(vertices), faces))
    if skipped:
        print(f"  [env] {skipped} solid(s) on layer {layer!r} carry no saved mesh: open {path.name} in Rhino, set "
              f"the viewport to Shaded so the meshes are built, and save.")
    print(f"  [env] {len(solids)} solids from {path.name}")
    return _packed_mesh(solids)


def cameras_3dm(path: Path, layer: str = CAMERA_LAYER) -> list[dict]:
    """The Motive camera rig on a Rhino layer: one {name, position, matrix} per camera, for the page's cones.

    Each camera is a point (or a text dot) with three short lines from it, its axes; the second is the line of sight.
    """
    model, r3 = _read_3dm(path)
    if model is None:
        return []
    layers = {each.Index for each in model.Layers if each.FullPath.lower() == layer.lower()}
    if not layers:
        print(f"  [env] layer {layer!r} not found in {path.name}")
        return []
    points, segments, labels = [], [], []

    def xyz(point) -> list[float]:
        return [point.X * 1e-3, point.Y * 1e-3, point.Z * 1e-3]

    for obj in model.Objects:
        if obj.Attributes.LayerIndex not in layers:
            continue
        geometry, kind = obj.Geometry, type(obj.Geometry).__name__
        if kind == "Point":
            points.append(xyz(geometry.Location))
        elif kind == "LineCurve":
            segments.append((xyz(geometry.PointAtStart), xyz(geometry.PointAtEnd)))
        elif kind == "TextDot":
            labels.append((geometry.Text, xyz(geometry.Point)))
    # * One camera per distinct position (rounded to a tenth of a millimetre: the repeated copies collapse).
    at: dict[tuple, list] = {}
    for point in points + [where for _text, where in labels]:
        at.setdefault(tuple(round(v, 4) for v in point), point)
    axes: dict[tuple, list] = {spot: [] for spot in at}
    for a, b in segments:
        spot, length = tuple(round(v, 4) for v in a), float(np.linalg.norm(np.subtract(b, a)))
        if spot not in at or not 0 < length <= 0.3:
            continue
        unit = (np.subtract(b, a) / length).tolist()
        if not any(np.linalg.norm(np.subtract(unit, seen)) < 1e-6 for seen in axes[spot]):
            axes[spot].append(unit)
    named = {tuple(round(v, 4) for v in where): text for text, where in labels}
    cameras = []
    for spot, origin in at.items():
        if len(axes[spot]) != 3:
            continue
        right, direction, up = axes[spot]
        matrix = np.eye(4)
        matrix[:3, 0], matrix[:3, 1], matrix[:3, 2], matrix[:3, 3] = right, direction, up, origin
        cameras.append({"name": named.get(spot, "") or "camera", "position": [round(v, 4) for v in origin],
                        "matrix": flatten(matrix)})
    print(f"  [env] {len(cameras)} cameras from {path.name}")
    return cameras


# * ================================================================ writing the page

def _check_three_js() -> tuple[str, str, str]:
    """The three library files' text, or how to fetch them.

    Raises:
        SystemExit: When the library files have not been downloaded.
    """
    missing = [path for path in (THREE_CORE, THREE_MODULE, ORBIT_CONTROLS) if not path.exists()]
    if missing:
        raise SystemExit("The 3D library files are missing:\n  " + "\n  ".join(map(str, missing))
                         + "\n\nThey are not kept in git. Download them once with:\n  scripts/fetch_three_js.sh")
    return THREE_CORE.read_text(), THREE_MODULE.read_text(), ORBIT_CONTROLS.read_text()


def embed_three_js() -> str:
    """The library inside the page as an import map of data: URLs, so the page needs no network."""
    core, module, controls = _check_three_js()
    rewritten = module.replace('"./three.core.min.js"', '"three-core"')
    if rewritten == module:
        raise SystemExit("three.module.min.js no longer refers to its core file the expected way")

    def as_url(text: str) -> str:
        return "data:text/javascript;base64," + base64.b64encode(text.encode()).decode("ascii")

    imports = {"three-core": as_url(core), "three": as_url(rewritten),
               "three/addons/controls/OrbitControls.js": as_url(controls)}
    return '<script type="importmap">' + json.dumps({"imports": imports}) + "</script>"


def _specifiers(block: str, importing: bool = False) -> list[tuple[str, str]]:
    """An import/export name list as (outside, inside) pairs.

    ! `export{a as Vector3}`: the file calls it `a`, the world `Vector3` (outside on the RIGHT); `import{Vector3 as e}`:
      the world calls it `Vector3`, the file `e` (outside on the LEFT).
    """
    pairs = []
    for piece in block.split(","):
        piece = " ".join(piece.split())
        if not piece:
            continue
        if " as " in piece:
            left, right = (part.strip() for part in piece.split(" as ", 1))
            pairs.append((left, right) if importing else (right, left))
        else:
            pairs.append((piece, piece))
    return pairs


def _as_scoped_module(source: str, name: str, incoming: str | None = None, from_object: str | None = None) -> str:
    """One ES module as a self-contained expression: its import a destructuring, its export a returned object.

    Each file in its OWN function keeps their minified names apart (the core and the renderer both use `e`, `t`).
    """
    # ! `export{...}from"other.js"` first: a re-export forwards names that never exist in this file.
    given = []
    for match in list(re.finditer(r'export\s*\{([^}]*)\}\s*from\s*[\'"][^\'"]*[\'"]\s*;?', source)):
        for outside, inside in _specifiers(match.group(1)):
            given.append((outside, f'{from_object}["{inside}"]'))
    source = re.sub(r'export\s*\{[^}]*\}\s*from\s*[\'"][^\'"]*[\'"]\s*;?', "", source)
    for match in list(re.finditer(r"export\s*\{([^}]*)\}\s*;?", source)):
        for outside, inside in _specifiers(match.group(1)):
            given.append((outside, inside))
    if not given:
        raise SystemExit(f"{name}: found no export list to bundle.")
    source = re.sub(r"export\s*\{[^}]*\}\s*;?", "", source)
    if incoming:
        match = re.search(incoming, source)
        if not match:
            raise SystemExit(f"{name}: its import statement no longer looks the way the bundler expects.")
        taken = _specifiers(match.group(1), importing=True)
        pulled = ",".join(f"{outside}:{inside}" for outside, inside in taken)
        source = source[:match.start()] + f"const {{{pulled}}}={from_object};" + source[match.end():]
    body = ",".join(f'"{outside}":{expression}' for outside, expression in given)
    return f"(function(){{\n{source}\nreturn {{{body}}};\n}})()"


def bundle_three_js(viewer_js: str) -> str:
    """The library and the viewer as ONE ordinary inline script: nothing imported, fetched or mapped.

    ! What a page needs when it is SERVED inside someone else's page (an artifact): no data: URL, no sibling file.
    """
    core, module, controls = _check_three_js()
    core_expr = _as_scoped_module(core, "three.core.min.js")
    module_expr = _as_scoped_module(module, "three.module.min.js",
                                    incoming=r'import\s*\{([^}]*)\}\s*from\s*"\./three\.core\.min\.js"\s*;?',
                                    from_object="__core")
    controls_expr = _as_scoped_module(controls, "OrbitControls.js",
                                      incoming=r'import\s*\{([^}]*)\}\s*from\s*[\'"]three[\'"]\s*;?',
                                      from_object="THREE")
    body = re.sub(r"^\s*import\s[^\n]*\n", "", viewer_js, flags=re.M)
    return ("(function(){\n\"use strict\";\n"
            f"const __core={core_expr};\nconst __three={module_expr};\n"
            "const THREE=Object.assign({},__core,__three);\n"
            f"const __controls={controls_expr};\nconst OrbitControls=__controls[\"OrbitControls\"];\n"
            f"\n{body}\n}})();")


def sibling_three_js(out_path: Path) -> str:
    """The library written beside the page, and an import map pointing at it (for a hosted page under a CSP)."""
    core, module, controls = _check_three_js()
    beside = Path(out_path).resolve().parent
    for name, text in (("three.core.min.js", core), ("three.module.min.js", module), ("OrbitControls.js", controls)):
        (beside / name).write_text(text)
    imports = {"three": "./three.module.min.js", "three/addons/controls/OrbitControls.js": "./OrbitControls.js"}
    return '<script type="importmap">' + json.dumps({"imports": imports}) + "</script>"


def viewer_script(page_js: str = "4_assembly_viewer.js") -> str:
    """The page's browser code and the shared viewer_common.js as ONE module (exports and imports dropped).

    ! Both files follow their headers' rules: one-line imports, `export` on the declaration, no name declared in both.
    """
    common = (DATA_FOLDER / "viewer_common.js").read_text()
    page = (DATA_FOLDER / page_js).read_text()
    common = re.sub(r"^export\s+(?=(?:function|const|let|class)\b)", "", common, flags=re.M)
    page = re.sub(r"^\s*import\s[^\n]*\n", "", page, flags=re.M)
    return common + "\n" + page


def write_page(scene: dict, out_path: Path, vendor: str, viewer_js: str, extra_css: str = "",
               controls: str = "") -> Path:
    """Write the finished page.

    Args:
        scene: Everything the page draws (`floor` is a 16-number matrix or None: the grid at z = 0).
        out_path: Where to write the .html.
        vendor: "inline" bundles the library into one ordinary script; "embed" keeps it as an import map of data:
            URLs; "sibling" writes it beside the page (a hosted copy).
        viewer_js: The page's browser code (`viewer_script`).
        extra_css: Style rules the page adds, ending with a newline.
        controls: Extra html laid over the scene, ending with a newline.
    """
    out_path = Path(out_path)
    scene = {"floor": None, **scene}
    if vendor == "inline":
        importmap, script_open, viewer_js = "", "<script>", bundle_three_js(viewer_js)
    else:
        importmap = sibling_three_js(out_path) if vendor == "sibling" else embed_three_js()
        script_open = '<script type="module">'
    html = _PAGE_TEMPLATE.format(title=scene["title"], importmap=importmap, script_open=script_open,
                                 data=json.dumps(scene, separators=(",", ":")), viewer_js=viewer_js,
                                 extra_css=extra_css, controls=controls)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html)
    external = len(re.findall(r'(?:src|href)\s*=\s*["\']https?://', html))
    print(f"[page] {out_path}: {out_path.stat().st_size / 1e6:.1f} MB, {external} external references "
          f"({'fully offline' if external == 0 else 'NOT offline!'})")
    return out_path


# ! Filled with str.format: every literal brace is doubled; {extra_css} and {controls} sit at the start of a line.
_PAGE_TEMPLATE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title>
{importmap}
<style>
:root {{ --bg:#ffffff; --panel:#f6f7f9; --line:#d9dee5; --text:#1d2430;
         --dim:#6b7686; --accent:#1763b8; }}
* {{ box-sizing:border-box; }}
html,body {{ margin:0; height:100%; background:var(--bg); color:var(--text);
  font:13px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }}
#app {{ display:flex; height:100%; }}
#scene {{ flex:1; position:relative; min-width:0; }}
#scene canvas {{ display:block; }}
#side {{ width:300px; flex:none; background:var(--panel);
  border-left:1px solid var(--line); overflow-y:auto; padding:16px; }}
h1 {{ font-size:15px; margin:0 0 2px; }}
h2 {{ font-size:11px; letter-spacing:.08em; text-transform:uppercase;
  color:var(--dim); margin:16px 0 6px; font-weight:600; }}
.sub {{ color:var(--dim); font-size:11px; margin-bottom:4px; }}
.row {{ display:flex; justify-content:space-between; gap:10px; padding:3px 0;
  border-bottom:1px solid rgba(0,0,0,.06); }}
.row span:last-child {{ font-variant-numeric:tabular-nums; }}
.muted {{ color:var(--dim); }}
.warn {{ color:#b45309; }}
select,button {{ background:#fff; color:var(--text);
  border:1px solid var(--line); border-radius:5px; padding:5px 8px;
  font:inherit; width:100%; }}
button {{ cursor:pointer; margin-top:6px; }}
button:hover {{ border-color:var(--accent); }}
label.chk {{ display:flex; align-items:center; gap:7px; padding:3px 0;
  cursor:pointer; }}
.card {{ position:absolute; left:14px; background:rgba(255,255,255,.96);
  border:1px solid var(--line); border-radius:8px; width:336px;
  box-shadow:0 2px 14px rgba(20,30,50,.13); }}
#legend {{ bottom:14px; padding:10px 12px; }}
#detail {{ padding:12px 14px; display:none;
  max-height:calc(100vh - 250px); overflow-y:auto; }}
#detail.open {{ display:block; }}
#detail h2:first-of-type {{ margin-top:0; }}
.block h2 {{ margin-top:0; }}
.block {{ margin-top:16px; }}
.block.focus {{ margin:16px -9px 0; padding:8px 9px 7px; border-radius:6px;
  background:#eef4fc; border:1px solid #c7dbf4; }}
.pill {{ margin-left:7px; padding:1px 5px; border-radius:3px;
  background:var(--accent); color:#fff; font-size:9px; letter-spacing:.06em; }}
#detail-close {{ position:absolute; top:7px; right:9px; width:auto; margin:0;
  padding:0 7px 2px; font-size:16px; line-height:1.3; color:var(--dim);
  background:transparent; border-color:transparent; }}
#detail-close:hover {{ color:var(--text); border-color:var(--line); }}
#legend-head {{ display:flex; align-items:center; gap:8px; margin-bottom:2px; }}
#layers {{ margin-top:9px; padding-top:8px; border-top:1px solid var(--line);
  display:flex; flex-wrap:wrap; gap:3px 10px; align-items:center; }}
#layers label {{ display:flex; align-items:center; gap:4px; cursor:pointer;
  font-size:11px; color:var(--dim); white-space:nowrap; }}
#layers input, #side input {{ margin:0; accent-color:#4a7fb5; }}
#reset {{ width:auto; margin:0 0 0 auto; padding:2px 8px; font-size:11px; }}
#legend-head span {{ color:var(--dim); font-size:10px; letter-spacing:.07em;
  text-transform:uppercase; white-space:nowrap; }}
#metric-legend {{ flex:1; padding:3px 6px; font-size:12px; }}
#ramp {{ height:11px; border-radius:3px; margin:6px 0 3px;
  border:1px solid rgba(0,0,0,.10); }}
#ticks {{ display:flex; justify-content:space-between; color:var(--dim);
  font-size:10px; font-variant-numeric:tabular-nums; }}
#bands {{ margin-top:7px; font-size:11px; color:var(--dim); }}
.swatch {{ display:inline-block; width:9px; height:9px; border-radius:2px;
  margin-right:5px; vertical-align:-1px; border:1px solid rgba(0,0,0,.12); }}
#hint {{ position:absolute; top:12px; left:14px; color:var(--dim);
  font-size:11px; line-height:1.7; }}
.formula {{ margin:9px 0 0; }}
.formula code {{ display:block; margin:3px 0 2px; padding:4px 7px;
  background:#eef2f7; border:1px solid #dde4ec; border-radius:4px;
  font-size:11.5px; color:#1d3f72; overflow-wrap:anywhere; }}
#formula-toggle {{ width:auto; padding:3px 9px; font-size:11px; }}
#detail {{ scrollbar-width:thin; scrollbar-color:#4a7fb5 transparent; }}
#detail::-webkit-scrollbar {{ width:9px; }}
#detail::-webkit-scrollbar-thumb {{ background:#4a7fb5; border-radius:5px; }}
#detail::-webkit-scrollbar-track {{ background:transparent; }}
.flag {{ margin:8px 0 0; padding:7px 9px; border-radius:6px;
  background:#fff6f5; border:1px solid #e2a79c; color:#7a2318; font-size:11.5px; }}
#boot {{ display:none; }}
#boot.boot-error {{ display:block; position:absolute; top:70px; left:14px;
  max-width:560px; background:#fff6f5; border:1px solid #e2a79c;
  border-radius:8px; padding:14px 16px; font-size:12.5px; line-height:1.6;
  color:#7a2318; }}
#boot code {{ background:#f3e2de; padding:1px 4px; border-radius:3px; }}
#status {{ margin-top:12px; padding-top:10px; border-top:1px solid var(--line);
  color:var(--dim); font-size:11px; }}
{extra_css}</style></head><body>
<div id="app">
  <div id="scene">
    <div id="hint">left drag to rotate, right drag to move, scroll to zoom<br>
      left click a bar to select and see detail, ESC or the card's &times;
      to unpin</div>
    <div class="card" id="detail"></div>
    <div class="card" id="legend">
      <div id="legend-head"><span>colour by</span><select id="metric-legend"></select></div>
      <div id="legend-head"><span style="min-width:52px">data by</span>
        <select id="data-legend"></select></div>
      <div id="legend-body"></div>
      <div id="layers">
        <label><input type="checkbox" id="show-env" checked> environment</label>
        <label><input type="checkbox" id="show-authored" checked> authored</label>
        <label><input type="checkbox" id="show-markers" checked> markers</label>
        <label><input type="checkbox" id="show-robots" checked> robots</label>
        <label><input type="checkbox" id="show-cameras"> mocap cameras</label>
        <label><input type="checkbox" id="show-labels" checked> labels</label>
        <button id="reset">reset view</button>
      </div>
    </div>
{controls}  </div>
  <div id="side">
    <h1>{title}</h1>
    <div class="sub" id="subtitle"></div>
    <div class="sub" id="metric-help" style="margin-top:10px"></div>
    <div id="status"></div>
  </div>
</div>
<div id="boot"></div>
<script>
/* ! A module that fails to load leaves a BLANK page: say so on the page instead of showing nothing. */
(function () {{
  var box = document.getElementById('boot');
  function fail(what) {{
    if (window.__viewerReady) return;
    box.className = 'boot-error';
    box.innerHTML = '<b>The 3D view did not start.</b><br>' + what
      + '<br><br>If this page is being served rather than opened from a file, '
      + 'its 3D library may have been blocked. Re-generate it with '
      + '<code>--vendor sibling</code> and keep the three .js files beside it.';
  }}
  window.addEventListener('error', function (e) {{
    fail(e.message ? ('<code>' + e.message + '</code>') : 'A script was blocked or failed to load.');
  }}, true);
  setTimeout(function () {{ fail('It did not finish starting.'); }}, 6000);
}})();
</script>
<script type="application/json" id="scene-data">{data}</script>
{script_open}
{viewer_js}
</script></body></html>
"""
