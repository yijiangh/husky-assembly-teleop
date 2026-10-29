"""The parts of a 3D session page that every viewer script shares.

``2_session_viewer.py`` (one bar-holding session) and the assembly viewer
(one lab day of as-built records) draw different data on the same kind of page:
the cell's solids, the Motive camera rig, the robot's link shapes, and a
``.html`` that carries the 3D library inside it. All of that lives here, so the
two pages cannot drift apart in how they read a ``.3dm``, pack a mesh or embed
the library. The session-specific measuring stays in each script.

The browser half is split the same way: ``viewer_common.js`` holds the scene,
the primitives and the controls, and each page's ``.js`` imports from it.
:func:`viewer_script` glues the two into the one module the page carries.
"""

import base64
import json
import os
import re
import sys
from datetime import datetime

# ! A script in this folder puts only its OWN folder on the import path, not
# ! the repo, so the package import below fails with ModuleNotFoundError even
# ! when you are sitting in the repo root. Put the repo root on the path first.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

import numpy as np

from husky_assembly_teleop.mocap_experiment import (
    read_env_obstacles_3dm,
    read_mocap_cameras_3dm,
)
from husky_assembly_teleop.mocap_floor import load_mocap_floor

# ! Imported lazily inside load_kinematics(): loading the URDF pulls in pybullet
# ! and costs a couple of seconds, which a --no-robots run should not pay.

# * This folder: the viewer .js files are read from here.
_HERE = os.path.dirname(os.path.abspath(__file__))

# * Where the three javascript files live. They are NOT in git (see .gitignore);
# * scripts/fetch_dashboard_vendor.sh downloads them once per checkout. They are
# * needed only to GENERATE the page -- the finished page carries its own copy.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
VENDOR_DIR = os.path.join(_REPO_ROOT, 'husky_assembly_teleop', 'dashboard',
                          'static', 'vendor', 'three-r185')
THREE_CORE = os.path.join(VENDOR_DIR, 'build', 'three.core.min.js')
THREE_MODULE = os.path.join(VENDOR_DIR, 'build', 'three.module.min.js')
ORBIT_CONTROLS = os.path.join(VENDOR_DIR, 'examples', 'jsm', 'controls',
                              'OrbitControls.js')

# * The Rhino layer holding the environment solids (desks, columns, walls), and
# * the one holding the Motive camera rig.
ENV_LAYER = 'env'
CAMERA_LAYER = 'Mocap::mocap_cameras'

# * How far to draw each camera's view cone, and how wide it opens. Motive does
# * not record a field of view, so this is a readable stand-in, not a measured
# * frustum -- it says where a camera points, not exactly what it can see.
CAMERA_CONE_M = 0.9
CAMERA_CONE_HALF_ANGLE_DEG = 22.0


# ---------------------------------------------------------------------------
# * Reading files
# ---------------------------------------------------------------------------

def file_timestamp(path: str) -> datetime:
    """When a take or run was saved, read from its own filename.

    ! Deliberately NOT the file's modification time. Editing a take -- fixing a
    ! wrong label, say -- rewrites that, and the record would then sort to the
    ! end of the session and pair with the wrong run. The name is written once,
    ! when the file is saved, and never changes.

    Takes are stamped to the minute (``..._1902.json``) and runs to the second
    (``..._185931.json``).

    Args:
        path (str): The file.

    Returns:
        datetime: Its saved time, or None when the name does not carry one.
    """
    stem = os.path.splitext(os.path.basename(path))[0]
    date, _, clock = stem.rpartition('_')
    date = date.rpartition('_')[2]
    # ! Pick the format by how many digits the clock has, rather than trying
    # ! both. strptime accepts one OR two digits per field, so 'HHMM' also
    # ! matches an 'HHMMSS' pattern -- 1514 would come back as 15:01:04.
    formats = {6: '%Y%m%d%H%M%S', 4: '%Y%m%d%H%M'}
    if not date.isdigit() or not clock.isdigit() or len(clock) not in formats:
        return None
    try:
        return datetime.strptime(date + clock, formats[len(clock)])
    except ValueError:
        return None


def load_json(path: str) -> dict:
    """Read a json file.

    Args:
        path (str): File to read.

    Returns:
        dict: Its contents.
    """
    with open(path, 'r') as handle:
        return json.load(handle)


# ---------------------------------------------------------------------------
# * The robot: its model, its link shapes, and placing them
# ---------------------------------------------------------------------------

# * One loaded model per URDF path, kept for the life of the process: reading a
# * URDF costs seconds, and one page may draw the same robot many times over.
_KINEMATICS_BY_PATH = {}


def load_kinematics(urdf_path: str) -> tuple:
    """Load a robot model once and keep it for every later call.

    Args:
        urdf_path (str): The URDF to load.

    Returns:
        tuple: ``(SceneKinematics, geometry)`` where geometry is
        ``{link_name: (points, faces)}`` for the links that have a shape.
    """
    if urdf_path not in _KINEMATICS_BY_PATH:
        from husky_assembly_teleop.dashboard.kinematics import SceneKinematics
        kinematics = SceneKinematics(urdf_path)
        _KINEMATICS_BY_PATH[urdf_path] = (kinematics, kinematics._link_geometry())
    return _KINEMATICS_BY_PATH[urdf_path]


def robot_matrices(kinematics, geometry: dict, world_from_base: np.ndarray,
                   joint_values: list, joint_names: list) -> dict:
    """Per-link placement matrices for one robot at one configuration.

    Args:
        kinematics (SceneKinematics): The loaded robot model.
        geometry (dict): Links that actually have a shape to draw.
        world_from_base (numpy.ndarray): The 4x4 base placement in the world.
        joint_values (list): One value per joint, in radians.
        joint_names (list): The joint each value belongs to.

    Returns:
        dict: ``{'base': 16 floats, 'links': {name: 16 floats}}``.
    """
    placements = kinematics.link_matrices(world_from_base, joint_values,
                                         joint_names)
    return {
        'base': flatten(world_from_base),
        'links': {name: flatten(matrix) for name, matrix in placements.items()
                  if name in geometry},
    }


def pack_link_meshes(geometry: dict) -> dict:
    """The robot's link shapes, packed small enough to carry in the page.

    Each link keeps its own vertices and triangles in its own frame, so drawing
    it later is one matrix away.

    Args:
        geometry (dict): ``{link_name: (points, faces)}`` from SceneKinematics.

    Returns:
        dict: ``{link_name: {'positions': base64, 'indices': base64}}``.
    """
    packed = {}
    for name, (points, faces) in geometry.items():
        # ! uint16 indices only work while a link stays under 65536 vertices;
        # ! the heaviest here is the chassis at a few thousand.
        index_type = np.uint16 if len(points) <= 65535 else np.uint32
        packed[name] = {
            'positions': pack(np.asarray(points, dtype=np.float32)),
            'indices': pack(np.asarray(faces, dtype=index_type)),
            'bits': 16 if index_type is np.uint16 else 32,
        }
    return packed


def _closest_point_on_segment(point, start, end) -> tuple:
    """The nearest point on a line segment, and how far away it is.

    Args:
        point (numpy.ndarray): The point to measure from.
        start (numpy.ndarray): One end of the segment.
        end (numpy.ndarray): The other end.

    Returns:
        tuple: ``(point_on_the_segment, distance_m)``.
    """
    along = end - start
    length = float(np.linalg.norm(along))
    direction = along / length
    # ! Clamped to the segment, not to its infinite line, so a wrist beyond a
    # ! bar's end is measured against the end rather than past it.
    reach = float(np.clip(np.dot(point - start, direction), 0.0, length))
    foot = start + reach * direction
    return foot, float(np.linalg.norm(point - foot))


def matrix_from_frame(frame) -> np.ndarray:
    """A 4x4 placement matrix from a frame's origin and two axes.

    The frame stores x and y; z is their cross product. The axes are
    re-squared first so small rounding in the export cannot skew the robot.

    Args:
        frame: A compas ``Frame`` with ``point``, ``xaxis`` and ``yaxis``, or
            the same three as a plain ``{'point', 'xaxis', 'yaxis'}`` dict --
            the raw shape a cell state keeps them in, so no compas is needed.

    Returns:
        numpy.ndarray: The 4x4 matrix.
    """
    if isinstance(frame, dict):
        origin, x_axis, y_axis = frame['point'], frame['xaxis'], frame['yaxis']
    else:
        origin, x_axis, y_axis = frame.point, frame.xaxis, frame.yaxis
    origin = np.asarray(origin, dtype=float)
    x_axis = np.asarray(x_axis, dtype=float)
    y_axis = np.asarray(y_axis, dtype=float)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis - x_axis * float(np.dot(x_axis, y_axis))
    y_axis = y_axis / np.linalg.norm(y_axis)
    matrix = np.eye(4)
    matrix[:3, 0] = x_axis
    matrix[:3, 1] = y_axis
    matrix[:3, 2] = np.cross(x_axis, y_axis)
    matrix[:3, 3] = origin
    return matrix


def matrix_from_pose(pos: list, quat: list) -> np.ndarray:
    """A 4x4 placement matrix from a position and an xyzw quaternion.

    Args:
        pos (list): ``[x, y, z]`` in metres.
        quat (list): ``[x, y, z, w]``.

    Returns:
        numpy.ndarray: The 4x4 matrix.
    """
    from husky_assembly_teleop.dashboard.kinematics import _matrix_from_pose
    return _matrix_from_pose({'pos': pos, 'quat_xyzw': quat})


def footprint_size(geometry: dict) -> list:
    """Overall size of the husky chassis, for the overview rectangles.

    Args:
        geometry (dict): ``{link_name: (points, faces)}`` in link frames.

    Returns:
        list: ``[length, width, height]`` in metres.
    """
    chassis = [name for name in geometry
               if 'wheel' in name or name in ('base_link', 'top_chassis_link',
                                              'front_bumper_link',
                                              'rear_bumper_link')]
    points = np.vstack([geometry[name][0] for name in chassis])
    return (points.max(axis=0) - points.min(axis=0)).tolist()


def flatten(matrix: np.ndarray) -> list:
    """A 4x4 matrix as the 16 numbers three.js reads, column by column.

    Args:
        matrix (numpy.ndarray): The 4x4 matrix.

    Returns:
        list: Sixteen floats.
    """
    return [round(float(v), 6) for v in np.asarray(matrix).T.reshape(-1)]


# ---------------------------------------------------------------------------
# * The environment, and packing geometry small
# ---------------------------------------------------------------------------

def build_environment(env_3dm: str) -> dict:
    """Read the cell's solids and merge them into one drawable mesh.

    Args:
        env_3dm (str): The ``.3dm`` to read, or None to skip the environment.

    Returns:
        dict: ``{'positions': base64, 'indices': base64, 'count': int}``.
    """
    if not env_3dm or not os.path.exists(env_3dm):
        print("[env] no .3dm given; the scene will have no environment")
        return None
    solids = read_env_obstacles_3dm(env_3dm, ENV_LAYER)
    if not solids:
        print(f"[env] layer {ENV_LAYER!r} held no solids")
        return None

    positions, triangles, offset = [], [], 0
    for solid in solids:
        vertices = solid['verts']
        positions.extend(vertices)
        for face in solid['faces']:
            # * Rhino gives three- or four-sided faces; a quad becomes 2 triangles.
            triangles.append([offset + face[0], offset + face[1], offset + face[2]])
            if len(face) == 4:
                triangles.append([offset + face[0], offset + face[2],
                                  offset + face[3]])
        offset += len(vertices)

    print(f"[env] {len(solids)} solids, {len(positions)} points, "
          f"{len(triangles)} triangles")
    return {
        'positions': pack(np.array(positions, dtype=np.float32)),
        'indices': pack(np.array(triangles, dtype=np.uint32)),
        'count': len(solids),
    }


def build_cameras(env_3dm: str) -> list:
    """The Motive camera rig, as a position and an orientation per camera.

    Args:
        env_3dm (str): The ``.3dm`` to read, or None to skip the cameras.

    Returns:
        list: One record per camera, ready for the page to draw a cone from.
    """
    if not env_3dm or not os.path.exists(env_3dm):
        return []
    cameras = read_mocap_cameras_3dm(env_3dm, CAMERA_LAYER)
    return [{
        'name': camera['name'] or 'camera',
        'position': [round(v, 4) for v in camera['position']],
        # * The three axes become one placement matrix, so the page can orient a
        # * cone with a single call instead of redoing this arithmetic.
        'matrix': flatten(_camera_matrix(camera)),
    } for camera in cameras]


def _camera_matrix(camera: dict) -> np.ndarray:
    """A placement matrix that stands a cone up along a camera's line of sight.

    The cone three.js builds points along its own +Y, which is also the axis the
    camera looks down, so the camera's three axes can be used as the matrix
    directly.

    Args:
        camera (dict): One entry from ``read_mocap_cameras_3dm``.

    Returns:
        numpy.ndarray: The 4x4 matrix.
    """
    matrix = np.eye(4)
    matrix[:3, 0] = camera['right']
    matrix[:3, 1] = camera['direction']
    matrix[:3, 2] = camera['up']
    matrix[:3, 3] = camera['position']
    return matrix


def pack(array: np.ndarray) -> str:
    """Encode a numeric array as text the page can turn back into numbers.

    Writing thousands of coordinates as plain json text would roughly triple
    the file; this keeps the raw bytes and encodes them once.

    Args:
        array (numpy.ndarray): A float32 or uint32 array.

    Returns:
        str: Base64 of the array's bytes.
    """
    return base64.b64encode(array.tobytes()).decode('ascii')


# ---------------------------------------------------------------------------
# * Writing the page
# ---------------------------------------------------------------------------

def _check_three_js() -> tuple:
    """Read the three library files, or explain how to fetch them.

    Returns:
        tuple: ``(core, module, controls)`` as text.

    Raises:
        SystemExit: When the library files have not been downloaded.
    """
    missing = [p for p in (THREE_CORE, THREE_MODULE, ORBIT_CONTROLS)
               if not os.path.exists(p)]
    if missing:
        raise SystemExit(
            "The 3D library files are missing:\n  "
            + "\n  ".join(missing)
            + "\n\nThey are not kept in git. Download them once with:\n"
              "  scripts/fetch_dashboard_vendor.sh")
    return (open(THREE_CORE).read(), open(THREE_MODULE).read(),
            open(ORBIT_CONTROLS).read())


def embed_three_js() -> str:
    """Put the 3D library inside the page, so it needs no network.

    The library is three files. ``three.core.min.js`` stands alone;
    ``three.module.min.js`` loads it by relative path, which an embedded module
    cannot follow, so those two references are rewritten to a name the import
    map below defines; ``OrbitControls.js`` (mouse rotate/zoom/pan) asks for
    "three", which the map also supplies.

    Returns:
        str: The ``<script type="importmap">`` block.
    """
    core, module, controls = _check_three_js()
    rewritten = module.replace('"./three.core.min.js"', '"three-core"')
    if rewritten == module:
        raise SystemExit("three.module.min.js no longer refers to its core file "
                         "the expected way; the embedding needs updating.")

    def as_url(text: str) -> str:
        """Carry one javascript file inside the page as an address of its own."""
        return ('data:text/javascript;base64,'
                + base64.b64encode(text.encode()).decode('ascii'))

    imports = {
        'three-core': as_url(core),
        'three': as_url(rewritten),
        'three/addons/controls/OrbitControls.js': as_url(controls),
    }
    return ('<script type="importmap">'
            + json.dumps({'imports': imports})
            + '</script>')


def _specifiers(block: str, importing: bool = False) -> list:
    """Split an import/export name list into ``(outside, inside)`` pairs.

    ! The two directions are mirror images and mixing them up is silent.
    ! ``export{a as Vector3}`` means the file calls it ``a`` and the world sees
    ! ``Vector3``, so the outside name is on the RIGHT. ``import{Vector3 as e}``
    ! means the world calls it ``Vector3`` and the file calls it ``e``, so the
    ! outside name is on the LEFT.

    Args:
        block (str): The text between the braces.
        importing (bool): True for an import list, False for an export list.

    Returns:
        list: ``[(outside_name, inside_name), ...]``.
    """
    pairs = []
    for piece in block.split(','):
        piece = ' '.join(piece.split())
        if not piece:
            continue
        if ' as ' in piece:
            left, right = (part.strip() for part in piece.split(' as ', 1))
            pairs.append((left, right) if importing else (right, left))
        else:
            pairs.append((piece, piece))
    return pairs


def _as_scoped_module(source: str, name: str, incoming: str = None,
                      from_object: str = None) -> str:
    """Rewrite one ES module as a self-contained expression.

    The file's ``import`` becomes a plain destructuring from an object the
    caller already built, and its ``export`` becomes a returned object. Wrapping
    each file in its OWN function keeps their short minified names apart -- the
    core and the renderer build both use names like ``e`` and ``t`` at the top
    level, so pasting them into one scope would collide.

    Args:
        source (str): The module's text.
        name (str): For error messages.
        incoming (str): Regex matching the import statement to replace, if any.
        from_object (str): The expression the imported names come from.

    Returns:
        str: ``(function(){ ... return {...}; })()``.
    """
    # ! Handle `export{...}from"other.js"` BEFORE the plain `export{...}`. It is
    # ! a re-export: the names are forwarded straight out of the other file and
    # ! never exist in this one. Matching only its `export{...}` half would
    # ! leave a dangling `from"..."` (a syntax error) and would also claim
    # ! local variables that were never declared.
    given = []
    for match in list(re.finditer(
            r'export\s*\{([^}]*)\}\s*from\s*[\'"][^\'"]*[\'"]\s*;?', source)):
        for outside, inside in _specifiers(match.group(1)):
            given.append((outside, f'{from_object}["{inside}"]'))
    source = re.sub(r'export\s*\{[^}]*\}\s*from\s*[\'"][^\'"]*[\'"]\s*;?',
                    '', source)

    for match in list(re.finditer(r'export\s*\{([^}]*)\}\s*;?', source)):
        for outside, inside in _specifiers(match.group(1)):
            given.append((outside, inside))
    if not given:
        raise SystemExit(f"{name}: found no export list to bundle.")
    source = re.sub(r'export\s*\{[^}]*\}\s*;?', '', source)

    if incoming:
        match = re.search(incoming, source)
        if not match:
            raise SystemExit(f"{name}: its import statement no longer looks the "
                             f"way the bundler expects.")
        taken = _specifiers(match.group(1), importing=True)
        pulled = ','.join(f'{outside}:{inside}' for outside, inside in taken)
        source = (source[:match.start()] + f'const {{{pulled}}}={from_object};'
                  + source[match.end():])

    body = ','.join(f'"{outside}":{expression}' for outside, expression in given)
    return f'(function(){{\n{source}\nreturn {{{body}}};\n}})()'


def bundle_three_js(viewer_js: str) -> str:
    """The 3D library and the viewer as ONE ordinary inline script.

    ! This is what a page needs when it is SERVED rather than opened from disk.
    ! A host may wrap the page inside a document of its own and serve it from a
    ! path where neither a ``data:`` URL nor a relative filename resolves, so
    ! an import map and sibling files both fail and the page comes up blank.
    ! Nothing here is imported, fetched or mapped: the library, its controls and
    ! the viewer are one script with no outside reference at all.

    Args:
        viewer_js (str): The viewer module, whose own imports are removed.

    Returns:
        str: A ``<script>`` body.
    """
    core, module, controls = _check_three_js()
    core_expr = _as_scoped_module(core, 'three.core.min.js')
    module_expr = _as_scoped_module(
        module, 'three.module.min.js',
        incoming=r'import\s*\{([^}]*)\}\s*from\s*"\./three\.core\.min\.js"\s*;?',
        from_object='__core')
    controls_expr = _as_scoped_module(
        controls, 'OrbitControls.js',
        incoming=r'import\s*\{([^}]*)\}\s*from\s*[\'"]three[\'"]\s*;?',
        from_object='THREE')

    # The viewer asks for the library by name; here it is already in scope.
    body = re.sub(r'^\s*import\s[^\n]*\n', '', viewer_js, flags=re.M)
    return (
        '(function(){\n'
        '"use strict";\n'
        f'const __core={core_expr};\n'
        f'const __three={module_expr};\n'
        # * The renderer build re-exports most of the core, but merging both is
        # * what guarantees every name the viewer reaches for is present.
        'const THREE=Object.assign({},__core,__three);\n'
        f'const __controls={controls_expr};\n'
        'const OrbitControls=__controls["OrbitControls"];\n'
        f'\n{body}\n'
        '})();'
    )


def sibling_three_js(out_path: str) -> str:
    """Write the 3D library next to the page and point the import map at it.

    ! Needed wherever the page is SERVED rather than opened from disk. A hosted
    ! page usually runs under a content-security policy, and those routinely
    ! refuse to execute a ``data:`` script -- which is how the embedded build
    ! carries the library, so the page comes up blank with the whole module
    ! blocked. Plain sibling files are fetched normally and pass.

    The page is then no longer a single file: the three ``.js`` files have to
    travel with it.

    Args:
        out_path (str): The ``.html`` being written; the library lands beside it.

    Returns:
        str: The ``<script type="importmap">`` block.
    """
    core, module, controls = _check_three_js()
    beside = os.path.dirname(os.path.abspath(out_path))
    for name, text in (('three.core.min.js', core),
                       ('three.module.min.js', module),   # keeps its own import
                       ('OrbitControls.js', controls)):
        with open(os.path.join(beside, name), 'w') as handle:
            handle.write(text)
    imports = {
        'three': './three.module.min.js',
        'three/addons/controls/OrbitControls.js': './OrbitControls.js',
    }
    return ('<script type="importmap">'
            + json.dumps({'imports': imports})
            + '</script>')


def viewer_script(page_js_path: str) -> str:
    """One page's browser code and the shared ``viewer_common.js``, as ONE module.

    The page's ``.js`` is written as a real ES module that imports from
    ``viewer_common.js``, so an editor can follow every name. The finished page
    carries the two as a single script instead: the shared file first with its
    ``export`` keywords dropped, then the page with its ``import`` lines
    dropped, so every shared name is simply in scope. What remains imports only
    the 3D library, exactly as a single-file viewer did, so the three
    ``--vendor`` modes need no change.

    ! This only works because both files follow the rules in their headers:
    ! one-line imports, ``export`` written on the declaration itself (never an
    ! ``export {...}`` list), and no top-level name declared in both files.

    Args:
        page_js_path (str): The page's ``.js``, by name when it sits in this
            folder, or as a full path.

    Returns:
        str: The module text to paste into the page.
    """
    with open(os.path.join(_HERE, 'viewer_common.js'), 'r') as handle:
        common = handle.read()
    with open(os.path.join(_HERE, page_js_path), 'r') as handle:
        page = handle.read()
    common = re.sub(r'^export\s+(?=(?:function|const|let|class)\b)', '',
                    common, flags=re.M)
    page = re.sub(r'^\s*import\s[^\n]*\n', '', page, flags=re.M)
    return common + '\n' + page


def write_page(scene: dict, out_path: str, vendor: str, viewer_js: str,
               extra_css: str = '', controls: str = '') -> None:
    """Write the finished page.

    Args:
        scene (dict): Everything the page draws.
        out_path (str): Where to write the ``.html``.
        vendor (str): ``'inline'`` bundles the 3D library into one ordinary
            script; ``'embed'`` keeps it inside the file as an import map, so
            the page travels alone; ``'sibling'`` writes the library beside it,
            which is what a hosted copy needs.
        viewer_js (str): The page's browser code, from :func:`viewer_script`.
        extra_css (str): Style rules a page adds to the shared ones; end it
            with a newline. Empty leaves the page exactly as before.
        controls (str): Extra html laid over the scene (a slider, say); end it
            with a newline. Empty leaves the page exactly as before.
    """
    # * The grid is drawn on the measured lab floor (tilted 0.5 deg against the
    # * MoCap XY plane), not at z = 0: floor frame -> world, or None for z = 0.
    floor = load_mocap_floor()
    scene = {**scene, 'floor': flatten(floor.world_from_floor) if floor is not None else None}
    if vendor == 'inline':
        # One ordinary script, nothing imported or fetched: the only form that
        # survives being served inside someone else's page.
        importmap, script_open = '', '<script>'
        viewer_js = bundle_three_js(viewer_js)
    else:
        importmap = (sibling_three_js(out_path) if vendor == 'sibling'
                     else embed_three_js())
        script_open = '<script type="module">'
    html = _PAGE_TEMPLATE.format(
        title=scene['title'],
        importmap=importmap,
        script_open=script_open,
        data=json.dumps(scene, separators=(',', ':')),
        viewer_js=viewer_js,
        extra_css=extra_css,
        controls=controls,
    )
    with open(out_path, 'w') as handle:
        handle.write(html)

    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    # ? Count only what the browser would actually FETCH. Plain text matches are
    # ? no use now that the library is inlined unencoded: its source carries an
    # ? XML namespace URI and a citation in a shader comment, neither a request.
    external = len(re.findall(r'(?:src|href)\s*=\s*["\']https?://', html))
    print(f"\n[page] {out_path}")
    print(f"       {size_mb:.1f} MB, {external} external references "
          f"({'fully offline' if external == 0 else 'NOT offline!'})")
    if vendor == 'sibling':
        print("       the 3D library sits beside it as three .js files; they "
              "must travel with the page")


# ! Filled with str.format, so every literal brace in the css and the boot script
# ! is doubled. The two page-specific slots, {extra_css} and {controls}, each
# ! sit at the START of a line: an empty value adds not a single byte.
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
/* * The two cards floating over the scene: the colour key, and the detail card
   * that opens directly above it once a bar is selected. */
.card {{ position:absolute; left:14px; background:rgba(255,255,255,.96);
  border:1px solid var(--line); border-radius:8px; width:336px;
  box-shadow:0 2px 14px rgba(20,30,50,.13); }}
#legend {{ bottom:14px; padding:10px 12px; }}
/* * Tall enough to show every number without scrolling, while still clearing
   * the hint text at the top and the colour key below. */
#detail {{ padding:12px 14px; display:none;
  max-height:calc(100vh - 250px); overflow-y:auto; }}
#detail.open {{ display:block; }}
#detail h2:first-of-type {{ margin-top:0; }}
/* * The card is one block per measurement, and the one the "data by" picker
   * asks for is marked and set in bold -- so clicking a bar answers the
   * question the picker asked instead of leaving it among the others. */
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
/* ! The layer toggles live in this floating card, NOT in the side panel: the
   ! panel is a fixed strip at the right edge and falls off screen entirely in a
   ! window wider than the display, taking its controls with it. */
#layers {{ margin-top:9px; padding-top:8px; border-top:1px solid var(--line);
  display:flex; flex-wrap:wrap; gap:3px 10px; align-items:center; }}
#layers label {{ display:flex; align-items:center; gap:4px; cursor:pointer;
  font-size:11px; color:var(--dim); white-space:nowrap; }}
/* * Tick boxes in the environment's own blue, so the controls read as part of
   * the same drawing rather than borrowing the browser's accent colour. */
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
/* ! A module that fails to load leaves a BLANK page -- no error, nothing. That
   ! is exactly what happens when a host's content-security policy refuses the
   ! script, so say so on the page instead of showing nothing. */
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
