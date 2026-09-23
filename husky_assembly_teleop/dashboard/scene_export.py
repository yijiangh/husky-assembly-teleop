"""Bake one design problem's planning scene into a single glTF binary file.

The dashboard's 3D viewer needs the same geometry the planner collision-checks:
the husky's links, the assembly tools, every bar/joint/obstacle of the cell and
the ground. That geometry never changes within a problem -- only the poses do --
so it is exported ONCE per problem and cached; each run file then just points at
it and supplies the poses.

Every node is written at the identity transform and named so the page can find
it: ``robot__<link>``, ``tool__<tool>__<link>``, ``body__<rigid_body>``. Double
underscores survive glTF's node-name sanitizing, so the browser needs no
fallback lookup.

! Collision meshes are exported by default, not visual ones: the dashboard's job
! is to explain collisions, so it must show the shapes the checker actually used.
"""
import hashlib
import json
import os
import struct

import numpy as np

# Colours by node kind, so the scene reads at a glance without any per-run data.
_MATERIALS = {
    'robot-link': (0.72, 0.73, 0.75, 1.0),
    'tool-link': (0.35, 0.37, 0.40, 1.0),
    'bar': (0.80, 0.68, 0.45, 1.0),
    'joint': (0.45, 0.52, 0.60, 1.0),
    'obstacle': (0.62, 0.66, 0.68, 1.0),
    'ground': (0.30, 0.32, 0.33, 1.0),
}

_GLB_MAGIC = 0x46546C67
_CHUNK_JSON = 0x4E4F534A
_CHUNK_BIN = 0x004E4942


def _kind_for_body(name):
    """Which material family a rigid body belongs to, from its name."""
    if name.startswith('bar_'):
        return 'bar'
    if name.startswith('joint_'):
        return 'joint'
    if 'ground' in name:
        return 'ground'
    return 'obstacle'


def _triangles(mesh, scale):
    """A compas mesh as flat numpy arrays ready for glTF.

    Faces may be triangles, quads or larger n-gons (the Rhino-exported cells
    carry all three), so every face is fanned into triangles.

    Args:
        mesh (Mesh): the compas mesh.
        scale (Sequence[float]): per-axis scale to bake into the vertices.

    Returns:
        tuple: ``(positions (N,3) float32, indices (M,3) uint32)``, or
        ``(None, None)`` when the mesh has no faces.
    """
    vertices, faces = mesh.to_vertices_and_faces()
    if not vertices or not faces:
        return None, None
    positions = np.asarray(vertices, dtype=np.float32) * np.asarray(scale, dtype=np.float32)
    tris = []
    for face in faces:
        for k in range(1, len(face) - 1):
            tris.append((face[0], face[k], face[k + 1]))
    if not tris:
        return None, None
    return positions, np.asarray(tris, dtype=np.uint32)


def _vertex_normals(positions, indices):
    """Area-weighted vertex normals, so the viewer shades smoothly."""
    normals = np.zeros_like(positions)
    a, b, c = (positions[indices[:, i]] for i in range(3))
    face_normals = np.cross(b - a, c - a)
    for i in range(3):
        np.add.at(normals, indices[:, i], face_normals)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths < 1e-12] = 1.0
    return (normals / lengths).astype(np.float32)


class GlbBuilder:
    """Accumulates meshes and nodes, then writes one ``.glb`` file.

    Meshes are de-duplicated by the hash of their vertex/index data: a cell
    with 81 identical bars pays for one bar mesh.
    """

    def __init__(self):
        self.json = {
            'asset': {'version': '2.0', 'generator': 'husky_assembly_teleop.dashboard'},
            'scenes': [{'nodes': []}], 'scene': 0,
            'nodes': [], 'meshes': [], 'materials': [],
            'accessors': [], 'bufferViews': [], 'buffers': [],
        }
        self._blob = bytearray()
        self._mesh_cache = {}
        self._material_cache = {}

    def _material(self, kind):
        """Index of the (shared) material for a node kind."""
        if kind not in self._material_cache:
            r, g, b, a = _MATERIALS.get(kind, _MATERIALS['obstacle'])
            self.json['materials'].append({
                'name': kind,
                'pbrMetallicRoughness': {
                    'baseColorFactor': [r, g, b, a],
                    'metallicFactor': 0.05, 'roughnessFactor': 0.75,
                },
                'doubleSided': True,
            })
            self._material_cache[kind] = len(self.json['materials']) - 1
        return self._material_cache[kind]

    def _buffer_view(self, data, target=None):
        """Append bytes to the binary blob and return the bufferView index."""
        while len(self._blob) % 4:          # every view must start 4-byte aligned
            self._blob.append(0)
        offset = len(self._blob)
        self._blob.extend(data)
        view = {'buffer': 0, 'byteOffset': offset, 'byteLength': len(data)}
        if target is not None:
            view['target'] = target
        self.json['bufferViews'].append(view)
        return len(self.json['bufferViews']) - 1

    def _accessor(self, data, component_type, count, type_, minmax=None, target=None):
        """Append an accessor over freshly written bytes; return its index."""
        accessor = {
            'bufferView': self._buffer_view(data, target),
            'componentType': component_type, 'count': count, 'type': type_,
        }
        if minmax is not None:
            accessor['min'], accessor['max'] = minmax
        self.json['accessors'].append(accessor)
        return len(self.json['accessors']) - 1

    def add_mesh(self, mesh, scale, kind):
        """Add one compas mesh, reusing an identical one already added.

        Args:
            mesh (Mesh): the compas mesh.
            scale (Sequence[float]): per-axis scale baked into the vertices.
            kind (str): material family (see ``_MATERIALS``).

        Returns:
            int | None: the glTF mesh index, or None when the mesh is empty.
        """
        positions, indices = _triangles(mesh, scale)
        if positions is None:
            return None
        digest = hashlib.sha1(positions.tobytes() + indices.tobytes()
                              + kind.encode()).hexdigest()
        if digest in self._mesh_cache:
            return self._mesh_cache[digest]

        normals = _vertex_normals(positions, indices)
        pos_acc = self._accessor(
            positions.tobytes(), 5126, len(positions), 'VEC3',
            minmax=(positions.min(axis=0).tolist(), positions.max(axis=0).tolist()),
            target=34962)
        nrm_acc = self._accessor(normals.tobytes(), 5126, len(normals), 'VEC3',
                                 target=34962)
        # uint16 indices where they fit -- most cell meshes are small.
        flat = indices.reshape(-1)
        if len(positions) <= 65535:
            idx_acc = self._accessor(flat.astype(np.uint16).tobytes(), 5123,
                                     len(flat), 'SCALAR', target=34963)
        else:
            idx_acc = self._accessor(flat.astype(np.uint32).tobytes(), 5125,
                                     len(flat), 'SCALAR', target=34963)
        self.json['meshes'].append({'primitives': [{
            'attributes': {'POSITION': pos_acc, 'NORMAL': nrm_acc},
            'indices': idx_acc, 'material': self._material(kind), 'mode': 4,
        }]})
        self._mesh_cache[digest] = len(self.json['meshes']) - 1
        return self._mesh_cache[digest]

    def add_root(self, name, kind, visuals):
        """Add one named scene entity with its meshes as children.

        Args:
            name (str): node name the page looks up (``robot__base_link`` etc).
            kind (str): material family, also stored in the node's extras.
            visuals (list): ``(mesh, matrix_16_or_None, scale)`` triples.

        Returns:
            bool: True when at least one mesh was written for this entity.
        """
        children = []
        for mesh, matrix, scale in visuals:
            mesh_index = self.add_mesh(mesh, scale, kind)
            if mesh_index is None:
                continue
            child = {'mesh': mesh_index}
            if matrix is not None:
                child['matrix'] = list(matrix)
            self.json['nodes'].append(child)
            children.append(len(self.json['nodes']) - 1)
        if not children:
            return False
        self.json['nodes'].append({
            'name': name, 'children': children,
            'extras': {'kind': kind},
        })
        self.json['scenes'][0]['nodes'].append(len(self.json['nodes']) - 1)
        return True

    def write(self, path):
        """Write the accumulated scene as a binary glTF file.

        Args:
            path (str): destination ``.glb`` path.

        Returns:
            str: ``path``.
        """
        while len(self._blob) % 4:
            self._blob.append(0)
        self.json['buffers'] = [{'byteLength': len(self._blob)}]
        json_bytes = json.dumps(self.json, separators=(',', ':')).encode('utf-8')
        json_bytes += b' ' * (-len(json_bytes) % 4)
        total = 12 + 8 + len(json_bytes) + 8 + len(self._blob)
        with open(path, 'wb') as handle:
            handle.write(struct.pack('<III', _GLB_MAGIC, 2, total))
            handle.write(struct.pack('<II', len(json_bytes), _CHUNK_JSON))
            handle.write(json_bytes)
            handle.write(struct.pack('<II', len(self._blob), _CHUNK_BIN))
            handle.write(self._blob)
        return path


def _matrix_from_frame(frame):
    """A compas Frame as a column-major 16-float glTF matrix (None = identity)."""
    if frame is None:
        return None
    x, y = np.asarray(frame.xaxis, dtype=float), np.asarray(frame.yaxis, dtype=float)
    z = np.cross(x, y)
    origin = np.asarray(frame.point, dtype=float)
    # glTF matrices are column-major: [col0(3) 0, col1(3) 0, col2(3) 0, t(3) 1]
    return [x[0], x[1], x[2], 0.0, y[0], y[1], y[2], 0.0,
            z[0], z[1], z[2], 0.0, origin[0], origin[1], origin[2], 1.0]


def _scale_of(shape):
    """The per-axis scale of a URDF shape, as three numbers.

    ! Only ``MeshDescriptor`` carries a scale list. The primitive proxies
    ! (box/cylinder/sphere) inherit compas geometry's ``scale()`` METHOD under
    ! the same name, so a plain ``getattr`` hands back a bound method and the
    ! vertex arithmetic then fails; anything that is not three numbers means
    ! "no scaling".

    Args:
        shape: a ``MeshDescriptor`` or one of the primitive proxies.

    Returns:
        list[float]: the three scale factors.
    """
    scale = getattr(shape, 'scale', None)
    if isinstance(scale, (list, tuple)) and len(scale) == 3:
        return [float(v) for v in scale]
    return [1.0, 1.0, 1.0]


def _visuals_for_link(link, geometry):
    """The drawable meshes of one robot/tool link.

    Args:
        link: a compas_robots ``Link``.
        geometry (str): ``'collision'`` or ``'visual'``.

    Returns:
        list: ``(mesh, matrix_16_or_None, scale)`` triples.
    """
    items = link.collision if geometry == 'collision' else link.visual
    # A link may carry only one of the two (the husky URDF has both, the
    # tool models sometimes only visual): fall back rather than draw nothing.
    if not items:
        items = link.visual if geometry == 'collision' else link.collision
    visuals = []
    for item in items or []:
        shape = getattr(getattr(item, 'geometry', None), 'shape', None)
        if shape is None:
            continue
        scale = _scale_of(shape)
        # Both mesh descriptors and the primitive proxies expose ``meshes``:
        # the former after the loader filled them, the latter by tessellating
        # the box/cylinder/sphere on demand.
        meshes = getattr(shape, 'meshes', None) or []
        if not meshes:
            continue
        matrix = _matrix_from_frame(getattr(item, 'origin', None))
        for mesh in meshes:
            visuals.append((mesh, matrix, scale))
    return visuals


def export_scene_glb(robot_cell, out_dir, geometry='collision'):
    """Write ``scene.glb`` + ``scene_nodes.json`` for one loaded RobotCell.

    Args:
        robot_cell: the compas_fab ``RobotCell`` a ``CfabSession`` built.
        out_dir (str): folder to write into (created if missing).
        geometry (str): ``'collision'`` (what the checker sees) or ``'visual'``.

    Returns:
        tuple[str, str]: the ``scene.glb`` and ``scene_nodes.json`` paths.
    """
    os.makedirs(out_dir, exist_ok=True)
    builder = GlbBuilder()
    nodes = []

    def _add(name, kind, visuals):
        if builder.add_root(name, kind, visuals):
            nodes.append({'name': name, 'kind': kind, 'n_meshes': len(visuals)})

    for link in robot_cell.robot_model.iter_links():
        _add(f'robot__{link.name}', 'robot-link', _visuals_for_link(link, geometry))

    for tool_name, tool_model in (robot_cell.tool_models or {}).items():
        for link in tool_model.iter_links():
            _add(f'tool__{tool_name}__{link.name}', 'tool-link',
                 _visuals_for_link(link, geometry))

    for body_name, body in (robot_cell.rigid_body_models or {}).items():
        meshes = (body.collision_meshes if geometry == 'collision' else body.visual_meshes)
        meshes = meshes or body.visual_meshes or body.collision_meshes or []
        native = float(getattr(body, 'native_scale', 1.0) or 1.0)
        scale = [native, native, native]
        _add(f'body__{body_name}', _kind_for_body(body_name),
             [(mesh, None, scale) for mesh in meshes])

    glb_path = builder.write(os.path.join(out_dir, 'scene.glb'))
    nodes_path = os.path.join(out_dir, 'scene_nodes.json')
    with open(nodes_path, 'w') as handle:
        json.dump({'geometry': geometry, 'nodes': nodes}, handle, indent=1)
    return glb_path, nodes_path


def ensure_scene_glb(robot_cell, problem, scenes_dir=None, geometry='collision'):
    """Export the problem's scene once; reuse it on every later run.

    Args:
        robot_cell: the loaded ``RobotCell``.
        problem (str): design problem name (the cache subfolder).
        scenes_dir (str | None): cache root; defaults to the dashboard's.
        geometry (str): ``'collision'`` or ``'visual'``.

    Returns:
        str: the relative path a run file stores, ``'<problem>/scene.glb'``.
    """
    from husky_assembly_teleop.dashboard.run_schema import scenes_dir_default
    root = scenes_dir or scenes_dir_default()
    out_dir = os.path.join(root, problem)
    if not os.path.exists(os.path.join(out_dir, 'scene.glb')):
        print(f'[dashboard] exporting the scene for {problem!r} (once) ...')
        export_scene_glb(robot_cell, out_dir, geometry=geometry)
        size_mb = os.path.getsize(os.path.join(out_dir, 'scene.glb')) / 1e6
        print(f'[dashboard] scene written: {out_dir}/scene.glb ({size_mb:.1f} MB)')
    return f'{problem}/scene.glb'
