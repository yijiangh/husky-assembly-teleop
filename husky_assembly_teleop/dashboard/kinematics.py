"""Forward kinematics for the dashboard's 3D viewer.

A run file stores configurations (twelve joint values per waypoint), not poses:
that keeps a thousand-candidate run down to a few megabytes. The browser cannot
evaluate a URDF, so the server turns the configuration the user clicked on into
one world matrix per scene node -- exactly the nodes ``scene_export`` wrote --
and the page just assigns them.

The same forward kinematics also draws the robot into the overview chart's
scatter plot -- see ``robot_triangles`` -- so the operator can see where the
candidate cloud sits relative to the machine that has to reach it.

! This runs on the URDF alone: no pybullet, no CfabSession, no 150 MB RobotCell.
! The bodies riding on the arms are placed from the run's own attachment table
! (link-relative poses captured at production time).
"""
import os

import numpy as np
from compas_robots import RobotModel
from compas_robots.resources import LocalPackageMeshLoader

from husky_assembly_teleop import DATA_DIRECTORY
from husky_assembly_teleop.dashboard.run_schema import describe_candidate
from husky_assembly_teleop.dashboard.scene_export import _triangles, _visuals_for_link

# * The mesh packages the husky URDFs point at with ``package://`` URIs. They
# * all live side by side under ``data/husky_urdf/`` -- same list cfab_session
# * uses when it builds the planning cell.
_MESH_PACKAGES = ('husky_description', 'husky_ur_description', 'ur_description')

# ! The husky's full collision geometry is about 27k triangles, and ONE link
# ! (the top chassis) is a third of that on its own -- too much to hand a plotly
# ! scatter that already carries 750 dots. Any link heavier than this is drawn
# ! as its own axis-aligned bounding box instead (12 triangles). The chassis and
# ! the bumpers really are box-shaped, so the robot still reads correctly, and
# ! this is a stand-in for the robot, not the shape the planner collision-checks.
_MAX_TRIANGLES_PER_LINK = 3000


def _matrix_from_pose(pose):
    """A ``{pos, quat_xyzw}`` pose as a 4x4 numpy matrix."""
    x, y, z, w = (float(v) for v in pose['quat_xyzw'])
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = [float(v) for v in pose['pos']]
    return matrix


def _matrix_from_frame(frame):
    """A compas Frame as a 4x4 numpy matrix."""
    xaxis = np.asarray(frame.xaxis, dtype=float)
    yaxis = np.asarray(frame.yaxis, dtype=float)
    matrix = np.eye(4)
    matrix[:3, 0] = xaxis
    matrix[:3, 1] = yaxis
    matrix[:3, 2] = np.cross(xaxis, yaxis)
    matrix[:3, 3] = np.asarray(frame.point, dtype=float)
    return matrix


def _matrix_from_gl(matrix16):
    """The 16 column-major floats of a glTF matrix back as a 4x4 numpy matrix.

    Args:
        matrix16 (Sequence[float] | None): the 16 floats, or None for identity.

    Returns:
        np.ndarray: the 4x4 matrix.
    """
    if matrix16 is None:
        return np.eye(4)
    return np.asarray(matrix16, dtype=float).reshape(4, 4).T


def _box_triangles(positions):
    """A point cloud replaced by its axis-aligned bounding box.

    Used for the few links whose mesh is far too heavy to send to the browser
    (see ``_MAX_TRIANGLES_PER_LINK``).

    Args:
        positions (np.ndarray): the (N,3) vertices to box in.

    Returns:
        tuple: ``(corners (8,3) float32, faces (12,3) uint32)``.
    """
    low, high = positions.min(axis=0), positions.max(axis=0)
    # Corner order is the bit pattern of (x, y, z): corner 5 is (high, low, high).
    corners = np.array([[high[0] if ix else low[0],
                         high[1] if iy else low[1],
                         high[2] if iz else low[2]]
                        for ix in (0, 1) for iy in (0, 1) for iz in (0, 1)],
                       dtype=np.float32)
    faces = np.array([(0, 1, 3), (0, 3, 2),        # the x = low side
                      (4, 6, 7), (4, 7, 5),        # the x = high side
                      (0, 4, 5), (0, 5, 1),        # the y = low side
                      (2, 3, 7), (2, 7, 6),        # the y = high side
                      (0, 2, 6), (0, 6, 4),        # the z = low side
                      (1, 5, 7), (1, 7, 3)],       # the z = high side
                     dtype=np.uint32)
    return corners, faces


def _gl(matrix):
    """A 4x4 numpy matrix as the 16 column-major floats three.js expects."""
    return [round(float(v), 6) for v in matrix.T.reshape(-1)]


class SceneKinematics:
    """Places every scene node for a given robot configuration.

    Args:
        urdf_path (str): the same dual-arm URDF the planner uses.
    """

    def __init__(self, urdf_path):
        self.model = RobotModel.from_urdf_file(urdf_path)
        self._zero = self.model.zero_configuration()
        self._root_link = self.model.get_base_link_name()
        # Link meshes are only needed by the 2D overview chart's robot overlay,
        # and reading them off disk takes a moment, so they are loaded on the
        # first request and kept (see ``_link_geometry``).
        self._link_geometry_cache = None

    def link_nodes(self):
        """Every robot link node name the exported scene may contain."""
        return [f'robot__{link.name}' for link in self.model.iter_links()]

    def link_matrices(self, world_from_mb, conf12, joint_names_12):
        """World matrices of all robot links at one configuration.

        Args:
            world_from_mb (np.ndarray): 4x4 pose of the mobile base.
            conf12 (Sequence[float]): the twelve arm joint values.
            joint_names_12 (Sequence[str]): their joint names, in order.

        Returns:
            dict[str, np.ndarray]: link name -> 4x4 world matrix.
        """
        configuration = self._zero.copy()
        for name, value in zip(joint_names_12, conf12):
            configuration[name] = float(value)
        matrices = {self._root_link: world_from_mb}
        for joint, frame in zip(self.model.iter_joints(),
                                self.model.transformed_frames(configuration)):
            matrices[joint.child_link.name] = world_from_mb @ _matrix_from_frame(frame)
        return matrices

    def _link_geometry(self):
        """Every link's collision triangles, in that link's own frame.

        The model this class builds carries no meshes (the viewer gets its
        geometry from the baked scene file instead), so the first call loads
        them from ``data/husky_urdf/`` and keeps the result: reading them is
        slow enough to be worth doing once per server.

        Each visual's own placement inside the link is baked into the vertices
        here, so drawing a link later costs exactly one matrix multiply.

        Returns:
            dict[str, tuple]: link name -> ``(vertices (N,3) float32,
            triangles (M,3) uint32)``, only for links that have geometry.
        """
        if self._link_geometry_cache is not None:
            return self._link_geometry_cache

        mesh_root = os.path.join(DATA_DIRECTORY, 'husky_urdf')
        self.model.load_geometry(*[LocalPackageMeshLoader(mesh_root, package)
                                   for package in _MESH_PACKAGES])
        geometry = {}
        for link in self.model.iter_links():
            vertices, triangles = [], []
            offset = 0
            # 'collision' is what the planner checks, and it is the lighter of
            # the two mesh sets the husky URDF carries.
            for mesh, matrix16, scale in _visuals_for_link(link, 'collision'):
                points, faces = _triangles(mesh, scale)
                if points is None:
                    continue
                matrix = _matrix_from_gl(matrix16)
                vertices.append(points @ matrix[:3, :3].T + matrix[:3, 3])
                triangles.append(faces + offset)
                offset += len(points)
            if not vertices:
                continue
            points = np.concatenate(vertices).astype(np.float32)
            faces = np.concatenate(triangles).astype(np.uint32)
            if len(faces) > _MAX_TRIANGLES_PER_LINK:
                points, faces = _box_triangles(points)
            geometry[link.name] = (points, faces)
        self._link_geometry_cache = geometry
        return geometry

    def robot_triangles(self, conf12, joint_names_12):
        """The whole robot as one triangle soup, in the MOBILE-BASE frame.

        Feeding ``link_matrices`` an identity base pose puts every link
        straight into the mobile-base frame -- which is the frame the overview
        chart's candidate dots already live in, so the two line up with no
        further transform.

        The six arrays are exactly what a plotly ``mesh3d`` trace wants:
        ``x/y/z`` are the vertices and ``i/j/k`` the three corner indices of
        every triangle.

        Args:
            conf12 (Sequence[float]): the twelve arm joint values.
            joint_names_12 (Sequence[str]): their joint names, in order.

        Returns:
            dict: ``{n_triangles, x, y, z, i, j, k}``.
        """
        links = self.link_matrices(np.eye(4), conf12, joint_names_12)
        vertices, triangles = [], []
        offset = 0
        for name, (points, faces) in self._link_geometry().items():
            matrix = links.get(name)
            if matrix is None:
                continue
            vertices.append(points @ matrix[:3, :3].T + matrix[:3, 3])
            triangles.append(faces + offset)
            offset += len(points)
        points = np.concatenate(vertices)
        faces = np.concatenate(triangles)
        # Round to a tenth of a millimetre: far finer than anything visible,
        # and it roughly halves the JSON the browser has to download.
        points = np.round(points, 4)
        return {
            'n_triangles': int(len(faces)),
            'x': points[:, 0].tolist(),
            'y': points[:, 1].tolist(),
            'z': points[:, 2].tolist(),
            'i': faces[:, 0].tolist(),
            'j': faces[:, 1].tolist(),
            'k': faces[:, 2].tolist(),
        }

    def _node_matrices(self, run, conf12):
        """Every animated node's world matrix at one configuration.

        Args:
            run (dict): the run record (for the base pose and attachments).
            conf12 (Sequence[float]): the twelve arm joint values.

        Returns:
            dict[str, np.ndarray]: node name -> 4x4 world matrix.
        """
        context = run['context']
        world_from_mb = _matrix_from_pose(context['world_from_mobile_base'])
        links = self.link_matrices(world_from_mb, conf12, context['joint_names_12'])
        out = {f'robot__{name}': matrix for name, matrix in links.items()}
        for attachment in context.get('attachments') or []:
            parent = links.get(attachment['parent_link'])
            if parent is None:
                continue
            out[attachment['node']] = parent @ _matrix_from_pose(attachment['link_from_body'])
        return out

    def frames_for_candidate(self, run, index, scene_nodes=None):
        """Everything the viewer needs to show one candidate's walk.

        The frames run from the goal pose along the accepted track to wherever
        the walk ended, with an extra frame at the first colliding waypoint
        when the straight path was blocked. Each frame carries the nodes that
        collide there, so the viewer can highlight them.

        Args:
            run (dict): the run record.
            index (int): index into ``run['candidates']``.
            scene_nodes (set[str] | None): the nodes the exported scene really
                has. Roughly a third of the URDF's links are bare frames with
                no geometry (``tool0``, ``flange``, ``imu_link`` ...), so
                sending matrices for them would be payload the viewer has
                nothing to apply to. None keeps every node.

        Returns:
            dict: ``{candidate, nodes, frames, ghosts, caption}``.
        """
        cand = run['candidates'][index]
        context = run['context']
        travel_cm = float(cand.get('travel_cm') or 0.0)
        n_total = max(1, int(cand.get('n_total') or 1) - 1)
        track = cand.get('track') or {'indices': [], 'confs': []}

        # Collect (label, conf, colliding nodes) in walk order.
        steps = []
        for position, conf in zip(track.get('indices') or [], track.get('confs') or []):
            along = travel_cm * (float(position) / n_total)
            label = ('goal pose (start of the walk)' if position == 0
                     else f'{along:.0f} of {travel_cm:.0f} cm along the walk')
            steps.append((label, conf, [], 'track'))

        collide_nodes = []
        for hit in cand.get('collisions') or []:
            collide_nodes.extend([hit.get('a'), hit.get('b')])
        collide_nodes = [node for node in collide_nodes if node]

        outcome = cand.get('outcome')
        if outcome == 'blocked' and cand.get('blocked_conf'):
            at = travel_cm * (float(cand.get('blocked_at') or 0.0))
            steps.append((f'first blocked waypoint ({at:.0f} of {travel_cm:.0f} cm)',
                          cand['blocked_conf'], collide_nodes, 'blocked'))
        elif outcome == 'arrival_collision' and steps:
            label, conf, _, _ = steps[-1]
            steps[-1] = ('arrived at home -- and collides there', conf,
                         collide_nodes, 'arrival')
        elif outcome == 'track_break' and steps:
            reason = ((cand.get('break') or {}).get('reason') or 'break')
            words = ('IK found no solution past here' if reason == 'ik_miss'
                     else 'the IK solution jumped to another branch here')
            label, conf, _, _ = steps[-1]
            steps[-1] = (f'{label} -- {words}', conf, [], 'last')

        if not steps:
            steps = [('start configuration', cand.get('last_conf') or [0.0] * 12, [], 'last')]

        # Node order is fixed across frames so the page can assign by index.
        nodes = sorted(self._node_matrices(run, steps[0][1]).keys())
        if scene_nodes is not None:
            nodes = [node for node in nodes if node in scene_nodes]
        frames = []
        for label, conf, colliding, kind in steps:
            matrices = self._node_matrices(run, conf)
            frames.append({
                'label': label,
                'kind': kind,
                'matrices': [_gl(matrices[node]) for node in nodes],
                'colliding': colliding,
            })

        world_from_mb = _matrix_from_pose(context['world_from_mobile_base'])
        home_pose = {'pos': cand.get('home_pos_mb') or [0, 0, 0],
                     'quat_xyzw': cand.get('home_quat_mb') or [0, 0, 0, 1]}
        ghosts = {
            'home_bar': _gl(world_from_mb @ _matrix_from_pose(home_pose)),
            'goal_bar': _gl(_matrix_from_pose(context['goal']['bar_pose_world'])),
            'break_bar': None,
        }
        break_pose = (cand.get('break') or {}).get('pose_world')
        if break_pose:
            ghosts['break_bar'] = _gl(_matrix_from_pose(break_pose))

        return {
            'candidate': index,
            'nodes': nodes,
            'frames': frames,
            'ghosts': ghosts,
            'active_bar_node': f'body__{run.get("active_bar")}',
            'caption': describe_candidate(run, cand),
        }
