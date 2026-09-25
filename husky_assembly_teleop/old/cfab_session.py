"""Long-lived compas_fab PyBullet session for a single design-study problem.

Owns a `PyBulletClient` + `PyBulletPlanner` and the deserialized
`RobotCell`. Single source of truth for scene materialization on the
planning side — replaces ad-hoc URDF / tool / robot-cell loading that
previously lived in `common.py` and `design_interface/`.

Usage:

    s = CfabSession("2026-05-08_dual-arm_transfer_test")
    s.planner.set_robot_cell_state(some_state)
    s.planner.check_collision(some_state, {"full_report": True})
    ...
    s.close()
"""

from __future__ import annotations

import json
import os
import tempfile
from typing import Optional, Tuple

import numpy as np
import pybullet_planning as pp

from compas.data import json_load
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.robots import RigidBody, RobotCell, RobotCellState, RobotSemantics
from compas_robots import RobotModel, ToolModel
from compas_robots.resources import LocalPackageMeshLoader

# Importing rs_data_structure registers the compas dtypes of the BarAction
# movement classes so BarAction JSONs deserialize correctly.
import rs_data_structure  # noqa: F401

from husky_assembly_teleop import DATA_DIRECTORY, DESIGN_DATA_DIRECTORY

# Robot description files for the two rig layouts. The dual-arm paths used to
# come from husky_assembly_tamp's run.py, but that module no longer resolves
# after the submodule prune, so this repo's own copies are the source now.
HUSKY_DUAL_URDF_PATH = os.path.join(
    DATA_DIRECTORY,
    'husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/'
    'husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf')
HUSKY_DUAL_SRDF_PATH = os.path.join(
    DATA_DIRECTORY,
    'husky_urdf/mt_husky_dual_ur5_e_moveit_config/config/dual_arm_husky.srdf')
# Per-robot calibrated single-arm files, keyed by robot name (see
# husky_world.init ROBOT_CONFIGS: Alice=0804, Belle=0805).
HUSKY_SINGLE_URDF_PATHS = {
    '0804': os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf'),
    '0805': os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Belle_Calibrated.urdf'),
}
HUSKY_SINGLE_SRDF_PATHS = {
    '0804': os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/config/husky.srdf'),
    '0805': os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/config/belle.srdf'),
}
# ToolModels exported once from the design-study RobotCell.json (meshes are
# embedded in the JSON). Re-export if the Rhino tool geometry changes.
TOOL_MODEL_DIR = os.path.join(DATA_DIRECTORY, 'tool_models')
ROBOTIQ_MESH_PATH = os.path.join(
    DATA_DIRECTORY, 'husky_urdf/robotiq_85/meshes/static/robotiq_85_close_20mm.obj')

# Planning group of the single-arm SRDFs (mt_husky_moveit_config).
SINGLE_ARM_GROUP = 'base_arm_manipulator'

# --- Ground / walkable-ground collision geometry ---------------------------
# Cell rigid-body name for the floor. The `obstacle_` prefix (matching the
# Rhino-exported `obstacle_env*` bodies) deliberately keeps it OUT of
# husky_monitor.BUILT_ASSEMBLY_RB_PREFIXES, so the mocap-accuracy hide never
# blanks the ground: it must stay collision-checked in every BarAction.
GROUND_RIGID_BODY_NAME = 'obstacle_ground'
# The floor sits at z=0 (faithful to reality) and is extruded DOWNWARD by this
# much. A flat, zero-thickness polygon would collapse into a zero-volume convex
# hull in PyBullet (compas_fab adds rigid bodies with concavity=False), which is
# useless for collision -- hence the slab.
GROUND_SLAB_THICKNESS = 0.05  # meters
# Half-extent of the fallback floor used when a problem ships no
# WalkableGround.json. Large enough to cover any cell workspace, so the arms and
# tools can never reach the real-world floor.
GROUND_FALLBACK_HALF_SIZE = 20.0  # meters (=> 40 x 40 m slab)
# Coordinates above this magnitude are treated as millimetres and scaled to
# metres. Same heuristic as mocap_experiment._walkable_ground_polygons.
_GROUND_MM_THRESHOLD = 50.0


def _slab_mesh_from_polygon(points_xy, thickness=GROUND_SLAB_THICKNESS):
    """Extrude a closed 2D polygon downward into a solid slab mesh.

    The top face lies at z=0 (the real floor height) and the bottom face at
    ``-thickness``, so the slab only ever occupies space BELOW the floor.

    Args:
        points_xy (Sequence): Polygon corners as ``(x, y)`` pairs in metres,
            in order and without repeating the first point.
        thickness (float): Slab depth in metres.

    Returns:
        Mesh: Closed mesh (top face, bottom face, and one quad per side), or
        None if fewer than 3 corners were given.
    """
    pts = [(float(x), float(y)) for x, y in points_xy]
    n = len(pts)
    if n < 3:
        return None
    # Vertices 0..n-1 are the top ring (z=0), n..2n-1 the bottom ring.
    vertices = [[x, y, 0.0] for x, y in pts]
    vertices += [[x, y, -float(thickness)] for x, y in pts]
    # Top face as given; bottom face reversed so both wind outward.
    faces = [list(range(n)), list(range(2 * n - 1, n - 1, -1))]
    # Side quads stitching the two rings.
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, j + n, i + n])
    return Mesh.from_vertices_and_faces(vertices, faces)


def _walkable_ground_slabs(problem_name):
    """Slab meshes (metres) for every patch in a problem's WalkableGround.json.

    Args:
        problem_name (str): Design-study problem folder name.

    Returns:
        list: One slab Mesh per ground face, empty when the file is missing or
        carries no usable polygon.
    """
    path = os.path.join(DESIGN_DATA_DIRECTORY, problem_name, 'WalkableGround.json')
    if not os.path.exists(path):
        return []
    try:
        with open(path, 'r') as f:
            raw = json.load(f)
    except Exception as exc:
        print(f"[ground] WARN: could not read {path}: {exc}")
        return []
    data = raw.get('data', raw)
    slabs = []
    for ground in (data.get('grounds') or {}).values():
        md = ground.get('data', ground)
        vertex = md.get('vertex') or {}
        if not vertex:
            continue
        # Rhino exports these in millimetres; scale only when they look like it.
        coords = np.array(
            [[v.get('x', 0.0), v.get('y', 0.0)] for v in vertex.values()], dtype=float)
        scale = 0.001 if np.abs(coords).max() > _GROUND_MM_THRESHOLD else 1.0
        for face_vertices in (md.get('face') or {}).values():
            ring = [(vertex[str(i)].get('x', 0.0) * scale,
                     vertex[str(i)].get('y', 0.0) * scale) for i in face_vertices]
            slab = _slab_mesh_from_polygon(ring)
            if slab is not None:
                slabs.append(slab)
    return slabs


def build_ground_rigid_body(problem_name):
    """Floor collision geometry for a design-study problem.

    Prefers the problem's exported ``WalkableGround.json`` patches. When that
    file is absent the robot would otherwise be free to drive its arms and tools
    straight through the real-world floor, so a large fallback slab is
    synthesized instead (with a warning).

    Every patch becomes its own mesh inside ONE RigidBody: compas_fab's
    ``_add_rigid_body`` turns each mesh into a separate PyBullet body (so
    disjoint patches are not merged into a single convex hull), while the cell
    still sees a single rigid-body name -- which means only one RigidBodyState
    has to be injected per movement.

    Args:
        problem_name (str): Design-study problem folder name.

    Returns:
        RigidBody: The floor, already in metres (``native_scale=1.0``).
    """
    slabs = _walkable_ground_slabs(problem_name)
    if slabs:
        print(f"[ground] {len(slabs)} walkable-ground patch(es) loaded as "
              f"{GROUND_RIGID_BODY_NAME!r} collision geometry.")
    else:
        half = GROUND_FALLBACK_HALF_SIZE
        print(f"[ground] WARN: no usable WalkableGround.json for problem "
              f"{problem_name!r}; falling back to a {2 * half:.0f} x {2 * half:.0f} m "
              f"ground slab so the arms/tools cannot reach the real floor.")
        slabs = [_slab_mesh_from_polygon(
            [(-half, -half), (half, -half), (half, half), (-half, half)])]
    return RigidBody(visual_meshes=slabs, collision_meshes=slabs, native_scale=1.0)


class CfabSession:
    """Per-problem cfab planner session.

    Materializes the entire RobotCell (robot URDF, tool URDFs, rigid body
    meshes) into the client's PyBullet world in one go via
    `planner.set_robot_cell`. Per-movement state is pushed in via
    `planner.set_robot_cell_state(state)`.
    """

    def __init__(self, problem_name: str, *,
                 connection_type: str = "direct",
                 enable_debug_gui: bool = False,
                 existing_client_id: int | None = None,
                 robot_cell: RobotCell | None = None):
        """Open a cfab planner session.

        Args:
            problem_name: Design-study problem folder holding RobotCell.json.
                Ignored (may be None) when ``robot_cell`` is given directly.
            connection_type: PyBullet connection type ("direct" or "gui").
            enable_debug_gui: Show the PyBullet sidebar in the cfab GUI window.
            existing_client_id: Adopt the monitor's already-open PyBullet
                connection instead of opening a new one.
            robot_cell: Pre-built RobotCell (e.g. from
                ``build_default_robot_cell``); skips the RobotCell.json load.
        """
        self.problem_name = problem_name if robot_cell is None else None
        self._owns_client_connection = existing_client_id is None
        # ``enable_debug_gui`` toggles ``pybullet.COV_ENABLE_GUI``. Off by
        # default (matches compas_fab); set to True to get the sidebar +
        # debug-parameter sliders in the cfab GUI window.
        self.client = PyBulletClient(
            connection_type=connection_type, verbose=False,
            enable_debug_gui=enable_debug_gui,
        )
        if existing_client_id is None:
            self.client.__enter__()  # open the PyBullet connection
        else:
            # Adopt the monitor's already-open PyBullet GUI connection. This
            # lets BarAction loading materialize the RobotCell in the visible
            # live-monitor scene instead of attempting to open a second GUI.
            self.client.client_id = existing_client_id
            self.client._cache_dir = tempfile.TemporaryDirectory(prefix="compas_fab")
        try:
            self.planner = PyBulletPlanner(self.client)
            if robot_cell is None:
                robot_cell_path = os.path.join(
                    DESIGN_DATA_DIRECTORY, problem_name, "RobotCell.json"
                )
                robot_cell = json_load(robot_cell_path)
                # The Rhino-exported cell has no floor, so the planners would
                # happily route the arms through it. Add the walkable ground
                # (or a fallback slab) as a static obstacle. Only for the
                # design-study cell -- a caller-supplied robot_cell (the
                # startup default rig) carries no design geometry and is left
                # alone. Every state pushed to this planner must then carry a
                # matching RigidBodyState (compas_fab asserts the cell and the
                # state hold exactly the same rigid-body ids); the monitor's
                # `_inject_ground_rigid_body_state` does that, and also grants
                # the wheels-only allowed collision.
                robot_cell.rigid_body_models[GROUND_RIGID_BODY_NAME] = (
                    build_ground_rigid_body(problem_name))
            self.planner.set_robot_cell(robot_cell)
            self.robot_cell = robot_cell
        except Exception:
            # If anything fails after the client is open, make sure we don't
            # leak the PyBullet connection.
            self.close()
            self.client = None
            raise

    def close(self):
        if self.client is not None:
            if self._owns_client_connection:
                self.client.__exit__(None, None, None)
            else:
                for tool_id in list(self.client.tools_puids.keys()):
                    self.client._remove_tool(tool_id)
                for rigid_body_id in list(self.client.rigid_bodies_puids.keys()):
                    self.client._remove_rigid_body(rigid_body_id)
                self.client._remove_robot()
                self.client._cache_dir.cleanup()
            self.client = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False  # don't suppress exceptions


def _attach_tool_in_state(state: RobotCellState, tool_name: str, group: str,
                          touch_links: list) -> None:
    """Mark a tool as attached to a planning group in a cell state.

    Mirrors how the Rhino producer authors ToolStates in BarAction JSONs:
    attached to the group with an identity attachment frame, and the two
    wrist links allowed to touch the tool body.

    Args:
        state: The RobotCellState to modify in place.
        tool_name: Key into ``state.tool_states``.
        group: Planning group the tool hangs off (its tool0 link).
        touch_links: Robot link names allowed to contact the tool mesh.
    """
    ts = state.tool_states[tool_name]
    ts.attached_to_group = group
    ts.attachment_frame = Frame.worldXY()
    ts.touch_links = list(touch_links)


def _cone_tool_mesh(tip_xyz, radius: float = 0.015, segments: int = 12) -> Mesh:
    """Cone mesh for the punch tool: base ring at tool0 (z=0), apex at tip.

    Mirrors the pp-side mesh built in common.create_end_effector so the
    cfab collision geometry matches the visualization proxy.

    Args:
        tip_xyz: Punch tip position relative to tool0 (the cone apex).
        radius: Base ring radius in meters.
        segments: Number of ring segments.

    Returns:
        The cone as a compas Mesh.
    """
    import math
    vertices = [[float(tip_xyz[0]), float(tip_xyz[1]), float(tip_xyz[2])]]
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        vertices.append([radius * math.cos(a), radius * math.sin(a), 0.0])
    base_center = len(vertices)
    vertices.append([0.0, 0.0, 0.0])
    faces = []
    for i in range(segments):
        nxt = (i + 1) % segments
        faces.append([0, i + 1, nxt + 1])          # side
        faces.append([base_center, nxt + 1, i + 1])  # base cap
    return Mesh.from_vertices_and_faces(vertices, faces)


def build_default_robot_cell(ee_types: list, *, dual_arm: bool,
                             robot_name: str = "",
                             punch_tool_offsets=None) -> Tuple[RobotCell, RobotCellState]:
    """Build a RobotCell + default state without a design-study RobotCell.json.

    Gives the monitor a working cfab planner from startup, for any of the
    three rigs (Alice 0804 / Belle 0805 single-arm, Cindy 0806 dual-arm),
    so free/single-arm planning can run before any BarAction is loaded.

    Args:
        ee_types: End-effector types as configured in husky_world.init, e.g.
            ['assembly_tool_v3_left', 'assembly_tool_v3_right'] or
            ['robotiq_gripper'].
        dual_arm: True for the dual-arm rig (Cindy 0806).
        robot_name: Robot id ('0804' | '0805' | ...) used to pick the
            per-robot calibrated single-arm URDF/SRDF.
        punch_tool_offsets: Per-arm tool0 -> punch-tip offsets (single
            [x,y,z] or a list of them), used to size punch_tool cones.

    Returns:
        (cell, state): The RobotCell and a default RobotCellState with the
        known tools attached and the robot at the zero configuration.
    """
    if dual_arm:
        urdf_path, srdf_path = HUSKY_DUAL_URDF_PATH, HUSKY_DUAL_SRDF_PATH
    else:
        urdf_path = HUSKY_SINGLE_URDF_PATHS.get(robot_name, HUSKY_SINGLE_URDF_PATHS['0804'])
        srdf_path = HUSKY_SINGLE_SRDF_PATHS.get(robot_name, HUSKY_SINGLE_SRDF_PATHS['0804'])
    robot_model = RobotModel.from_urdf_file(urdf_path)
    # The URDFs reference link meshes via package:// URIs; those mesh
    # packages live side by side under data/husky_urdf/.
    mesh_root = os.path.join(DATA_DIRECTORY, 'husky_urdf')
    robot_model.load_geometry(*[
        LocalPackageMeshLoader(mesh_root, pkg)
        for pkg in ('husky_description', 'husky_ur_description', 'ur_description')
    ])
    semantics = RobotSemantics.from_srdf_file(srdf_path, robot_model)

    # Map each configured end effector to a (ToolModel, group, touch links)
    # triple. Every known ee_type gets a ToolModel so its geometry is
    # collision-checked on the cfab path; mesh sources mirror the pp-side
    # proxies in common.create_end_effector.
    tool_models = {}
    attachments = []  # (tool_name, group, touch_links)
    for i, ee_type in enumerate(ee_types):
        # Group + allowed-contact wrist links for this arm slot.
        if dual_arm:
            side = ('left', 'right')[min(i, 1)]
            group = f'base_{side}_arm_manipulator'
            touch_links = [f'{side}_ur_arm_wrist_2_link',
                           f'{side}_ur_arm_wrist_3_link']
        else:
            side = ''
            group = SINGLE_ARM_GROUP
            touch_links = ['ur_arm_wrist_2_link', 'ur_arm_wrist_3_link']

        if ee_type in ('assembly_tool_v3_left', 'assembly_tool_v3_right'):
            side = ee_type.rsplit('_', 1)[-1]                # 'left' | 'right'
            tool_name = 'AT3L' if side == 'left' else 'AT3R'
            tool_models[tool_name] = json_load(
                os.path.join(TOOL_MODEL_DIR, f'{tool_name}.json'))
            attachments.append((
                tool_name,
                f'base_{side}_arm_manipulator',
                [f'{side}_ur_arm_wrist_2_link', f'{side}_ur_arm_wrist_3_link'],
            ))
        elif ee_type == 'robotiq_gripper':
            # Static closed-gripper mesh (metres). TCP frame is unused by
            # collision checking, so identity is fine.
            name = f'robotiq_{side or i}'
            tool_models[name] = ToolModel(
                Mesh.from_obj(ROBOTIQ_MESH_PATH), Frame.worldXY(), name=name)
            attachments.append((name, group, touch_links))
        elif ee_type == 'punch_tool':
            # Calibration punch: cone with base at tool0 and apex at the
            # calibrated punch-tip offset (same mesh as the pp proxy).
            offsets = punch_tool_offsets
            if offsets is not None and not isinstance(offsets, (list, tuple)):
                offsets = [offsets]
            tip = (offsets[min(i, len(offsets) - 1)]
                   if offsets else [0.0, 0.0, 0.15])
            name = f'punch_{side or i}'
            tool_models[name] = ToolModel(
                _cone_tool_mesh(tip), Frame.worldXY(), name=name)
            attachments.append((name, group, touch_links))
        elif ee_type == 'custom_gripper':
            # Thin plate proxy, matching create_end_effector's pp fallback
            # (a 0.12 x 0.12 x 0.01 box centered at tool0).
            from compas.geometry import Box
            name = f'custom_gripper_{side or i}'
            tool_models[name] = ToolModel(
                Mesh.from_shape(Box(0.12, 0.12, 0.01)), Frame.worldXY(), name=name)
            attachments.append((name, group, touch_links))
        else:
            print(f"[cfab] ee_type {ee_type!r} has no cfab ToolModel; its "
                  f"geometry will not be collision-checked on the cfab path.")

    cell = RobotCell(robot_model, semantics, tool_models, {})
    state = cell.default_cell_state()
    for tool_name, group, touch_links in attachments:
        _attach_tool_in_state(state, tool_name, group, touch_links)
    state.robot_configuration = cell.zero_full_configuration()
    return cell, state


def arm_joint_names_for_group(robot_cell: RobotCell, group: str) -> list:
    """Return the group's UR arm joint names in canonical order.

    Filters the group's configurable joints down to the 6 UR arm joints,
    same convention as husky_assembly_tamp's ``_arm_joint_names``.
    """
    from husky_assembly_tamp.motion_planner.api import _ARM_SUFFIXES
    return [n for n in robot_cell.get_configurable_joint_names(group)
            if any(n.endswith(s) for s in _ARM_SUFFIXES)]


def plan_free_motion(planner, start_state: RobotCellState, goal_conf, *,
                     group: str, max_time: float = 10.0, max_iterations: int = 20,
                     joint_resolution: float = 0.05, smooth_iterations: int = 20,
                     debug: bool = False) -> Tuple[Optional[list], dict]:
    """Joint-space BiRRT for ONE planning group, with cfab collision checks.

    Group-generic version of husky_assembly_tamp's ``plan_free_dual_arm``
    (which is hard-wired to the two dual-arm groups). Used for single-arm
    free motion on Alice/Belle.

    Args:
        planner: compas_fab PyBulletPlanner with the robot cell loaded.
        start_state: Start RobotCellState; arm start values are read from
            its ``robot_configuration``.
        goal_conf: Goal as a compas Configuration or a sequence matching the
            group's arm joint count (6 for single-arm).
        group: Planning group name, e.g. ``SINGLE_ARM_GROUP``.
        max_time: BiRRT time budget in seconds.
        max_iterations: BiRRT restart budget.
        joint_resolution: Extend-step resolution in radians.
        smooth_iterations: Post-plan shortcut smoothing iterations.
        debug: Verbose diagnosis of start/goal collision rejections.

    Returns:
        (path, info): path is a list of per-waypoint joint-value arrays, or
        None on failure with info['failure_reason'] set.
    """
    from husky_assembly_tamp.motion_planner.api import _build_cfab_collision_fn

    joint_names = arm_joint_names_for_group(planner.client.robot_cell, group)
    start_conf = np.asarray(
        [float(start_state.robot_configuration[n]) for n in joint_names])
    try:
        goal_arr = np.asarray([float(goal_conf[n]) for n in joint_names])
    except (TypeError, KeyError, IndexError):
        goal_arr = np.asarray(list(goal_conf), dtype=float)
        if goal_arr.shape != (len(joint_names),):
            raise ValueError(
                f"goal_conf must be a Configuration with the {len(joint_names)} "
                f"arm joints or a same-length sequence; got shape {goal_arr.shape}")

    robot_puid = planner.client.robot_puid
    arm_joints = pp.joints_from_names(robot_puid, joint_names)

    planner.set_robot_cell_state(start_state)
    collision_fn = _build_cfab_collision_fn(planner, start_state, joint_names)
    resolutions = np.ones(len(joint_names)) * float(joint_resolution)
    sample_fn = pp.get_sample_fn(robot_puid, arm_joints)
    distance_fn = pp.get_distance_fn(robot_puid, arm_joints)
    extend_fn = pp.get_extend_fn(robot_puid, arm_joints, resolutions=resolutions)

    info = {'group': group, 'max_time': float(max_time)}
    with pp.WorldSaver():
        pp.set_joint_positions(robot_puid, arm_joints, start_conf)
        if not pp.check_initial_end(start_conf, goal_arr, collision_fn, diagnosis=debug):
            info['failure_reason'] = 'start_or_goal_in_collision'
            return None, info
        raw_path = pp.solve_motion_plan(
            start_conf, goal_arr, distance_fn, sample_fn, extend_fn, collision_fn,
            algorithm='birrt', max_time=max_time, max_iterations=int(max_iterations),
            smooth=int(smooth_iterations), diagnosis=debug, coarse_waypoints=False,
        )
    if raw_path is None:
        info['failure_reason'] = 'birrt_failed'
        return None, info
    return [np.asarray(q, dtype=float) for q in raw_path], info
