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
from contextlib import contextmanager
from types import MethodType
from typing import Iterable, Iterator, Optional, Tuple

import numpy as np
import pybullet_planning as pp

from compas.data import json_load
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas_fab.backends import (
    BackendError, PyBulletClient, PyBulletPlanner,
)
from compas_fab.robots import (
    FrameWaypoints, JointTrajectory, RigidBody, RigidBodyState, RobotCell, RobotCellState,
    RobotSemantics, TargetMode,
)
from compas_robots import RobotModel, ToolModel
from compas_robots.resources import LocalPackageMeshLoader

# Importing rs_data_structure registers the compas dtypes of the BarAction
# movement classes so BarAction JSONs deserialize correctly.
import rs_data_structure  # noqa: F401

from husky_assembly_teleop import DATA_DIRECTORY, DESIGN_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import find_bar_body, is_ground_joint_body
from husky_assembly_teleop.robot_registry import ROBOTS, robot_by_name

# * Robot description files now come from the robot registry (one RobotSpec per
# * husky). The module-level names below are kept as aliases so every existing
# * `from cfab_session import HUSKY_DUAL_URDF_PATH` keeps working.
HUSKY_DUAL_URDF_PATH = robot_by_name('Cindy').urdf_path
HUSKY_DUAL_SRDF_PATH = robot_by_name('Cindy').srdf_path
# Per-robot calibrated single-arm files, keyed by serial (Alice=0804, Belle=0805).
HUSKY_SINGLE_URDF_PATHS = {spec.serial: spec.urdf_path
                           for spec in ROBOTS.values() if not spec.dual_arm}
HUSKY_SINGLE_SRDF_PATHS = {spec.serial: spec.srdf_path
                           for spec in ROBOTS.values() if not spec.dual_arm}
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
# bar_action_io.BUILT_ASSEMBLY_RB_PREFIXES, so the mocap-accuracy hide never
# blanks the ground: it must stay collision-checked in every BarAction.
GROUND_RIGID_BODY_NAME = 'obstacle_ground'
# Robot links allowed to touch the ground (GROUND_RIGID_BODY_NAME). The floor is
# modelled honestly at z=0, and the husky's wheels rest exactly on it: the URDF
# puts base_footprint->base_link at 0.13228 m and base_link->wheel at 0.03282 m,
# so each wheel centre is at 0.1651 m -- exactly the wheel radius. The wheels are
# therefore permanently tangent to the floor, while the chassis clears it by
# 132 mm. Without this allowed-collision every configuration would read as
# colliding. Only the four wheels are exempt, so an arm or tool dipping below
# the floor is still caught. (Same link names on the single- and dual-arm URDFs.)
GROUND_TOUCH_LINKS = (
    'front_left_wheel_link', 'front_right_wheel_link',
    'rear_left_wheel_link', 'rear_right_wheel_link',
)
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


def _slab_mesh_from_polygon(points_xy, thickness=GROUND_SLAB_THICKNESS, top_z=0.0):
    """Extrude a closed 2D polygon downward into a solid slab mesh.

    The top face lies at ``top_z`` (the floor height) and the bottom face
    ``thickness`` below it, so the slab only ever occupies space BELOW the floor.

    Args:
        points_xy (Sequence): Polygon corners as ``(x, y)`` pairs in metres,
            in order and without repeating the first point.
        thickness (float): Slab depth in metres.
        top_z (float): Height of the floor surface in metres. The design
            exports put their ground a little below zero (260921: -15.55 mm);
            a floor left at z=0 would then report the feet of a ground bar,
            authored to stand on the real ground, as buried in the floor.

    Returns:
        Mesh: Closed mesh (top face, bottom face, and one quad per side), or
        None if fewer than 3 corners were given.
    """
    pts = [(float(x), float(y)) for x, y in points_xy]
    n = len(pts)
    if n < 3:
        return None
    # Vertices 0..n-1 are the top ring (z=top_z), n..2n-1 the bottom ring.
    vertices = [[x, y, float(top_z)] for x, y in pts]
    vertices += [[x, y, float(top_z) - float(thickness)] for x, y in pts]
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
            # The patch is flat: its mean height is the floor surface.
            top_z = float(np.mean([vertex[str(i)].get('z', 0.0) * scale
                                   for i in face_vertices]))
            slab = _slab_mesh_from_polygon(ring, top_z=top_z)
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


def ground_touch_bodies(rb_names: Iterable[str], obstacle_tools: Iterable[str],
                        existing: Optional[Iterable[str]] = None) -> list[str]:
    """Bodies and tools the floor may touch in one cell state.

    * A ground joint (``joint_*_ground``, see ``is_ground_joint_body``) stands on
    * the floor by construction, whether it still rides with the bar or is
    * already built, so the floor allows it. compas_fab's attached-body check is
    * symmetric (listing either body in the other's ``touch_bodies`` skips the
    * pair), so the floor's list is enough.

    Args:
        rb_names (Iterable[str]): Rigid-body names of the state
            (e.g. ``state.rigid_body_states``).
        obstacle_tools (Iterable[str]): ``ObstacleRobot<Name>`` tools present in
            the cell.
        existing (Iterable[str] | None): Allowances the ground entry already
            carries; they are kept.

    Returns:
        list[str]: Sorted names, each listed once.
    """
    ground_joints = {name for name in rb_names if is_ground_joint_body(name)}
    return sorted(set(existing or []) | set(obstacle_tools) | ground_joints)


def inject_ground_rigid_body_state(cell: RobotCell, state: RobotCellState) -> None:
    """Give a cell state the ground body, with the wheels-only allowance.

    ``CfabSession`` adds the floor (``GROUND_RIGID_BODY_NAME``) to the design
    cell, and compas_fab asserts that a cell and any state pushed to it hold
    exactly the same rigid-body ids -- so every freshly parsed movement state
    needs a matching entry or ``set_robot_cell_state`` raises. The floor is
    stationary at the world origin, and lists the four wheel links in
    ``touch_links`` so resting on it is not reported as a collision (see
    GROUND_TOUCH_LINKS); anything else that reaches the floor still is.

    ! The OTHER robots (the ``ObstacleRobot<Name>`` tools) stand on the same
    ! floor, so their wheels touch it too. Both are static, so that contact would
    ! veto every plan while no arm motion could change it; the ground lists them
    ! in ``touch_bodies`` (compas_fab then skips the tool <-> floor check).
    ! The state's ground joints are listed there too (see ``ground_touch_bodies``).

    Args:
        cell (RobotCell): The session's cell (``CfabSession.robot_cell``).
        state (RobotCellState): State to edit in place. Left unchanged when the
            cell has no ground body. When the state already carries a ground
            entry (e.g. a sidecar written by an older monitor), the wheel links,
            obstacle robots and ground joints are ADDED to its allowances instead.
    """
    if state is None or cell is None:
        return
    if GROUND_RIGID_BODY_NAME not in (cell.rigid_body_models or {}):
        return
    rb_states = getattr(state, 'rigid_body_states', None)
    if rb_states is None:
        return
    obstacle_tools = [spec.obstacle_tool_name for spec in ROBOTS.values()
                      if spec.obstacle_tool_name in (cell.tool_models or {})]
    ground = rb_states.get(GROUND_RIGID_BODY_NAME)
    if ground is not None:
        # An older file may carry a ground without the obstacle-robot allowance;
        # without it a robot standing on the floor vetoes every plan.
        ground.touch_links = sorted(set(ground.touch_links or []) | set(GROUND_TOUCH_LINKS))
        ground.touch_bodies = ground_touch_bodies(rb_states, obstacle_tools, ground.touch_bodies)
        return
    rb_states[GROUND_RIGID_BODY_NAME] = RigidBodyState(
        frame=Frame.worldXY(),
        touch_links=list(GROUND_TOUCH_LINKS),
        touch_bodies=ground_touch_bodies(rb_states, obstacle_tools),
    )


class CfabSession:
    """Per-problem cfab planner session.

    Materializes the entire RobotCell (robot URDF, tool URDFs, rigid body
    meshes) into the client's PyBullet world in one go via
    `planner.set_robot_cell`. Per-movement state is pushed in via
    `planner.set_robot_cell_state(state)`.
    """

    def __init__(self, problem_name: str, *,
                 cell_filename: str = "RobotCell.json",
                 connection_type: str = "direct",
                 enable_debug_gui: bool = False,
                 existing_client_id: int | None = None,
                 robot_cell: RobotCell | None = None):
        """Open a cfab planner session.

        Args:
            problem_name (str): Design-study problem folder holding the cell file.
                Ignored (may be None) when ``robot_cell`` is given directly.
            cell_filename (str): Which robot's cell to load from the problem
                folder: ``'RobotCell.json'`` (Cindy) or ``'RobotCell_<Name>.json'``
                (a support robot, see ``RobotSpec.cell_file``).
            connection_type (str): PyBullet connection type ("direct" or "gui").
            enable_debug_gui (bool): Show the PyBullet sidebar in the cfab GUI window.
            existing_client_id (int | None): Adopt the monitor's already-open
                PyBullet connection instead of opening a new one.
            robot_cell (RobotCell | None): Pre-built RobotCell (e.g. from
                ``build_default_robot_cell``); skips the cell file load.
        """
        self.problem_name = problem_name if robot_cell is None else None
        # None for a caller-supplied cell, so a later design-problem load always
        # replaces it (the monitor compares problem_name AND cell_filename).
        self.cell_filename = cell_filename if robot_cell is None else None
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
                    DESIGN_DATA_DIRECTORY, problem_name, cell_filename
                )
                robot_cell = json_load(robot_cell_path)
                # The Rhino-exported cell has no floor, so the planners would
                # happily route the arms through it. Add the walkable ground
                # (or a fallback slab) as a static obstacle. Only for the
                # design-study cell -- a caller-supplied robot_cell (the
                # startup default rig) carries no design geometry and is left
                # alone. Every state pushed to this planner must then carry a
                # matching RigidBodyState (compas_fab asserts the cell and the
                # state hold exactly the same rigid-body ids);
                # `inject_ground_rigid_body_state` below does that, and also
                # grants the wheels-only allowed collision.
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


# ============================================================================
# * compas_fab forward-kinematics workaround (used by plan_linear_motion only)
# ============================================================================
# ! The pinned compas_fab (external/compas_fab @ 995122d) has a bug in
# ! PyBulletForwardKinematics.forward_kinematics
# ! (backends/pybullet/backend_features/pybullet_forward_kinematics.py:99-102):
# ! set_robot_cell_state already places the robot base at robot_base_frame in
# ! PyBullet, so the link pose PyBullet reports is in WORLD coordinates -- yet the
# ! function multiplies it by robot_base_frame a second time. With the robot
# ! anywhere but the world origin (always, with mocap bases) the "start pose" that
# ! plan_cartesian_motion interpolates from is metres off and the plan fails.
# ? TODO: drop this workaround once compas_fab is fixed upstream and the pin bumped.

def _forward_kinematics_world(planner, robot_cell_state: RobotCellState, target_mode,
                              group: Optional[str] = None,
                              native_scale: Optional[float] = None,
                              options: Optional[dict] = None) -> Frame:
    """compas_fab's forward_kinematics without the second robot-base transform.

    Same steps and signature as
    ``PyBulletForwardKinematics.forward_kinematics``, minus the final
    multiplication by ``robot_base_frame`` (the PyBullet link pose is already in
    the world frame).

    Args:
        planner: The PyBulletPlanner this is bound to.
        robot_cell_state (RobotCellState): State to evaluate.
        target_mode (TargetMode): Which frame to return (ROBOT = the group's flange).
        group (str, optional): Planning group; defaults to the cell's main group.
        native_scale (float, optional): Unit scale, as in compas_fab.
        options (dict, optional): Unused, kept for the same signature.

    Returns:
        Frame: The requested frame in world coordinates.
    """
    client = planner.client
    robot_cell = client.robot_cell
    group = group or robot_cell.main_group_name
    robot_cell_state.assert_target_mode_match(target_mode, group)
    planner.set_robot_cell_state(robot_cell_state)
    link_name = robot_cell.get_end_effector_link_name(group)
    pcf_frame = client._get_link_frame(client.robot_link_puids[link_name], client.robot_puid)
    target_frame = robot_cell.pcf_to_target_frames(robot_cell_state, pcf_frame, target_mode, group)
    if native_scale:
        target_frame.scale(1 / native_scale)
    return target_frame


@contextmanager
def _world_frame_forward_kinematics(planner) -> Iterator[None]:
    """Use the corrected forward kinematics on ONE planner while the block runs.

    The fix is set on the planner instance only (an instance attribute hides the
    class method), and removed again on exit, so no other planner or later call
    in the process sees a changed compas_fab.

    Args:
        planner: The PyBulletPlanner to patch for the duration of the block.

    Yields:
        None.
    """
    had_own = 'forward_kinematics' in vars(planner)
    previous = vars(planner).get('forward_kinematics')
    planner.forward_kinematics = MethodType(_forward_kinematics_world, planner)
    try:
        yield
    finally:
        if had_own:
            planner.forward_kinematics = previous
        else:
            del planner.forward_kinematics


def plan_linear_motion(planner, start_state: RobotCellState, target_frame: Frame, *,
                       group: str, max_step_distance: float = 0.005,
                       max_step_angle: float = 0.05, max_jump_revolute: float = 0.35,
                       check_collision: bool = True,
                       verbose: bool = False) -> Optional[JointTrajectory]:
    """Straight-line tool0 motion for ONE planning group, with cfab collision checks.

    Used for a support robot's linear approach / retreat (SingleArmLinearMovement).
    The flange (tool0 of ``group``) moves on a straight line from where the
    start configuration puts it to ``target_frame``; each interpolated pose is
    solved by IK seeded from the previous point.

    Args:
        planner: compas_fab PyBulletPlanner with the robot cell loaded.
        start_state (RobotCellState): Start state; its ``robot_configuration``
            (full configuration) is where the motion starts.
        target_frame (Frame): World-frame goal of the group's flange (tool0).
        group (str): Planning group, e.g. ``'manipulator'``.
        max_step_distance (float): Max flange travel between two points, meters.
        max_step_angle (float): Max flange rotation between two points, radians.
        max_jump_revolute (float): Max joint change between two points, radians;
            a bigger jump is subdivided, and fails the plan when it cannot be.
        check_collision (bool): Collision-check the start and every point.
        verbose (bool): Print compas_fab's step-by-step planning log.

    Returns:
        JointTrajectory | None: The group's joints per point, ``points[0]``
        being the start configuration; None when planning failed (the reason
        is printed).
    """
    # ! Equal tolerances on purpose: compas_fab's plan_cartesian_motion_frame_waypoints
    # ! builds every interpolated FrameTarget with
    # ! tolerance_position=waypoints.tolerance_ORIENTATION (upstream bug), so a
    # ! different position tolerance here would be silently ignored.
    waypoints = FrameWaypoints([target_frame], target_mode=TargetMode.ROBOT,
                               tolerance_position=1e-3, tolerance_orientation=1e-3)
    options = {
        'max_step_distance': float(max_step_distance),
        'max_step_angle': float(max_step_angle),
        'max_jump_revolute': float(max_jump_revolute),
        'check_collision': bool(check_collision),
        'verbose': bool(verbose),
        # * Passed on to every step's IK. compas_fab's default of 20 descent
        # * iterations is too few for the 1 mm / 1 mrad tolerance: on B3__H's
        # * approach the IK gave up a quarter of the way along the line.
        'max_descend_iterations': 200,
    }
    try:
        # * The planner reads the start pose through forward kinematics; use the
        # * corrected one (see _world_frame_forward_kinematics above).
        with _world_frame_forward_kinematics(planner):
            return planner.plan_cartesian_motion(waypoints, start_state, group, options=options)
    except (BackendError, ValueError) as e:
        # BackendError covers every compas_fab planning / kinematics / collision
        # error (MotionPlanningError, PlanningGroupNotSupported, KinematicsError);
        # ValueError covers a start state without the group's joints.
        # e.message, not str(e): compas_fab rewrites the message of a joint-jump
        # error after creating it, and only .message carries the rewrite.
        print(f"[linear plan] group {group!r} FAILED ({type(e).__name__}): "
              f"{getattr(e, 'message', None) or e}")
        return None


def apply_obstacle_robot_beliefs(state: RobotCellState, robot_cell: RobotCell,
                                 active_robot: str, beliefs: dict,
                                 holding_bars: Optional[dict] = None) -> None:
    """Pose the OTHER robots' obstacle tools in a cell state from what we believe.

    Each robot appears in the other robots' cells as a frozen tool
    ``ObstacleRobot<Name>``: its ``frame`` is the robot base in the world and its
    ``configuration`` the arm joints. The exporter writes one guess per state;
    the monitor knows better (live mocap, or the end state of the entry that
    robot last finished), so this overwrites it. Mirrors the Rhino side's
    ``configure_robot_obstacle`` and ``whitelist_frozen_contact``
    (``bar_joint_rhino_design_workflow/scripts/core/robot_obstacles.py``).

    ! The configuration is merged BY JOINT NAME into the tool's zero
    ! configuration, never by position: Cindy's URDF declares her right arm
    ! first, while the beliefs list the left arm first.

    Args:
        state (RobotCellState): The state to edit in place.
        robot_cell (RobotCell): The cell the state belongs to (for the obstacle
            tools' zero configurations).
        active_robot (str): The connected robot's short name; its own obstacle
            tool (never in its own cell) and its own hold are skipped.
        beliefs (dict): ``{obstacle_tool_name: (Frame, Configuration | None)}``,
            e.g. from ``progress_io.obstacle_tool_states``. A None configuration
            keeps the state's joints and only moves the base. Tools the state
            does not carry are ignored.
        holding_bars (dict | None): ``{robot_name: bar_id}`` for support robots
            clamped onto a bar right now. That robot's obstacle tool is allowed to
            touch the bar (added to the bar's ``touch_bodies``) -- the contact is
            real by construction, and both are static, so it would otherwise
            veto every plan.
    """
    own_tool = robot_by_name(active_robot).obstacle_tool_name
    tool_states = state.tool_states or {}
    for tool_name, (frame, configuration) in beliefs.items():
        if tool_name == own_tool or tool_name not in tool_states:
            continue
        tool_state = tool_states[tool_name]
        tool_state.frame = frame.copy()
        if configuration is None:
            continue
        merged = robot_cell.tool_models[tool_name].zero_configuration()
        values = configuration.joint_dict
        matched = 0
        for name in merged.joint_names:
            if name in values:
                merged[name] = float(values[name])
                matched += 1
        if matched < len(values):
            # ! Some believed joints have no counterpart in the tool: those stay at
            # ! zero, so the obstacle robot is drawn and collision-checked wrongly.
            print(f"[obstacle robots] WARNING: only {matched}/{len(values)} believed joints "
                  f"of {tool_name!r} match the tool's joint names; the rest stay at zero.")
        tool_state.configuration = merged

    rb_states = state.rigid_body_states or {}
    for robot, bar_id in (holding_bars or {}).items():
        if robot == active_robot:
            continue
        tool_name = robot_by_name(robot).obstacle_tool_name
        bar_name = find_bar_body(rb_states, bar_id)
        if bar_name is None:
            # ! Say so: without the entry the frozen contact stays forbidden.
            print(f"[obstacle robots] NOTE: {robot} holds bar {bar_id}, but this state "
                  f"has no rigid body for it to whitelist {tool_name!r} on.")
            continue
        rb = rb_states[bar_name]
        existing = list(rb.touch_bodies or [])
        if tool_name not in existing:
            rb.touch_bodies = sorted(set(existing) | {tool_name})
