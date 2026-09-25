"""Human-in-the-loop start pose for the bar-held transfer (M1).

The automatic start derivation walks the bar back from the goal to a thousand
home poses and can spend its whole budget (see ``doc/m1_planner_changelog.md``).
For the mocap bar-reaching session the operator picks the start bar pose
instead: a carry anchor (``HOME_BAR_ANCHORS``: horizontal / vertical / back),
then small adjustments -- slide along the bar, roll about the bar, and two
shifts perpendicular to it -- and confirms. This module turns that choice into
a start configuration: the dual-arm IK that holds the bar there with the same
grasps as at the goal, on the joint branch nearest the goal, checked against
the full cell collision model. No sweep, no RRT; the caller plans the M1 path
from the returned configuration.

* The bar pose is built in the mobile-base frame, so the same slider values
* mean the same thing wherever the base is parked.
"""
import numpy as np
import pybullet_planning as pp
from husky_assembly_tamp.motion_planner.api import (
    _build_cfab_collision_fn, _conf12_from_target, _fk_link_pose_pp, _pp_pose_from_frame,
)

IDENTITY_POSE = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
# Mobile-base axes, in the order the two perpendicular shifts pick them.
BASE_AXES = (('x', np.array([1.0, 0.0, 0.0])),
             ('y', np.array([0.0, 1.0, 0.0])),
             ('z', np.array([0.0, 0.0, 1.0])))


def perpendicular_base_axes(bar_axis_mb) -> list:
    """The two mobile-base axes that are not the bar's axis, x before y before z.

    For the horizontal carry (bar along base y) that is forward (x) and up (z);
    for the vertical carry (bar along z) forward (x) and left (y); for the back
    carry (bar along x) left (y) and up (z).

    Args:
        bar_axis_mb: the anchor's bar axis in the mobile-base frame.

    Returns:
        list[tuple[str, np.ndarray]]: two ``(label, unit_vector)`` pairs.
    """
    axis = np.asarray(bar_axis_mb, dtype=float)
    return [(label, vec) for label, vec in BASE_AXES if abs(float(np.dot(vec, axis))) < 0.5]


def manual_m1_start(planner, state, active_bar_id: str, goal_conf, joint_names_12,
                    robot_puid: int, arm_joints, tool_link_left: int, tool_link_right: int, *,
                    anchor: str = 'horizontal', slide_m: float = 0.0, roll_deg: float = 0.0,
                    perp1_m: float = 0.0, perp2_m: float = 0.0, max_attempts: int = 20) -> dict:
    """Solve the collision-checked start configuration for an operator-chosen bar pose.

    Args:
        planner: the compas_fab ``PyBulletPlanner`` holding the design cell.
        state (RobotCellState): M1's start state (live base already applied,
            a full configuration present, the bar attached to a tool0 link).
        active_bar_id (str): the held bar's rigid-body name, e.g. ``'bar_B3'``.
        goal_conf: M1's goal configuration (M2's authored start), a compas
            ``Configuration`` or a 12-sequence; defines the grasps and seeds
            the IK so the start sits on the branch nearest the goal.
        joint_names_12: the twelve arm joint names, left arm first.
        robot_puid (int): PyBullet id of the cfab robot.
        arm_joints: PyBullet joint indices of the twelve arm joints.
        tool_link_left (int): PyBullet link index of the left tool0.
        tool_link_right (int): PyBullet link index of the right tool0.
        anchor (str): a ``HOME_BAR_ANCHORS`` label, or ``'all'`` to try them
            in order and keep the first that gives a collision-free start.
        slide_m (float): shift along the bar's axis, metres (+ toward the
            left gripper's end).
        roll_deg (float): turn about the bar's own axis, degrees.
        perp1_m (float): shift along the first perpendicular base axis, metres
            (see ``perpendicular_base_axes``).
        perp2_m (float): shift along the second perpendicular base axis, metres.
        max_attempts (int): IK re-seeds (gradient backend only; ssik enumerates
            its branches instead).

    Returns:
        dict: ``start_conf`` (12-vector, or None), ``goal_conf`` (12-vector),
        ``world_from_bar_start`` / ``world_from_bar_goal`` (pybullet poses),
        ``grasp_bar_from_left`` / ``grasp_bar_from_right``, ``bar_start_mb``
        (the chosen bar pose in the mobile-base frame), ``bar_mid_mb`` (the
        point midway between the two grippers, in the mobile-base frame -- the
        bar frame's own origin sits at one tip on some exports), ``anchor``
        (the one used), ``perp_axes`` (labels of the two perpendicular base axes),
        ``reason`` (None on success, else ``'bar_not_attached'`` / ``'no_ik'``
        / ``'collision'``), ``collision_conf`` (the colliding configuration on
        the goal's branch, for drawing) and ``reseeded`` (True when that
        branch collided and another one was taken).
    """
    # Kept local like the monitor's own core imports: core pulls in the whole
    # RRT stack, which the monitor otherwise never imports at module level.
    from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import (
        HOME_BAR_ANCHORS, home_bar_anchor_pose_mb, resolve_home_anchors,
        solve_endpoint_dual_arm_ik,
    )
    result = {
        'start_conf': None, 'goal_conf': None,
        'world_from_bar_start': None, 'world_from_bar_goal': None,
        'grasp_bar_from_left': None, 'grasp_bar_from_right': None,
        'bar_start_mb': None, 'bar_mid_mb': None, 'anchor': None, 'perp_axes': None,
        'reason': None, 'collision_conf': None, 'reseeded': False,
    }
    # FK probes read the PyBullet robot where it stands: put it at the state first.
    planner.set_robot_cell_state(state)

    # * Goal geometry, the same way _derive_constrained_start_for_plan reads it
    # * (api.py): the bar hangs off one tool0 link with an authored attachment
    # * frame; FK at the goal gives the bar pose and both grasps.
    bar_state = (state.rigid_body_states or {}).get(active_bar_id)
    if (bar_state is None or not getattr(bar_state, 'attached_to_link', None)
            or getattr(bar_state, 'attachment_frame', None) is None):
        result['reason'] = 'bar_not_attached'
        return result
    goal_arr = np.asarray(_conf12_from_target(goal_conf, joint_names_12), dtype=float)
    tool0_from_bar = _pp_pose_from_frame(bar_state.attachment_frame)
    attach_link = pp.link_from_name(robot_puid, bar_state.attached_to_link)
    world_from_bar_goal = pp.multiply(
        _fk_link_pose_pp(planner, goal_arr, robot_puid, arm_joints, attach_link), tool0_from_bar)
    grasp_l = pp.multiply(pp.invert(world_from_bar_goal),
                          _fk_link_pose_pp(planner, goal_arr, robot_puid, arm_joints, tool_link_left))
    grasp_r = pp.multiply(pp.invert(world_from_bar_goal),
                          _fk_link_pose_pp(planner, goal_arr, robot_puid, arm_joints, tool_link_right))
    base_frame = getattr(state, 'robot_base_frame', None)
    world_from_mb = _pp_pose_from_frame(base_frame) if base_frame is not None else IDENTITY_POSE
    mb_from_bar_goal = pp.multiply(pp.invert(world_from_mb), world_from_bar_goal)
    # ! The bar's axis for the sliders is the right->left grasp direction in
    # ! the bar frame (what the anchors align), not the bar's local Z: some
    # ! exported bar frames point Z the other way, and "+slide" must always
    # ! mean "toward the left gripper's end".
    axis_local = np.asarray(grasp_l[0], dtype=float) - np.asarray(grasp_r[0], dtype=float)
    axis_local /= max(1e-9, float(np.linalg.norm(axis_local)))
    grasp_mid_local = 0.5 * (np.asarray(grasp_l[0], dtype=float) + np.asarray(grasp_r[0], dtype=float))
    result.update(goal_conf=goal_arr, world_from_bar_goal=world_from_bar_goal,
                  grasp_bar_from_left=grasp_l, grasp_bar_from_right=grasp_r)

    anchors = resolve_home_anchors(None) if anchor in (None, 'all') else [anchor]
    collides = _build_cfab_collision_fn(planner, state, joint_names_12)
    rng = np.random.default_rng(0)
    try:
        for label in anchors:
            # Canonical pose for the anchor, then the operator's adjustments:
            # roll first (re-anchoring keeps the grasp midpoint on the anchor
            # point), then the shifts.
            pos, quat = home_bar_anchor_pose_mb(mb_from_bar_goal, grasp_l, grasp_r, anchor=label)
            if abs(roll_deg) > 1e-9:
                roll = pp.quat_from_axis_angle(tuple(axis_local.tolist()), float(np.deg2rad(roll_deg)))
                quat = pp.multiply(((0.0, 0.0, 0.0), tuple(quat)), ((0.0, 0.0, 0.0), tuple(roll)))[1]
                pos, quat = home_bar_anchor_pose_mb(mb_from_bar_goal, grasp_l, grasp_r,
                                                    bar_quat_override=tuple(quat), anchor=label)
            pos = np.asarray(pos, dtype=float)
            pos = pos + slide_m * (np.asarray(pp.matrix_from_quat(quat), dtype=float) @ axis_local)
            perps = perpendicular_base_axes(HOME_BAR_ANCHORS[label]['bar_axis_mb'])
            pos = pos + perp1_m * perps[0][1] + perp2_m * perps[1][1]
            bar_start_mb = (tuple(pos.tolist()), tuple(float(v) for v in quat))
            world_from_bar_start = pp.multiply(world_from_mb, bar_start_mb)
            bar_mid_mb = pos + np.asarray(pp.matrix_from_quat(quat), dtype=float) @ grasp_mid_local
            result.update(anchor=label, perp_axes=(perps[0][0], perps[1][0]),
                          bar_start_mb=bar_start_mb, bar_mid_mb=tuple(bar_mid_mb.tolist()),
                          world_from_bar_start=world_from_bar_start)

            # * IK twice with the same solver: first for reachability, then with
            # * the cell collision check; the difference tells the operator
            # * whether the pose is out of reach or merely blocked.
            common = dict(robot=robot_puid, arm_joints=arm_joints,
                          tool_link_left=tool_link_left, tool_link_right=tool_link_right,
                          bar_pose=world_from_bar_start,
                          grasp_bar_from_left=grasp_l, grasp_bar_from_right=grasp_r,
                          seed_conf=goal_arr, rng=rng, max_attempts=max_attempts)
            first = solve_endpoint_dual_arm_ik(collision_fn=None, **common)
            if first is None:
                result['reason'] = 'no_ik'
                continue
            final = solve_endpoint_dual_arm_ik(collision_fn=collides, **common)
            if final is None:
                result['reason'] = 'collision'
                result['collision_conf'] = np.asarray(first, dtype=float)
                continue
            result.update(start_conf=np.asarray(final, dtype=float), reason=None,
                          reseeded=not np.allclose(np.asarray(first), np.asarray(final), atol=1e-6))
            break
    finally:
        # The collision checks and the IK's FK gate moved the planning robot.
        planner.set_robot_cell_state(state)
    return result
