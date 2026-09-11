"""Collision-checked approach motion from the live arm pose to a trajectory start.

* The open-loop engine executes a precomputed trajectory that begins at an
* authored configuration, which is generally NOT where the arms are standing.
* Bridging that gap with a blind joint-space move is the one uncontrolled
* motion in the pipeline, so this module plans it properly: both arms in one
* 12-joint search (hence checked against each other), against robot
* self-collision, the attached Robotiq grippers, the floor and a placeholder
* work table.
*
* Everything happens in the caller's PyBullet world with the robot the engine
* already displays -- planning and visualization are the same scene, so what
* the operator sees is exactly what was checked.
*
* Nothing here commands the robot. The output is a time-parameterized
* OpenLoopTraj that the engine's normal tracker executes, so the approach
* inherits the same shared clock, tracking-error abort and logging as the
* trajectory it precedes.
"""

import numpy as np
import pybullet_planning as pp

from husky_assembly_teleop.common import HUSKY_DUAL_UR5e_JOINT_NAMES
from husky_assembly_teleop.open_loop_traj import traj_from_arrays
from husky_assembly_teleop.utils import get_custom_limits, plan_transit_motion

JOINT_NAMES_12 = HUSKY_DUAL_UR5e_JOINT_NAMES[0] + HUSKY_DUAL_UR5e_JOINT_NAMES[1]

# Planning resolution and the (finer) resolution the resulting path is
# re-checked at. Budgets are modest because the search blocks the UI.
PLAN_PASSES = ((0.01, 30.0, 'fine'), (0.02, 60.0, 'coarse'))
VALIDATION_STEP_RAD = 0.005

# Time-parameterization: the planner returns bare waypoints, so the engine has
# to invent the timing. Waypoints are first resampled this finely along the
# planned polyline, which keeps the tracked reference on the path that was
# actually collision-checked.
DENSIFY_STEP_RAD = 0.02
APPROACH_DT = 0.05               # reference sample spacing [s]

# --- Obstacles -------------------------------------------------------------
# ? The work table is a stand-in until the workspace is calibrated: better to
# ? plan around a guessed obstacle than around nothing. Measured from the
# ? calibrated URDF, both arm base links sit at z = 0.490 m, so "half the arm
# ? base height" puts the top at 0.245 m. The chassis front edge is at
# ? x = 0.495 m, hence a near edge just clear of it.
ARM_BASE_LINK_Z = 0.490          # m, left/right_ur_arm_base_link in base_footprint
TABLE_TOP_Z = 0.5 * ARM_BASE_LINK_Z
TABLE_NEAR_X = 0.55              # m, near edge, in front of the chassis
TABLE_DEPTH = 0.80               # m, extent along +x (away from the robot)
TABLE_WIDTH = 1.60               # m, extent along y
# The floor is modelled honestly at z=0 and extruded downward; a flat plane
# would have no volume to collide with. 6 x 6 m covers any arm's reach.
GROUND_SIZE = 6.0
GROUND_THICKNESS = 0.10

# ? PyBullet gives a body created with create_box no name, so a collision
# ? report would only be able to call it "/link0". build_obstacles writes a
# ? readable label here for every body it makes, and describe_collision reads
# ? it back. Rebuilding the obstacles replaces the entries for the new ids.
OBSTACLE_LABELS = {}


def build_obstacles(table_top_z: float = TABLE_TOP_Z,
                    with_table: bool = True, layout: dict = None) -> list:
    """Create the static collision bodies in the current PyBullet world.

    The bodies are both drawn and collision-checked, so the operator sees the
    exact volumes the planner treats as blocked.

    With a measured `layout` (from the pickup_calib tool) the guessed table is
    replaced by the surveyed one AND every part gets a box, so the approach
    cannot sweep the arms through the parts waiting to be picked. Without one,
    the placeholder table is the honest best guess it always was.

    ! Only links moved by the planned joints are tested against these, so the
    ! wheels resting on the floor need no allowed-collision exemption.

    Args:
        table_top_z (float): Height of the placeholder table top [m]. Ignored
            when `layout` is given, which carries the measured height.
        with_table (bool): Include the table (the floor is always added).
        layout (dict): Measured layout, or None for the placeholder table.

    Returns:
        list: PyBullet body ids to pass to the planner as obstacles.
    """
    ground = pp.create_box(GROUND_SIZE, GROUND_SIZE, GROUND_THICKNESS,
                           color=(0.75, 0.75, 0.78, 1.0))
    pp.set_pose(ground, pp.Pose(pp.Point(0.0, 0.0, -0.5 * GROUND_THICKNESS)))
    OBSTACLE_LABELS[ground] = 'ground'
    obstacles = [ground]
    if not with_table:
        return obstacles

    if layout is None:
        table = pp.create_box(TABLE_DEPTH, TABLE_WIDTH, table_top_z,
                              color=(0.55, 0.45, 0.35, 0.45))
        pp.set_pose(table, pp.Pose(pp.Point(TABLE_NEAR_X + 0.5 * TABLE_DEPTH,
                                            0.0, 0.5 * table_top_z)))
        OBSTACLE_LABELS[table] = 'table (placeholder)'
        obstacles.append(table)
        print(f'[approach] placeholder work table: '
              f'x {TABLE_NEAR_X:.2f}..{TABLE_NEAR_X + TABLE_DEPTH:.2f} m, '
              f'y +/-{TABLE_WIDTH / 2:.2f} m, top at z={table_top_z:.3f} m '
              f'(half the {ARM_BASE_LINK_Z:.3f} m arm base height)')
        return obstacles

    spec = layout['table']
    (cx, cy), (sx, sy) = spec['center_xy'], spec['size_xy']
    thickness, top_z = float(spec['thickness']), float(spec['top_z'])
    table = pp.create_box(sx, sy, thickness, color=(0.55, 0.45, 0.35, 0.45))
    pp.set_pose(table, pp.Pose(pp.Point(cx, cy, top_z - 0.5 * thickness)))
    OBSTACLE_LABELS[table] = 'table (measured)'
    obstacles.append(table)
    print(f'[approach] MEASURED work table: {sx:.2f} x {sy:.2f} m centred at '
          f'({cx:.3f}, {cy:.3f}), top at z={top_z:.3f} m')

    # One box per part, sized by its footprint and standing on the table. A box
    # covers the part's visual mesh, so the approach keeps clear of the real
    # thing even where the mesh is slimmer than its bounding box.
    for name, entry in sorted(layout['parts'].items()):
        hx, hy = entry['footprint_half_xy']
        height = float(entry['height'])
        body = pp.create_box(2 * hx, 2 * hy, height, color=(0.85, 0.55, 0.25, 0.45))
        pp.set_pose(body, pp.Pose(
            pp.Point(entry['xy'][0], entry['xy'][1], top_z + 0.5 * height),
            pp.Euler(yaw=float(entry['yaw']))))
        OBSTACLE_LABELS[body] = f'part {name}'
        obstacles.append(body)
    print(f"[approach] {len(layout['parts'])} measured part(s) added as obstacles")
    return obstacles


def unwrap_goal(goal_q12, start_q12, traj_q12=None, lower=None, upper=None,
                margin: float = 0.0) -> np.ndarray:
    """Shift each goal joint by whole turns to the branch nearest the start.

    A goal 2*pi away from the start describes the same arm pose but forces the
    sampler to traverse more than pi in that joint, which it almost never
    manages inside the time budget (documented at husky_world.py:2115).

    ! Unwrapping the goal commits the WHOLE trajectory that follows to that
    ! branch, because the arms end the approach there. A single goal point
    ! always fits -- but the trajectory around it need not: measured on the
    ! bench file, unwrapping the left shoulder pan moved its range from
    ! +0.04..+1.50 rad (4.78 rad of room) to -6.24..-4.78 rad, which is 42
    ! MILLIRADIANS from the -2*pi end stop. Tracking error alone would trip a
    ! protective stop there. So a joint is only unwrapped when the trajectory
    ! still fits on the new branch with `margin` to spare; otherwise it keeps
    ! the authored branch and the approach simply turns the long way round.

    Args:
        goal_q12: Target 12-vector [rad].
        start_q12: Start 12-vector [rad].
        traj_q12: (n, 12) joint samples that follow the goal, or None to skip
            the range check (a bare goal with nothing after it).
        lower: Per-joint lower limits [rad], or None to skip the check.
        upper: Per-joint upper limits [rad], or None to skip the check.
        margin (float): Room the trajectory must keep to each limit [rad].

    Returns:
        np.ndarray: The unwrapped goal.
    """
    goal = np.asarray(goal_q12, dtype=float)
    start = np.asarray(start_q12, dtype=float)
    turns = np.round((goal - start) / (2.0 * np.pi))
    if traj_q12 is not None and lower is not None and upper is not None:
        traj_q12 = np.asarray(traj_q12, dtype=float)
        for j in np.nonzero(turns)[0]:
            shift = turns[j] * 2.0 * np.pi
            if (traj_q12[:, j].min() - shift < lower[j] + margin
                    or traj_q12[:, j].max() - shift > upper[j] - margin):
                turns[j] = 0.0                 # keep the authored branch
    return goal - turns * 2.0 * np.pi


def build_collision_fn(robot, attachments, obstacles):
    """Collision predicate over the 12 arm joints: q12 -> True when colliding.

    Built with the same arguments plan_transit_motion uses internally, so the
    post-plan validation agrees with what the planner accepted. Self-collision
    is on with an empty disabled set, which is how the repo's other pp-native
    planners run: pp excludes adjacent/fixed link pairs itself, and since both
    arms are moving links of one body this is also what checks the arms
    against each other and against the husky.

    Args:
        robot (int): Robot body id.
        attachments (list): The two gripper Attachments (left, right).
        obstacles (list): Static body ids.

    Returns:
        callable: The collision function.
    """
    joints = pp.joints_from_names(robot, JOINT_NAMES_12)
    extra_disabled = [
        ((robot, pp.link_from_name(robot, f'{side}_ur_arm_wrist_3_link')),
         (attachments[i].child, pp.BASE_LINK))
        for i, side in enumerate(('left', 'right'))]
    return pp.get_collision_fn(
        robot, joints, obstacles=obstacles, attachments=attachments,
        self_collisions=1, disabled_collisions={},
        extra_disabled_collisions=extra_disabled,
        custom_limits=get_custom_limits(robot, {}), max_distance=0.0)


def _text(value) -> str:
    """PyBullet hands back names as bytes; print them as plain text.

    Args:
        value: A name from PyBullet, bytes or str.

    Returns:
        str: The decoded name.
    """
    return value.decode() if isinstance(value, bytes) else str(value)


def _pair_name(body: int, link: int) -> str:
    """Readable "body/link" label for one side of a collision pair.

    Args:
        body (int): PyBullet body id.
        link (int): Link index, or `pp.BASE_LINK` for a single-link body.

    Returns:
        str: e.g. ``husky/left_ur_arm_wrist_3_link``.
    """
    label = OBSTACLE_LABELS.get(body)
    if label is not None and pp.get_num_joints(body) == 0:
        return label                               # a box we made: no URDF names at all
    try:
        name = _text(pp.get_link_name(body, link))
    except Exception:                              # a body with no link names
        name = f'link{link}'
    return f'{label or _text(pp.get_body_name(body))}/{name}'


def _depth_mm(body1: int, link1: int, body2: int, link2: int) -> float:
    """How deep two links overlap, in millimetres (0.0 if it cannot be read).

    Args:
        body1 (int): First body id.
        link1 (int): First link index.
        body2 (int): Second body id.
        link2 (int): Second link index.

    Returns:
        float: Penetration depth [mm], positive when the links overlap.
    """
    try:
        pts = pp.pairwise_link_collision_info(body1, link1, body2, link2)
        # PyBullet's closest-point tuples carry the signed distance at index 8;
        # it is negative while the shapes overlap.
        return -1000.0 * min(float(pt[8]) for pt in pts)
    except Exception:
        return 0.0


def describe_collision(robot, attachments, obstacles, q12, log=print,
                       max_report: int = 12) -> list:
    """Say WHY `build_collision_fn` rejects a configuration, pair by pair.

    ! The collision function returns on the FIRST problem it meets and reports
    ! only True, which is no help when the scene looks clear on screen. This
    ! walks the same three families of pairs it checks -- and the joint limits
    ! it checks FIRST -- and reports every hit with its penetration depth.
    ! A limit violation is the usual answer to "but nothing is touching": the
    ! goal branch chosen by `unwrap_goal` can land outside a joint's range,
    ! and that is reported as "in collision" with nothing visibly wrong.

    The pair families mirror `build_collision_fn` exactly, so a configuration
    this reports as clean is one that function also accepts.

    Args:
        robot (int): Robot body id.
        attachments (list): The two gripper Attachments (left, right).
        obstacles (list): Static body ids.
        q12: The 12 arm joint values to explain [rad].
        log (callable): Message sink.
        max_report (int): Stop listing after this many pairs.

    Returns:
        list: One string per reason found, in the order they were checked.
    """
    joints = pp.joints_from_names(robot, JOINT_NAMES_12)
    reasons = []

    # * 1. joint limits -- checked before any geometry, so check them first here too
    lower, upper = pp.get_custom_limits(robot, joints,
                                        get_custom_limits(robot, {}))
    for i, value in enumerate(np.asarray(q12, dtype=float)):
        if value < lower[i] or value > upper[i]:
            reasons.append(
                f'JOINT LIMIT {_text(pp.get_joint_name(robot, joints[i]))} = '
                f'{value:.4f} rad outside [{lower[i]:.4f}, {upper[i]:.4f}]')
    if reasons:                                    # geometry is not even reached
        log(f'[approach] {len(reasons)} joint-limit violation(s) -- the '
            f'configuration is out of range, NOT touching anything:')
        for line in reasons:
            log(f'[approach]   {line}')
        return reasons

    extra_disabled = set()
    for i, side in enumerate(('left', 'right')):
        pair = ((robot, pp.link_from_name(robot, f'{side}_ur_arm_wrist_3_link')),
                (attachments[i].child, pp.BASE_LINK))
        extra_disabled.add(pair)
        extra_disabled.add(pair[::-1])
        # ? Both gripper URDFs are named "robotiq_85_gripper", so a report could
        # ? not say WHICH arm's finger it meant. Label them by the arm instead.
        OBSTACLE_LABELS.setdefault(attachments[i].child, f'{side} gripper')

    with pp.WorldSaver():
        pp.set_joint_positions(robot, joints, list(q12))
        for attachment in attachments:
            attachment.assign()
        moving = frozenset(pp.get_moving_links(robot, joints))

        # * 2. the robot against itself (this is also arm-against-arm and
        # * arm-against-husky: both arms are moving links of one body)
        for link1, link2 in pp.get_self_link_pairs(robot, joints, {}):
            if pp.pairwise_link_collision(robot, link1, robot, link2):
                reasons.append(
                    f'SELF {_pair_name(robot, link1)} vs '
                    f'{_pair_name(robot, link2)} '
                    f'({_depth_mm(robot, link1, robot, link2):.1f} mm deep)')

        # * 3. each gripper against the robot links it is not allowed to touch
        for attachment in attachments:
            for link in moving:
                if link == attachment.parent_link:
                    continue
                if ((robot, link), (attachment.child, pp.BASE_LINK)) in extra_disabled:
                    continue
                if pp.pairwise_link_collision(robot, link, attachment.child,
                                              pp.BASE_LINK):
                    reasons.append(
                        f'GRIPPER {_pair_name(attachment.child, pp.BASE_LINK)} '
                        f'vs {_pair_name(robot, link)} '
                        f'({_depth_mm(robot, link, attachment.child, pp.BASE_LINK):.1f} mm deep)')

        # * 4. robot and grippers against every static obstacle. The moving
        # * side is the robot's MOVING links plus each gripper, exactly the
        # * `moving_bodies` list pp builds.
        moving_bodies = [(robot, moving)] + [a.child for a in attachments]
        for mover in moving_bodies:
            body1, links1 = pp.expand_links(mover)
            for obstacle in obstacles:
                body2, links2 = pp.expand_links(obstacle)
                if body1 == body2:
                    continue
                for link1 in links1:
                    for link2 in links2:
                        if ((body1, link1), (body2, link2)) in extra_disabled:
                            continue
                        if pp.pairwise_link_collision(body1, link1, body2, link2):
                            reasons.append(
                                f'OBSTACLE {_pair_name(body1, link1)} vs '
                                f'{_pair_name(body2, link2)} '
                                f'({_depth_mm(body1, link1, body2, link2):.1f} mm deep)')

    if not reasons:
        log('[approach] no colliding pair and no limit violation found -- if '
            'the collision function still rejects this configuration, the two '
            'have drifted apart and describe_collision needs updating')
        return reasons
    log(f'[approach] {len(reasons)} colliding pair(s):')
    for line in reasons[:max_report]:
        log(f'[approach]   {line}')
    if len(reasons) > max_report:
        log(f'[approach]   ... and {len(reasons) - max_report} more')
    return reasons


def plan_approach(robot, attachments, obstacles, start_q12, goal_q12,
                  log=print, traj_q12=None, branch_margin: float = 0.0) -> tuple:
    """Plan a collision-free dual-arm motion from the live pose to the goal.

    Args:
        robot (int): Robot body id in the current world.
        attachments (list): The two gripper Attachments (left, right).
        obstacles (list): Static body ids from `build_obstacles`.
        start_q12: Where the arms are now [rad].
        goal_q12: Where they must end up [rad].
        log (callable): Message sink.
        traj_q12: (n, 12) samples of the trajectory that starts at the goal,
            so a joint is only unwrapped onto a branch the whole trajectory
            fits on. None skips that check.
        branch_margin (float): Room the trajectory must keep to each joint
            limit on an unwrapped branch [rad].

    Returns:
        tuple: ``(path12, info)`` -- a list of 12-vectors including both
        endpoints, or ``(None, info)`` with ``info['failure_reason']`` set.
    """
    joints = pp.joints_from_names(robot, JOINT_NAMES_12)
    start = np.asarray(start_q12, dtype=float)
    lower, upper = pp.get_custom_limits(robot, joints,
                                        get_custom_limits(robot, {}))
    goal = unwrap_goal(goal_q12, start, traj_q12, np.asarray(lower),
                       np.asarray(upper), branch_margin)
    wrapped = np.abs(goal - np.asarray(goal_q12, dtype=float)) > 1e-6
    if wrapped.any():
        names = ', '.join(JOINT_NAMES_12[j] for j in np.nonzero(wrapped)[0])
        log(f'[approach] unwrapped to the branch nearest the start: {names}')
    kept = (np.abs(np.round((np.asarray(goal_q12, dtype=float) - start)
                            / (2.0 * np.pi))) > 0) & ~wrapped
    if kept.any():
        names = ', '.join(JOINT_NAMES_12[j] for j in np.nonzero(kept)[0])
        log(f'[approach] NOT unwrapped (the trajectory would run too close to '
            f'the joint limit on the near branch): {names} -- the approach '
            f'turns the long way round instead')
    log(f'[approach] joint deltas: max {np.abs(goal - start).max():.3f} rad, '
        f'L2 {np.linalg.norm(goal - start):.3f} rad')

    collision_fn = build_collision_fn(robot, attachments, obstacles)
    for label, q in (('live', start), ('goal', goal)):
        if collision_fn(q):
            log(f'[approach] the {label} configuration is already in '
                f'collision -- cannot plan out of it')
            describe_collision(robot, attachments, obstacles, q, log=log)
            return None, {'failure_reason': f'{label} configuration in collision'}

    path, info = None, {'failure_reason': 'birrt_failed'}
    # ! plan_transit_motion reads its start from the robot's current pose, and
    # ! its dual-arm branch deliberately leaves the renderer unlocked (a
    # ! leftover cfab-debugging hack). Lock it here, or the GUI repaints on
    # ! every sample and the search crawls.
    with pp.WorldSaver(), pp.LockRenderer():
        pp.set_joint_positions(robot, joints, list(start))
        for resolution, max_time, tag in PLAN_PASSES:
            log(f'[approach] {tag} pass: joint_resolution={resolution:.3f} '
                f'rad, max_time={max_time:.0f}s ...')
            path = plan_transit_motion(
                robot, list(goal), attachments, obstacles,
                dual_arm_index='both', joint_resolution=resolution,
                max_time=max_time, max_iterations=50, disabled_collisions={})
            if path is not None:
                log(f'[approach] {tag} pass succeeded: {len(path)} waypoints')
                info = {'failure_reason': None, 'pass': tag,
                        'waypoints': len(path)}
                break
            log(f'[approach] {tag} pass found no path')
            pp.set_joint_positions(robot, joints, list(start))
    if path is None:
        return None, info

    verdict = validate_path(collision_fn, path, log=log)
    info.update(verdict)
    if not verdict['ok']:
        log('[approach] plan REJECTED: it sweeps through the scene between '
            'waypoints. Plan again for a different sample.')
        return None, info
    return [np.asarray(q, dtype=float) for q in path], info


def validate_path(collision_fn, path12, step_rad=VALIDATION_STEP_RAD,
                  log=print) -> dict:
    """Re-check a planned path densely, between the waypoints as well.

    ! BiRRT does NO swept checking: it only tests the configurations its
    ! extend function produces, and its post-plan shortcutting lengthens those
    ! gaps further. At a shoulder joint with a ~0.9 m lever, one 0.05 rad step
    ! is ~45 mm of tool travel -- wider than an assembly bar. So every segment
    ! is walked at a finer step here and the path is REJECTED (never merely
    ! flagged) if anything hits. Any exception also fails the verdict: an
    ! unverifiable path must not be executed.

    Args:
        collision_fn (callable): From `build_collision_fn`.
        path12 (list): Waypoints to check.
        step_rad (float): Interpolation step [rad].
        log (callable): Message sink.

    Returns:
        dict: ``{'ok': bool, 'samples': int, 'bad_segments': list}``.
    """
    verdict = {'ok': True, 'samples': 0, 'bad_segments': []}
    try:
        for k in range(len(path12) - 1):
            a, b = np.asarray(path12[k]), np.asarray(path12[k + 1])
            steps = max(1, int(np.ceil(np.abs(b - a).max() / step_rad)))
            for s in range(1, steps + 1):
                verdict['samples'] += 1
                if collision_fn(a + (b - a) * (s / steps)):
                    verdict['ok'] = False
                    verdict['bad_segments'].append(k)
                    break
    except Exception as exc:                       # fail closed, never open
        log(f'[approach] validation could not complete ({exc}); '
            f'treating the path as unsafe')
        verdict['ok'] = False
    log(f'[approach] swept re-check at {step_rad} rad: {verdict["samples"]} '
        f'samples, ' + ('clear' if verdict['ok']
                        else f'COLLISION in segments {verdict["bad_segments"][:5]}'))
    return verdict


def densify(path12, step_rad=DENSIFY_STEP_RAD) -> np.ndarray:
    """Resample a waypoint polyline finely, staying exactly on it.

    The points are linear interpolations of the planned edges, so they inherit
    the collision check the edges just passed -- unlike a smoothing spline
    fitted through the waypoints, which cuts corners by up to a couple of
    tenths of a radian and can leave the checked path.

    Args:
        path12 (list): Planned waypoints.
        step_rad (float): Maximum per-joint spacing of the output [rad].

    Returns:
        np.ndarray: (m, 12) densified path including both endpoints.
    """
    dense = [np.asarray(path12[0], dtype=float)]
    for a, b in zip(path12[:-1], path12[1:]):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        steps = max(1, int(np.ceil(np.abs(b - a).max() / step_rad)))
        dense.extend(a + (b - a) * (s / steps) for s in range(1, steps + 1))
    return np.asarray(dense)


def approach_traj_from_path(path12, max_joint_vel=0.25, dt=APPROACH_DT):
    """Turn planned waypoints into a timed trajectory the tracker can follow.

    The path is densified, then walked with a smooth-step progress profile:
    speed starts at zero, peaks at `max_joint_vel` in the middle and returns
    to zero at the goal, so the arms ease in and out instead of stepping to
    full speed. Positions are read straight off the densified polyline, which
    is why the tracked reference stays on the collision-checked path.

    Args:
        path12 (list): Planned waypoints (>= 2).
        max_joint_vel (float): Peak joint speed [rad/s].
        dt (float): Reference sample spacing [s].

    Returns:
        OpenLoopTraj: Approach motion, no gripper events.
    """
    dense = densify(path12)
    # Progress measured as the largest single-joint travel, matching how the
    # speed limit is expressed.
    seg = np.abs(np.diff(dense, axis=0)).max(axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(arc[-1])
    if total <= 1e-9:                     # already at the goal
        times = np.array([0.0, dt])
        q12 = np.vstack([dense[0], dense[0]])
        return traj_from_arrays(times, q12, np.zeros_like(q12),
                                label='<approach: already at start>')

    # smooth-step progress s(u) = 3u^2 - 2u^3 peaks at 1.5 * total / T, so a
    # duration of 1.5 * total / vmax puts the peak exactly at vmax.
    duration = 1.5 * total / float(max_joint_vel)
    times = np.arange(0.0, duration + dt, dt)
    u = np.clip(times / duration, 0.0, 1.0)
    progress = total * (3.0 * u ** 2 - 2.0 * u ** 3)
    # Position at a given progress = the point that far along the polyline.
    q12 = np.column_stack([np.interp(progress, arc, dense[:, j])
                           for j in range(12)])
    qd12 = np.gradient(q12, times, axis=0)
    qd12[0] = qd12[-1] = 0.0              # exact standstill at both ends
    return traj_from_arrays(times, q12, qd12, label='<approach>')
