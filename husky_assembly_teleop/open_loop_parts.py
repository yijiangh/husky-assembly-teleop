"""Where every assembly part is, at every moment of an open-loop trajectory.

* The engine's 3D view already shows the robot; this module adds the PARTS --
* the seat and the legs -- so the operator can see what each gripper carries
* and watch a leg go into the seat instead of guessing from the arm motion.
* Input: the layout JSON (nominal from export_layout, or measured by
* pickup_calib) for the part meshes and their poses on the table, plus the
* planner's plan.json to know which gripper-open is a MATE and which is a
* set-down. Output: one PyBullet body per part, and a pose for each of them
* at any sample of the trajectory.

The bookkeeping is a parent chain per part: a part sits on the world, or rides
an arm's tool0 flange, or rides another part (once assembled). Every gripper
event re-parents one part WITHOUT moving it, so the view never jumps.

! The bodies carry a collision shape, but ONLY so `fit_jaws` can measure the
! finger pads against them. Nothing steps physics here, and the approach
! planner checks its own boxes from `open_loop_approach.build_obstacles`, so
! no planning or execution behaviour reads these shapes.

! This module imports only json/numpy/scipy/pybullet -- no ROS, no UI -- so it
! can be exercised headlessly.
"""

import json
import os
from bisect import bisect_right

import numpy as np
import pybullet as p
import pybullet_planning as pp
from pybullet_planning.interfaces.env_manager.shape_creation import (
    get_mesh_geometry)
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.open_loop_traj import ARM_SIDES

# Part colours by what the part is currently doing, so a glance at the 3D view
# says which piece is in a gripper and which ones are already built in.
COLOR_TABLE = (0.60, 0.60, 0.62, 1.0)       # resting on the table
COLOR_HELD = (0.95, 0.60, 0.20, 1.0)        # carried by a gripper
COLOR_ASSEMBLED = (0.35, 0.80, 0.45, 1.0)   # mated into another part

# How deep a part-rides-part chain may go before we call it malformed. The
# real assembly is two levels (leg on seat, seat in a gripper).
MAX_PARENT_DEPTH = 8

# ! How far outside a part's own box the gripper may be when it closes on it
# ! before we call the layout a mismatch. Fingers grasp a SURFACE, and the
# ! trajectory may approach a part from its edge, so this is deliberately
# ! generous -- it is there to catch the wrong layout file, not a poor grasp.
GRASP_MARGIN_M = 0.05


def part_world_pose(entry: dict, table_top_z: float,
                    part_float: float = 0.0) -> tuple:
    """Pose of one part resting on the table, exactly as the planner places it.

    Mirrors the planner's own placement (`problem.add_disassembled_obj`): the
    part keeps the orientation the assembly says it lies in
    (`scatter_quat_wxyz`), the layout's measured yaw is turned on top of that
    about world z, and it rests on the table top plus the planner's small
    float gap.

    Args:
        entry (dict): One "parts" entry of a layout JSON.
        table_top_z (float): Height of the table's top surface [m].
        part_float (float): Gap the planner leaves under a resting part [m].

    Returns:
        tuple: A pybullet_planning pose, i.e. (point, quaternion xyzw).
    """
    scatter = np.asarray(entry.get('scatter_quat_wxyz') or [1.0, 0.0, 0.0, 0.0],
                         dtype=float)
    # ! Layout and rai quaternions are wxyz; PyBullet and scipy are xyzw.
    rot = (Rotation.from_euler('z', float(entry['yaw']))
           * Rotation.from_quat(scatter[[1, 2, 3, 0]]))
    x, y = entry['xy']
    z = table_top_z + part_float + 0.5 * float(entry['height'])
    return ((float(x), float(y), float(z)), tuple(rot.as_quat()))


def load_part_bodies(layout: dict) -> dict:
    """Create one visual-only PyBullet body per part of a layout.

    Uses each part's exact visual mesh when the layout names one that exists,
    so the seat's pockets and the legs' real shape are visible; otherwise a
    box of the part's footprint stands in.

    ! The bodies are collidable so `fit_jaws` has a surface to close the
    ! fingers onto, and that surface is the EXACT TRIANGLE MESH. PyBullet's
    ! default for a mesh is its convex hull, which on a leg is a lie: the
    ! snap-fit lug at one end drags the hull out along the whole length, so
    ! the leg measures 22.9-28.4 mm across instead of its true 22.0 mm and the
    ! jaws stop up to 4 mm per side too early. A concave trimesh is allowed
    ! here because these bodies are static and nothing steps physics.

    Args:
        layout (dict): A loaded layout JSON (`pickup_calib.load_layout`).

    Returns:
        dict: Part name -> PyBullet body id, posed on the table.
    """
    top_z = float(layout['table']['top_z'])
    part_float = float(layout.get('part_float', 0.0))
    bodies = {}
    for name, entry in sorted(layout['parts'].items()):
        mesh = entry.get('visual_mesh')
        with pp.HideOutput():
            if mesh and os.path.exists(mesh):
                geometry = get_mesh_geometry(mesh)
                body = pp.create_body(
                    pp.create_collision_shape(
                        {**geometry, 'flags': p.GEOM_FORCE_CONCAVE_TRIMESH}),
                    pp.create_visual_shape(geometry, color=COLOR_TABLE))
            else:
                # No mesh on this machine: the footprint box still shows where
                # the part is and how big it is.
                hx, hy = entry['footprint_half_xy']
                body = pp.create_box(2 * hx, 2 * hy, float(entry['height']),
                                     color=COLOR_TABLE)
        pp.set_pose(body, part_world_pose(entry, top_z, part_float))
        bodies[name] = body
    return bodies


# The two finger pads, by URDF link name. These are what actually touch a part
# (`robotiq_85_gripper_simple.urdf`), and they are what a jaw fit measures.
JAW_TIP_LINKS = ('robotiq_85_left_finger_tip_link',
                 'robotiq_85_right_finger_tip_link')
# ! How close the two pads must get to the part before the jaws are called
# ! fitted, measured as the SUM of the two gaps. Summing is what makes an
# ! off-centre grasp work: a part lying free on the table slides toward
# ! whichever pad reaches it first, so demanding that BOTH pads touch where
# ! the part stands would never trigger and the jaws would sweep shut.
JAW_FIT_TOLERANCE_M = 0.001
# How far the closest-point query looks. Anything past this reads as "far".
JAW_QUERY_RANGE_M = 0.2
# ! The table query needs its own, much longer reach: a pad held high above the
# ! table is genuinely far, and clamping that to JAW_QUERY_RANGE_M would report
# ! a made-up 200 mm as if it were a measurement. Past THIS the clearance is
# ! reported as None -- "so far away the number does not matter" -- rather than
# ! as a number the operator might compare against anything.
TABLE_QUERY_RANGE_M = 0.5


def _closest_distance(body1, link1, body2, link2=None,
                      max_distance: float = JAW_QUERY_RANGE_M) -> float:
    """Signed distance between two links, positive apart and negative overlapping.

    Args:
        body1 (int): First body id.
        link1 (int): Link index on the first body.
        body2 (int): Second body id.
        link2 (int): Link index on the second body; its base link by default.
        max_distance (float): How far the query looks [m].

    Returns:
        float: The distance [m], or `max_distance` when they are further apart
        than the query looks -- a saturated value, not a measurement.
    """
    if link2 is None:
        link2 = pp.BASE_LINK
    points = pp.pairwise_link_collision_info(body1, link1, body2, link2,
                                             max_distance=max_distance)
    # PyBullet's closest-point tuple carries the signed distance at index 8.
    return min((float(pt[8]) for pt in points), default=max_distance)


def _pad_gap(grip_body: int, tips: list) -> float:
    """The gap between the two finger pads right now -- how wide the jaws are.

    Measured surface to surface between the two tip links, so it is the number
    an operator can hold a part up against. (The tip links' ORIGINS sit ~16 mm
    apart with the jaws shut, so their separation overstates the opening by
    that much and is no use as a reading.) Sanity: fully open reads 83 mm
    against the 2F-85's 85 mm stroke.

    Args:
        grip_body (int): The articulated gripper body.
        tips (list): The two tip link indices.

    Returns:
        float: The pad-to-pad gap [m], negative once the pads overlap.
    """
    return _closest_distance(grip_body, tips[0], grip_body, tips[1])


def fit_jaws(grip_body: int, grip_joints: list, factors: list, part_bodies,
             table_body: int, open_angle: float, close_angle: float,
             step: float = 0.005) -> dict:
    """Close the jaws until the pads meet the part, and measure what that costs.

    ! The viewer used to snap the fingers to fully closed on every grasp, which
    ! drove the finger meshes straight through the part and made the picture
    ! useless for judging table clearance. This walks the knuckle angle shut
    ! one step at a time and stops where the pads actually meet the part, so
    ! what is on screen is a grasp the real gripper could hold.
    !
    ! The four-bar drop comes out for free: the inner finger swings ~13 mm
    ! FURTHER along the approach axis as it closes, so the pad tip measured at
    ! the fitted angle is the one that would really sweep the table.

    ! `part_bodies` is the WHOLE sub-assembly the arm carries, not one part.
    ! Once legs are mated onto a seat the arm may pinch a LEG while the part
    ! bookkeeping still roots the chain at the seat -- measured on the 5-part
    ! stool, the pads sat 148-186 mm from the seat's centre and would never
    ! have met it. The real gripper closes until it meets whatever it holds,
    ! so every body in the chain counts and the nearest one wins.

    The world is restored on the way out (`pp.WorldSaver`), fingers included;
    the caller re-applies the returned angle when it wants to show it.

    Args:
        grip_body (int): The articulated gripper body.
        grip_joints (list): Its six driven joint indices.
        factors (list): Per joint, the multiplier on the knuckle angle.
        part_bodies: The body the jaws are closing on, or every body of the
            sub-assembly the arm carries.
        table_body (int): The table to measure clearance against, or None.
        open_angle (float): Knuckle angle the fingers start from [rad].
        close_angle (float): Fully-closed knuckle angle, the sweep's end [rad].
        step (float): Knuckle-angle increment per test [rad].

    Returns:
        dict: ``{'angle', 'gaps_mm', 'opening_mm', 'table_clearance_mm',
        'closed_on_nothing'}``. `opening_mm` is the PAD-TO-PAD gap the fit
        stops at, so it reads the pinched thickness plus PyBullet's collision
        margin (~1 mm per side on a mesh).
        `table_clearance_mm` is None without a table and when the pads are
        further than `TABLE_QUERY_RANGE_M` from it; it is NEGATIVE when a pad
        has gone below the table top.
    """
    tips = [pp.link_from_name(grip_body, name) for name in JAW_TIP_LINKS]
    if isinstance(part_bodies, int):
        part_bodies = [part_bodies]

    def gaps(angle):
        pp.set_joint_positions(grip_body, grip_joints,
                               [factor * angle for factor in factors])
        # A pad already inside a part reads negative; clamp so that pad simply
        # counts as touching rather than paying off the other one's gap.
        return [max(0.0, min(_closest_distance(grip_body, tip, body)
                             for body in part_bodies))
                for tip in tips]

    with pp.WorldSaver():
        angle, gap_pair = close_angle, gaps(close_angle)
        closed_on_nothing = sum(gap_pair) > JAW_FIT_TOLERANCE_M
        if not closed_on_nothing:
            steps = int(np.ceil((close_angle - open_angle) / step))
            for k in range(steps + 1):
                here = min(open_angle + k * step, close_angle)
                pair = gaps(here)
                if sum(pair) <= JAW_FIT_TOLERANCE_M:
                    angle, gap_pair = here, pair
                    break
        gaps(angle)                                # read everything at the fit
        opening = _pad_gap(grip_body, tips)
        clearance = None
        if table_body is not None:
            near = min(_closest_distance(grip_body, tip, table_body,
                                         max_distance=TABLE_QUERY_RANGE_M)
                       for tip in tips)
            # Saturated = the pads are nowhere near the table; say nothing
            # rather than report the query's own range as a distance.
            clearance = None if near >= TABLE_QUERY_RANGE_M else 1000.0 * near
    return {'angle': float(angle),
            'gaps_mm': tuple(1000.0 * g for g in gap_pair),
            'opening_mm': 1000.0 * opening,
            'table_clearance_mm': clearance,
            'closed_on_nothing': bool(closed_on_nothing)}


def read_mate_parents(plan_json: str) -> dict:
    """Which part each part gets assembled ONTO, from the planner's plan.

    Args:
        plan_json (str): Path to the planner's plan.json.

    Returns:
        dict: Child part name -> list of (parent part, robot name), one entry
        per `assemble` action in plan order.

    Raises:
        ValueError: If the file has no "plan" action list.
    """
    with open(plan_json) as f:
        plan = json.load(f)
    actions = plan.get('plan')
    if not actions:
        raise ValueError(f'{plan_json}: no "plan" action list')
    mates = {}
    for action in actions:
        if len(action) >= 4 and action[0] == 'assemble':
            _, robot, child, parent = action[:4]
            mates.setdefault(child, []).append((parent, robot))
    return mates


class PartTracker:
    """Follows every part through a trajectory: table -> gripper -> assembly.

    Built once from the trajectory's gripper events and the PLANNED joint
    configurations, so the whole timeline is known up front; `poses_at` then
    only resolves parent chains against whatever tool0 poses the caller passes
    in. That is what lets the execute mode draw the parts where the REAL arms
    are while still using the planned grasps.

    ! The grasps are the planned ones. A part is drawn where the plan says it
    ! sits in the gripper, not where slip actually left it.

    Args:
        traj (OpenLoopTraj): The loaded trajectory (needs `attached` and
            `events`, i.e. a file-loaded one, not a generated motion).
        layout (dict): Loaded layout JSON, for the parts' start poses.
        tool0_pose_fn (callable): `fn(arm_index, q12) -> pose`, the tool0 pose
            of one arm at a 12-joint configuration. Injected so this module
            needs no robot model of its own.
        plan_json (str): Planner plan.json naming which opens are mates, or
            None to treat every open as a set-down.
        log (callable): One-line logger for the mismatches worth telling the
            operator about; defaults to `print`.
        tcp_offset (tuple): tool0 -> gripper TCP pose
            (`utils.TOOL0_FROM_GRIPPER_TCP`). Given, the first grasp of every
            part is checked against the layout; None skips that check.
    """

    def __init__(self, traj, layout: dict, tool0_pose_fn, plan_json: str = None,
                 log=print, tcp_offset=None):
        self.traj = traj
        self.log = log
        self.layout = layout
        self.tcp_offset = tcp_offset
        self.grasp_misses = {}
        top_z = float(layout['table']['top_z'])
        part_float = float(layout.get('part_float', 0.0))
        self.names = sorted(layout['parts'])

        # ? Mates keyed by the CHILD part, not by (robot, child): a plan.json
        # ? re-solved after its trajectory was exported can hand the same
        # ? assemble to the other arm, and the trajectory is the truth about
        # ? which arm does what. The robot is kept only to warn about it.
        mates = read_mate_parents(plan_json) if plan_json else {}

        # Per part: the episodes it lives through, as
        # (first sample, parent, pose relative to that parent). The pose of a
        # world-parented episode is simply its world pose.
        self.timeline = {
            name: [(0, 'world', part_world_pose(layout['parts'][name],
                                                top_z, part_float))]
            for name in self.names}
        state = {name: self.timeline[name][0][1:] for name in self.names}

        # The attachment record leads a close by one sample and lags an open by
        # one, so each edge is read from the side that still knows which part
        # was involved.
        steps = []
        for ev in traj.events:
            i = traj.state_at(ev.time)
            if ev.kind == 'close':
                part = traj.attached[i].get(ev.robot)
            else:
                part = (traj.attached[max(i - 1, 0)].get(ev.robot)
                        or traj.attached[i].get(ev.robot))
            if part is not None and part in self.timeline:
                steps.append((i, ev, part))

        # ? Which open actually MATES a part: its last one. A part is opened
        # ? twice when it is handed over -- the giver lets go and the receiver
        # ? takes it -- and once a part is mated it is never picked up again,
        # ? so the final open is the release that leaves it in the assembly.
        final_open = {}
        for i, ev, part in steps:
            if ev.kind == 'open':
                final_open[part] = i

        for i, ev, part in steps:
            tool0 = [tool0_pose_fn(k, traj.q12[i]) for k in range(2)]
            here = self._resolve(part, state, tool0)

            if ev.kind == 'close':
                if state[part][0] == 'world' and part not in self.grasp_misses:
                    self._check_grasp(part, here, tool0[ev.arm_index])
                parent = ('arm', ev.arm_index)
                rel = pp.multiply(pp.invert(tool0[ev.arm_index]), here)
            else:
                # ! A handover's give arrives once the part already rides the
                # ! other arm: in a v2 file the giver's open and the receiver's
                # ! close share one instant, in a v3 file the open is delayed to
                # ! the end of the receiver's close dwell (both arms are listed
                # ! as attached meanwhile). Either way, releasing the part to
                # ! the world here would drop it in mid-air, so a give is a
                # ! no-op for the part's parent chain.
                if state[part][0] != ('arm', ev.arm_index):
                    continue
                mate = mates.get(part, []) if i == final_open[part] else []
                if mate:
                    parent_name, plan_robot = mate.pop(0)
                    if plan_robot != ev.robot:
                        self.log(
                            f'[parts] plan.json has {parent_name} <- {part} '
                            f'assembled by {plan_robot}, the trajectory does '
                            f'it with {ev.robot} -- following the trajectory')
                    parent = ('part', parent_name)
                    rel = pp.multiply(
                        pp.invert(self._resolve(parent_name, state, tool0)),
                        here)
                else:
                    parent, rel = 'world', here
            state[part] = (parent, rel)
            self.timeline[part].append((i,) + state[part])

        # Sample indices per part, for the bisect in `_episode`.
        self.starts = {name: [ep[0] for ep in eps]
                       for name, eps in self.timeline.items()}
        leftover = {p: [m[0] for m in v] for p, v in mates.items() if v}
        if leftover:
            self.log(f'[parts] plan.json mates never seen in the trajectory: '
                     f'{leftover} -- those parts stay where they were released')

    def _check_grasp(self, name: str, part_pose: tuple, tool0_pose: tuple):
        """Is the gripper actually AT the part when it first closes on it?

        The layout says where a part rests; the trajectory says where the
        gripper goes to fetch it. If those disagree the layout does not belong
        to this trajectory -- most often a table height from another cell --
        and every part would be drawn in the wrong place. The miss is recorded
        per part and reported by `summary`.

        Args:
            name (str): Part name.
            part_pose (tuple): The part's world pose at the grasp.
            tool0_pose (tuple): The grasping arm's tool0 world pose.
        """
        if self.tcp_offset is None:
            return
        entry = self.layout['parts'][name]
        tcp = np.asarray(pp.multiply(tool0_pose, self.tcp_offset)[0])
        # The layout's extents are given for the part AS IT LIES, in world
        # axes turned by its yaw -- so undo only the yaw to compare.
        local = (Rotation.from_euler('z', -float(entry['yaw']))
                 .apply(tcp - np.asarray(part_pose[0])))
        half = np.array([entry['footprint_half_xy'][0],
                         entry['footprint_half_xy'][1],
                         0.5 * float(entry['height'])])
        outside = np.abs(local) - half - GRASP_MARGIN_M
        if outside.max() > 0.0:
            self.grasp_misses[name] = (float(outside.max()), local.copy())

    def _episode(self, name: str, index: int) -> tuple:
        """The (parent, relative pose) this part lives in at one sample.

        Args:
            name (str): Part name.
            index (int): Trajectory sample index.

        Returns:
            tuple: (parent, pose), the parent being 'world', ('arm', i) or
            ('part', other name).
        """
        k = bisect_right(self.starts[name], index) - 1
        return self.timeline[name][max(k, 0)][1:]

    def _resolve(self, name: str, state: dict, tool0_poses: list,
                 depth: int = 0) -> tuple:
        """World pose of one part by walking its parent chain.

        Args:
            name (str): Part name.
            state (dict): Part name -> (parent, relative pose).
            tool0_poses (list): Per arm, the tool0 pose to attach held parts to.
            depth (int): Recursion guard counter.

        Returns:
            tuple: The part's world pose.

        Raises:
            RuntimeError: If the parent chain is longer than MAX_PARENT_DEPTH,
                which means it is cyclic.
        """
        if depth > MAX_PARENT_DEPTH:
            raise RuntimeError(f'cyclic parent chain at part {name!r}')
        parent, rel = state[name]
        if parent == 'world':
            return rel
        if parent[0] == 'arm':
            return pp.multiply(tool0_poses[parent[1]], rel)
        return pp.multiply(self._resolve(parent[1], state, tool0_poses,
                                         depth + 1), rel)

    def poses_at(self, index: int, tool0_poses: list) -> dict:
        """World pose of every part at one trajectory sample.

        Args:
            index (int): Trajectory sample index.
            tool0_poses (list): Per arm [left, right], the tool0 pose to hang
                held parts on -- the LIVE one during execution, the planned
                one in preview.

        Returns:
            dict: Part name -> world pose.
        """
        state = {name: self._episode(name, index) for name in self.names}
        return {name: self._resolve(name, state, tool0_poses)
                for name in self.names}

    def role_at(self, index: int) -> dict:
        """What every part is doing at one sample, for the colours.

        Args:
            index (int): Trajectory sample index.

        Returns:
            dict: Part name -> 'table', 'held' or 'assembled'.
        """
        roles = {}
        for name in self.names:
            parent = self._episode(name, index)[0]
            roles[name] = ('table' if parent == 'world' else
                           'held' if parent[0] == 'arm' else 'assembled')
        return roles

    def episode_start(self, name: str, index: int) -> int:
        """First sample of the episode one part is living through at `index`.

        Identifies a grasp uniquely: a part can be grasped several times, by
        the same arm and with different grasps (a handover regrasp), so the
        arm and the part alone do not name one.

        Args:
            name (str): Part name.
            index (int): Trajectory sample index.

        Returns:
            int: The sample the current episode began at.
        """
        k = bisect_right(self.starts[name], index) - 1
        return self.starts[name][max(k, 0)]

    def held_chain(self, arm_index: int, index: int) -> list:
        """Every part riding one arm at a sample, mated members included.

        A part is on the arm when its parent chain REACHES that arm: the seat
        hangs off the flange, the legs hang off the seat. All of them travel
        with the gripper, and any of them can be the one its pads are on.

        Args:
            arm_index (int): 0 = left, 1 = right.
            index (int): Trajectory sample index.

        Returns:
            list: The part names, in the tracker's own order.
        """
        state = {name: self._episode(name, index)[0] for name in self.names}
        held = []
        for name in self.names:
            parent, depth = state[name], 0
            while (parent != 'world' and parent[0] == 'part'
                   and depth < MAX_PARENT_DEPTH):
                parent, depth = state[parent[1]], depth + 1
            if parent != 'world' and parent[0] == 'arm' and parent[1] == arm_index:
                held.append(name)
        return held

    def held_by(self, arm_index: int, index: int) -> str:
        """Which part one arm is holding at a sample, if any.

        Args:
            arm_index (int): 0 = left, 1 = right.
            index (int): Trajectory sample index.

        Returns:
            str: The part's name, or None when that arm holds nothing.
        """
        for name in self.names:
            if self._episode(name, index)[0] == ('arm', arm_index):
                return name
        return None

    def summary(self) -> str:
        """One line per part listing the episodes it goes through.

        Returns:
            str: Human-readable timeline, for the startup log.
        """
        lines = []
        for name in self.names:
            steps = []
            for index, parent, _rel in self.timeline[name]:
                where = ('table' if parent == 'world' else
                         f'{ARM_SIDES[parent[1]]} arm' if parent[0] == 'arm'
                         else f'on {parent[1]}')
                steps.append(f't={self.traj.times[index]:.2f}s {where}')
            lines.append(f'  {name}: ' + ' -> '.join(steps))
        if self.grasp_misses:
            worst = max(self.grasp_misses.items(), key=lambda kv: kv[1][0])
            lines.append(
                f'  ! this layout does NOT match this trajectory: the gripper '
                f'closes {worst[1][0] * 1000:.0f} mm outside {worst[0]} '
                f'(offset in the part frame {np.round(worst[1][1], 3)} m), and '
                f'{len(self.grasp_misses)} of {len(self.names)} parts are off. '
                f'Check the table height and that this layout was exported for '
                f'this plan -- the parts will be drawn in the wrong place.')
        return '\n'.join(lines)
