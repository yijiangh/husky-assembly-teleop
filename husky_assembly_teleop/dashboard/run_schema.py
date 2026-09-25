"""What an M1 derivation run file contains, and how to describe it in words.

One run file is a complete record of a single ``derive_constrained_start``
sweep: the goal it walked back from, every home bar pose it tried, why each
one failed, and where the time went. ``validate_run`` is the gate the producer
passes a record through before writing it; the ``describe_*`` helpers turn the
numbers into the sentences the dashboard shows, because the raw fields
("reached 0.17", "blocked_at 0.06") mean nothing to a reader.

! Keep the wording physical: centimetres of bar travel, degrees of joint jump,
! named links -- never fractions or internal field names.
"""
import os
from math import atan2, degrees


SCHEMA = 'm1-derive-run/1'
RUN_KIND = 'm1_derive'

# Why a candidate home pose was rejected, in the order the dashboard lists them.
#   corridor             walked the whole way AND the straight path is clear ->
#                        this IS a finished M1 path, the RRT is skipped
#   blocked              walked the whole way, home pose is fine, but the
#                        straight path hits something in between
#   arrival_collision    walked the whole way, but the arm collides AT the home
#   track_break          never arrived: IK missed, or the solution jumped branch
#   fine_reverify_failed a corridor that passed the coarse screen but not the
#                        fine re-walk
OUTCOMES = ('corridor', 'blocked', 'arrival_collision', 'track_break',
            'fine_reverify_failed')

OUTCOME_PLAIN = {
    'corridor': 'clear path found',
    'blocked': 'reached home, path blocked on the way',
    'arrival_collision': 'reached home, but collides there',
    'track_break': 'never reached home',
    'fine_reverify_failed': 'clear at coarse steps, not at fine steps',
}

# How an RRT search ended, in words.
RRT_OUTCOME_PLAIN = {
    'connected': 'the two trees met',
    'corridor': 'not needed -- the straight walk from the start was already clear',
    'max_time': 'the time budget ran out before the trees met',
    'max_iterations': 'the iteration cap was reached before the trees met',
    'start_in_collision': 'the start configuration itself collides',
    'goal_in_collision': 'the goal configuration itself collides',
    'failed': 'no path was found',
}

# Where a run's mobile-base pose came from. EVERY position in a run file is
# relative to that base, so a run that stood the robot in the wrong place is
# wrong everywhere -- which is why the dashboard says this out loud.
BASE_SOURCE_PLAIN = {
    'mocap_live': 'measured live by the mocap while the derivation ran',
    'bar_action_file': "read from the BarAction file's own M1 start state, "
                       'because nothing was tracking the base',
    'base_placement_heuristic': 'chosen by the headless base-placement search '
                                '(saved as the .solved_keyframe sidecar), because the '
                                'export itself left the robot at the world origin',
    'unknown': 'not recorded (this run file predates the base readout)',
}

# The three carry modes, in words.
ANCHOR_PLAIN = {
    'horizontal': 'bar carried across the front',
    'vertical': 'bar upright in front',
    'back': 'bar fore-aft over the robot',
}

REQUIRED_RUN_KEYS = ('schema', 'kind', 'id', 'created', 'source', 'problem',
                     'active_bar', 'anchor_selection', 'result', 'stage_times',
                     'context', 'profile', 'candidates')
REQUIRED_CONTEXT_KEYS = ('joint_names_12', 'world_from_mobile_base', 'goal',
                         'grasps', 'attachments', 'static_bodies', 'scene')
REQUIRED_CANDIDATE_KEYS = ('i', 'variant', 'anchor', 'rotation', 'home_pos_mb',
                           'home_quat_mb', 'travel_cm', 'outcome', 'last_conf')


def _check_pose(value, where, problems):
    """Record a problem unless ``value`` is a ``{pos, quat_xyzw}`` pose."""
    if not isinstance(value, dict):
        problems.append(f'{where}: expected a pose dict, got {type(value).__name__}')
        return
    if len(value.get('pos', [])) != 3:
        problems.append(f'{where}.pos: expected 3 numbers')
    if len(value.get('quat_xyzw', [])) != 4:
        problems.append(f'{where}.quat_xyzw: expected 4 numbers')


def validate_run(run):
    """Check a run record before it is written or served.

    Collects every problem rather than raising on the first, so a producer
    change that breaks the schema is reported in one go.

    Args:
        run (dict): the run record.

    Raises:
        ValueError: listing every missing or ill-typed field.
    """
    problems = []
    for key in REQUIRED_RUN_KEYS:
        if key not in run:
            problems.append(f'missing top-level key {key!r}')
    if run.get('schema') != SCHEMA:
        problems.append(f'schema must be {SCHEMA!r}, got {run.get("schema")!r}')

    context = run.get('context') or {}
    for key in REQUIRED_CONTEXT_KEYS:
        if key not in context:
            problems.append(f'missing context key {key!r}')
    if len(context.get('joint_names_12', [])) != 12:
        problems.append('context.joint_names_12: expected 12 names')
    if 'world_from_mobile_base' in context:
        _check_pose(context['world_from_mobile_base'], 'context.world_from_mobile_base', problems)
    goal = context.get('goal') or {}
    for key in ('conf', 'bar_pose_world', 'bar_pos_mb', 'bar_quat_mb'):
        if key not in goal:
            problems.append(f'missing context.goal key {key!r}')
    if 'bar_pose_world' in goal:
        _check_pose(goal['bar_pose_world'], 'context.goal.bar_pose_world', problems)
    if 'conf' in goal and len(goal['conf']) != 12:
        problems.append('context.goal.conf: expected 12 joint values')
    for i, att in enumerate(context.get('attachments') or []):
        for key in ('node', 'parent_link', 'link_from_body'):
            if key not in att:
                problems.append(f'context.attachments[{i}]: missing {key!r}')
        if 'link_from_body' in att:
            _check_pose(att['link_from_body'], f'context.attachments[{i}].link_from_body', problems)
    for i, body in enumerate(context.get('static_bodies') or []):
        if 'node' not in body or 'world' not in body:
            problems.append(f'context.static_bodies[{i}]: needs node + world')
        elif 'world' in body:
            _check_pose(body['world'], f'context.static_bodies[{i}].world', problems)

    candidates = run.get('candidates')
    if not isinstance(candidates, list):
        problems.append('candidates: expected a list')
        candidates = []
    for cand in candidates:
        i = cand.get('i', '?')
        for key in REQUIRED_CANDIDATE_KEYS:
            if key not in cand:
                problems.append(f'candidates[{i}]: missing {key!r}')
        if cand.get('outcome') not in OUTCOMES:
            problems.append(f'candidates[{i}]: outcome {cand.get("outcome")!r} '
                            f'not one of {OUTCOMES}')
        if len(cand.get('home_pos_mb', [])) != 3:
            problems.append(f'candidates[{i}].home_pos_mb: expected 3 numbers')
        if len(cand.get('home_quat_mb', [])) != 4:
            problems.append(f'candidates[{i}].home_quat_mb: expected 4 numbers')
        if len(cand.get('last_conf', [])) != 12:
            problems.append(f'candidates[{i}].last_conf: expected 12 joint values')
        track = cand.get('track') or {}
        indices = track.get('indices') or []
        if indices != sorted(indices):
            problems.append(f'candidates[{i}].track.indices: must be increasing')
        if len(indices) != len(track.get('confs') or []):
            problems.append(f'candidates[{i}].track: indices and confs differ in length')
        for conf in track.get('confs') or []:
            if len(conf) != 12:
                problems.append(f'candidates[{i}].track.confs: expected 12 joint values')
                break

    if problems:
        raise ValueError('invalid M1 derivation run:\n  - ' + '\n  - '.join(problems))


def variant_plain(label):
    """A home variant label as a readable phrase.

    Args:
        label (str): e.g. ``'back/canonical'``, ``'vertical/roll-60'``.

    Returns:
        str: e.g. ``'bar fore-aft over the robot'`` or
        ``'bar upright in front, rolled -60 deg about its own axis'``.
    """
    anchor, _, rotation = str(label).partition('/')
    base = ANCHOR_PLAIN.get(anchor, anchor)
    if not rotation or rotation == 'canonical':
        return base
    kind = 'rolled' if rotation.startswith('roll') else 'swung'
    about = 'about its own axis' if rotation.startswith('roll') else 'about the base vertical'
    return f'{base}, {kind} {rotation[4:]} deg {about}'


def _joint_plain(joint_names_12, index):
    """A joint name as 'left wrist 1' instead of 'left_ur_arm_wrist_1_joint'."""
    try:
        raw = joint_names_12[int(index)]
    except (IndexError, TypeError, ValueError):
        return f'joint {index}'
    return (raw.replace('_ur_arm', '').replace('_joint', '')
            .replace('_', ' ').strip())


def _where_plain(pos):
    """A position in the robot frame as 'x cm ahead, y cm left, z cm up'."""
    x, y, z = (float(v) * 100.0 for v in pos)
    parts = [f'{abs(x):.0f} cm {"ahead" if x >= 0 else "behind"}',
             f'{abs(y):.0f} cm {"left" if y >= 0 else "right"}',
             f'{abs(z):.0f} cm {"up" if z >= 0 else "down"}']
    return ', '.join(parts)


def _part_plain(node_name):
    """A scene node name as a readable part name.

    Robot links become everyday words ("left forearm"); rigid bodies and tools
    keep their exact names, because those are what the cell and the BarAction
    call them and the reader will want to look them up.

    Args:
        node_name (str): e.g. ``'robot__left_ur_arm_forearm_link'``.

    Returns:
        str: e.g. ``'left forearm'``, or ``'bar_B1'`` unchanged.
    """
    name = str(node_name or '?')
    if name.startswith('robot__'):
        return (name[len('robot__'):].replace('_ur_arm', '')
                .replace('_link', '').replace('_', ' ').strip())
    return name.replace('body__', '').replace('tool__', '')


def _collisions_plain(collisions):
    """Name the colliding pairs, deepest first: 'left forearm against bar_B1 (12 mm)'."""
    if not collisions:
        return 'something in the scene'
    said = []
    for hit in collisions[:2]:
        a, b = _part_plain(hit.get('a')), _part_plain(hit.get('b'))
        said.append(f'{a} against {b} ({hit.get("depth_mm", 0.0):.0f} mm deep)')
    more = len(collisions) - len(said)
    return ' and '.join(said) + (f', and {more} more pair(s)' if more > 0 else '')


def describe_candidate(run, cand):
    """One sentence saying what this home pose was and why it was rejected.

    Args:
        run (dict): the run record (for the joint names and the step size).
        cand (dict): one entry of ``run['candidates']``.

    Returns:
        str: e.g. "bar fore-aft over the robot, 12 cm ahead, 6 cm left, 3 cm up
        of the base: the walk died after 11 of 63 cm -- left wrist 1 jumped
        47 deg in one 1 cm step (limit 10 deg), so IK switched branch."
    """
    names = (run.get('context') or {}).get('joint_names_12') or []
    step_cm = ((run.get('context') or {}).get('budget') or {}).get('screen_step_m', 0.01) * 100.0
    travel = float(cand.get('travel_cm') or 0.0)
    middle = bar_middle(run, cand.get('home_pos_mb') or [0, 0, 0],
                        cand.get('home_quat_mb') or [0, 0, 0, 1])
    head = (f'{variant_plain(cand.get("variant"))}, {middle_plain(run)} at '
            f'{_where_plain(middle)} of the base; '
            f'{travel:.0f} cm of bar travel from the goal')
    outcome = cand.get('outcome')

    if outcome == 'track_break':
        got = travel * float(cand.get('reached') or 0.0)
        brk = cand.get('break') or {}
        if brk.get('reason') == 'branch_flip':
            why = (f'{_joint_plain(names, brk.get("joint"))} jumped '
                   f'{float(brk.get("jump_deg") or 0.0):.0f} deg in a single '
                   f'{step_cm:.0f} cm step, so IK switched to another branch')
        else:
            why = 'IK found no way to hold the bar there'
        return f'{head}. The walk died after {got:.0f} of {travel:.0f} cm: {why}.'

    if outcome == 'arrival_collision':
        return (f'{head}. The arm reaches home, but collides there: '
                f'{_collisions_plain(cand.get("collisions"))}.')

    if outcome == 'blocked':
        at = travel * float(cand.get('blocked_at') or 0.0)
        return (f'{head}. Home is reachable and clear, but the straight path '
                f'collides {at:.0f} cm into the {travel:.0f} cm walk: '
                f'{_collisions_plain(cand.get("collisions"))}.')

    if outcome == 'corridor':
        return (f'{head}. The whole straight path is clear -- this is a '
                f'finished M1 motion, no search needed.')

    return (f'{head}. A clear path at coarse steps turned out not to be clear '
            f'when re-walked at fine steps.')


def _yaw_deg(quat_xyzw):
    """The heading of a quaternion about the world vertical, in degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return degrees(atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def base_source_plain(run):
    """Where this run's mobile-base pose came from, in words."""
    source = (run.get('context') or {}).get('base_source') or 'unknown'
    return BASE_SOURCE_PLAIN.get(source, source)


def grasp_span_along_bar(run):
    """How far along the bar's own axis the two grippers hold it.

    The grasps are stored as tool0 poses in the BAR's frame, so their third
    coordinate is the distance from the bar's origin along the bar.

    Args:
        run (dict): the run record.

    Returns:
        tuple[float, float]: ``(near, far)`` in metres from the bar's origin.
    """
    grasps = (run.get('context') or {}).get('grasps') or {}
    along = [float(((grasps.get(key) or {}).get('pos') or [0.0, 0.0, 0.0])[2])
             for key in ('bar_from_left_tool0', 'bar_from_right_tool0')]
    return min(along), max(along)


def bar_axis(quat_xyzw):
    """The bar's own direction -- its local Z -- in the frame the pose is given in."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return (2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y))


def bar_mid_along(run):
    """How far along the bar, from its own origin, the middle of the bar sits.

    From the recorded bar extent when there is one; otherwise the midpoint of
    the two grips, which is the best a run file written before the extent was
    recorded can offer.

    Args:
        run (dict): the run record.

    Returns:
        float: the distance in metres along the bar's own axis.
    """
    extent = (run.get('context') or {}).get('active_bar_extent_local')
    if extent:
        return (float(extent['min'][2]) + float(extent['max'][2])) / 2.0
    near, far = grasp_span_along_bar(run)
    return (near + far) / 2.0


def middle_plain(run):
    """What the plotted point is, in words: the bar's middle, or the grips'.

    A run file written before the bar extent was recorded cannot know where the
    bar's middle is, so the grips' midpoint stands in -- and the sentences say
    so rather than claiming the bar's middle.
    """
    if (run.get('context') or {}).get('active_bar_extent_local'):
        return 'the middle of the bar'
    return 'the midpoint of the two grips'


def bar_middle(run, pos, quat_xyzw):
    """The middle of the bar, for a bar pose stored in the run.

    ! Every bar pose in a run file is the pose of the bar's own FRAME, and that
    ! frame's origin is at one end of the bar. The middle is what a reader
    ! pictures when they look at a bar position, so it is what the page plots
    ! and what these sentences quote.

    Args:
        run (dict): the run record.
        pos (Sequence[float]): the stored bar position (its frame's origin).
        quat_xyzw (Sequence[float]): the stored bar orientation.

    Returns:
        list[float]: the middle of the bar, in the same frame as ``pos``.
    """
    axis = bar_axis(quat_xyzw)
    along = bar_mid_along(run)
    return [float(pos[i]) + axis[i] * along for i in range(3)]


def describe_base(run):
    """Which base pose every position in the run is measured from.

    Args:
        run (dict): the run record.

    Returns:
        list[str]: two lines -- what the frame is, and where that base stood.
    """
    context = run.get('context') or {}
    pose = context.get('world_from_mobile_base') or {}
    pos = pose.get('pos') or [0.0, 0.0, 0.0]
    quat = pose.get('quat_xyzw') or [0.0, 0.0, 0.0, 1.0]
    return [
        "Every position here is in the robot's own base frame -- base_footprint, "
        'on the ground between the wheels: x forward, y left, z up.',
        f'That base stood at x {float(pos[0]):.3f}, y {float(pos[1]):.3f}, '
        f'z {float(pos[2]):.3f} m in the cell, facing {_yaw_deg(quat):.0f} deg, and its '
        f'pose was {base_source_plain(run)}.',
    ]


def describe_bar_shape(run):
    """Where the bar's own origin sits on the bar, and where it is held.

    Every bar pose in the file is the pose of the bar's own frame, whose origin
    is at the END the assembly joint is on -- so a reader who takes a plotted
    dot for the middle of the bar misreads the whole picture by up to a metre.

    Args:
        run (dict): the run record.

    Returns:
        list[str]: zero or one line (one only when the bar extent was recorded).
    """
    extent = (run.get('context') or {}).get('active_bar_extent_local')
    if not extent:
        near, far = grasp_span_along_bar(run)
        return ['This run file predates the bar-extent readout, so every dot below sits '
                f'at the midpoint of the two grips ({100.0 * near:.0f} cm and '
                f'{100.0 * far:.0f} cm along the bar from its own origin) rather than at '
                'the middle of the bar itself, and the stick is the gripped piece only.']
    low, high = float(extent['min'][2]), float(extent['max'][2])
    near, far = grasp_span_along_bar(run)
    from_end_cm = abs(low) * 100.0
    where = ('right at one end of it' if from_end_cm < 1.0
             else f'{from_end_cm:.0f} cm in from one end')
    return [f'{run.get("active_bar")} is {100.0 * (high - low):.0f} cm long, and every bar '
            f'pose in this run is the pose of the bar\'s own frame, whose origin sits '
            f'{where} -- not in the middle. The grippers hold it {100.0 * near:.0f} cm and '
            f'{100.0 * far:.0f} cm from that origin, so every dot below is moved onto the '
            f'MIDDLE of the bar and the stick through it is the bar itself.']


def describe_run(run):
    """Headline sentences for one run.

    Args:
        run (dict): the run record.

    Returns:
        list[str]: lines for the dashboard header, in reading order.
    """
    context = run.get('context') or {}
    goal = context.get('goal') or {}
    profile = run.get('profile') or {}
    result = run.get('result') or {}
    budget = context.get('budget') or {}
    candidates = run.get('candidates') or []
    lines = []

    lines.append(f'Bar {run.get("active_bar")} of {run.get("problem")}, '
                 f'{"all three carry anchors" if run.get("anchor_selection") in (None, "all") else variant_plain(run.get("anchor_selection"))}.')
    lines.append(f'Every position below is relative to the robot\'s mobile base, '
                 f'whose pose was {base_source_plain(run)}.')
    lines.append(f'The goal has {middle_plain(run)} '
                 f'{_where_plain(bar_middle(run, goal.get("bar_pos_mb") or [0, 0, 0], goal.get("bar_quat_mb") or [0, 0, 0, 1]))} '
                 f'of the robot base; the sweep walks the bar backwards from there '
                 f'to every home pose it tries.')
    lines.extend(describe_bar_shape(run))

    lines.extend(describe_goal_probe(run))
    rebranch = float(goal.get('rebranch_max_deg') or 0.0)
    if goal.get('rebranched'):
        cross = goal.get('pairing_cross_distance_deg')
        cross_txt = f' (worst-arm branch distance {cross:.0f} deg)' if cross else ''
        lines.append(f'The goal configuration was moved onto a different IK branch '
                     f'before the sweep, by up to {rebranch:.0f} deg on one joint{cross_txt} '
                     f'-- so M1 no longer ends exactly at the configuration M2 starts from.')

    t_total = float(profile.get('t_total') or 0.0)
    t_found = result.get('t_found_s')
    if result.get('found') and t_found is not None:
        lines.append(f'A usable start was found after {float(t_found):.0f} s, but the sweep '
                     f'kept looking for a fully clear path for the remaining '
                     f'{max(0.0, t_total - float(t_found)):.0f} s of its '
                     f'{float(budget.get("max_time_s") or 0.0):.0f} s budget.')
    elif not result.get('found'):
        lines.append(f'No start was found in {t_total:.0f} s '
                     f'({result.get("failure_reason") or "unknown reason"}).')

    n_ik = int(profile.get('n_ik') or 0)
    t_track = float(profile.get('t_track') or 0.0)
    lines.append(f'{len(candidates)} home poses were tried over '
                 f'{len(context.get("variants_mb") or [])} orientations; '
                 f'{n_ik} IK solves took {t_track:.0f} s of the {t_total:.0f} s.')

    for cut in profile.get('budget_cuts') or []:
        anchor = cut[0] if len(cut) > 0 else '?'
        variant = cut[1] if len(cut) > 1 else '?'
        n_at = f' after {cut[2]} candidates' if len(cut) > 2 else ''
        # The variant already names its anchor, so only the ROTATION part is
        # worth repeating here -- otherwise the line says the carry twice.
        _, _, rotation = str(variant).partition('/')
        where = ('its default orientation' if rotation in ('', 'canonical')
                 else f'the {rotation} orientation')
        lines.append(f'The "{ANCHOR_PLAIN.get(anchor, anchor)}" carry used up its '
                     f'{float(budget.get("per_anchor_s") or 0.0):.0f} s share at '
                     f'{where}{n_at}, so it stopped trying more positions for it.')
    lines.extend(describe_rrt(run))
    return lines


def describe_goal_probe(run):
    """One sentence on whether M2's own goal configuration was kept.

    Args:
        run (dict): the run record.

    Returns:
        list[str]: zero or one line.
    """
    probe = ((run.get('context') or {}).get('goal') or {}).get('branch_probe')
    if not probe:
        return []
    if probe.get('authored_reaches_home'):
        return [f'M2\'s own goal configuration was kept: a test walk from it reached the '
                f'"{variant_plain(probe.get("variant") or "")}" home continuously '
                f'in {float(probe.get("t_s") or 0.0):.1f} s.']
    return [f'M2\'s own goal configuration could not walk to any home pose in '
            f'{float(probe.get("t_s") or 0.0):.0f} s ({probe.get("candidates_walked", 0)} '
            f'tried), so the goal was moved to another IK branch before the sweep.']


def describe_rrt(run):
    """Sentences on the RRT search, when the run includes one.

    Args:
        run (dict): the run record.

    Returns:
        list[str]: zero, one or two lines.
    """
    rrt = run.get('rrt')
    if not rrt:
        return []
    outcome = rrt.get('outcome') or 'failed'
    head = RRT_OUTCOME_PLAIN.get(outcome, outcome)
    if outcome == 'corridor':
        return [f'RRT search: {head}; the plan has {rrt.get("n_waypoints", 0)} waypoints.']
    t = float(rrt.get('time_s') or 0.0)
    line = (f'RRT search: {head} after {t:.0f} s and {rrt.get("iterations") or 0} iterations '
            f'({rrt.get("attempts") or 0} attempt(s)); the start tree grew to '
            f'{rrt.get("n_nodes_start") or 0} nodes and the goal tree to '
            f'{rrt.get("n_nodes_goal") or 0}')
    gap = rrt.get('closest_gap_cm')
    if gap is not None and not rrt.get('path_found'):
        line += f'; at their closest the two trees were {gap:.0f} cm apart'
    line += '.'
    out = [line]
    reasons = rrt.get('stop_reasons') or {}
    total = sum(int(v) for v in reasons.values()) or 0
    if total:
        # Group the extend/connect/stitch variants of each cause together.
        buckets = {}
        for key, count in reasons.items():
            cause = ('collision' if 'collision' in key else 'ik' if 'ik' in key
                     else 'continuity' if 'continuity' in key or 'endpoint' in key
                     else 'reached' if 'reached' in key else key)
            buckets[cause] = buckets.get(cause, 0) + int(count)
        words = {'collision': 'hit something', 'ik': 'found no IK solution',
                 'continuity': 'jumped joint branch', 'reached': 'reached their target'}
        # Buckets that round to nothing (a couple of connects out of a
        # thousand extensions) only add noise to the line.
        parts = [f'{100.0 * n / total:.0f} % {words.get(cause, cause)}'
                 for cause, n in sorted(buckets.items(), key=lambda kv: -kv[1])
                 if 100.0 * n / total >= 0.5]
        out.append('Tree extensions: ' + ', '.join(parts) + '.')
    return out


def run_id(problem, bar_action, anchor_selection, when):
    """Build the run file's id (also its filename stem).

    Args:
        problem (str): design problem name.
        bar_action (str): e.g. ``'B3'``.
        anchor_selection (str | None): anchor label or None for all.
        when (datetime): run timestamp.

    Returns:
        str: e.g. ``'20260923-150412_260716_phase1_test_B3_all'``.
    """
    return (f'{when:%Y%m%d-%H%M%S}_{problem}_{bar_action}_'
            f'{anchor_selection or "all"}')


def runs_dir_default():
    """Default folder the producers write runs into and the server watches."""
    from husky_assembly_teleop import RECORD_DIRECTORY
    return os.path.abspath(os.path.join(RECORD_DIRECTORY, 'm1_derive_runs'))


def scenes_dir_default():
    """Default folder holding one exported ``scene.glb`` per design problem."""
    from husky_assembly_teleop import RECORD_DIRECTORY
    return os.path.abspath(os.path.join(RECORD_DIRECTORY, 'm1_dashboard_scenes'))
