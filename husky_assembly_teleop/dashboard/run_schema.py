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
    head = (f'{variant_plain(cand.get("variant"))}, bar at '
            f'{_where_plain(cand.get("home_pos_mb") or [0, 0, 0])} of the base; '
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
    lines.append(f'The goal has the bar {_where_plain(goal.get("bar_pos_mb") or [0, 0, 0])} '
                 f'of the robot base; the sweep walks the bar backwards from there '
                 f'to every home pose it tries.')

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
    return lines


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
