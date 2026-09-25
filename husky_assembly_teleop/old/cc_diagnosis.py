"""Show WHERE a compas_fab collision check failed, inside the monitor's PyBullet window.

When the goal IK gives up, compas_fab only tells us WHICH pairs collided, as text:
``PyBulletClient._check_collision`` computes the contact points with
``pybullet.getClosestPoints`` and then throws them away, raising a
``CollisionCheckError`` that carries a message and a list of compas model objects --
no PyBullet ids, no geometry. This module recovers that geometry and draws it.

Two facts make this possible:

1. The monitor and the cfab planner share ONE PyBullet server. ``husky_monitor.start_pybullet``
   connects (``pp.CLIENT == 0``) and ``cfab_session.CfabSession`` adopts that same client id
   rather than opening its own, so every planner body is a real body in the visible window
   and can be drawn on and recoloured directly.
2. ``PyBulletCheckCollision.check_collision`` already passes human-readable names into every
   ``_check_collision`` call (``'robot_' + link_name``, the tool name, the rigid-body name).
   Wrapping that one method therefore hands us the body id, the link index AND the cfab-side
   name for both sides at once -- the join that ``CollisionCheckError.collision_pairs``
   cannot provide.

! compas_fab is installed non-editable at
  ``/home/su/ros2_ws/venv/lib/python3.10/site-packages/compas_fab``; the ``external/compas_fab``
  submodule is only a mirror and editing it has no runtime effect. Hence the runtime wrapper
  below, which delegates to the original method so only its parameter names are coupled.

Unlike ``pybullet_planning.draw_collision_diagnosis``, which this is modelled on, nothing
here blocks and nothing moves the camera. That function ends in ``wait_for_user`` (a blocking
``input()``) and calls ``set_camera_pose``; either would be unusable from the live monitor,
whose ROS spin and Dear PyGui panel are driven from the same thread.
"""

from contextlib import contextmanager

import numpy as np
import pybullet as p
import pybullet_planning as pp
from compas_fab.backends import CollisionCheckError

# * Highlight colours. Deliberately NOT red/green: the cfab robot and its tools are
# already tinted translucent red by `HuskyMonitor._hide_cfab_robot`, and trajectory
# previews are green, so those two would be invisible against the existing scene.
# Orange / cyan read clearly against both.
CC_HIGHLIGHT_A = [1.0, 0.55, 0.0, 1.0]   # first body of the pair
CC_HIGHLIGHT_B = [0.0, 0.9, 1.0, 1.0]    # second body of the pair
CC_LINE_COLOR = [1.0, 1.0, 0.0]          # penetration line
CC_POINT_COLOR = [0.0, 0.0, 0.0]         # witness-point crosses
CC_TEXT_COLOR = [1.0, 1.0, 0.0]

# ! Layout of a pybullet.getClosestPoints tuple, for readers of `pts` below:
#   0 flag | 1 bodyA | 2 bodyB | 3 linkA | 4 linkB | 5 posOnA | 6 posOnB
#   7 normalOnB | 8 contactDistance (negative == penetrating) | 9 normalForce
_PT_LINK_A = 3
_PT_LINK_B = 4
_PT_POS_A = 5
_PT_POS_B = 6
_PT_DISTANCE = 8


@contextmanager
def capture_closest_points(client):
    """Record the contact geometry that compas_fab's collision check discards.

    Replaces ``client._check_collision`` -- on this one instance, by shadowing the
    bound class method with an instance attribute -- with a wrapper that delegates to
    the original and, whenever the original reports a collision, re-queries
    ``getClosestPoints`` for the same pair and stores the result before re-raising.

    Re-raising the original exception untouched is essential: the caller
    (``PyBulletCheckCollision.check_collision``) relies on it to build its own per-pair
    report, so swallowing it here would silently turn a colliding state into a passing
    one. The extra ``getClosestPoints`` call is paid only for pairs that actually
    collide; pairs that pass go straight through to the original.

    Args:
        client: The compas_fab ``PyBulletClient`` to wrap (``monitor.cfab.client``).

    Yields:
        list: Filled while the context is open, one dict per colliding pair, with keys
            ``name_a`` / ``name_b`` (cfab-side names such as
            ``'robot_left_ur_arm_forearm_link'`` or ``'bar_12'``) and ``pts`` (the raw
            ``getClosestPoints`` tuples for that pair).
    """
    records = []
    # Grab the BOUND original before shadowing it, so the wrapper cannot recurse.
    original = client._check_collision

    def _recording_check_collision(body_1_id, body_1_name, body_2_id, body_2_name,
                                   link_index_1=None, link_index_2=None):
        try:
            original(body_1_id, body_1_name, body_2_id, body_2_name,
                     link_index_1, link_index_2)
        except CollisionCheckError:
            # Same query arguments compas_fab used, so we see exactly the contact
            # set that produced the error (see PyBulletClient._check_collision).
            kwargs = {
                "bodyA": body_1_id,
                "bodyB": body_2_id,
                "distance": 0,
                "physicsClientId": client.client_id,
                "linkIndexA": link_index_1,
                "linkIndexB": link_index_2,
            }
            kwargs = {key: value for key, value in kwargs.items() if value is not None}
            records.append({
                "name_a": body_1_name,
                "name_b": body_2_name,
                "pts": p.getClosestPoints(**kwargs),
            })
            raise

    client._check_collision = _recording_check_collision
    try:
        yield records
    finally:
        # Removing the instance attribute restores the class method underneath, which
        # was never touched. Unconditional, so an escaping exception cannot leak it.
        client.__dict__.pop("_check_collision", None)


def collect_collision_contacts(planner, state, skip_env_collisions: bool = False) -> list:
    """Re-run the collision check on a colliding state and return its contacts.

    Runs ``planner.check_collision`` with ``full_report=True`` -- so every offending
    pair is visited instead of stopping at the first -- inside `capture_closest_points`,
    then swallows the single ``CollisionCheckError`` raised at the very end, once all
    pairs have already been recorded.

    The check pushes ``state`` into PyBullet itself (via ``set_robot_cell_state``), so
    when this returns the bodies in the window stand exactly at the configuration the
    contacts were measured in. That is what makes the drawing line up with what the
    operator sees.

    Args:
        planner: The compas_fab ``PyBulletPlanner`` (``monitor.cfab.planner``).
        state: The colliding ``RobotCellState`` to inspect.
        skip_env_collisions: Mirrors the flag of the same name on the goal IK. When
            True the CC.3 / CC.4 / CC.5 steps (everything involving rigid bodies) are
            skipped, so only robot self-collision and robot-versus-tool are reported.

    Returns:
        list: One record per colliding pair, deepest penetration first. Empty when the
            state turns out to be collision-free after all.
    """
    options = {"verbose": False, "full_report": True}
    if skip_env_collisions:
        options["_skip_cc3"] = True
        options["_skip_cc4"] = True
        options["_skip_cc5"] = True

    with capture_closest_points(planner.client) as records:
        try:
            planner.check_collision(state, options)
        except CollisionCheckError:
            # Expected: every pair has already been recorded by the wrapper.
            pass

    records.sort(key=_penetration_depth, reverse=True)
    return records


def _penetration_depth(record) -> float:
    """Deepest penetration of a captured pair, in metres (positive when overlapping).

    Args:
        record: One entry from `capture_closest_points`.

    Returns:
        float: Depth of the most-penetrating contact point, 0.0 when there are none.
    """
    if not record["pts"]:
        return 0.0
    # contactDistance is negative while the shapes overlap, so the deepest contact is
    # the most negative one.
    return -min(pt[_PT_DISTANCE] for pt in record["pts"])


def _deepest_point(record):
    """The single most-penetrating contact tuple of a pair, or None if it has none."""
    if not record["pts"]:
        return None
    return min(record["pts"], key=lambda pt: pt[_PT_DISTANCE])


def print_collision_contacts(records, header: str = "") -> None:
    """Print every captured pair with its penetration depth and witness points.

    Always runs, GUI or not, so the diagnosis also lands in the log of a headless run
    and survives after the drawing has been cleared.

    Args:
        records: Records from `collect_collision_contacts`.
        header: Optional first line, e.g. which IK attempt is being shown.
    """
    if header:
        print(header)
    if not records:
        print("[cc diag] no colliding pair found (the state checks out clean).")
        return
    print(f"[cc diag] {len(records)} colliding pair(s), deepest first:")
    for i, record in enumerate(records, start=1):
        pt = _deepest_point(record)
        depth_mm = _penetration_depth(record) * 1000.0
        print(f"  {i}. {record['name_a']} <-> {record['name_b']}  "
              f"pen={depth_mm:.2f} mm")
        if pt is not None:
            print(f"       A body {pt[1]} link {pt[_PT_LINK_A]} at "
                  f"({pt[_PT_POS_A][0]:.4f}, {pt[_PT_POS_A][1]:.4f}, {pt[_PT_POS_A][2]:.4f})")
            print(f"       B body {pt[2]} link {pt[_PT_LINK_B]} at "
                  f"({pt[_PT_POS_B][0]:.4f}, {pt[_PT_POS_B][1]:.4f}, {pt[_PT_POS_B][2]:.4f})")


def _cache_link_color(cache: dict, body: int, link: int) -> None:
    """Remember a link's current RGBA before it gets recoloured (first write wins).

    Reads it back from ``p.getVisualShapeData``, whose entries carry the link index at
    position 1 and the RGBA at position 7. Caching whatever is on screen right now --
    rather than assuming a default -- is what lets the restore keep
    ``_hide_cfab_robot``'s translucent red robot and the fully transparent hidden
    bodies. ``pybullet_planning.draw_collision_diagnosis`` gets this wrong: it restores
    everything to opaque grey.

    Args:
        cache: The monitor's ``(body, link) -> RGBA`` cache, mutated in place.
        body: PyBullet body id.
        link: Link index (-1 for a single-shape rigid body's base link).
    """
    key = (body, link)
    if key in cache:
        return
    try:
        for entry in p.getVisualShapeData(body):
            if entry[1] == link:
                cache[key] = list(entry[7])
                return
    except Exception:
        pass
    # Nothing reported for this link; a mid-grey restore is the best we can do.
    cache[key] = [0.7, 0.7, 0.7, 1.0]


def _highlight_link(monitor, body: int, link: int, color) -> list:
    """Recolour one link and outline it, caching its previous colour for the restore.

    The recolour alone is easy to miss when the offending link is buried inside other
    geometry, so this also draws the link's world-axis-aligned bounding box as a
    wireframe -- cheap, made only of debug lines, and it survives the link being
    hidden behind something else.

    Args:
        monitor: The HuskyMonitor holding the colour cache.
        body: PyBullet body id.
        link: Link index.
        color: RGBA to paint the link with.

    Returns:
        list: Debug-item uids for the wireframe box, to be removed on clear.
    """
    _cache_link_color(monitor._cc_diag_orig_colors, body, link)
    try:
        pp.set_color(body, color, link=link)
    except Exception as e:
        print(f"[cc diag] could not recolour body {body} link {link}: {e}")
    try:
        return pp.draw_aabb(pp.get_aabb(body, link=link), color=color[:3], width=2)
    except Exception as e:
        print(f"[cc diag] could not outline body {body} link {link}: {e}")
        return []


def draw_collision_contacts(monitor, records, max_pairs: int = 4) -> int:
    """Draw the captured contacts in the shared PyBullet window, without blocking.

    For each of the ``max_pairs`` deepest pairs this draws a yellow line between the
    two witness points, a black cross at each end, a text label naming both sides with
    the penetration depth, and recolours plus outlines the two colliding links (orange
    for the first side, cyan for the second).

    Nothing here waits for input and the camera is never moved, so it is safe to call
    from the live ROS spin loop. Every debug handle and every recoloured link is
    remembered on the monitor and undone by `clear_collision_diagnosis`.

    Args:
        monitor: The HuskyMonitor that owns the drawing handles and colour cache.
        records: Records from `collect_collision_contacts`, deepest first.
        max_pairs: How many pairs to draw. The rest are only printed, so a badly-posed
            state cannot flood the window with hundreds of lines.

    Returns:
        int: The number of pairs actually drawn.
    """
    handles = monitor._cc_diag_handles
    drawn = 0
    # Batch the drawing so the window repaints once instead of per line.
    with pp.LockRenderer():
        for record in records[:max_pairs]:
            pt = _deepest_point(record)
            if pt is None:
                continue
            pos_a = pt[_PT_POS_A]
            pos_b = pt[_PT_POS_B]
            depth_mm = _penetration_depth(record) * 1000.0

            handles.append(pp.add_line(pos_a, pos_b, color=CC_LINE_COLOR, width=5))
            handles.extend(pp.draw_point(pos_a, size=0.01, color=CC_POINT_COLOR))
            handles.extend(pp.draw_point(pos_b, size=0.01, color=CC_POINT_COLOR))

            # Label just above the midpoint so it does not sit inside the geometry.
            midpoint = (np.asarray(pos_a) + np.asarray(pos_b)) / 2.0 + np.array([0, 0, 0.02])
            handles.append(pp.add_text(
                f"{record['name_a']} <-> {record['name_b']} ({depth_mm:.1f} mm)",
                position=midpoint, color=CC_TEXT_COLOR,
            ))

            # Link indices come from the contact point itself, not from what
            # compas_fab passed in: it checks rigid bodies whole-body (link index
            # None), and only the contact tells us which link actually touched.
            handles.extend(_highlight_link(monitor, pt[1], pt[_PT_LINK_A], CC_HIGHLIGHT_A))
            handles.extend(_highlight_link(monitor, pt[2], pt[_PT_LINK_B], CC_HIGHLIGHT_B))
            drawn += 1

    if len(records) > max_pairs:
        print(f"[cc diag] drew the {max_pairs} deepest pair(s); "
              f"{len(records) - max_pairs} more were printed but not drawn.")
    return drawn


def clear_collision_diagnosis(monitor) -> None:
    """Erase a previous diagnosis: its debug items and its link recolouring.

    Called at the start of every new diagnosis and by the monitor's 'Remove all
    drawing' button. Each removal is wrapped defensively because a body can disappear
    between drawing and clearing (loading another movement rebuilds the cfab rigid
    bodies), and because ``pp.remove_all_debug()`` may already have deleted the debug
    items we hold handles for.

    Args:
        monitor: The HuskyMonitor holding the handles and the colour cache.
    """
    for handle in getattr(monitor, '_cc_diag_handles', []):
        try:
            pp.remove_debug(handle)
        except Exception:
            pass
    monitor._cc_diag_handles = []

    for (body, link), color in getattr(monitor, '_cc_diag_orig_colors', {}).items():
        try:
            pp.set_color(body, color, link=link)
        except Exception:
            pass
    monitor._cc_diag_orig_colors = {}


def visualize_goal_ik_collision(monitor, colliding_state,
                                skip_env_collisions: bool = False,
                                max_pairs: int = 4) -> int:
    """Explain a failed goal IK by showing where its best candidate collides.

    Clears any previous diagnosis, re-runs the collision check on ``colliding_state``
    while capturing the contact geometry, prints every offending pair, and -- when a
    PyBullet GUI is attached -- draws the deepest ones in the shared window. The
    printout happens either way, so this is equally useful from the headless
    ``scripts/movement_collision_inspector.py``.

    A diagnostic must never turn a soft IK failure into a crash (this runs inside the
    servo loop), so the whole body is guarded and any error is reported and swallowed.

    Args:
        monitor: The HuskyMonitor holding the cfab session and the drawing handles.
        colliding_state: The merged goal ``RobotCellState`` whose collision check
            failed (see ``husky_world._solve_bar_action_goal_ik``).
        skip_env_collisions: Mirrors the goal IK flag, so the report covers the same
            CC steps the IK actually ran.
        max_pairs: How many pairs to draw. All of them are printed.

    Returns:
        int: The number of colliding pairs found, or 0 if the diagnosis could not run.
    """
    cfab = getattr(monitor, 'cfab', None)
    if cfab is None or getattr(cfab, 'planner', None) is None:
        print("[cc diag] no cfab planner on the monitor; nothing to diagnose.")
        return 0

    # The stub monitors in scripts/ are built with object.__new__ and never run
    # __init__, so make sure the caches exist before anything writes to them.
    if not hasattr(monitor, '_cc_diag_handles'):
        monitor._cc_diag_handles = []
    if not hasattr(monitor, '_cc_diag_orig_colors'):
        monitor._cc_diag_orig_colors = {}

    try:
        clear_collision_diagnosis(monitor)
        records = collect_collision_contacts(
            cfab.planner, colliding_state, skip_env_collisions=skip_env_collisions,
        )
        print_collision_contacts(records)
        if records and pp.has_gui():
            draw_collision_contacts(monitor, records, max_pairs=max_pairs)
        return len(records)
    except Exception as e:
        print(f"[cc diag] ERROR while diagnosing the collision: {e}")
        return 0
