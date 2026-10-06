"""Thin convenience helpers for BarAction.json files.

The data classes live in `rs_data_structure.bar_action`. compas's
`json_load` reconstructs them faithfully (including nested
`RobotCellState`, `Frame`, `Configuration`, etc.). This module exposes:

- `parse_bar_action(path)`  → any BarSceneAction (jointing, release, legacy)
- `load_action_cycle(path)` → both halves of one bar's cycle, in order
- `list_bar_actions(dir)`   → sorted list of *.json filenames
- `find_movement(action, key)` → (index, movement)
- `cycle_start_ee_sources(movements, side_keys)` → per movement, which movement
  authored where each tool flange STARTS
- `sibling_action_path(path)` → the release file of a jointing file (and back)
- `clean_action_path` / `sidecar_action_path` / `preferred_action_path` /
  `write_path_for` → the clean export vs its `.live-solved.json` sidecar
- `movement_kind(mv)`       → what KIND of step a movement is (`MovementKind`),
  from its class; `step_kind`, `tool_event`, `default_trajectory_time` build on it
- `COMPLIANT_KINDS`         → the kinds that run under the Cartesian compliance
  controller (Cindy's insert and retreat)
- `is_free_home(action, mv)` → whether a free dual-arm move is the move HOME
  (the other one is the travel out to the loading pose)
- `check_action_kinds(action)` → refuse an action whose transfer / insert /
  retreat is missing or doubled
- `tool_runs_with_next_motion(mv)` → whether a tool step is carried out by the
  arm movement after it (its own button only marks it done)
- `OperatorStep` / `operator_steps(movements)` → the steps the operator walks
  through: each such tool step joins the next arm movement
- `step_index_of(steps, movement_index)` → which step holds a movement
- `bar_body_name` / `find_bar_body` / `is_built_assembly_body` → the built
  bars' rigid-body names, which differ between Cindy's and the support cells
- `held_ground_joints(state)` → the ground joints held in the tools: a GROUNDED
  bar (its insert runs rigid and never tightens)

To know what a movement does, use `movement_kind` (it reads the concrete
Movement subclass): in Cindy's cycle each of the transfer, the insert and the
retreat has its own class, and the one class used twice (the free dual-arm
move) is told apart with `is_free_home`. The old M0..M4 roles live in
`legacy_bar_action_io`, only for reading old takes that were stamped with them.

* Two export schemas exist. The legacy one (up to 260716_phase1_test) writes
* ONE file per bar, `B6.json`, holding M0..M4. The current one
* (260921_motion_sample onwards) splits the cycle into `B6__J.json`
* (BarAssemblyJointingAction: load, mount, grasp, transfer, tighten, insert)
* and `B6__R.json` (BarAssemblyReleaseAction: untighten, ungrasp, retreat,
* home). The helpers here hide that difference.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from enum import Enum
from typing import Container, Optional, Union

from compas.data import json_load

# Importing the movement classes registers their compas dtypes so json_load
# can rebuild them (the package import also registers the support robots'
# hold actions from rs_data_structure.hold_action). Concrete class =
# coordination x motion type: Independent vs EndEffectorConstrained (bar held
# by both arms) vs SingleArm (support robot), Free vs Linear.
import rs_data_structure.bar_action as _bar_action_module
from rs_data_structure.bar_action import (
    BarSceneAction,
    BarAssemblyJointingAction,
    BarAssemblyReleaseAction,
    Movement,
    ManualMovement,
    ToolMovement,
    GripperToolMovement,
    ScaffoldingToolMovement,
    IndependentDualArmFreeMovement,
    EndEffectorConstrainedDualArmFreeMovement,
    EndEffectorConstrainedDualArmLinearMovement,
    IndependentDualArmLinearMovement,
    SingleArmFreeMovement,
    SingleArmLinearMovement,
)

# * The two kinds of step where NO ARM MOVES: the operator mounting the bar by
# * hand, and a tool (the scaffolding screws, a support gripper) acting on its
# * own. Every other movement in a cycle drives the arms.
# ! Do not test this with "the movement has no target_ee_frames": the free
# ! movements M0 and M4 drive both arms and carry no EE targets either (they are
# ! authored as joint-space goals), so an empty target dict says nothing about
# ! whether the robot stood still.
STATIONARY_MOVEMENT_TYPES = (ManualMovement, ToolMovement)


class BarAssemblyAction(BarSceneAction):
    """The legacy single-file export: one action holding M0..M4 for a bar.

    ``rs_data_structure`` dropped this class when it split the cycle into
    ``BarAssemblyJointingAction`` + ``BarAssemblyReleaseAction``, but the older
    design problems (e.g. ``260716_phase1_test``) still name it in their JSON
    ``dtype``. Keeping it here, with the same fields as ``BarSceneAction``,
    keeps those files loadable.
    """


# ! compas rebuilds an object by looking its dtype's class name up on the named
# ! module, so the legacy name has to exist on ``rs_data_structure.bar_action``.
if not hasattr(_bar_action_module, "BarAssemblyAction"):
    _bar_action_module.BarAssemblyAction = BarAssemblyAction

# * Cindy's actions (the assembly robot): the jointing and release halves, and
# * the legacy single file holding both.
# Take the legacy class from the module (not the local one above): that is the
# class compas actually builds, even if a later rs_data_structure ships its own.
CINDY_ACTION_TYPES = (BarAssemblyJointingAction, BarAssemblyReleaseAction,
                      _bar_action_module.BarAssemblyAction)


def parse_bar_action(path: str) -> BarSceneAction:
    """Load a bar action (jointing, release or legacy all-in-one) from JSON."""
    obj = json_load(path)
    if not isinstance(obj, BarSceneAction):
        raise TypeError(
            f"Expected a bar action at {path!r}, got {type(obj).__name__}"
        )
    return obj


def slot_of_index(slots: list, idx: int):
    """Which loaded half holds movement ``idx`` of the concatenated cycle.

    The cycle can span a jointing and a release file, so writing a movement back
    -- or naming it in a saved take -- has to address its own half rather than
    whichever one happens to be first.

    Args:
        slots (list): ``(action, path)`` pairs, as ``load_action_cycle`` returns.
        idx (int): Index into the concatenated movement list.

    Returns:
        tuple | None: ``(action, path, index_within_that_action)``, or None when
        the index is out of range.
    """
    if idx is None or idx < 0:
        return None
    start = 0
    for action, path in slots or []:
        n = len(action.movements)
        if idx < start + n:
            return action, path, idx - start
        start += n
    return None


def cycle_start_ee_sources(movements: list, side_keys: tuple = ("left", "right")) -> list:
    """Per movement, which movement says where each tool flange STARTS.

    A movement's own ``target_ee_frames`` is where it ENDS. Its START pose is the
    authored target of the movement that ran before it, and that is what the
    monitor needs in two places: the servo loop's live-base IK aims at it, and a
    held bar's reference pose is composed from it as ``flange_world x grasp``.
    Neither may fall back to FK on ``start_state.robot_configuration`` -- the
    servo loop overwrites that with the live arm pose, so an FK-derived target
    would drift along with the robot instead of staying where it was authored.

    ! Resolved ONCE for the whole cycle, not per call site. Both call sites used
    ! to read ``movements[idx - 1]``, which the split export breaks: between the
    ! insert and the retreat sit two screw events that author nothing, so the
    ! retreat's list neighbour has no targets at all and the caller gave up.

    Walking the cycle forward, each movement inherits the flange poses left by
    the one before it::

        idx 5  B1_J_M5_LM_insert             authors both targets -> becomes the source
        idx 6  B1_R_M0_tool_untighten_joint  no arm moves         -> source unchanged
        idx 7  B1_R_M1_tool_ungrasp_bar      no arm moves         -> source unchanged
        idx 8  B1_R_M2_LM_retreat            starts at the insert's targets

    ! Carrying the source across the screw events is only legitimate because NO
    ! ARM MOVES in them (``STATIONARY_MOVEMENT_TYPES``), so the flange is still
    ! exactly where the insert left it. That is a fact about the movement's
    ! class, NOT "the movement has no EE targets": M0 and M4 drive both arms and
    ! author no targets either, being joint-space goals. A movement that moves an
    ! arm without authoring a target for it therefore CLEARS that side, so the
    ! caller fails loudly rather than aiming at a pose the flange has left.

    Each side is carried separately, so a movement that authors only one of them
    leaves the other's source untouched.

    Args:
        movements (list): The loaded cycle's movements, in order.
        side_keys (tuple): The robot's ``target_ee_frames`` keys:
            ``('left', 'right')`` for Cindy, ``('arm',)`` for a support robot
            (``RobotSpec.side_keys``).

    Returns:
        list: One ``{side: Movement | None}`` dict per movement (e.g.
        ``{'left': ..., 'right': ...}``), in the same order.
    """
    carried = {side: None for side in side_keys}
    sources = []
    for mv in movements:
        sources.append(dict(carried))  # the poses this movement STARTS from
        targets = getattr(mv, "target_ee_frames", None) or {}
        moves_an_arm = not isinstance(mv, STATIONARY_MOVEMENT_TYPES)
        for side in carried:
            if side in targets:
                carried[side] = mv
            elif moves_an_arm:
                carried[side] = None  # that flange moved with nothing authored
    return sources


def sibling_action_path(path: str) -> Optional[str]:
    """Return the release file that pairs with a jointing file, or vice versa.

    ``.../B6__J.json`` <-> ``.../B6__R.json`` (any sidecar suffix such as
    ``B6__J.solved_keyframe.json`` is kept). Legacy single-file actions have
    no sibling.

    Args:
        path (str): Path of a jointing or release action file.

    Returns:
        Optional[str]: The sibling's path, or None for a legacy file.
    """
    folder, name = os.path.split(path)
    if "__J." in name:
        return os.path.join(folder, name.replace("__J.", "__R.", 1))
    if "__R." in name:
        return os.path.join(folder, name.replace("__R.", "__J.", 1))
    return None


# * ------------------------------------------------ clean export vs sidecar
# The Rhino exporter writes the CLEAN file ``B3__J.json``. Anything the monitor
# solves live is saved next to it as ``B3__J.live-solved.json`` (the sidecar),
# so the clean export is only ever written by the exporter.
LIVE_SOLVED_TAG = "live-solved"


def clean_action_path(path: str) -> str:
    """The plain ``<stem>.json`` behind a sidecar path.

    ``B6__R.live-solved.json`` -> ``B6__R.json``; a path that is already plain
    comes back unchanged.

    Args:
        path (str): Any action path, with or without a sidecar suffix.

    Returns:
        str: The clean export's path.
    """
    folder, name = os.path.split(path)
    return os.path.join(folder, f"{name.split('.', 1)[0]}.json")


# The older private name keeps working (load_action_cycle below uses it).
_clean_action_path = clean_action_path


def sidecar_action_path(path: str, tag: str = LIVE_SOLVED_TAG) -> str:
    """The tagged sidecar next to an action file.

    ``B3__J.json`` and ``B3__J.live-solved.json`` both give
    ``B3__J.live-solved.json``, so calling it twice changes nothing.

    Args:
        path (str): The clean export or any of its sidecars.
        tag (str): The sidecar tag.

    Returns:
        str: ``<folder>/<stem>.<tag>.json``.
    """
    clean = clean_action_path(path)
    return f"{os.path.splitext(clean)[0]}.{tag}.json"


def preferred_action_path(path: str, tag: str = LIVE_SOLVED_TAG) -> str:
    """The file to load for an action: the sidecar when it exists, else the clean export.

    Args:
        path (str): The clean export or any of its sidecars.
        tag (str): The sidecar tag.

    Returns:
        str: The sidecar's path if that file is on disk, else the clean path.
    """
    sidecar = sidecar_action_path(path, tag)
    return sidecar if os.path.isfile(sidecar) else clean_action_path(path)


def write_path_for(path: str) -> str:
    """Which file a loaded action may be written back to (no file access).

    Same rule as ``HuskyMonitor._bar_action_write_path``: a CLEAN export (a
    basename with a single dot, ``B3__J.json``) is never overwritten, the write
    goes to ``B3__J.live-solved.json`` instead; an already tagged file is
    written in place, so repeated saves do not pile up tags.

    Args:
        path (str): The path the action was loaded from.

    Returns:
        str: The path to write to.
    """
    if os.path.basename(path).count(".") > 1:
        return path
    stem, ext = os.path.splitext(path)
    return f"{stem}.{LIVE_SOLVED_TAG}{ext}"


def load_action_cycle(path: str) -> list:
    """Load one bar's WHOLE cycle: both halves of the split export, in order.

    The split export spreads the classic roles over two files -- M0/M1/M2 in the
    jointing half, M3/M4 in the release half -- so a movement can no longer see
    its own neighbours. The bar-holding accuracy test needs exactly that: while
    it works on the retreat (M3, release half) it reads the insert's authored EE
    targets (M2, jointing half) for the reference pose, and copies the insert's
    grasp to re-attach the bar. Loading both halves and reading them as ONE
    ordered list of movements puts every role back within reach.

    Handles every export this repo can open:

    - ``B6.json`` (legacy, one file holding M0..M4) -> one slot, unchanged;
    - ``B6__J.json`` or ``B6__R.json`` -> both halves, jointing first, whichever
      one was picked;
    - ``B6__J.live-solved.json`` -> the matching release sidecar when it exists,
      else the clean ``B6__R.json``. Step A saves only the jointing sidecar (M0
      and M1 both live there), so without that fallback reloading it would leave
      Step B without an M3.

    The halves stay separate objects, each paired with its own path, so anything
    written back goes to the file it came from and no file ends up holding the
    other half's movements.

    Args:
        path (str): The action file the operator selected.

    Returns:
        list: ``(action, path)`` per loaded file, jointing half first. A single
        entry for a legacy action, or when no release half can be found.
    """
    sibling = sibling_action_path(path)
    if sibling is None:
        return [(parse_bar_action(path), path)]  # legacy: one file holds M0..M4

    if not os.path.isfile(sibling):
        # A sidecar's partner may never have been written; the clean export of
        # that half is the right stand-in for the movements it carries.
        clean = _clean_action_path(sibling)
        sibling = clean if os.path.isfile(clean) else sibling

    # Order by half, not by the one the caller happened to pick: the jointing
    # movements always come first in the cycle.
    picked_is_jointing = "__J." in os.path.basename(path)
    ordered = (path, sibling) if picked_is_jointing else (sibling, path)

    slots = []
    for p in ordered:
        if p != path and not os.path.isfile(p):
            print(f"[BarAction] {os.path.basename(p)} is not there; loading "
                  f"{os.path.basename(path)} on its own -- only the roles that "
                  f"half carries will be available.")
            continue
        slots.append((parse_bar_action(p), p))
    return slots


def _natural_key(s: str) -> list:
    """Sort key that orders embedded numbers numerically (B9 < B12 < B81),
    not lexicographically (B12 < B81 < B9)."""
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", s)]


def list_bar_actions(action_dir: str, cycle_only: bool = False) -> list[str]:
    """Return *.json filenames in the BarActions directory, natural-sorted
    by bar number (B1, B2, ... B9, B12, ... B81).

    Args:
        action_dir (str): The design problem's ``BarActions`` folder.
        cycle_only (bool): Drop a release (``__R``) file when its jointing
            (``__J``) partner is there, so one entry stands for one bar's whole
            cycle -- ``load_action_cycle`` opens the pair either way. Both files
            stay on disk. Legacy folders have no ``__R`` files, so nothing is
            dropped there.

    Returns:
        list[str]: Filenames (not paths), natural-sorted.
    """
    if not os.path.isdir(action_dir):
        return []
    names = sorted(
        (f for f in os.listdir(action_dir) if f.endswith(".json")),
        key=_natural_key,
    )
    if cycle_only:
        names = [f for f in names if not _release_half_covered(action_dir, f)]
    return names


def _release_half_covered(action_dir: str, fname: str) -> bool:
    """True when ``fname`` is a release half whose jointing partner is present.

    Args:
        action_dir (str): Folder holding both halves.
        fname (str): A file name from that folder.

    Returns:
        bool: True when listing this file separately would be redundant.
    """
    if "__R." not in fname:
        return False
    partner = sibling_action_path(os.path.join(action_dir, fname))
    return partner is not None and os.path.isfile(partner)


def find_movement(action: BarSceneAction, key: Union[int, str]) -> tuple[int, Movement]:
    """Resolve a movement by integer index OR by movement_id substring/equality.

    Examples:
        find_movement(action, 0)     → first movement
        find_movement(action, "J_M3")  → first movement whose movement_id
                                         contains "_J_M3_" (or equals "J_M3")
        find_movement(action, "B6_R_M2_LM_retreat") → exact-id match
    """
    n = len(action.movements)
    if isinstance(key, int):
        if key < 0 or key >= n:
            raise IndexError(f"movement index {key} out of range [0, {n})")
        return key, action.movements[key]

    if not isinstance(key, str):
        raise TypeError(f"movement key must be int or str, got {type(key).__name__}")

    # Exact match first
    for idx, mv in enumerate(action.movements):
        if mv.movement_id == key:
            return idx, mv

    # Substring match (e.g. "J_M3" → "*_J_M3_*")
    needle = f"_{key}_"
    for idx, mv in enumerate(action.movements):
        if needle in mv.movement_id:
            return idx, mv

    # Fallback: bare substring
    for idx, mv in enumerate(action.movements):
        if key in mv.movement_id:
            return idx, mv

    available = [mv.movement_id for mv in action.movements]
    raise KeyError(f"No movement matches {key!r}. Available: {available}")


# * ------------------------------------------------------------- movement kinds
# What KIND of step a movement is, read from its class (the exporter's
# discriminator). Kinds cover every robot and are what the planner / executor /
# UI dispatch on.
class MovementKind(str, Enum):
    """The kind of step a movement is, one per concrete Movement class."""

    DUAL_FREE = "dual_free"
    DUAL_CONSTRAINED_FREE = "dual_constrained_free"
    DUAL_CONSTRAINED_LINEAR = "dual_constrained_linear"
    DUAL_INDEPENDENT_LINEAR = "dual_independent_linear"
    SINGLE_FREE = "single_free"
    SINGLE_LINEAR = "single_linear"
    GRIPPER_TOOL = "gripper_tool"
    SCAFFOLDING_TOOL = "scaffolding_tool"
    MANUAL = "manual"


# ! Looked up by the EXACT class, so a new Movement subclass fails loudly
# ! (TypeError) instead of being handled like its parent. The bare ToolMovement
# ! base is never exported and has no kind.
_KIND_BY_CLASS = {
    IndependentDualArmFreeMovement: MovementKind.DUAL_FREE,
    EndEffectorConstrainedDualArmFreeMovement: MovementKind.DUAL_CONSTRAINED_FREE,
    EndEffectorConstrainedDualArmLinearMovement: MovementKind.DUAL_CONSTRAINED_LINEAR,
    IndependentDualArmLinearMovement: MovementKind.DUAL_INDEPENDENT_LINEAR,
    SingleArmFreeMovement: MovementKind.SINGLE_FREE,
    SingleArmLinearMovement: MovementKind.SINGLE_LINEAR,
    GripperToolMovement: MovementKind.GRIPPER_TOOL,
    ScaffoldingToolMovement: MovementKind.SCAFFOLDING_TOOL,
    ManualMovement: MovementKind.MANUAL,
}

DUAL_ARM_KINDS = frozenset({
    MovementKind.DUAL_FREE,
    MovementKind.DUAL_CONSTRAINED_FREE,
    MovementKind.DUAL_CONSTRAINED_LINEAR,
    MovementKind.DUAL_INDEPENDENT_LINEAR,
})
SINGLE_ARM_KINDS = frozenset({MovementKind.SINGLE_FREE, MovementKind.SINGLE_LINEAR})
ARM_KINDS = DUAL_ARM_KINDS | SINGLE_ARM_KINDS
# Steps where no arm moves (same idea as STATIONARY_MOVEMENT_TYPES above).
STATIONARY_KINDS = frozenset({
    MovementKind.GRIPPER_TOOL, MovementKind.SCAFFOLDING_TOOL, MovementKind.MANUAL,
})
# * Cindy's insert (tightens the joint screws) and retreat (loosens the
# * grippers): both run under the Cartesian compliance controller.
COMPLIANT_KINDS = frozenset({
    MovementKind.DUAL_CONSTRAINED_LINEAR, MovementKind.DUAL_INDEPENDENT_LINEAR,
})

# * Default duration of a planned arm trajectory, per kind (seconds). 'Load
# * Movement' writes it into the monitor's "traj time" slider, so the slider
# * comes back at a sane value for the movement about to run instead of
# * whatever the previous one left behind.
# The split is by what the path IS, not by how many waypoints it has:
#   free moves, transfer   long articulated paths (a free transit, the
#                          ~270-waypoint constrained bar-loading sweep, the free
#                          return home) -- give them room so the arms move at a
#                          watchable speed.
#   insert, retreat        the ~15 mm linear insert and retreat. Only ~5
#                          waypoints, and the insert keeps holding under
#                          compliance after the nominal duration anyway (until
#                          the joint motor stalls), so a long budget here buys
#                          nothing and just makes the approach crawl.
# The operator can always override on the slider before pressing execute; that
# value is re-read at execution time.
TRAJECTORY_TIME_BY_KIND_S = {
    MovementKind.DUAL_FREE: 30.0,
    MovementKind.DUAL_CONSTRAINED_FREE: 10.0,
    MovementKind.DUAL_CONSTRAINED_LINEAR: 5.0,
    MovementKind.DUAL_INDEPENDENT_LINEAR: 5.0,
    MovementKind.SINGLE_FREE: 15.0,
    MovementKind.SINGLE_LINEAR: 5.0,
}
# Cindy's free move home is shorter than her travel out to the loading pose
# (the 30 s in the table above), so it gets its own default.
FREE_HOME_TRAJECTORY_TIME_S = 10.0


def movement_kind(mv: Movement) -> MovementKind:
    """The kind of step a movement is, from its exact class.

    Args:
        mv (Movement): A loaded movement.

    Returns:
        MovementKind: Its kind.

    Raises:
        TypeError: The class is not one of the known concrete movement classes.
    """
    kind = _KIND_BY_CLASS.get(type(mv))
    if kind is None:
        raise TypeError(f"no MovementKind for {type(mv).__name__} "
                        f"({getattr(mv, 'movement_id', '?')!r})")
    return kind


def kind_fits_robot(kind: MovementKind, dual_arm: bool) -> bool:
    """Whether a robot with one or two arms can run a kind of step.

    Args:
        kind (MovementKind): The step kind.
        dual_arm (bool): True for Cindy.

    Returns:
        bool: Dual-arm motions only fit the dual-arm robot, single-arm motions only
        a single-arm robot; tool and manual steps fit both.
    """
    if kind in DUAL_ARM_KINDS:
        return dual_arm
    if kind in SINGLE_ARM_KINDS:
        return not dual_arm
    return True


def is_free_home(action: BarSceneAction, mv: Movement) -> bool:
    """Whether a ``DUAL_FREE`` movement is the free move HOME (not the travel out to the loading pose).

    Cindy's two free moves share one class, so the action holding them tells
    them apart: the release half travels home, the jointing half travels out to
    the loading pose, and the legacy single file holds both (out first, home
    last).

    Args:
        action (BarSceneAction): The action that holds ``mv``.
        mv (Movement): One of that action's movements.

    Returns:
        bool: True only for the free move home. False for any other kind (an
        unknown class included), and for every movement of a jointing, hold or
        hold-release action.
    """
    if _KIND_BY_CLASS.get(type(mv)) is not MovementKind.DUAL_FREE:
        return False
    if isinstance(action, BarAssemblyReleaseAction):
        return True
    if isinstance(action, _bar_action_module.BarAssemblyAction):
        # * Legacy single file: home is the LAST free move. Compared by identity,
        # * since the movement objects all come from this one loaded action.
        free_moves = [m for m in action.movements
                      if _KIND_BY_CLASS.get(type(m)) is MovementKind.DUAL_FREE]
        return free_moves[-1] is mv
    return False


# * How many movements of each kind Cindy's actions must hold. The monitor finds
# * the transfer, the insert and the retreat by their kind, so each must be there
# * exactly once. The legacy single file also holds both free moves (travel out
# * to the loading pose, then home). Support robots' actions are not checked.
_REQUIRED_KIND_COUNTS = {
    BarAssemblyJointingAction: {
        MovementKind.DUAL_CONSTRAINED_FREE: 1,
        MovementKind.DUAL_CONSTRAINED_LINEAR: 1,
    },
    BarAssemblyReleaseAction: {
        MovementKind.DUAL_INDEPENDENT_LINEAR: 1,
    },
    _bar_action_module.BarAssemblyAction: {
        MovementKind.DUAL_CONSTRAINED_FREE: 1,
        MovementKind.DUAL_CONSTRAINED_LINEAR: 1,
        MovementKind.DUAL_INDEPENDENT_LINEAR: 1,
        MovementKind.DUAL_FREE: 2,
    },
}
# What each checked kind is in Cindy's cycle, for the error message.
_KIND_MEANING = {
    MovementKind.DUAL_FREE: "free move",
    MovementKind.DUAL_CONSTRAINED_FREE: "transfer",
    MovementKind.DUAL_CONSTRAINED_LINEAR: "insert",
    MovementKind.DUAL_INDEPENDENT_LINEAR: "retreat",
}


def check_action_kinds(action: BarSceneAction, source: Optional[str] = None) -> None:
    """Refuse one of Cindy's actions whose transfer, insert or retreat is missing or doubled.

    The monitor finds the transfer, the insert and the retreat by their movement
    kind, so an action holding two of them, or none, would silently make it pick
    the wrong movement. The legacy single file must also hold exactly two free
    moves, so ``is_free_home`` ("the last one") is unambiguous. Other action
    types (a support robot's hold / hold release) are not checked.

    Args:
        action (BarSceneAction): A loaded action.
        source (str | None): How to name the action in the error, e.g. its file
            name. Defaults to ``action.action_id``.

    Raises:
        ValueError: A kind count differs from what the action type needs.
        TypeError: A movement's class has no kind (see ``movement_kind``).
    """
    required = next((counts for cls, counts in _REQUIRED_KIND_COUNTS.items()
                     if isinstance(action, cls)), None)
    if required is None:
        return
    kinds = [movement_kind(mv) for mv in action.movements]
    wrong = [f"{kind.value} ({_KIND_MEANING[kind]}) expected {expected}, found {kinds.count(kind)}"
             for kind, expected in required.items() if kinds.count(kind) != expected]
    if wrong:
        raise ValueError(f"{source or action.action_id}: this {type(action).__name__} has the "
                         f"wrong number of movements: {'; '.join(wrong)}")


def step_kind(mv: Movement) -> str:
    """Which UI action runs a movement: the arm, the gripper, the screws, or the operator.

    Args:
        mv (Movement): A loaded movement.

    Returns:
        str: ``'arm'``, ``'gripper'``, ``'scaffold'`` or ``'manual'``.
    """
    kind = movement_kind(mv)
    if kind in ARM_KINDS:
        return "arm"
    return {
        MovementKind.GRIPPER_TOOL: "gripper",
        MovementKind.SCAFFOLDING_TOOL: "scaffold",
        MovementKind.MANUAL: "manual",
    }[kind]


def tool_event(mv: ToolMovement) -> tuple:
    """What a tool step does.

    Args:
        mv (ToolMovement): A gripper or scaffolding tool movement.

    Returns:
        tuple: ``(tool_action, tool_names, overlaps_next)``, e.g.
        ``('tighten', ['AT3L', 'AT3R'], True)``.
    """
    return mv.tool_action, list(mv.tool_names), bool(mv.overlaps_next)


# * ------------------------------------------------------------ operator steps
# Some of Cindy's screw-tool steps send nothing on their own: the arm movement
# after them does the tool work (or nobody does it on purpose). The operator
# should not have to stop on such a step, so it is folded into that movement.
def tool_runs_with_next_motion(mv: Movement) -> bool:
    """Whether a tool step is carried out by the arm movement after it.

    These are the scaffolding-tool steps whose own button only marks them done:

    - 'tighten' with ``overlaps_next`` (the joint tighten before the insert):
      the compliant insert starts the joint motors TIGHTENING;
    - 'ungrasp' (the release's gripper loosen): the compliant retreat starts the
      gripper motors LOOSENING;
    - 'untighten' (the release's joint loosen): never sent, on purpose. The
      compliant retreat does not reverse the joint motor, since that may back
      the just-tightened screw off the bar; the operator has the manual
      'Loosen Joint' button if the tool must back off.

    Args:
        mv (Movement): A loaded movement.

    Returns:
        bool: True for those steps; False for every other movement (arm, manual,
        gripper, a scaffolding 'grasp', or a class with no kind).
    """
    if _KIND_BY_CLASS.get(type(mv)) is not MovementKind.SCAFFOLDING_TOOL:
        return False
    tool_action, _names, overlaps_next = tool_event(mv)
    return overlaps_next or tool_action in ("ungrasp", "untighten")


@dataclass(frozen=True)
class OperatorStep:
    """One step the operator runs: an arm movement plus the tool steps that run with it.

    Attributes:
        primary (int): Index of the movement the step loads and runs.
        absorbed (tuple): Indices of the tool steps carried out by that movement
            (``tool_runs_with_next_motion``), in order. Empty for most steps.
    """

    primary: int
    absorbed: tuple = ()


def operator_steps(movements: list) -> list[OperatorStep]:
    """Group an action's movements into the steps the operator walks through.

    A tool step that runs with the next motion (``tool_runs_with_next_motion``)
    joins the next arm movement when that movement is compliant (the insert or
    the retreat: ``COMPLIANT_KINDS``, whose exec sends the tool command). Every
    other movement is a step of its own. Any other step met while such tool
    steps are still waiting ends the wait (a manual, gripper or other tool
    step, or an arm movement that is not compliant): the waiting ones become
    steps of their own first, and so do any left over at the end of the action.
    ! So a tool step is never hidden behind a movement that would not send it.

    Example (Cindy's jointing half, by movement index)::

        0 free move, 1 manual mount, 2 grasp, 3 transfer, 4 tighten, 5 insert
        -> [0] [1] [2] [3] [5 (+4)]

    Args:
        movements (list): The loaded action's movements, in order.

    Returns:
        list[OperatorStep]: The steps, in order; every movement index appears
        exactly once (as a primary or absorbed).
    """
    steps = []
    waiting = []
    for i, mv in enumerate(movements):
        if tool_runs_with_next_motion(mv):
            waiting.append(i)
        elif _KIND_BY_CLASS.get(type(mv)) in COMPLIANT_KINDS:
            steps.append(OperatorStep(i, tuple(waiting)))
            waiting = []
        else:
            steps.extend(OperatorStep(j) for j in waiting)
            waiting = []
            steps.append(OperatorStep(i))
    steps.extend(OperatorStep(j) for j in waiting)
    return steps


def step_index_of(steps: list, movement_index: Optional[int]) -> Optional[int]:
    """Which step holds a movement, as its primary or as an absorbed tool step.

    Args:
        steps (list): ``operator_steps`` of the loaded action.
        movement_index (int | None): A movement index of that action.

    Returns:
        int | None: The step's index, or None when no step holds the movement
        (no movement, or an index out of range).
    """
    for k, step in enumerate(steps):
        if movement_index == step.primary or movement_index in step.absorbed:
            return k
    return None


def default_trajectory_time(mv: Movement, free_home: bool = False) -> Optional[float]:
    """Default duration of a movement's planned trajectory.

    Args:
        mv (Movement): A loaded movement.
        free_home (bool): The movement is Cindy's free move home (see
            ``is_free_home``); it then gets ``FREE_HOME_TRAJECTORY_TIME_S``
            instead of the free-move default. Ignored for every other kind.

    Returns:
        float | None: Seconds, or None for a step where no arm moves.

    Raises:
        TypeError: The movement's class has no kind (see ``movement_kind``).
    """
    kind = movement_kind(mv)
    if free_home and kind is MovementKind.DUAL_FREE:
        return FREE_HOME_TRAJECTORY_TIME_S
    return TRAJECTORY_TIME_BY_KIND_S.get(kind)


# * ------------------------------------------------------ rigid-body naming
# The built assembly's rigid bodies: bars 'bar_<id>' and connectors
# 'joint_<id>_male/female' in Cindy's cell, the same with an 'env_' prefix in
# the support robots' cells. Environment obstacles ('obstacle_*') are NOT in
# this list on purpose -- they must stay visible and collision-checked.
BUILT_ASSEMBLY_RB_PREFIXES = ("bar_", "joint_", "env_bar_", "env_joint_")
_SUPPORT_RB_PREFIX = "env_"


def is_built_assembly_body(name: str) -> bool:
    """Whether a rigid body is one of the built assembly's bars or joints.

    Args:
        name (str): Rigid-body name in a RobotCell.

    Returns:
        bool: True for ``bar_*``, ``joint_*``, ``env_bar_*``, ``env_joint_*``.
    """
    return name.startswith(BUILT_ASSEMBLY_RB_PREFIXES)


def is_ground_joint_body(name: str) -> bool:
    """Whether a rigid body is a ground joint (the foot that sets a bar on the floor).

    Args:
        name (str): Rigid-body name in a RobotCell, e.g.
            ``'joint_G1-T20Ground-1_ground'`` or ``'env_joint_G1-T20Ground-0_ground'``.

    Returns:
        bool: True for ``joint_*_ground`` and ``env_joint_*_ground``.
    """
    if name.startswith(_SUPPORT_RB_PREFIX):
        name = name[len(_SUPPORT_RB_PREFIX):]
    return name.startswith("joint_") and name.endswith("_ground")


def held_ground_joints(state) -> list:
    """The ground joints a robot holds in its tools in a cell state.

    * A GROUNDED bar (B1, B5) is grasped on its ground joints, which stand
    * straight on the floor: there is no female joint to mate with, so its insert
    * runs rigid and the joint motors never tighten. A normal bar is grasped on
    * its male joints (``joint_*_male``) instead. No exported field says which bar
    * is grounded, so this reads it from the insert's start state.

    Args:
        state (RobotCellState | None): A cell state, e.g. the insert's start state.

    Returns:
        list: Names of the attached (``attached_to_link`` or ``attached_to_tool``)
        ground joints, sorted; empty for a normal bar or no state.
    """
    rb_states = getattr(state, 'rigid_body_states', None) or {}
    return sorted(name for name, rb in rb_states.items()
                  if is_ground_joint_body(name)
                  and (getattr(rb, 'attached_to_link', None) or getattr(rb, 'attached_to_tool', None)))


def bar_body_name(bar_id: str, rb_prefix: str = "") -> str:
    """Rigid-body name of a bar in a cell.

    Args:
        bar_id (str): e.g. ``'B3'``.
        rb_prefix (str): The cell's prefix (``RobotSpec.rb_prefix``).

    Returns:
        str: e.g. ``'bar_B3'`` or ``'env_bar_B3'``.
    """
    return f"{rb_prefix}bar_{bar_id}"


def find_bar_body(rb_names: Container[str], bar_id: str) -> Optional[str]:
    """The name a bar actually has in a cell, whichever prefix that cell uses.

    Args:
        rb_names (Container[str]): The cell's rigid-body names (a list, set,
            dict keys, ... anything supporting ``in``).
        bar_id (str): e.g. ``'B3'``.

    Returns:
        str | None: ``'bar_<id>'`` or ``'env_bar_<id>'``, whichever is present,
        else None.
    """
    for prefix in ("", _SUPPORT_RB_PREFIX):
        name = bar_body_name(bar_id, prefix)
        if name in rb_names:
            return name
    return None


def bar_id_of_body(name: str) -> Optional[str]:
    """The bar id behind a bar's rigid-body name.

    Args:
        name (str): e.g. ``'bar_B3'`` or ``'env_bar_B3'``.

    Returns:
        str | None: ``'B3'``, or None when the body is not a bar.
    """
    if name.startswith(_SUPPORT_RB_PREFIX):
        name = name[len(_SUPPORT_RB_PREFIX):]
    return name[len("bar_"):] if name.startswith("bar_") else None
