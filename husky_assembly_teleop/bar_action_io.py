"""Thin convenience helpers for BarAction.json files.

The data classes live in `rs_data_structure.bar_action`. compas's
`json_load` reconstructs them faithfully (including nested
`RobotCellState`, `Frame`, `Configuration`, etc.). This module exposes:

- `parse_bar_action(path)`  → any BarSceneAction (jointing, release, legacy)
- `load_action_cycle(path)` → both halves of one bar's cycle, in order
- `list_bar_actions(dir)`   → sorted list of *.json filenames
- `find_movement(action, key)` → (index, movement)
- `movement_role(mv)`       → the classic 'M0'..'M4' role of a movement
- `cycle_roles(slots)`      → those roles from the recorded movement CLASSES
- `roles_for_action(action)` → the same for ONE action (all None for a support
  robot's hold / hold-release action, which has no classic role)
- `cycle_start_ee_sources(movements, side_keys)` → per movement, which movement
  authored where each tool flange STARTS
- `sibling_action_path(path)` → the release file of a jointing file (and back)
- `clean_action_path` / `sidecar_action_path` / `preferred_action_path` /
  `write_path_for` → the clean export vs its `.live-solved.json` sidecar
- `movement_kind(mv)`       → what KIND of step a movement is (`MovementKind`),
  from its class; `step_kind`, `movement_controller`, `tool_event`,
  `default_trajectory_time` build on it
- `bar_body_name` / `find_bar_body` / `is_built_assembly_body` → the built
  bars' rigid-body names, which differ between Cindy's and the support cells

To classify a movement's motion type, use `movement_kind` (it reads the concrete
Movement subclass); to know what it does in Cindy's assembly cycle (transfer,
insert, retreat, ...), use `movement_role` / `roles_for_action`.

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
    CONTROLLER_CARTESIAN_COMPLIANT,
    CONTROLLER_JOINT_TRACKING,
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

# * The actions of the assembly robot (Cindy). Only these carry the classic
# * M0..M4 roles; a support robot's hold / hold-release action has none.
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


# * ------------------------------------------------------------- movement roles
# The classic roles name what a movement does in the assembly cycle:
#   M0 free travel to the loading pose      M1 bar-held transfer to the approach
#   M2 linear insertion (bar held)          M3 per-arm linear retreat (bar released)
#   M4 free travel home
# The legacy export tags them as ``<bar>_M<n>_<desc>`` directly. The split
# export tags ``<bar>_J_M<n>_<desc>`` / ``<bar>_R_M<n>_<desc>``, where the
# number is the position INSIDE that file, so it has to be translated: the
# manual mount and the screw-tool movements have no classic role (None).
_LEGACY_ROLE_RE = re.compile(r"_M([0-9])_")
_SPLIT_ROLE_RE = re.compile(r"_([JR])_M([0-9])_")
# ! A support robot's hold (``B3_H_M2_...``) and hold-release (``B3_HR_M1_...``)
# ! ids would otherwise hit the legacy fallback above and read as 'M2' / 'M1',
# ! routing a support movement into Cindy's compliant insert flow.
_HOLD_ROLE_RE = re.compile(r"_HR?_M[0-9]_")
_SPLIT_ROLES = {
    ("J", 0): "M0",  # free travel to the loading pose
    ("J", 3): "M1",  # bar-held transfer to the approach
    ("J", 5): "M2",  # linear insertion
    ("R", 2): "M3",  # linear retreat
    ("R", 3): "M4",  # free travel home
}


def movement_role(mv) -> Optional[str]:
    """Return the classic role 'M0'..'M4' of a movement, or None.

    Works for both export schemas (see the module note). Movements without a
    classic role -- the operator mounting the bar, the screw tools running,
    anything a support robot does in a hold / hold-release action -- return None.

    Args:
        mv: A Movement (only its ``movement_id`` is read).

    Returns:
        Optional[str]: 'M0'..'M4', or None.
    """
    mid = getattr(mv, "movement_id", "") or ""
    if _HOLD_ROLE_RE.search(mid):
        return None  # support robot movement: no classic role
    split = _SPLIT_ROLE_RE.search(mid)
    if split:
        return _SPLIT_ROLES.get((split.group(1), int(split.group(2))))
    legacy = _LEGACY_ROLE_RE.search(mid)
    return f"M{legacy.group(1)}" if legacy else None


# The role a movement's own CLASS pins down, whatever its id says. Each of these
# appears once per cycle. The fourth moving class, IndependentDualArmFreeMovement,
# is used TWICE (travel out and travel home), so it is resolved below from the
# action that owns it instead.
_ROLE_BY_CLASS = {
    EndEffectorConstrainedDualArmFreeMovement: "M1",   # bar-held transfer
    EndEffectorConstrainedDualArmLinearMovement: "M2",  # bar-held linear insert
    IndependentDualArmLinearMovement: "M3",            # per-arm linear retreat
}


def cycle_roles(slots: list) -> list:
    """Classic role of every movement in a loaded cycle, from the recorded classes.

    Preferred over calling ``movement_role`` per movement: the ids number
    movements by their POSITION inside their file, so the split export's
    translation table is only right as long as nothing is ever inserted or
    reordered. The class is what the movement *is*, and both export schemas
    record it in the JSON ``dtype``. Any disagreement with the id is reported,
    so a re-export that changes the naming is noticed rather than silently
    mis-steering the operator.

    Args:
        slots (list): ``(action, path)`` pairs in cycle order, as
            ``load_action_cycle`` returns them.

    Returns:
        list: One role ('M0'..'M4') or None per movement, in the same order as
        the concatenated ``action.movements``. A support robot's action
        (hold / hold-release) contributes None for each of its movements.
    """
    roles, legacy_free_idx = [], []
    for action, _path in slots:
        if not isinstance(action, CINDY_ACTION_TYPES):
            # A support robot's action has no classic roles at all.
            roles.extend([None] * len(action.movements))
            continue
        for mv in action.movements:
            role = _ROLE_BY_CLASS.get(type(mv))
            if role is None and type(mv) is IndependentDualArmFreeMovement:
                # M0 and M4 share this class. The split export separates them by
                # the action holding them (the jointing half travels OUT to the
                # loading pose, the release half travels HOME); the legacy
                # all-in-one file has both, so there they are first and last.
                if isinstance(action, BarAssemblyJointingAction):
                    role = "M0"
                elif isinstance(action, BarAssemblyReleaseAction):
                    role = "M4"
                else:
                    legacy_free_idx.append(len(roles))
            roles.append(role)
    if legacy_free_idx:
        roles[legacy_free_idx[0]] = "M0"
        roles[legacy_free_idx[-1]] = "M4"

    # Cross-check against the ids, only for Cindy's actions (the ones that
    # actually carry roles).
    movements = [(mv, isinstance(action, CINDY_ACTION_TYPES))
                 for action, _ in slots for mv in action.movements]
    for (mv, is_cindy), role in zip(movements, roles):
        if not is_cindy:
            continue
        by_id = movement_role(mv)
        if by_id != role:
            print(f"[BarAction] role mismatch on {mv.movement_id!r}: the "
                  f"{type(mv).__name__} class says {role}, the id says {by_id}. "
                  f"Using {role}; check whether the export's naming changed.")
    return roles


def roles_for_action(action: BarSceneAction) -> list:
    """Classic role of every movement in ONE loaded action.

    The schedule loads one file per entry, so this is ``cycle_roles`` for a
    single action. Only Cindy's actions carry roles; a support robot's hold or
    hold-release action gives None for every movement (and nothing is printed).

    Args:
        action (BarSceneAction): A loaded action of any kind.

    Returns:
        list: One role ('M0'..'M4') or None per movement.
    """
    if isinstance(action, CINDY_ACTION_TYPES):
        return cycle_roles([(action, None)])
    return [None] * len(action.movements)


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
        find_movement(action, "M1")  → first movement whose movement_id
                                       contains "_M1_" (or equals "M1")
        find_movement(action, "B6_M3_LM_retreat") → exact-id match
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

    # Substring match (e.g. "M1" → "*_M1_*")
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
# discriminator). The roles above only exist for Cindy; kinds cover every robot
# and are what the planner / executor / UI dispatch on.
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

# Default duration of a planned arm trajectory, per kind (seconds). For Cindy the
# monitor's role table (MOVEMENT_TRAJECTORY_TIME_S) takes precedence.
TRAJECTORY_TIME_BY_KIND_S = {
    MovementKind.DUAL_FREE: 30.0,
    MovementKind.DUAL_CONSTRAINED_FREE: 10.0,
    MovementKind.DUAL_CONSTRAINED_LINEAR: 5.0,
    MovementKind.DUAL_INDEPENDENT_LINEAR: 5.0,
    MovementKind.SINGLE_FREE: 15.0,
    MovementKind.SINGLE_LINEAR: 5.0,
}

# The controller Cindy's proven flow runs per role: only the insert (M2) is
# compliant. Used to warn when an export's Movement.controller disagrees.
_CINDY_CONTROLLER_BY_ROLE = {"M2": CONTROLLER_CARTESIAN_COMPLIANT}


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


def movement_controller(mv: Movement, role: Optional[str] = None) -> str:
    """The arm controller the export asks for, with a check against Cindy's roles.

    Cindy's M2 (insert) runs compliant and every other role runs joint tracking;
    that flow is proven on hardware and stays role-keyed. When ``role`` is given
    and the exported ``Movement.controller`` says otherwise, one warning is
    printed so a re-export that changes it is noticed.

    Args:
        mv (Movement): A loaded movement.
        role (str | None): Its classic role ('M0'..'M4') for Cindy, else None.

    Returns:
        str: ``mv.controller`` as exported.
    """
    controller = mv.controller
    if role is not None:
        expected = _CINDY_CONTROLLER_BY_ROLE.get(role, CONTROLLER_JOINT_TRACKING)
        if controller != expected:
            print(f"[BarAction] controller mismatch on {mv.movement_id!r}: the "
                  f"export says {controller!r}, role {role} runs {expected!r}.")
    return controller


def tool_event(mv: ToolMovement) -> tuple:
    """What a tool step does.

    Args:
        mv (ToolMovement): A gripper or scaffolding tool movement.

    Returns:
        tuple: ``(tool_action, tool_names, overlaps_next)``, e.g.
        ``('tighten', ['AT3L', 'AT3R'], True)``.
    """
    return mv.tool_action, list(mv.tool_names), bool(mv.overlaps_next)


def default_trajectory_time(mv: Movement, role: Optional[str] = None,
                            role_table: Optional[dict] = None) -> Optional[float]:
    """Default duration of a movement's planned trajectory.

    Args:
        mv (Movement): A loaded movement.
        role (str | None): Its classic role for Cindy, else None.
        role_table (dict | None): Seconds per role (the monitor's
            MOVEMENT_TRAJECTORY_TIME_S). Wins over the per-kind default when
            ``role`` is in it.

    Returns:
        float | None: Seconds, or None for a step where no arm moves.
    """
    if role is not None and role_table and role in role_table:
        return role_table[role]
    return TRAJECTORY_TIME_BY_KIND_S.get(movement_kind(mv))


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
