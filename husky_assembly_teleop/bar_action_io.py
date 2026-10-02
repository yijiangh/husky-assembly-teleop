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
- `cycle_start_ee_sources(movements)` → per movement, which movement authored
  where each tool flange STARTS
- `sibling_action_path(path)` → the release file of a jointing file (and back)

To classify a movement's motion type, use `isinstance(mv, ...)` against the
concrete Movement subclasses; to know what it does in the assembly cycle
(transfer, insert, retreat, ...), use `movement_role`.

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
from typing import Optional, Union

from compas.data import json_load

# Importing the movement classes registers their compas dtypes so json_load
# can rebuild them. Concrete class = coordination x motion type:
# Independent vs EndEffectorConstrained (bar held by both arms), Free vs Linear.
import rs_data_structure.bar_action as _bar_action_module
from rs_data_structure.bar_action import (
    BarSceneAction,
    BarAssemblyJointingAction,
    BarAssemblyReleaseAction,
    Movement,
    ManualMovement,
    ToolMovement,
    IndependentDualArmFreeMovement,
    EndEffectorConstrainedDualArmFreeMovement,
    EndEffectorConstrainedDualArmLinearMovement,
    IndependentDualArmLinearMovement,
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
    classic role -- the operator mounting the bar, the screw tools running --
    return None.

    Args:
        mv: A Movement (only its ``movement_id`` is read).

    Returns:
        Optional[str]: 'M0'..'M4', or None.
    """
    mid = getattr(mv, "movement_id", "") or ""
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
        the concatenated ``action.movements``.
    """
    roles, legacy_free_idx = [], []
    for action, _path in slots:
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

    movements = [mv for action, _ in slots for mv in action.movements]
    for mv, role in zip(movements, roles):
        by_id = movement_role(mv)
        if by_id != role:
            print(f"[BarAction] role mismatch on {mv.movement_id!r}: the "
                  f"{type(mv).__name__} class says {role}, the id says {by_id}. "
                  f"Using {role}; check whether the export's naming changed.")
    return roles


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


def cycle_start_ee_sources(movements: list) -> list:
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

    Returns:
        list: One ``{'left': Movement | None, 'right': Movement | None}`` per
        movement, in the same order.
    """
    carried = {"left": None, "right": None}
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


def _clean_action_path(path: str) -> str:
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


def resolve_take_movement(bar_action_path: str, key: Union[int, str]) -> tuple:
    """Find the movement a marker take was recorded at, over the bar's whole cycle.

    Takes stamp the classic ROLE ('M3'), which the split export does not put in
    the movement id -- the retreat is ``B6_R_M2_LM_retreat``, so searching the
    ids for ``_M3_`` lands on ``B6_R_M3_free_home``, which is M4. Loading the
    cycle also picks up the other half, so a take that named the jointing file
    for an M3 measurement still resolves.

    Args:
        bar_action_path (str): The action file the take named.
        key (int | str): The stamped ``movement_id``, or a legacy index.

    Returns:
        tuple: ``(index, movement, action, path)`` -- the last two say which
        file the movement was found in.
    """
    slots = load_action_cycle(bar_action_path)
    movements = [mv for action, _p in slots for mv in action.movements]
    roles = cycle_roles(slots)

    if isinstance(key, str) and key in roles:
        idx = roles.index(key)
    elif isinstance(key, int):
        idx = key
    else:
        # An exact id or a substring: ask each half in turn.
        idx, offset = None, 0
        for action, _p in slots:
            try:
                local, _mv = find_movement(action, key)
            except (KeyError, IndexError):
                offset += len(action.movements)
                continue
            idx = offset + local
            break
        if idx is None:
            raise KeyError(f"No movement matches {key!r} in either half of "
                           f"{os.path.basename(bar_action_path)}.")

    action, path, _local = slot_of_index(slots, idx)
    return idx, movements[idx], action, path
