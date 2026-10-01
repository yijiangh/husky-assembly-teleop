"""The old M0..M4 roles, kept only for offline tools and their tests.

Offline tools (the bar-holding accuracy analysis) use them to read takes that
were stamped with a role (e.g. 'M3'). The monitor must not import this module:
it finds the transfer, the insert and the retreat by their movement kind
(``bar_action_io.movement_kind``) and the free move home with
``bar_action_io.is_free_home``.

The two export schemas these roles have to read (the legacy single file
``B6.json`` and the split ``B6__J.json`` / ``B6__R.json``) are described in
``bar_action_io``'s module note.
"""

from __future__ import annotations

import re
from typing import Optional

from rs_data_structure.bar_action import (
    BarSceneAction,
    BarAssemblyJointingAction,
    BarAssemblyReleaseAction,
    EndEffectorConstrainedDualArmFreeMovement,
    EndEffectorConstrainedDualArmLinearMovement,
    IndependentDualArmLinearMovement,
    IndependentDualArmFreeMovement,
)

# Importing bar_action_io also registers the legacy BarAssemblyAction dtype, so
# json_load can rebuild the old single-file exports.
from husky_assembly_teleop.bar_action_io import CINDY_ACTION_TYPES


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
        # ! A split id numbers the movement by its position INSIDE its file, so a
        # ! re-export that adds or drops a tool step shifts every number after
        # ! it. The class is right either way; comparing it with such an id
        # ! would only print noise, so only the legacy ids are cross-checked.
        if _SPLIT_ROLE_RE.search(mv.movement_id):
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
