"""Thin convenience helpers for BarAction.json files.

The data classes live in `rs_data_structure.bar_action`. compas's
`json_load` reconstructs them faithfully (including nested
`RobotCellState`, `Frame`, `Configuration`, etc.). This module exposes:

- `parse_bar_action(path)`  → any BarSceneAction (jointing, release, legacy)
- `list_bar_actions(dir)`   → sorted list of *.json filenames
- `find_movement(action, key)` → (index, movement)
- `movement_role(mv)`       → the classic 'M0'..'M4' role of a movement
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
    IndependentDualArmFreeMovement,
    EndEffectorConstrainedDualArmFreeMovement,
    EndEffectorConstrainedDualArmLinearMovement,
    IndependentDualArmLinearMovement,
)


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


def _natural_key(s: str) -> list:
    """Sort key that orders embedded numbers numerically (B9 < B12 < B81),
    not lexicographically (B12 < B81 < B9)."""
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", s)]


def list_bar_actions(action_dir: str) -> list[str]:
    """Return *.json filenames in the BarActions directory, natural-sorted
    by bar number (B1, B2, ... B9, B12, ... B81)."""
    if not os.path.isdir(action_dir):
        return []
    return sorted(
        (f for f in os.listdir(action_dir) if f.endswith(".json")),
        key=_natural_key,
    )


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
