"""Thin convenience helpers for BarAction.json files.

The data classes live in `rs_data_structure.bar_action`. compas's
`json_load` reconstructs them faithfully (including nested
`RobotCellState`, `Frame`, `Configuration`, etc.). This module exposes:

- `parse_bar_action(path)`  → BarAssemblyAction
- `list_bar_actions(dir)`   → sorted list of *.json filenames
- `find_movement(action, key)` → (index, movement)
- `movement_type(mv)`       → "constrained-free" | "constrained-linear"
                              | "linear" | "free"
"""

from __future__ import annotations

import os
from typing import Union

from compas.data import json_load

# Importing the movement classes registers their compas dtypes so json_load
# can rebuild them. Concrete class = coordination x motion type:
# Independent vs EndEffectorConstrained (bar held by both arms), Free vs Linear.
from rs_data_structure.bar_action import (
    BarAssemblyAction,
    Movement,
    IndependentDualArmFreeMovement,
    EndEffectorConstrainedDualArmFreeMovement,
    EndEffectorConstrainedDualArmLinearMovement,
    IndependentDualArmLinearMovement,
)


def parse_bar_action(path: str) -> BarAssemblyAction:
    """Load a BarAssemblyAction from a JSON file."""
    obj = json_load(path)
    if not isinstance(obj, BarAssemblyAction):
        raise TypeError(
            f"Expected BarAssemblyAction at {path!r}, got {type(obj).__name__}"
        )
    return obj


def list_bar_actions(action_dir: str) -> list[str]:
    """Return sorted *.json filenames in the BarActions directory."""
    if not os.path.isdir(action_dir):
        return []
    return sorted(f for f in os.listdir(action_dir) if f.endswith(".json"))


def find_movement(action: BarAssemblyAction, key: Union[int, str]) -> tuple[int, Movement]:
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


def movement_type(mv: Movement) -> str:
    """Classify a movement by its concrete class type.

    "constrained-*" means both arms rigidly hold one bar (fixed relative
    tool0_left -> tool0_right transform); "free"/"linear" without the prefix
    means the arms move independently (bar not held, e.g. M0/M3/M4).
    """
    if isinstance(mv, EndEffectorConstrainedDualArmFreeMovement):
        return "constrained-free"       # M1: home -> approach, bar gripped
    if isinstance(mv, EndEffectorConstrainedDualArmLinearMovement):
        return "constrained-linear"     # M2: approach -> mated, bar gripped
    if isinstance(mv, IndependentDualArmLinearMovement):
        return "linear"                 # M3: per-arm linear retreat
    if isinstance(mv, IndependentDualArmFreeMovement):
        return "free"                   # M0/M4: staging / return home
    return "unknown"
