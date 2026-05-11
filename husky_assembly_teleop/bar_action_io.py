"""Thin convenience helpers for BarAction.json files.

The data classes live in `rs_data_structure.bar_action`. compas's
`json_load` reconstructs them faithfully (including nested
`RobotCellState`, `Frame`, `Configuration`, etc.). This module exposes:

- `parse_bar_action(path)`  → BarAssemblyAction
- `list_bar_actions(dir)`   → sorted list of *.json filenames
- `find_movement(action, key)` → (index, movement)
- `movement_type(mv)`       → "constrained" | "linear" | "free"
"""

from __future__ import annotations

import os
from typing import Union

from compas.data import json_load

# Importing rs_data_structure also registers the "core.bar_action" legacy
# dtype alias so old JSONs still load.
from rs_data_structure.bar_action import (
    BarAssemblyAction,
    Movement,
    RoboticDualArmConstrainedMovement,
    RoboticLinearMovement,
    RoboticFreeMovement,
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
    """Classify a movement by its concrete class type."""
    if isinstance(mv, RoboticDualArmConstrainedMovement):
        return "constrained"
    if isinstance(mv, RoboticLinearMovement):
        return "linear"
    if isinstance(mv, RoboticFreeMovement):
        return "free"
    return "unknown"
