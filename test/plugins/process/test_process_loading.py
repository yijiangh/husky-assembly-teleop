"""Loading a process: shoulder lifts written a whole turn off come back inside their window, with a warning each."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from process_fixtures import build_process

from husky_assembly_teleop.plugins.process import loading
from husky_assembly_teleop.plugins.process.loading import load_process
from scaffolding_assembly_core.legacy.conversion import PROCESS_FILE

LIFT = "left_ur_arm_shoulder_lift_joint"


def test_a_lift_a_turn_off_is_moved_in_on_load(tmp_path, monkeypatch):
    """A target's lift of 328.3° loads as -31.7°, the same pose but reachable, and says so once."""
    process = build_process(tmp_path)
    action, index, movement = next((action, index, movement) for action in process.actions.values()
                                   for index, movement in enumerate(action.movements)
                                   if movement.target is not None and movement.target.joints)
    robot = next(iter(movement.target.joints))
    part = movement.target.robots[robot]
    target = replace(movement.target, robots={**movement.target.robots,
                                              robot: replace(part, joints={**part.joints,
                                                                           LIFT: float(np.radians(328.3))})})
    movements = list(action.movements)
    movements[index] = replace(movement, target=target)
    written = replace(process, actions={**process.actions, action.id: replace(action, movements=tuple(movements))})
    folder = tmp_path / "process"
    folder.mkdir()
    (folder / PROCESS_FILE).write_text("{}")
    # ? Read is skipped: the fake robot's URDF has no UR joints, so the file checks would refuse the lift.
    monkeypatch.setattr(loading, "read", lambda _folder: written)

    warnings = []
    loaded = load_process(folder, tmp_path, warn=warnings.append)
    lift = loaded.actions[action.id].movements[index].target.robots[robot].joints[LIFT]
    assert np.degrees(lift) == pytest.approx(-31.7)
    assert len(warnings) == 1 and warnings[0].endswith(f"{movement.id}: {robot} {LIFT} 328.3° -> -31.7°")

    # * A process with every lift in range loads as it was read, without a word.
    monkeypatch.setattr(loading, "read", lambda _folder: process)
    warnings.clear()
    assert load_process(folder, tmp_path, warn=warnings.append) is process and not warnings
