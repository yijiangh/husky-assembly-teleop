"""Simulated parts: one counts while its announcements keep coming, and is real again once they stop."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from husky_assembly_teleop.robot_interface.simulated_parts import SIMULATED_FOR, SimulatedParts

pytestmark = pytest.mark.safety


def _announcement(part: str) -> SimpleNamespace:
    """A crl_husky_msgs/SimulatedPart for `part`."""
    return SimpleNamespace(part=part, node="husky_sim", kind="base simulator")


def test_a_part_is_simulated_while_it_announces_itself():
    """Never heard: real. Heard: simulated, for SIMULATED_FOR seconds after its last announcement."""
    now = [10.0]
    parts = SimulatedParts(lambda: now[0])
    assert parts.get("base") is None and parts.all() == []
    parts.heard(_announcement("base"))
    assert parts.get("base").node == "husky_sim" and [p.part for p in parts.all()] == ["base"]
    now[0] += SIMULATED_FOR
    assert parts.get("base") is not None
    now[0] += 0.01
    assert parts.get("base") is None and parts.all() == [], "a stopped simulator stops counting"


def test_parts_are_kept_apart():
    """An arm's fake hardware says nothing about the base or the other arm."""
    parts = SimulatedParts(lambda: 10.0)
    parts.heard(_announcement("left_ur5e"))
    assert parts.get("left_ur5e") is not None
    assert parts.get("right_ur5e") is None and parts.get("base") is None
