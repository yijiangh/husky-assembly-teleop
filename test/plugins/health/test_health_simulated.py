"""The sim chip: blue while a part announces itself as simulated, absent for a real one; it is no problem."""

from __future__ import annotations

from husky_assembly_teleop.plugins.health import simulated_check
from husky_assembly_teleop.robot_interface.simulated_parts import SimulatedPart
from husky_assembly_teleop.world.checks import INFO


def test_sim_chip_only_for_a_simulated_part():
    """A real part gets no chip; a simulated one a short blue "sim" chip, its simulator in the hover text."""
    assert simulated_check(None) == []
    chip, = simulated_check(SimulatedPart("base", "husky_sim", "base simulator", 10.0))
    assert chip.label == "sim" and chip.level == INFO and "husky_sim" in chip.detail
