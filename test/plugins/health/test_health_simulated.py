"""The SIM chip: amber while a part announces itself as simulated, absent for a real one."""

from __future__ import annotations

from husky_assembly_teleop.plugins.health import simulated_check
from husky_assembly_teleop.robot_interface.simulated_parts import SimulatedPart
from husky_assembly_teleop.world.checks import WARN


def test_sim_chip_only_for_a_simulated_part():
    """A real part gets no chip; a simulated one an amber SIM chip naming its simulator."""
    assert simulated_check(None) == []
    chip, = simulated_check(SimulatedPart("base", "husky_sim", "base simulator", 10.0))
    assert chip.label == "SIM" and chip.level == WARN and "husky_sim" in chip.detail


def test_a_simulated_tool_gets_a_sim_chip_named_for_it():
    """On the arm's line, after the tool's own chip: the label says which part is simulated."""
    chip, = simulated_check(SimulatedPart("left_gripper", "left_gripper/scaffolding_tool_sim", "scaffolding tool sim",
                                          10.0), "scaffolding_v3 SIM")
    assert chip.label == "scaffolding_v3 SIM" and "scaffolding tool sim" in chip.detail
