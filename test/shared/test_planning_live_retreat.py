"""The retreat's goal from the live tool: each flange backed out along its own -z, the export's distance."""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.shared.assembly_planning.movement import ArmPart, MovementSpec
from husky_assembly_teleop.shared.assembly_planning.route_result import (RETREAT_AXIS_WARN, axis_angle,
                                                                         live_line_goal, live_retreat_goals)
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process.types import LineSpec


def tool(direction: tuple, at=(1.0, 2.0, 0.5)) -> Pose:
    """A tool0 whose z axis points along `direction`."""
    z = np.asarray(direction, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.cross([0.0, 0.0, 1.0], z) if abs(z[2]) < 0.9 else np.cross([1.0, 0.0, 0.0], z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return Pose(tuple(at), tuple(Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()))


def test_the_live_goal_backs_out_along_the_tools_own_minus_z():
    """A tool pointing +x backs out 50 mm towards -x, turned the same; the angle to the export's axis reads right."""
    start = tool((1.0, 0.0, 0.0))
    goal = live_line_goal(start, 0.05)
    np.testing.assert_allclose(goal.position, (0.95, 2.0, 0.5), atol=1e-12)
    assert goal.orientation == start.orientation
    assert axis_angle(start, LineSpec((-1.0, 0.0, 0.0), 0.05)) == pytest.approx(0.0, abs=1e-9)
    assert math.degrees(axis_angle(start, LineSpec((-0.9397, 0.342, 0.0), 0.05))) == pytest.approx(20.0, abs=0.01)
    assert math.degrees(RETREAT_AXIS_WARN) == 20.0


def test_live_retreat_goals_say_each_arm_and_warn_past_twenty_degrees():
    """Both flanges get their goal; the line names each arm's axis, angle and gap; a tool turned 25° warns."""
    line = LineSpec((-1.0, 0.0, 0.0), 0.05)
    spec = MovementSpec.__new__(MovementSpec)
    object.__setattr__(spec, "arms", (
        ArmPart("robots/cindy/left_ur_arm_tool0", "left_ur_arm", "left", (), line, None),
        ArmPart("robots/cindy/right_ur_arm_tool0", "right_ur_arm", "right", (), line, None)))
    flanges = {"robots/cindy/left_ur_arm_tool0": tool((1.0, 0.0, 0.0)),
               "robots/cindy/right_ur_arm_tool0": tool((math.cos(math.radians(25)), math.sin(math.radians(25)), 0.0),
                                                       at=(1.0, 3.0, 0.5))}
    ends = {"robots/cindy/left_ur_arm_tool0": Pose((0.949, 2.0, 0.5))}
    goals, text, warnings = live_retreat_goals("B20_R_retreat", spec, ends, flanges)
    np.testing.assert_allclose(goals["robots/cindy/left_ur_arm_tool0"].position, (0.95, 2.0, 0.5), atol=1e-9)
    assert text.startswith("[retreat live] B20_R_retreat: goals = the live tool0 poses moved 50 mm back along their "
                           "own -z (encoders): left along (-1.00, +0.00, +0.00), 0.0 deg from the Rhino axis, 1.0 mm "
                           "from the exported goal; right along (")
    assert text.endswith("; right along (-0.91, -0.42, +0.00), 25.0 deg from the Rhino axis.")
    assert warnings == ["[retreat live] B20_R_retreat: the right tool axis is 25.0 deg from the Rhino retreat axis; "
                        "check the tool before Exec."]
    with pytest.raises(KeyError):
        live_retreat_goals("B20_R_retreat", spec, {}, {})
