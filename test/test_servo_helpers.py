"""Pure helpers of the visual-servo loop (husky_world).

husky_world imports the ROS stack, so these tests skip when it is not sourced.
"""

import pytest

husky_world = pytest.importorskip('husky_assembly_teleop.husky_world')


def test_recommended_traj_time_scales_with_travel_and_clamps():
    """Time = travel / SERVO_RECOMMENDED_DEG_PER_S, never under the floor nor over the slider's max."""
    rate = husky_world.SERVO_RECOMMENDED_DEG_PER_S
    floor = husky_world.SERVO_MIN_TRAJ_TIME_S
    # A small correction sits at the floor.
    assert husky_world.recommended_traj_time(0.0, 90) == floor
    assert husky_world.recommended_traj_time(0.5 * floor * rate, 90) == floor
    # In between: proportional to the travel.
    assert husky_world.recommended_traj_time(10.0 * rate, 90) == pytest.approx(10.0)
    # A long path is capped at the slider's upper bound.
    assert husky_world.recommended_traj_time(1000.0 * rate, 90) == 90.0
