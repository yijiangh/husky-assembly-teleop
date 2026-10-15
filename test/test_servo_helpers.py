"""Pure helpers of the visual-servo loop (husky_world).

husky_world imports the ROS stack, so these tests skip when it is not sourced.
"""

import numpy as np
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


# * --- The insert's spiral search (pure math; the executor is covered by the smoke) ---

def test_lateral_basis_is_orthonormal_and_perpendicular():
    """u, v are unit, mutually perpendicular and perpendicular to the axis, also for a vertical axis."""
    for axis in ([0.0, 0.0, -1.0], [1.0, 2.0, 3.0]):
        u, v = husky_world.lateral_basis(axis)
        a = np.asarray(axis) / np.linalg.norm(axis)
        assert np.linalg.norm(u) == pytest.approx(1.0)
        assert np.linalg.norm(v) == pytest.approx(1.0)
        assert abs(float(np.dot(u, v))) < 1e-12
        assert abs(float(np.dot(u, a))) < 1e-12
        assert abs(float(np.dot(v, a))) < 1e-12
    with pytest.raises(ValueError):
        husky_world.lateral_basis([0.0, 0.0, 0.0])


def test_spiral_search_offset_grows_with_progress():
    """Radius 0 -> max over t_move, always perpendicular to the axis, held (clamped) after t_move."""
    axis = [0.0, 0.0, -1.0]
    spiral = husky_world.SpiralSearch(axis, t_move=20.0, turns=4)
    radius = husky_world.M2_SPIRAL_MAX_RADIUS_MM / 1000.0
    assert np.linalg.norm(spiral.offset(0.0)) == pytest.approx(0.0)
    assert np.linalg.norm(spiral.offset(10.0)) == pytest.approx(0.5 * radius)
    assert np.linalg.norm(spiral.offset(20.0)) == pytest.approx(radius)
    # Every offset lies in the plane perpendicular to the axis.
    for t in np.linspace(0.0, 20.0, 41):
        assert abs(float(np.dot(spiral.offset(t), axis))) < 1e-12
    # Past t_move the offset is held: the clamp IS the hold at the spiral's end.
    assert np.allclose(spiral.offset(40.0), spiral.offset(20.0))
    # 4 whole turns bring the angle back to 0, so the end offset points along u.
    assert np.allclose(spiral.offset(20.0) / radius, spiral.u)
    assert spiral.frac(5.0) == pytest.approx(0.25)
    spiral.offset(19.9)
    assert not spiral.done()
    spiral.offset(20.0)
    assert spiral.done()
    assert '100%' in spiral.text() and '4.0 of 4 turns' in spiral.text()
