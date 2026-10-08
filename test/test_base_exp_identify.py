"""The response identification recovers a known robot model from a simulated run."""

import math

import pytest
from base_exp_fixtures import model_run

from husky_assembly_teleop.plugins.base_exp.identify import identify


def test_recovers_a_known_model():
    """Delay, efficiencies and the turning-centre offset come back within a few percent."""
    found = identify([model_run(), model_run(seed=1)])
    assert found.delay == pytest.approx(0.2, abs=0.02)
    assert found.speed_efficiency == pytest.approx(0.9, rel=0.05)
    assert found.steering_efficiency == pytest.approx(0.95, rel=0.05)
    assert found.x_icr == pytest.approx(-0.1, abs=0.01)
    assert found.clock == "stamps" and min(found.fit.values()) > 0.9


def test_ideal_robot_has_no_offset_and_no_delay():
    """The robot the controllers assume: no delay, gains of one, no offset."""
    found = identify([model_run(delay=0.0, speed=1.0, steering=1.0, x_icr=0.0)])
    assert found.delay == pytest.approx(0.0, abs=0.02)
    assert found.speed_efficiency == pytest.approx(1.0, rel=0.02) and found.x_icr == pytest.approx(0.0, abs=0.005)


def test_unexcited_values_stay_unknown():
    """Driving straight says nothing about turning: those values are NaN."""
    run = model_run(stamps=False)
    run["follower_command"][:, 1] = 0.0
    found = identify([run])
    assert math.isnan(found.x_icr) and math.isnan(found.steering_efficiency)
    assert not math.isnan(found.speed_efficiency) and found.clock == "tick"


def test_wheels_split_the_delay():
    """With wheel odometry, the delay splits into command to wheels and wheels to motion."""
    runs = [model_run(wheel_delay=0.1), model_run(seed=1, wheel_delay=0.1)]
    to_wheels = identify(runs, "commands → wheels")
    to_motion = identify(runs, "wheels → motion")
    assert to_wheels.delay == pytest.approx(0.1, abs=0.02)
    assert to_wheels.speed_efficiency == pytest.approx(1.0, rel=0.02)
    assert math.isnan(to_wheels.x_icr)
    assert to_motion.delay == pytest.approx(0.1, abs=0.02) and to_motion.x_icr == pytest.approx(-0.1, abs=0.01)


def test_no_wheels_no_wheel_response():
    """Runs without wheel odometry (the simulator) identify nothing from the wheels."""
    assert identify([model_run()], "commands → wheels") is None
