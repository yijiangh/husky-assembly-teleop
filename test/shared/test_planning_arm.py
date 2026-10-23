"""Tests for shared.assembly_planning.arm: the one-arm search samples within the URDF's and the UR's limits."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np

from husky_assembly_teleop.robot_interface.arm import UR_JOINT_LIMITS
from husky_assembly_teleop.shared.assembly_planning import arm
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names


def test_the_search_samples_within_the_urdf_and_ur_limits(monkeypatch):
    """A pan of ±π and a wrist of ±4.19 in the URDF bound the samples; a joint without URDF limits keeps the UR's."""
    names = arm_joint_names("ur_arm")
    mirror = SimpleNamespace(state=SimpleNamespace(robot_configuration=dict.fromkeys(names, 0.0)),
                             collisions=lambda: [], search_check=lambda _names: lambda _q: None)
    lower = np.array([-np.pi, -np.inf, -np.pi, -4.19, -4.19, -4.19])
    upper = -lower
    samples = []

    def connect(_start, _goal, _distance, sample, *_args):
        samples.extend(sample() for _ in range(2000))
        return SimpleNamespace(states=None, reason="no path")

    monkeypatch.setattr(arm, "connect", connect)
    found = arm.plan_arm(mirror, "ur_arm", [0.1] * 6, threading.Event(), limits=(lower, upper))
    assert found.path is None and found.reason == "no path"
    samples = np.array(samples)
    bound = np.minimum(upper, UR_JOINT_LIMITS)
    assert np.all(np.abs(samples) <= bound) and np.all(np.abs(samples).max(axis=0) > 0.9 * bound)
