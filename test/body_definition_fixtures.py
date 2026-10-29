"""Rigid body definitions for the tests of reading and comparing them: a marker plate, and the same markers moved."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.robot_interface.mocap_definitions import BodyDefinition

#: A marker layout like a calibration marker plate: five markers, no symmetry, metres from the pivot.
MARKERS = np.array([[0.0, 0.0, 0.0], [0.061, 0.004, 0.0], [0.013, 0.072, 0.002], [-0.044, 0.031, 0.011],
                    [0.027, -0.038, 0.019]]) - [0.01, 0.014, 0.006]


def body(points=MARKERS, streaming_id: int = 1866, name: str = "calibration_tool") -> BodyDefinition:
    """A body with these marker offsets, as float32 like NatNet sends them."""
    points = np.asarray(points, dtype=np.float32).astype(float)
    return BodyDefinition(streaming_id, name, tuple(tuple(xyz) for xyz in points), (0,) * len(points))


def moved(points=MARKERS, shift=(0.0, 0.0, 0.0), turn_degrees=(0.0, 0.0, 0.0), noise: float = 0.0,
          order=None, seed: int = 0) -> np.ndarray:
    """The markers as a new definition sees them: the frame shifted and turned, each marker off by up to `noise`."""
    rotation = Rotation.from_euler("xyz", turn_degrees, degrees=True)
    points = rotation.apply(points) + shift
    points += np.random.default_rng(seed).uniform(-noise, noise, points.shape)
    return points[list(order)] if order is not None else points
