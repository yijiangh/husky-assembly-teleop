"""Bars and their markers for the marker-fit tests: a made-up T20 bar, the real B20, random poses, relay samples.

Both bars are in the process frame (origin at the bar's MIDDLE).
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from husky_assembly_teleop.shared.bars.layouts import BarLayout
from husky_assembly_teleop.shared.bars.pose import bar_slide_roll
from husky_assembly_teleop.world.marker_match import Sensed, sensed_from_raw, transform_points

#: Length of B20 and of the made-up T20 bar, metres: the process frame sits L/2 up the bar from the old origin.
BAR_LENGTH = 1.8

#: The real B20 of the 2ModulesDemo: four T20_MoCap halves, two balls each, ball centres in the process bar frame
#: (metres), in `bar_layouts` order (by half id, then label).
B20_MARKERS = (
    ("joints/J20-169_mocap/M1", (0.02727, 0.02214, 0.2453)),
    ("joints/J20-169_mocap/M2", (0.02358, -0.02602, 0.2453)),
    ("joints/J20-21_mocap/M1", (-0.0255, 0.02415, -0.48)),
    ("joints/J20-21_mocap/M2", (-0.0255, -0.02415, -0.48)),
    ("joints/J20-22_mocap/M1", (-0.02735, -0.02203, -0.8151)),
    ("joints/J20-22_mocap/M2", (-0.02348, 0.02612, -0.8151)),
    ("joints/J20-25_mocap/M1", (-0.0255, -0.02415, 0.78)),
    ("joints/J20-25_mocap/M2", (-0.0255, 0.02415, 0.78)),
)

# The T20_MoCap plate: its two balls in the joint block frame (metres).
T20_BLOCK_M1 = (-0.010, 0.02415, 0.0255)
T20_BLOCK_M2 = (-0.010, -0.02415, 0.0255)

#: A sample's ROS time in these checks, seconds.
NOW = 10.0


def t20_design(joint_z: tuple = (0.075, 0.430, 1.135, 1.670), rolls_deg: tuple = (0, 90, 180, 45)) -> np.ndarray:
    """A made-up T20 bar: one T20_MoCap plate per joint, joint frame = slide along z, then roll.

    Args:
        joint_z: Joint positions along the bar from its START, metres.
        rolls_deg: Joint rolls about the bar, degrees.

    Returns:
        np.ndarray: (2 * joints, 3) ball centres in the process bar frame, ``[J1 M1, J1 M2, J2 M1, ...]``.
    """
    block = np.array([T20_BLOCK_M1, T20_BLOCK_M2])
    return np.vstack([transform_points(bar_slide_roll(z - BAR_LENGTH / 2.0, roll), block)
                      for z, roll in zip(joint_z, rolls_deg)])


def b20_design() -> np.ndarray:
    """The real B20 ball centres in the process bar frame, (8, 3) metres."""
    return np.array([point for _name, point in B20_MARKERS])


def b20_layout() -> BarLayout:
    """The real B20 as `bar_layouts` builds it, without the Drive."""
    return BarLayout("bars/B20", tuple(name for name, _point in B20_MARKERS), b20_design(), BAR_LENGTH)


def random_rigid(seed: int) -> np.ndarray:
    """A random bar pose somewhere in a 6 m room, as a 4x4 proper rigid transform."""
    rng = np.random.default_rng(seed)
    q, _r = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1.0
    T = np.eye(4)
    T[:3, :3] = q
    T[:3, 3] = rng.uniform(-3.0, 3.0, 3)
    return T


def small_rigid(offset_mm: float, angle_deg: float, seed: int) -> np.ndarray:
    """A small rigid move: ``offset_mm`` along a random direction, ``angle_deg`` about a random axis."""
    rng = np.random.default_rng(seed)
    axis, direction = rng.normal(size=3), rng.normal(size=3)
    axis, direction = axis / np.linalg.norm(axis), direction / np.linalg.norm(direction)
    # Rodrigues: R = I + sin(a) K + (1 - cos(a)) K^2, K the cross-product matrix of the axis.
    K = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    angle = np.radians(angle_deg)
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * K @ K
    T[:3, 3] = direction * offset_mm / 1000.0
    return T


def raw_markers(points: np.ndarray, *, measured=None, residuals=0.0008, sizes=0.0188, capture_time=NOW):
    """A relay sample of these points with ids ``1..N``, shaped like `RawMarkers` with M7's extra fields.

    Built as a namespace, so the checks hold with or without residuals, sizes and a capture time on the class.

    Args:
        points: (N, 3) marker positions in world, metres.
        measured: Per marker, or None for all measured.
        residuals: Per marker or one for all, metres; None = the relay sent none.
        sizes: Per marker or one for all, metres; None = the relay sent none.
        capture_time: ROS time of the sample, seconds; None = the relay sent none.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    n = len(points)
    return SimpleNamespace(ids=np.arange(1, n + 1), positions=points,
                           measured=np.ones(n, dtype=bool) if measured is None else np.asarray(measured, dtype=bool),
                           residuals=None if residuals is None else np.broadcast_to(residuals, n).astype(float),
                           sizes=None if sizes is None else np.broadcast_to(sizes, n).astype(float),
                           capture_time=capture_time)


def sensed_of(points: np.ndarray, **fields) -> Sensed:
    """These points as a sample from the relay, every marker measured and trusted unless `fields` say otherwise."""
    return sensed_from_raw(raw_markers(points, **fields), NOW)
