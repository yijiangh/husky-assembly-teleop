"""
A bar's pose as a 4x4 matrix: the rigid fit of points, its deviation from the design, and the servo's correction.

Numpy only, ported from the old monitor's bar_mocap_fit.py (BMF).

! The process puts a bar's frame at its MIDDLE; the old monitor put it at the bar START. `bar_start` gives the old
  origin, where the servo's stop test and records are evaluated (spec D3).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PoseDeviation:
    """How far a measured bar pose is from its designed pose.

    * Everything per axis is in the DESIGNED bar's frame: x and y across the bar, z along it, so "slid 5 mm along
      the bar" reads ``dpos_bar_mm = (0, 0, 5)``.

    Attributes:
        dpos_mm: Distance between the two bar origins, mm.
        dang_deg: Angle of the rotation from designed to measured, degrees.
        dpos_world_mm: Measured minus designed origin, in world, mm.
        dpos_bar_mm: The same offset in the designed bar frame, mm.
        rotvec_bar_deg: The rotation from designed to measured as a rotation vector in the designed bar frame,
            degrees (its length is ``dang_deg``; its z part is the roll about the bar).
    """

    dpos_mm: float
    dang_deg: float
    dpos_world_mm: np.ndarray
    dpos_bar_mm: np.ndarray
    rotvec_bar_deg: np.ndarray

    def to_json(self) -> dict:
        """The deviation as plain lists and numbers, ready for ``json.dump``.

        Returns:
            dict: Same fields.
        """
        return {"dpos_mm": self.dpos_mm, "dang_deg": self.dang_deg,
                "dpos_world_mm": np.asarray(self.dpos_world_mm).tolist(),
                "dpos_bar_mm": np.asarray(self.dpos_bar_mm).tolist(),
                "rotvec_bar_deg": np.asarray(self.rotvec_bar_deg).tolist()}


# * ================================================================ rigid transforms

def kabsch(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """The rigid transform that best moves ``src`` points onto ``dst`` points (least squares), never a mirror.

    SVD of the cross-covariance (BMF:362-387); when the best fit would be a mirror, the weakest singular direction
    is flipped, so the result is always a proper rotation.

    Args:
        src: (K, 3) points, K >= 3.
        dst: (K, 3) points, row k pairs with ``src[k]``.

    Returns:
        np.ndarray: 4x4 ``T`` with ``dst ~= T @ src`` and ``det(T[:3, :3]) = +1``.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
    u, _s, vh = np.linalg.svd((dst - dst_mean).T @ (src - src_mean))
    R = u @ vh
    if np.linalg.det(R) < 0.0:
        # ! A mirror is not a rigid motion: flip the weakest direction instead (BMF:381).
        R -= np.outer(u[:, 2], vh[2, :] * 2.0)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = dst_mean - R @ src_mean
    return T


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to points (BMF:390).

    Args:
        T: 4x4 transform.
        pts: (K, 3) points or a single (3,) point.

    Returns:
        np.ndarray: The moved points, same shape as ``pts``.
    """
    T = np.asarray(T, dtype=float)
    return np.asarray(pts, dtype=float) @ T[:3, :3].T + T[:3, 3]


def bar_start(T: np.ndarray, length: float) -> np.ndarray:
    """The bar pose with its origin moved from the bar's MIDDLE (the process frame) to its START (the old frame).

    The servo's stop test and records are evaluated here, as the old monitor did (spec D3): two process-frame poses
    deviate at `bar_start` exactly as the old-frame poses did.

    Args:
        T: 4x4 bar pose, origin at the middle, z along the bar.
        length: Bar length, metres.

    Returns:
        np.ndarray: 4x4 ``T @ Trans(0, 0, -length / 2)``.
    """
    shift = np.eye(4)
    shift[2, 3] = -length / 2.0
    return np.asarray(T, dtype=float) @ shift


def bar_ends(T: np.ndarray, length: float) -> np.ndarray:
    """The two ends of a bar's centre line in world, for drawing.

    Args:
        T: 4x4 bar pose, origin at the middle, z along the bar.
        length: Bar length, metres.

    Returns:
        np.ndarray: (2, 3) the ends at z = -length / 2 and +length / 2, metres.
    """
    half = length / 2.0
    return transform_points(T, np.array([[0.0, 0.0, -half], [0.0, 0.0, half]]))


def bar_slide_roll(slide_m: float, roll_deg: float) -> np.ndarray:
    """A move along and about a bar's own z axis (slide, then roll) (BMF:417).

    Args:
        slide_m: Slide along the bar, metres.
        roll_deg: Roll about the bar, degrees.

    Returns:
        np.ndarray: 4x4 transform in the bar frame; ``T_bar @ bar_slide_roll(s, r)`` is the bar slid by ``s`` and
        rolled by ``r``.
    """
    angle = np.radians(roll_deg)
    T = np.eye(4)
    T[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    T[2, 3] = slide_m
    return T


def _rotation_vector(R: np.ndarray) -> np.ndarray:
    """Axis times angle (radians) of a rotation matrix, right at every angle (BMF:435).

    Args:
        R: 3x3 rotation matrix.

    Returns:
        np.ndarray: (3,) rotation vector.
    """
    skew = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    two_sin = np.linalg.norm(skew)
    angle = np.arctan2(two_sin / 2.0, (np.trace(R) - 1.0) / 2.0)
    if two_sin > 1e-9:
        return angle * skew / two_sin
    if angle < 1e-6:
        return np.zeros(3)
    # * A half turn: skew vanishes, so read the axis from R = 2 a a^T - I (BMF:450-453).
    B = (R + np.eye(3)) / 2.0
    k = int(np.argmax(np.diag(B)))
    return angle * B[:, k] / np.sqrt(B[k, k])


# * ================================================================ deviation and the servo's correction

def pose_deviation(T_meas: np.ndarray, T_design: np.ndarray) -> PoseDeviation:
    """How far a measured bar pose is from its designed pose, in the designed bar's frame (BMF:457-476).

    Args:
        T_meas: 4x4 measured bar pose in world.
        T_design: 4x4 designed bar pose in world.

    Returns:
        PoseDeviation: ``T_meas = T_design @ T_rel``; ``dpos_bar_mm`` and ``rotvec_bar_deg`` are the translation and
        rotation of ``T_rel``.
    """
    T_meas = np.asarray(T_meas, dtype=float)
    T_design = np.asarray(T_design, dtype=float)
    T_rel = np.linalg.inv(T_design) @ T_meas
    dpos_world_mm = (T_meas[:3, 3] - T_design[:3, 3]) * 1000.0
    rotvec_bar_deg = np.degrees(_rotation_vector(T_rel[:3, :3]))
    return PoseDeviation(dpos_mm=float(np.linalg.norm(dpos_world_mm)),
                         dang_deg=float(np.linalg.norm(rotvec_bar_deg)),
                         dpos_world_mm=dpos_world_mm, dpos_bar_mm=T_rel[:3, 3] * 1000.0,
                         rotvec_bar_deg=rotvec_bar_deg)


def bar_servo_correction(T_mocap_bar: np.ndarray, T_design_bar: np.ndarray) -> np.ndarray:
    """The world-frame move that carries the measured bar onto its design pose (BMF:479-492).

    * The servo applies it to both flanges (``T_corr @ T_flange``): one rigid move for the whole held bar, so the
      grasp actually held is kept.

    Args:
        T_mocap_bar: 4x4 bar pose measured by MoCap, in world.
        T_design_bar: 4x4 bar pose the servo aims at, in world.

    Returns:
        np.ndarray: 4x4 ``T_corr = T_design @ inv(T_mocap)``, so ``T_corr @ T_mocap == T_design``.
    """
    return np.asarray(T_design_bar, dtype=float) @ np.linalg.inv(np.asarray(T_mocap_bar, dtype=float))


def apply_authored_move(T_start_authored: np.ndarray, T_end_authored: np.ndarray,
                        T_start_actual: np.ndarray) -> np.ndarray:
    """The authored world-frame move of a flange, done from where the flange actually starts (BMF:495-513).

    * The insert after a MoCap-bar servo: the flanges stand where the servo put them (not on the authored start),
      and the insert must still make the authored move (e.g. 15 mm straight down) from there.

    Args:
        T_start_authored: 4x4 authored start pose, in world.
        T_end_authored: 4x4 authored end pose, in world.
        T_start_actual: 4x4 actual start pose, in world.

    Returns:
        np.ndarray: 4x4 ``T_end_authored @ inv(T_start_authored) @ T_start_actual``; equal to ``T_end_authored``
        when the actual start is the authored one.
    """
    T_move = np.asarray(T_end_authored, dtype=float) @ np.linalg.inv(np.asarray(T_start_authored, dtype=float))
    return T_move @ np.asarray(T_start_actual, dtype=float)
