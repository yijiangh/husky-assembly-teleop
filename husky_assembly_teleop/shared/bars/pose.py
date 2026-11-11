"""
A bar's pose as a 4x4 matrix: its deviation from the design, the servo's correction, its start and ends.

`bar_gap` is how far a measured bar sits from where it should be (its start, middle and end, its turn and tilt) and
`gap_color` how bad a gap is: the one place the as-built report, the Bars arrows and the frame check get both from.

! The process puts a bar's frame at its MIDDLE; the old monitor put it at the bar START. `bar_start` gives the old
  origin, where the servo's stop test and records are evaluated.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from ...colors import FAIL, NONE, OK, WARN
from ...world.marker_match import transform_points
from scaffolding_assembly_core.geometry import Pose, compose, invert


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

    @property
    def across_mm(self) -> float:
        """Distance of the measured origin from the designed bar axis, mm."""
        return float(np.hypot(self.dpos_bar_mm[0], self.dpos_bar_mm[1]))

    @property
    def along_mm(self) -> float:
        """Offset of the measured origin along the designed bar axis, mm."""
        return float(self.dpos_bar_mm[2])

    @property
    def tilt_deg(self) -> float:
        """Angle between the measured and the designed bar axis, degrees: the rotation without its roll."""
        z_of_measured = Rotation.from_rotvec(np.radians(self.rotvec_bar_deg)).as_matrix()[2, 2]
        return float(np.degrees(np.arccos(np.clip(z_of_measured, -1.0, 1.0))))

    def to_json(self) -> dict:
        """The deviation as plain lists and numbers, ready for ``json.dump``.

        Returns:
            dict: Same fields.
        """
        return {"dpos_mm": self.dpos_mm, "dang_deg": self.dang_deg,
                "dpos_world_mm": np.asarray(self.dpos_world_mm).tolist(),
                "dpos_bar_mm": np.asarray(self.dpos_bar_mm).tolist(),
                "rotvec_bar_deg": np.asarray(self.rotvec_bar_deg).tolist()}


def _pose(T: np.ndarray) -> Pose:
    """A 4x4 transform as a core `Pose`."""
    return Pose.from_matrix(np.asarray(T, dtype=float))


# * ================================================================ the bar's start and ends

def bar_start(T: np.ndarray, length: float) -> np.ndarray:
    """The bar pose with its origin moved from the bar's MIDDLE (the process frame) to its START (the old frame).

    The servo's stop test and records are evaluated here, as the old monitor did: two process-frame poses deviate at
    `bar_start` exactly as the old-frame poses did.

    Args:
        T: 4x4 bar pose, origin at the middle, z along the bar.
        length: Bar length, metres.

    Returns:
        np.ndarray: 4x4 ``T @ Trans(0, 0, -length / 2)``.
    """
    return compose(_pose(T), Pose((0.0, 0.0, -length / 2.0))).matrix()


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


# * ================================================================ how far a bar sits from where it should be

#: A gap within OK_MM is fine (green), within WARN_MM close (amber), beyond it off (red), mm; WARN_MM is also the
#: as-built report's placement scale.
OK_MM = 2.0
WARN_MM = 5.0
#: Where along a bar its gaps are measured: `BarGap`'s rows.
GAP_PLACES = ("start", "middle", "end")


@dataclass(frozen=True)
class BarGap:
    """How far a measured bar sits from where it should be: at its start, middle and end, and how it is turned.

    Attributes:
        design: (3, 3) the start, middle and end of the bar where it should be, world, metres.
        measured: (3, 3) the same points of the measured bar.
        deviation: `pose_deviation(measured, design)`: the turn about each designed axis, the tilt.
    """

    design: np.ndarray
    measured: np.ndarray
    deviation: PoseDeviation

    @property
    def gaps_mm(self) -> np.ndarray:
        """(3, 3) measured minus design at the start, middle and end, world axes, mm."""
        return (self.measured - self.design) * 1e3

    @property
    def distances_mm(self) -> np.ndarray:
        """(3,) how far apart the start, middle and end are, mm."""
        return np.linalg.norm(self.gaps_mm, axis=1)

    @property
    def worst_mm(self) -> float:
        """The largest of the three, mm: always an end's (the middle's gap is their mean)."""
        return float(self.distances_mm.max())

    @property
    def tilt_deg(self) -> float:
        """Angle between the measured and the designed centre line, degrees."""
        return self.deviation.tilt_deg


def bar_gap(T_meas: np.ndarray, T_design: np.ndarray, length: float) -> BarGap:
    """How far a measured bar sits from where it should be.

    Args:
        T_meas: 4x4 measured bar pose (origin at the middle, z along the bar).
        T_design: 4x4 pose it should have.
        length: Bar length, metres.
    """
    half = length / 2.0
    places = np.array([[0.0, 0.0, -half], [0.0, 0.0, 0.0], [0.0, 0.0, half]])
    return BarGap(transform_points(T_design, places), transform_points(T_meas, places),
                  pose_deviation(T_meas, T_design))


def gap_color(gap_mm: float | None) -> str:
    """The colour of a gap: OK, WARN or FAIL by its size (OK_MM, WARN_MM); NONE without one."""
    if gap_mm is None:
        return NONE
    return OK if gap_mm <= OK_MM else WARN if gap_mm <= WARN_MM else FAIL


def bar_slide_roll(slide_m: float, roll_deg: float) -> np.ndarray:
    """A move along and about a bar's own z axis (slide, then roll).

    Args:
        slide_m: Slide along the bar, metres.
        roll_deg: Roll about the bar, degrees.

    Returns:
        np.ndarray: 4x4 transform in the bar frame; ``T_bar @ bar_slide_roll(s, r)`` is the bar slid by ``s`` and
        rolled by ``r``.
    """
    roll = Rotation.from_euler("z", roll_deg, degrees=True).as_quat()
    return Pose((0.0, 0.0, float(slide_m)), tuple(roll)).matrix()


# * ================================================================ deviation and the servo's correction

def pose_deviation(T_meas: np.ndarray, T_design: np.ndarray) -> PoseDeviation:
    """How far a measured bar pose is from its designed pose, in the designed bar's frame.

    Args:
        T_meas: 4x4 measured bar pose in world.
        T_design: 4x4 designed bar pose in world.

    Returns:
        PoseDeviation: ``T_meas = T_design @ T_rel``; ``dpos_bar_mm`` and ``rotvec_bar_deg`` are the translation and
        rotation of ``T_rel``.
    """
    meas, design = _pose(T_meas), _pose(T_design)
    rel = compose(invert(design), meas)
    dpos_world_mm = (np.asarray(meas.position) - np.asarray(design.position)) * 1000.0
    rotvec_bar_deg = np.degrees(Rotation.from_quat(rel.orientation).as_rotvec())
    return PoseDeviation(dpos_mm=float(np.linalg.norm(dpos_world_mm)),
                         dang_deg=float(np.linalg.norm(rotvec_bar_deg)),
                         dpos_world_mm=dpos_world_mm, dpos_bar_mm=np.asarray(rel.position) * 1000.0,
                         rotvec_bar_deg=rotvec_bar_deg)


def bar_servo_correction(T_mocap_bar: np.ndarray, T_design_bar: np.ndarray) -> np.ndarray:
    """The world-frame move that carries the measured bar onto its design pose.

    * The servo applies it to both flanges (``T_corr @ T_flange``): one rigid move for the whole held bar, so the
      grasp actually held is kept.

    Args:
        T_mocap_bar: 4x4 bar pose measured by mocap, in world.
        T_design_bar: 4x4 bar pose the servo aims at, in world.

    Returns:
        np.ndarray: 4x4 ``T_corr = T_design @ inv(T_mocap)``, so ``T_corr @ T_mocap == T_design``.
    """
    return compose(_pose(T_design_bar), invert(_pose(T_mocap_bar))).matrix()


def apply_authored_move(T_start_authored: np.ndarray, T_end_authored: np.ndarray,
                        T_start_actual: np.ndarray) -> np.ndarray:
    """The authored world-frame move of a flange, done from where the flange actually starts.

    * The insert after a mocap-bar servo: the flanges stand where the servo put them (not on the authored start),
      and the insert must still make the authored move (e.g. 15 mm straight down) from there.

    Args:
        T_start_authored: 4x4 authored start pose, in world.
        T_end_authored: 4x4 authored end pose, in world.
        T_start_actual: 4x4 actual start pose, in world.

    Returns:
        np.ndarray: 4x4 ``T_end_authored @ inv(T_start_authored) @ T_start_actual``; equal to ``T_end_authored``
        when the actual start is the authored one.
    """
    move = compose(_pose(T_end_authored), invert(_pose(T_start_authored)))
    return compose(move, _pose(T_start_actual)).matrix()
