"""Measure a scaffold bar's 6-DoF pose from the MoCap markers on its joints (numpy + stdlib only).

A ``T20_MoCap`` joint is a T20 female joint with a plate carrying two OptiTrack
balls, ``M1`` and ``M2``, 48.3 mm apart. Several of them sit on one bar, so the
bar's full pose can be read from Motive. The Rhino export writes, next to
``RobotCell.json``, a ``MoCapMarkers.json`` (schema 2) holding every ball centre
in its bar's body frame (``bar_m``, metres)::

    {"schema_version": 2, "units": "m",
     "marker_blocks": {"T20_MoCap": {"marker_diameter_m": 0.019,
                                     "points_block_m": {"M1": [...], "M2": [...]}}},
     "bars": {"bar_B20": {"bar_id": "B20", "assembly_seq": 1,
                          "markers": [{"name": "J20-22_mocap_M1", "joint": "joint_J20-22_mocap",
                                       "label": "M1", "block": "T20_MoCap",
                                       "bar_m": [-0.02735, -0.02203, 0.0849]}, ...]}},
     "bars_without_markers": ["bar_B22", ...]}

``bar_m`` is in the same frame as ``rigid_body_states['bar_B20'].frame`` of a
BarAction state, so the measured ``T_world_from_bar`` compares directly with it.

What is here:

- `load_mocap_markers(problem_root)` -> the design marker table (`MoCapMarkerSet`)
- `match_and_fit(sensed, design)` -> `BarFit`: which sensed marker is which
  design marker, WITHOUT Motive's marker ids (they shift), and the bar pose
- `fit_bar_from_cache(cache, design)` -> the same from the monitor's marker
  cache ``{id: {'pos', 'size', 'error'}}`` (drops bad markers and old snapshots);
  with ``T_hint`` (the last pose, following live) it first tries a quick
  nearest-ball match around that pose, and runs `match_and_fit` only when the
  bar moved too far for that
- `pose_deviation(T_meas, T_design)` -> how far the measured bar is from its
  designed pose, in the designed bar's frame (x, y across the bar, z along it)
- `bar_servo_correction(T_mocap, T_design)`, `apply_authored_move(...)` -> the
  servo loop's correction on the MoCap bar, and the insert carried from it
- `fake_sensed_markers(...)` -> a made-up marker snapshot for a desk rehearsal
- `kabsch`, `transform_points`, `design_bar_axis_endpoints`, `bar_slide_roll`
  -> small geometry helpers

! No ROS, no pybullet, no compas, no scipy, so this file is unit-tested on any
! machine. ``pp.superimposition_matrix`` would pull pybullet in through the
! package, so `kabsch` is our own (same SVD and mirror fix).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from itertools import combinations
from json import load as json_load_std
from math import ceil
from typing import Optional, Union

import numpy as np

MOCAP_MARKERS_FILENAME = 'MoCapMarkers.json'
SCHEMA_VERSION = 2

# * A design triangle used to start a match must be at least this high (its
# * smallest altitude). Balls with the same label on joints with the same roll
# * lie on one line along the bar, and three points on a line leave the turn
# * about that line open. A pair plus any far ball is at least 25 mm high.
MIN_TRIANGLE_HEIGHT_M = 0.015

# * A sensed and a placed design marker closer than this match (4 mm: ten times
# * below the 48 mm pair spacing, above Motive's ~0.8 mm noise).
MATCH_TOL_M = 0.004
# * Following live: a sensed marker this close to a design marker placed at the
# * last pose is taken as that marker. 20 mm is well under half the 48 mm pair
# * spacing, so a ball can never be taken for its neighbour.
TRACK_GATE_M = 0.02

# * Motive markers we do not trust: a fit residual above 3 mm, or a ball size
# * far from the 19 mm balls on the plates (a reflection, a merged blob).
MAX_MARKER_ERROR_M = 0.003
# Motive's 'occluded' marker flag: the marker was not seen, its position is
# Motive's guess from the asset's own model (not a measurement).
MARKER_OCCLUDED_FLAG = 0x01
MARKER_SIZE_RANGE_M = (0.012, 0.026)

# What a fake marker reports as its Motive size and residual.
_FAKE_MARKER_SIZE_M = 0.019
_FAKE_MARKER_ERROR_M = 0.0005
# How far the fake stray marker sits from the bar.
_FAKE_STRAY_DISTANCE_M = 0.3


# * ================================================================ data classes

class MoCapMarkersError(RuntimeError):
    """``MoCapMarkers.json`` is missing or cannot be used; the message names the file."""


@dataclass(frozen=True)
class DesignMarker:
    """One designed marker ball on a bar.

    Attributes:
        name (str): The name the Motive marker should carry, e.g. ``'J20-21_mocap_M1'``.
        joint (str): Rigid-body name of the joint it sits on, e.g. ``'joint_J20-21_mocap'``.
        label (str): The ball's label on its plate, ``'M1'`` or ``'M2'``.
        bar_m (tuple): Ball centre ``(x, y, z)`` in the bar body frame, metres.
        block (str): The joint block, a key of ``MoCapMarkerSet.blocks``,
            e.g. ``'T20_MoCap'``; with ``label`` it finds the ball in the block frame.
    """

    name: str
    joint: str
    label: str
    bar_m: tuple
    block: str = ''


@dataclass
class BarMarkers:
    """The designed markers of one bar.

    Attributes:
        bar_name (str): The bar's rigid-body name, e.g. ``'bar_B20'``.
        assembly_seq (int | None): Its place in the assembly order, as exported.
        markers (list): One `DesignMarker` per ball, in file order.
    """

    bar_name: str
    assembly_seq: Optional[int]
    markers: list

    def points(self) -> np.ndarray:
        """The ball centres in the bar frame.

        Returns:
            np.ndarray: (M, 3) metres, in marker order.
        """
        return np.array([m.bar_m for m in self.markers], dtype=float)

    def names(self) -> list:
        """The ball names, in marker order.

        Returns:
            list[str]: e.g. ``['J20-22_mocap_M1', 'J20-22_mocap_M2', ...]``.
        """
        return [m.name for m in self.markers]


@dataclass
class MoCapMarkerSet:
    """A loaded ``MoCapMarkers.json``.

    Attributes:
        schema_version (int): Always 2 (other versions are refused).
        source (str): The Rhino file it was exported from.
        units (str): Always ``'m'``.
        bar_frame_note (str): The file's own note on the bar frame.
        blocks (dict): The file's ``marker_blocks``: ``{block: {'marker_diameter_m',
            'points_block_m': {label: [x, y, z]}}}``, metres in the block frame.
        bars (dict): ``{bar body name: BarMarkers}``, only bars that carry markers.
        bars_without_markers (list): The other bar bodies of the same export.
        path (str): The file's full path.
        warnings (list): Plain-English problems that do not stop the load.
    """

    schema_version: int
    source: str
    units: str
    bar_frame_note: str
    blocks: dict
    bars: dict
    bars_without_markers: list
    path: str
    warnings: list = field(default_factory=list)


@dataclass
class BarFit:
    """The result of matching sensed markers to one bar's design markers.

    Index conventions: a "sensed index" is a row of the sensed points handed to
    `match_and_fit`; a "design index" is a row of the design points.

    Attributes:
        ok (bool): True when enough markers matched and no other assignment fits as well.
        reason (str): Why the fit is not ok, in plain English; empty when ok.
        bar_name (str): The bar's body name, when known.
        T_world_from_bar (np.ndarray | None): 4x4 pose of the bar frame in world
            (always a proper rotation, never a mirror). The best pose found even
            when not ok; None when no hypothesis was found at all.
        correspondences (dict): ``{sensed index: design index}`` of the matched markers.
        sensed_ids (list): Sensed index -> the marker id it came from (the cache
            key in `fit_bar_from_cache`, else simply ``0..N-1``).
        residuals_m (dict): ``{sensed index: distance in metres}`` between each
            matched sensed marker and its design marker placed at the fitted pose.
        rms_m (float | None): Root mean square of ``residuals_m``.
        max_residual_m (float | None): Largest of ``residuals_m``.
        n_matched (int): How many markers matched.
        n_design (int): How many design markers the bar has.
        unmatched_sensed (list): Sensed indices that match no design marker (strays,
            another bar's markers).
        missing_design (list): Design indices that no sensed marker matched
            (occluded balls).
        ambiguous (bool): A different marker assignment fits equally well.
        second_best (dict | None): The best assignment that disagrees with the
            winner: ``{'n_matched', 'rms_m', 'max_residual_m', 'correspondences'}``;
            None when there is none.
        n_hypotheses (int): How many (design triangle, sensed triple) guesses were tried.
        tracked (bool): True when the fit came from the quick match around the
            last pose (``T_hint`` of `fit_bar_from_cache`) instead of the full
            search; such a fit skips the ambiguity check (``ambiguous`` False,
            ``second_best`` None, ``n_hypotheses`` 0).
    """

    ok: bool
    reason: str
    bar_name: str = ''
    T_world_from_bar: Optional[np.ndarray] = None
    correspondences: dict = field(default_factory=dict)
    sensed_ids: list = field(default_factory=list)
    residuals_m: dict = field(default_factory=dict)
    rms_m: Optional[float] = None
    max_residual_m: Optional[float] = None
    n_matched: int = 0
    n_design: int = 0
    unmatched_sensed: list = field(default_factory=list)
    missing_design: list = field(default_factory=list)
    ambiguous: bool = False
    second_best: Optional[dict] = None
    n_hypotheses: int = 0
    tracked: bool = False

    def to_json(self) -> dict:
        """The fit as plain lists, dicts and numbers, ready for ``json.dump``.

        Returns:
            dict: Same fields; the pose as a nested 4x4 list (or None).
        """
        T = self.T_world_from_bar
        return {
            'ok': self.ok, 'reason': self.reason, 'bar_name': self.bar_name,
            'T_world_from_bar': None if T is None else np.asarray(T).tolist(),
            'correspondences': dict(self.correspondences),
            'sensed_ids': list(self.sensed_ids),
            'residuals_m': dict(self.residuals_m),
            'rms_m': self.rms_m, 'max_residual_m': self.max_residual_m,
            'n_matched': self.n_matched, 'n_design': self.n_design,
            'unmatched_sensed': list(self.unmatched_sensed),
            'missing_design': list(self.missing_design),
            'ambiguous': self.ambiguous, 'second_best': self.second_best,
            'n_hypotheses': self.n_hypotheses, 'tracked': self.tracked,
        }


@dataclass
class PoseDeviation:
    """How far a measured bar pose is from its designed pose.

    * Everything per-axis is in the DESIGNED bar's frame: x and y across the
    * bar, z along it, so "slid 5 mm along the bar" reads ``dpos_bar_mm = (0, 0, 5)``.

    Attributes:
        dpos_mm (float): Distance between the two bar origins, mm.
        dang_deg (float): Angle of the rotation from designed to measured, degrees.
        dpos_world_mm (np.ndarray): Measured minus designed origin, in world, mm.
        dpos_bar_mm (np.ndarray): The same offset in the designed bar frame, mm.
        rotvec_bar_deg (np.ndarray): The rotation from designed to measured as a
            rotation vector in the designed bar frame, degrees (its length is
            ``dang_deg``; its z part is the roll about the bar).
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
        return {'dpos_mm': self.dpos_mm, 'dang_deg': self.dang_deg,
                'dpos_world_mm': np.asarray(self.dpos_world_mm).tolist(),
                'dpos_bar_mm': np.asarray(self.dpos_bar_mm).tolist(),
                'rotvec_bar_deg': np.asarray(self.rotvec_bar_deg).tolist()}


# * ================================================================ loader

def load_mocap_markers(problem_root: str) -> MoCapMarkerSet:
    """Read ``<problem_root>/MoCapMarkers.json`` (schema 2 only).

    Args:
        problem_root (str): The design problem folder (next to ``RobotCell.json``).

    Returns:
        MoCapMarkerSet: The marker table; ``warnings`` lists bars whose markers
        lie on one line (their roll cannot be measured).

    Raises:
        MoCapMarkersError: The file is missing or unreadable, is not schema 2,
            is not in metres, has a bar with fewer than 3 markers, or has a
            malformed marker. The message holds the full path.
    """
    path = os.path.join(problem_root, MOCAP_MARKERS_FILENAME)
    if not os.path.isfile(path):
        raise MoCapMarkersError(f"{path}: file not found; export MoCapMarkers.json from Rhino")
    try:
        with open(path, 'r') as f:
            data = json_load_std(f)
    except ValueError as exc:
        raise MoCapMarkersError(f"{path}: not valid JSON ({exc})") from exc

    version = data.get('schema_version')
    if version != SCHEMA_VERSION:
        raise MoCapMarkersError(f"{path}: schema_version {version!r}, but only {SCHEMA_VERSION} can be read; "
                                f"re-export MoCapMarkers.json from Rhino")
    if data.get('units') != 'm':
        raise MoCapMarkersError(f"{path}: units {data.get('units')!r}, expected 'm'")

    bars, warnings = {}, []
    for bar_name, bar in (data.get('bars') or {}).items():
        markers = [_design_marker(path, bar_name, k, m) for k, m in enumerate(bar.get('markers') or [])]
        if len(markers) < 3:
            raise MoCapMarkersError(f"{path}: {bar_name} has {len(markers)} marker(s); "
                                    f"at least 3 are needed to measure a bar")
        bars[bar_name] = BarMarkers(bar_name=bar_name, assembly_seq=bar.get('assembly_seq'), markers=markers)
        if not _design_triangles(bars[bar_name].points(), MIN_TRIANGLE_HEIGHT_M):
            warnings.append(f"{bar_name}: its markers are collinear (no three of them form a triangle "
                            f"{MIN_TRIANGLE_HEIGHT_M * 1000:.0f} mm high), so its roll cannot be measured")

    return MoCapMarkerSet(schema_version=version, source=data.get('source', ''), units=data['units'],
                          bar_frame_note=data.get('bar_frame', ''), blocks=data.get('marker_blocks') or {},
                          bars=bars, bars_without_markers=list(data.get('bars_without_markers') or []),
                          path=path, warnings=warnings)


def _design_marker(path: str, bar_name: str, index: int, data: dict) -> DesignMarker:
    """One marker entry of the file as a `DesignMarker`.

    Args:
        path (str): The file, for the error message.
        bar_name (str): The bar it belongs to, for the error message.
        index (int): Its position in the bar's list, for the error message.
        data (dict): The entry: ``name``, ``joint``, ``label``, ``block``, ``bar_m``.

    Returns:
        DesignMarker: The marker.

    Raises:
        MoCapMarkersError: A key is missing or ``bar_m`` is not three numbers.
    """
    try:
        bar_m = tuple(float(v) for v in data['bar_m'])
        if len(bar_m) != 3:
            raise ValueError(f"bar_m {data['bar_m']!r} is not [x, y, z]")
        return DesignMarker(name=data['name'], joint=data['joint'], label=data['label'],
                            bar_m=bar_m, block=data['block'])
    except (KeyError, TypeError, ValueError) as exc:
        raise MoCapMarkersError(f"{path}: {bar_name} marker #{index} is malformed ({exc!r})") from exc


# * ================================================================ geometry helpers

def kabsch(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """The rigid transform that best moves ``src`` points onto ``dst`` points (least squares).

    Same method as ``pybullet_planning.utils.transformations.superimposition_matrix``
    (SVD of the cross-covariance): when the best fit would be a mirror, the
    last singular direction is flipped, so the result is always a proper rotation.

    Args:
        src (np.ndarray): (K, 3) points, K >= 3.
        dst (np.ndarray): (K, 3) points, row k pairs with ``src[k]``.

    Returns:
        np.ndarray: 4x4 ``T`` with ``dst ~= T @ src`` and ``det(T[:3, :3]) = +1``.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
    u, _s, vh = np.linalg.svd((dst - dst_mean).T @ (src - src_mean))
    R = u @ vh
    if np.linalg.det(R) < 0.0:
        # ! A mirror is not a rigid motion: flip the weakest direction instead.
        R -= np.outer(u[:, 2], vh[2, :] * 2.0)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = dst_mean - R @ src_mean
    return T


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to points.

    Args:
        T (np.ndarray): 4x4 transform.
        pts (np.ndarray): (K, 3) points or a single (3,) point.

    Returns:
        np.ndarray: The moved points, same shape as ``pts``.
    """
    T = np.asarray(T, dtype=float)
    return np.asarray(pts, dtype=float) @ T[:3, :3].T + T[:3, 3]


def design_bar_axis_endpoints(T: np.ndarray, length_m: float) -> np.ndarray:
    """The two ends of a bar's centre line in world, for drawing.

    Args:
        T (np.ndarray): 4x4 bar pose (origin at the bar start, z along the bar).
        length_m (float): Bar length, metres.

    Returns:
        np.ndarray: (2, 3) start and end points, metres.
    """
    return transform_points(T, np.array([[0.0, 0.0, 0.0], [0.0, 0.0, length_m]]))


def bar_slide_roll(slide_m: float, roll_deg: float) -> np.ndarray:
    """A move along and about a bar's own z axis (slide, then roll).

    Args:
        slide_m (float): Slide along the bar, metres.
        roll_deg (float): Roll about the bar, degrees.

    Returns:
        np.ndarray: 4x4 transform in the bar frame; ``T_bar @ bar_slide_roll(s, r)``
        is the bar slid by ``s`` and rolled by ``r``.
    """
    angle = np.radians(roll_deg)
    T = np.eye(4)
    T[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    T[2, 3] = slide_m
    return T


def _rotation_vector(R: np.ndarray) -> np.ndarray:
    """Axis times angle (radians) of a rotation matrix, right at every angle.

    Args:
        R (np.ndarray): 3x3 rotation matrix.

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
    # * A half turn: skew vanishes, so read the axis from R = 2 a a^T - I.
    B = (R + np.eye(3)) / 2.0
    k = int(np.argmax(np.diag(B)))
    return angle * B[:, k] / np.sqrt(B[k, k])


def pose_deviation(T_meas: np.ndarray, T_design: np.ndarray) -> PoseDeviation:
    """How far a measured bar pose is from its designed pose, in the designed bar's frame.

    Args:
        T_meas (np.ndarray): 4x4 measured bar pose in world.
        T_design (np.ndarray): 4x4 designed bar pose in world.

    Returns:
        PoseDeviation: ``T_meas = T_design @ T_rel``; ``dpos_bar_mm`` and
        ``rotvec_bar_deg`` are the translation and rotation of ``T_rel``.
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
    """The world-frame move that carries the measured bar onto its design pose.

    * The servo loop applies it to both flanges (``T_corr @ T_flange``): one
    * rigid move for the whole held bar, so the grasp actually held is kept.

    Args:
        T_mocap_bar (np.ndarray): 4x4 bar pose measured by MoCap, in world.
        T_design_bar (np.ndarray): 4x4 bar pose the servo aims at, in world.

    Returns:
        np.ndarray: 4x4 ``T_corr = T_design @ inv(T_mocap)``, so ``T_corr @ T_mocap == T_design``.
    """
    return np.asarray(T_design_bar, dtype=float) @ np.linalg.inv(np.asarray(T_mocap_bar, dtype=float))


def apply_authored_move(T_start_authored: np.ndarray, T_end_authored: np.ndarray,
                        T_start_actual: np.ndarray) -> np.ndarray:
    """The authored world-frame move of a flange, done from where the flange actually starts.

    * The insert after a MoCap-bar servo: the flanges stand where the servo put
    * them (not on the authored start), and the insert must still make the
    * authored move (e.g. 15 mm straight down) from there.

    Args:
        T_start_authored (np.ndarray): 4x4 authored start pose, in world.
        T_end_authored (np.ndarray): 4x4 authored end pose, in world.
        T_start_actual (np.ndarray): 4x4 actual start pose, in world.

    Returns:
        np.ndarray: 4x4 ``T_end_authored @ inv(T_start_authored) @ T_start_actual``;
        equal to ``T_end_authored`` when the actual start is the authored one.
    """
    T_move = np.asarray(T_end_authored, dtype=float) @ np.linalg.inv(np.asarray(T_start_authored, dtype=float))
    return T_move @ np.asarray(T_start_actual, dtype=float)


# * ================================================================ matcher
#
# * Why the matcher works the way it does (B20 / B21 of 261016_2ModulesDemo_YH):
# *  - M1 and M2 of one plate cannot be told apart by distances to the other
# *    balls (all ~1 m away, the lists differ by ~0.35 mm). So every guess is
# *    judged on ALL markers after the fit: the wrong M1/M2 guess leaves
# *    ~48 mm errors elsewhere.
# *  - A three-ball triangle cannot reject a mirror (a flipped triangle is a
# *    rotated one). Mirrors are rejected by `kabsch` never mirroring plus a
# *    minimum of 4 matched balls.
# *  - Balls on a line (same label, same roll) are skipped as start triangles.

def _pairwise_distances(pts: np.ndarray) -> np.ndarray:
    """Distance between every two points.

    Args:
        pts (np.ndarray): (K, 3) points.

    Returns:
        np.ndarray: (K, K) distances.
    """
    return np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)


def _design_triangles(design: np.ndarray, min_height_m: float) -> list:
    """Every three design markers whose triangle is high enough to fix a pose.

    Args:
        design (np.ndarray): (M, 3) design points.
        min_height_m (float): Smallest allowed altitude of the triangle.

    Returns:
        list[tuple]: ``(a, b, c)`` index triples, a < b < c.
    """
    dist = _pairwise_distances(design)
    triangles = []
    for a, b, c in combinations(range(len(design)), 3):
        twice_area = np.linalg.norm(np.cross(design[b] - design[a], design[c] - design[a]))
        longest = max(dist[a, b], dist[b, c], dist[a, c])
        # The smallest altitude stands on the longest side: 2 * area / longest side.
        if longest > 0.0 and twice_area / longest >= min_height_m:
            triangles.append((a, b, c))
    return triangles


def _assign(sensed: np.ndarray, design_world: np.ndarray, tol_m: float) -> dict:
    """Pair sensed and design markers one-to-one, closest pairs first.

    Args:
        sensed (np.ndarray): (N, 3) sensed points.
        design_world (np.ndarray): (M, 3) design points already placed in world.
        tol_m (float): Pairs farther apart than this never match.

    Returns:
        dict: ``{sensed index: design index}``.
    """
    dist = np.linalg.norm(sensed[:, None, :] - design_world[None, :, :], axis=2)
    s_idx, d_idx = np.nonzero(dist < tol_m)
    pairs, used_design = {}, set()
    for k in np.argsort(dist[s_idx, d_idx], kind='stable'):
        s, d = int(s_idx[k]), int(d_idx[k])
        if s not in pairs and d not in used_design:
            pairs[s] = d
            used_design.add(d)
    return pairs


def _fit_pairs(sensed: np.ndarray, design: np.ndarray, pairs: dict) -> np.ndarray:
    """Kabsch on matched pairs.

    Args:
        sensed (np.ndarray): (N, 3) sensed points.
        design (np.ndarray): (M, 3) design points.
        pairs (dict): ``{sensed index: design index}``, at least 3.

    Returns:
        np.ndarray: 4x4 ``T_world_from_bar``.
    """
    return kabsch(design[list(pairs.values())], sensed[list(pairs.keys())])


def _refine(sensed: np.ndarray, design: np.ndarray, pairs: dict, tol_m: float) -> tuple:
    """Refit on all matched pairs, re-pair once, refit again.

    Args:
        sensed (np.ndarray): (N, 3) sensed points.
        design (np.ndarray): (M, 3) design points.
        pairs (dict): ``{sensed index: design index}`` of a guess, at least 3.
        tol_m (float): Match distance.

    Returns:
        tuple: ``(T, pairs, residuals)``: the 4x4 pose, the final pairs (sorted
        by sensed index) and ``{sensed index: residual in metres}``.
    """
    T = _fit_pairs(sensed, design, pairs)
    repaired = _assign(sensed, transform_points(T, design), tol_m)
    # Keep the guess' pairs in the (theoretical) case the refit loses too many.
    if len(repaired) >= 3:
        pairs = repaired
    pairs = dict(sorted(pairs.items()))
    T = _fit_pairs(sensed, design, pairs)
    gaps = np.linalg.norm(transform_points(T, design[list(pairs.values())]) - sensed[list(pairs.keys())], axis=1)
    return T, pairs, {s: float(g) for s, g in zip(pairs, gaps)}


def _rms(residuals: dict) -> float:
    """Root mean square of residuals.

    Args:
        residuals (dict): ``{index: metres}``, not empty.

    Returns:
        float: The rms, metres.
    """
    return float(np.sqrt(np.mean(np.square(list(residuals.values())))))


def _conflicts(a: dict, b: dict) -> bool:
    """Whether two marker assignments disagree about some marker.

    One being a part of the other (a marker more or less) is NOT a disagreement.

    Args:
        a (dict): ``{sensed index: design index}``.
        b (dict): ``{sensed index: design index}``.

    Returns:
        bool: True when a sensed marker goes to different design markers, or a
        design marker to different sensed markers.
    """
    a_by_design = {d: s for s, d in a.items()}
    return any(a.get(s, d) != d or a_by_design.get(d, s) != s for s, d in b.items())


def _default_min_inliers(n_design: int) -> int:
    """How many markers must match for a fit to count, when the caller does not say.

    Args:
        n_design (int): How many design markers the bar has.

    Returns:
        int: ``max(4, ceil(0.75 * n_design))``, i.e. 6 of 8.
    """
    return max(4, ceil(0.75 * n_design))


def _failed_fit(reason: str, n_sensed: int, n_design: int, n_hypotheses: int = 0) -> BarFit:
    """A fit that found no pose at all.

    Args:
        reason (str): Why, in plain English.
        n_sensed (int): Number of sensed markers.
        n_design (int): Number of design markers.
        n_hypotheses (int): Guesses tried before giving up.

    Returns:
        BarFit: ``ok=False``, no pose, every marker unmatched.
    """
    return BarFit(ok=False, reason=reason, sensed_ids=list(range(n_sensed)), n_design=n_design,
                  unmatched_sensed=list(range(n_sensed)), missing_design=list(range(n_design)),
                  n_hypotheses=n_hypotheses)


def match_and_fit(sensed_points_m: np.ndarray, design_points_bar_m: np.ndarray, *, tol_m: float = MATCH_TOL_M,
                  side_tol_m: float = 0.002, min_inliers: Optional[int] = None,
                  min_triangle_height_m: float = MIN_TRIANGLE_HEIGHT_M, max_sensed: int = 24) -> BarFit:
    """Find which sensed marker is which design marker and the bar pose that fits them.

    Motive's marker ids are not used. Every high-enough design triangle is
    matched against every sensed triple with the same side lengths; each such
    guess is fitted and judged on ALL markers (most matched, then smallest rms).
    The winner is refitted on all its markers. If a different assignment
    matches as many markers within ``tol_m``, the layout is ambiguous.

    Args:
        sensed_points_m (np.ndarray): (N, 3) sensed marker centres in world, metres.
        design_points_bar_m (np.ndarray): (M, 3) design marker centres in the bar frame, metres.
        tol_m (float): A sensed and a placed design marker closer than this match.
            4 mm: ten times below the 48 mm pair spacing, above Motive's ~0.8 mm noise.
        side_tol_m (float): Triangle sides must agree this well to start a guess
            (a quick filter only). Raise to 0.003 if live fits show residuals above 2 mm.
        min_inliers (int | None): Matched markers needed for ``ok``; None means
            ``max(4, ceil(0.75 * M))``, i.e. 6 of 8.
        min_triangle_height_m (float): Smallest altitude of a start triangle.
        max_sensed (int): Refuse more sensed markers than this (time grows fast).

    Returns:
        BarFit: The fit; ``ok`` False with a plain-English ``reason`` when it
        cannot be trusted.
    """
    sensed = np.asarray(sensed_points_m, dtype=float).reshape(-1, 3)
    design = np.asarray(design_points_bar_m, dtype=float).reshape(-1, 3)
    n_sensed, n_design = len(sensed), len(design)
    if min_inliers is None:
        min_inliers = _default_min_inliers(n_design)

    # * 1. Enough markers on both sides, not too many in view.
    if n_design < 3:
        return _failed_fit(f"the design has only {n_design} markers (need at least 3)", n_sensed, n_design)
    if n_sensed < 3:
        return _failed_fit(f"only {n_sensed} markers in view (need at least 3)", n_sensed, n_design)
    if n_sensed > max_sensed:
        return _failed_fit(f"too many markers in view ({n_sensed}, at most {max_sensed}); "
                           f"disable other assets or use the per-bar asset", n_sensed, n_design)

    # * 2-3. Distance tables and the start triangles of the design.
    sensed_dist = _pairwise_distances(sensed)
    design_dist = _pairwise_distances(design)
    triangles = _design_triangles(design, min_triangle_height_m)
    if not triangles:
        return _failed_fit(f"the design markers are collinear (no three of them form a triangle "
                           f"{min_triangle_height_m * 1000:.0f} mm high), so the bar's roll cannot be measured",
                           n_sensed, n_design)

    # * 4-5. Every sensed triple shaped like a design triangle is one guess.
    # * Ordered triples on purpose: M1/M2-swapped and mirrored guesses are tried
    # * too, and lose on the full-set score.
    guesses = {}
    n_hypotheses = 0
    for a, b, c in triangles:
        like_ab = np.abs(sensed_dist - design_dist[a, b]) <= side_tol_m
        like_bc = np.abs(sensed_dist - design_dist[b, c]) <= side_tol_m
        like_ac = np.abs(sensed_dist - design_dist[a, c]) <= side_tol_m
        # hits[i, j, k]: i-j looks like a-b, j-k like b-c, i-k like a-c.
        hits = like_ab[:, :, None] & like_bc[None, :, :] & like_ac[:, None, :]
        for i, j, k in zip(*np.nonzero(hits)):
            n_hypotheses += 1
            T = kabsch(design[[a, b, c]], sensed[[i, j, k]])
            pairs = _assign(sensed, transform_points(T, design), tol_m)
            if len(pairs) >= 3:
                guesses.setdefault(tuple(sorted(pairs.items())), pairs)
    if not guesses:
        return _failed_fit("no three sensed markers match any design triangle "
                           "(wrong bar, or wrong unit / axis convention?)", n_sensed, n_design, n_hypotheses)

    # * 6. Refine every distinct guess, so equal assignments end up equal; best first.
    fits = {}
    for pairs in guesses.values():
        T, pairs, residuals = _refine(sensed, design, pairs, tol_m)
        fits.setdefault(tuple(pairs.items()), (T, pairs, residuals))
    ranked = sorted(fits.values(), key=lambda fit: (-len(fit[1]), _rms(fit[2])))
    T, pairs, residuals = ranked[0]

    # * 7. Ambiguity: the best assignment that DISAGREES with the winner.
    second = next((fit for fit in ranked[1:] if _conflicts(pairs, fit[1])), None)
    second_best = None
    ambiguous = False
    if second is not None:
        second_best = {'n_matched': len(second[1]), 'rms_m': _rms(second[2]),
                       'max_residual_m': max(second[2].values()), 'correspondences': dict(second[1])}
        ambiguous = (second_best['n_matched'] == len(pairs) and second_best['max_residual_m'] <= tol_m)

    # * 8. Verdict.
    n_matched = len(pairs)
    if n_matched < min_inliers:
        reason = f"only {n_matched} of {n_design} design markers matched (need {min_inliers})"
    elif ambiguous:
        reason = "two different marker assignments fit equally well (symmetric layout?)"
    else:
        reason = ''
    return BarFit(ok=not reason, reason=reason, T_world_from_bar=T, correspondences=pairs,
                  sensed_ids=list(range(n_sensed)), residuals_m=residuals, rms_m=_rms(residuals),
                  max_residual_m=max(residuals.values()), n_matched=n_matched, n_design=n_design,
                  unmatched_sensed=[s for s in range(n_sensed) if s not in pairs],
                  missing_design=sorted(set(range(n_design)) - set(pairs.values())),
                  ambiguous=ambiguous, second_best=second_best, n_hypotheses=n_hypotheses)


def _fit_near_hint(sensed: np.ndarray, design: np.ndarray, T_hint: np.ndarray, *, gate_m: float,
                   tol_m: float, min_inliers: Optional[int]) -> Optional[BarFit]:
    """The quick fit while following a bar live: match around its last pose, no full search.

    The design markers are placed at ``T_hint``; each sensed marker within
    ``gate_m`` of one is taken as that marker (closest pairs first, one to
    one), then the pose is refitted and the markers re-paired within
    ``tol_m``, until the pairs stop changing (the refinement `match_and_fit`
    ends with, repeated).

    ! Only a fit that matches EVERY design marker is kept. With a marker
    ! missing, the full search runs instead (with its check for a second,
    ! equally good assignment): following a pose with balls missing could stay
    ! locked on a wrong, partly matching assignment (B20 has one that matches
    ! 6 of its 8 balls), and the full search is what a click shows too.

    Args:
        sensed (np.ndarray): (N, 3) sensed points.
        design (np.ndarray): (M, 3) design points in the bar frame.
        T_hint (np.ndarray): 4x4 bar pose to start from (the last good fit).
        gate_m (float): How far a sensed marker may be from its design marker at ``T_hint``.
        tol_m (float): Match distance of the refined fit; every residual must stay below it.
        min_inliers (int | None): Matched markers needed; None = as in `match_and_fit`.

    Returns:
        BarFit | None: An ok fit marked ``tracked``; None when a design marker
        is not matched (hidden, or the bar moved too far from ``T_hint``) or a
        residual is above ``tol_m``, so the caller runs the full search.
    """
    n_sensed, n_design = len(sensed), len(design)
    if min_inliers is None:
        min_inliers = _default_min_inliers(n_design)
    if n_sensed < 3 or n_design < 3:
        return None
    pairs = _assign(sensed, transform_points(T_hint, design), gate_m)
    if len(pairs) < 3:
        return None
    # Re-pair and refit until the pairs stop changing (a few rounds at most).
    for _ in range(4):
        T, refined, residuals = _refine(sensed, design, pairs, tol_m)
        if refined == pairs:
            break
        pairs = refined
    if len(pairs) < max(min_inliers, n_design) or max(residuals.values()) > tol_m:
        return None
    return BarFit(ok=True, reason='', T_world_from_bar=T, correspondences=pairs,
                  sensed_ids=list(range(n_sensed)), residuals_m=residuals, rms_m=_rms(residuals),
                  max_residual_m=max(residuals.values()), n_matched=len(pairs), n_design=n_design,
                  unmatched_sensed=[s for s in range(n_sensed) if s not in pairs],
                  missing_design=sorted(set(range(n_design)) - set(pairs.values())), tracked=True)


# * ================================================================ monitor cache

def _marker_size(size: Union[float, list, tuple]) -> float:
    """A Motive marker size as one number.

    Args:
        size (float | list | tuple): NatNet hands a 1-tuple, saved takes a 1-element list.

    Returns:
        float: The size, metres.
    """
    return float(size[0]) if isinstance(size, (list, tuple)) else float(size)


def fit_bar_from_cache(cache: dict, design: Union[BarMarkers, np.ndarray], *, age_s: float = 0.0,
                       max_age_s: Optional[float] = None, T_hint: Optional[np.ndarray] = None,
                       gate_m: float = TRACK_GATE_M, **kwargs) -> BarFit:
    """Fit a bar from one snapshot of the monitor's labelled-marker cache.

    Markers with a Motive residual above 3 mm, a size outside 12-26 mm, or
    Motive's 'occluded' flag (a hidden marker of an asset, placed by Motive's
    guess) are dropped first.

    * With ``T_hint`` (following live: the bar's last good pose) the markers
    * are first matched around that pose (~1 ms instead of ~20 ms, see
    * `_fit_near_hint`); when the bar moved too far for that, the full search
    * (`match_and_fit`) runs as without a hint.

    Args:
        cache (dict): ``{marker id: {'pos': [x, y, z], 'size': s, 'error': e, 'flags': f}}``,
            world metres in the z-up ("rhino") convention. Ids may be ints or
            strings (saved takes); ``size`` a number or a 1-element list;
            ``flags`` optional (older takes have none).
        design (BarMarkers | np.ndarray): The bar's design markers, or (M, 3) points.
        age_s (float): How old the snapshot is, seconds.
        max_age_s (float | None): Refuse snapshots older than this; None to never refuse.
        T_hint (np.ndarray | None): 4x4 pose to match around first; None = full search only.
        gate_m (float): With ``T_hint``: how far a marker may be from where that pose puts it.
        **kwargs: Passed on to `match_and_fit` (``tol_m``, ``min_inliers``, ...).

    Returns:
        BarFit: The fit; ``sensed_ids[i]`` is the cache id of sensed index ``i``;
        ``tracked`` True when the match around ``T_hint`` was used.
    """
    is_bar = isinstance(design, BarMarkers)
    bar_name = design.bar_name if is_bar else ''
    points = design.points() if is_bar else np.asarray(design, dtype=float)
    if max_age_s is not None and age_s > max_age_s:
        fit = _failed_fit(f"the marker snapshot is {age_s:.1f} s old (limit {max_age_s:.1f} s); is Motive streaming?",
                          0, len(points))
        fit.bar_name = bar_name
        return fit

    ids, sensed, n_occluded = [], [], 0
    for marker_id, marker in cache.items():
        # ! A hidden marker of an asset still streams, at Motive's guess: never fit it.
        if int(marker.get('flags', 0)) & MARKER_OCCLUDED_FLAG:
            n_occluded += 1
            continue
        size = _marker_size(marker['size'])
        if marker['error'] > MAX_MARKER_ERROR_M or not MARKER_SIZE_RANGE_M[0] <= size <= MARKER_SIZE_RANGE_M[1]:
            continue
        ids.append(marker_id)
        sensed.append(marker['pos'])
    sensed = np.array(sensed, dtype=float).reshape(-1, 3)
    fit = None
    if T_hint is not None:
        fit = _fit_near_hint(sensed, points, np.asarray(T_hint, dtype=float), gate_m=gate_m,
                             tol_m=kwargs.get('tol_m', MATCH_TOL_M), min_inliers=kwargs.get('min_inliers'))
    if fit is None:
        fit = match_and_fit(sensed, points, **kwargs)
    fit.bar_name = bar_name
    fit.sensed_ids = ids
    n_dropped = len(cache) - len(ids) - n_occluded
    if not fit.ok and n_occluded:
        fit.reason += f" ({n_occluded} marker(s) occluded in Motive)"
    if not fit.ok and n_dropped:
        fit.reason += (f" ({n_dropped} marker(s) dropped: size outside {MARKER_SIZE_RANGE_M[0] * 1000:.0f}-"
                       f"{MARKER_SIZE_RANGE_M[1] * 1000:.0f} mm or error above {MAX_MARKER_ERROR_M * 1000:.0f} mm)")
    return fit


# * ================================================================ desk rehearsal

def fake_sensed_markers(T_world_from_bar: np.ndarray, design_points: np.ndarray, *, slide_m: float = 0.03,
                        roll_deg: float = 2.0, noise_m: float = 0.0005, seed: int = 0) -> dict:
    """A made-up marker snapshot of a bar that sits a known bit off its pose.

    * The fit of this snapshot should come out as
    * ``T_world_from_bar @ bar_slide_roll(slide_m, roll_deg)``: slid ``slide_m``
    * along the bar and rolled ``roll_deg`` about it (up to the noise), so
    * `pose_deviation` against ``T_world_from_bar`` reads
    * ``dpos_bar_mm ~ (0, 0, slide_m * 1000)`` and ``dang_deg ~ roll_deg``.

    One stray marker is added 0.3 m from the bar (along the bar's x, in world)
    and the rows are shuffled, so the matcher has to work for it.

    Args:
        T_world_from_bar (np.ndarray): 4x4 pose the offset starts from, e.g. the designed one.
        design_points (np.ndarray): (M, 3) design markers in the bar frame, metres.
        slide_m (float): Slide along the bar, metres.
        roll_deg (float): Roll about the bar, degrees.
        noise_m (float): Standard deviation of the Gaussian noise added to each coordinate.
        seed (int): Random seed for the noise and the shuffle.

    Returns:
        dict: ``{id: {'pos': [x, y, z], 'size': [0.019], 'error': 0.0005}}`` with
        int ids ``1..M+1``, shaped like the monitor's marker cache.
    """
    T_world_from_bar = np.asarray(T_world_from_bar, dtype=float)
    rng = np.random.default_rng(seed)
    moved = transform_points(T_world_from_bar @ bar_slide_roll(slide_m, roll_deg), design_points)
    moved = moved + rng.normal(0.0, noise_m, moved.shape)
    stray = moved.mean(axis=0) + _FAKE_STRAY_DISTANCE_M * T_world_from_bar[:3, 0]
    rows = np.vstack([moved, stray])[rng.permutation(len(moved) + 1)]
    return {k + 1: {'pos': row.tolist(), 'size': [_FAKE_MARKER_SIZE_M], 'error': _FAKE_MARKER_ERROR_M}
            for k, row in enumerate(rows)}
