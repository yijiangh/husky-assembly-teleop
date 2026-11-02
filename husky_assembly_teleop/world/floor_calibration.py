"""
The mocap probe's floor calibration: a plane fitted to the probed floor points, and the lab frame placed on it.

Everything is raw (Motive's, Z-up): each sample keeps its position with the relay's floor calibration taken off. The
lab frame's position and turn on the plane are fitted to the marks probed and the anchors read, against their
places in the lab frame file (`lab_frame.py`); before any are stored, the origin mark and Motive's x place it. The
relay reads what `floor_calibration_json` gives (`mocap_frame.read_floor_calibration`). Pure data and maths; no UI.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert

from ..drive import started_at
from .lab_frame import MARK_WARN, MARKS, FloorPointsFit, fit_floor_points, floor_point, floor_xy, plane_axes
from .mocap_frame import volume_moved

#: Labels of the floor calibration's samples: FLOOR_LABEL for floor points, a mark's name for it (`lab_frame.MARKS`);
#: all of them are fitted into the plane.
FLOOR_LABEL, ORIGIN_LABEL = "floor", MARKS[0]
#: Label of an anchor sample, named "<anchor>_<n>": an unlabeled marker pointed at with the probe.
ANCHOR_LABEL = "anchor"
#: A plane fit counts as good from this many points and with its normal known to within PLANE_STD_GOOD.
#: ? Guesses: 0.02° tips the floor by 2 mm at 6 m from the points.
PLANE_MIN_POINTS = 6
PLANE_STD_GOOD = np.radians(0.02)
#: A fit less sure than PLANE_STD_MAX or tilted more than PLANE_TILT_MAX from mocap z is not used (no axes, no
#: export): its points sit in one spot or on a line, or not on the floor.
PLANE_STD_MAX = np.radians(1.0)
PLANE_TILT_MAX = np.radians(10.0)


@dataclass
class Sample:
    """One recorded probe point, in the lab frame (as the relay publishes poses), metres.

    Attributes:
        id: Unique within this run, in recording order.
        name: `label` plus its running number, e.g. "table_3"; unique within this run.
        label: What the operator typed, e.g. "table".
        position: Rigid-body origin, (x, y, z).
        orientation: Rigid-body orientation, quaternion (x, y, z, w).
        mocap_id: Rigid-body id the probe was tracked by.
        marker_error: Mean marker error of the pose, metres.
        stamp: ROS time of the recording, seconds.
        raw_position: `position` raw (Motive's, Z-up), the relay's floor calibration taken off; None from older files.
    """

    id: int
    name: str
    label: str
    position: list[float]
    orientation: list[float]
    mocap_id: int
    marker_error: float
    stamp: float
    raw_position: list[float] | None = None

    @property
    def raw(self) -> np.ndarray:
        """np.ndarray: The raw position, or `position` where it is not known."""
        return np.asarray(self.raw_position if self.raw_position is not None else self.position, dtype=float)


def samples_from_json(data: dict) -> list[Sample]:
    """The samples of a file written by Export, with id 0; the caller numbers them.

    Raises:
        ValueError: If `data` is not a probe points file.
    """
    if not isinstance(data, dict) or not isinstance(data.get("samples"), list):
        raise ValueError("not a probe points file (no samples list)")
    if data.get("kind") == "mocap_coverage":
        raise ValueError("a coverage map, not a probe points file")
    try:
        return [Sample(id=0, name=str(item["name"]), label=str(item["label"]),
                       position=[float(v) for v in item["position"]],
                       orientation=[float(v) for v in item["orientation"]], mocap_id=int(item["mocap_id"]),
                       marker_error=float(item["marker_error"]), stamp=float(item["stamp"]),
                       raw_position=[float(v) for v in item["raw_position"]] if item.get("raw_position") else None)
                for item in data["samples"]]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"bad sample: {error}") from error


@dataclass
class PlaneFit:
    """A least-squares plane through probe points, in the mocap frame, metres.

    Attributes:
        centroid: Mean of the points; on the plane.
        normal: Unit normal, on the side of mocap +z.
        residuals: Each point's signed height above the plane.
        normal_std: Standard error of the normal's direction about the points' narrower spread, radians; inf for
            points on a line.
    """

    centroid: np.ndarray
    normal: np.ndarray
    residuals: np.ndarray
    normal_std: float

    @property
    def rms(self) -> float:
        """float: Root mean square of the residuals: the probe's and the floor's noise together, metres."""
        return float(np.sqrt(np.mean(self.residuals ** 2)))

    @property
    def max_error(self) -> float:
        """float: The largest distance of a point from the plane, metres."""
        return float(np.max(np.abs(self.residuals)))

    @property
    def tilt(self) -> float:
        """float: Angle between the normal and mocap +z, radians."""
        return float(np.arccos(np.clip(self.normal[2], -1.0, 1.0)))

    def height(self, point) -> float:
        """float: How far `point` is above the plane, metres."""
        return float((np.asarray(point, dtype=float) - self.centroid) @ self.normal)

    def is_good(self) -> bool:
        """bool: Enough points, and the normal known to within PLANE_STD_GOOD."""
        return len(self.residuals) >= PLANE_MIN_POINTS and self.normal_std <= PLANE_STD_GOOD

    def is_usable(self) -> bool:
        """bool: The normal known to within PLANE_STD_MAX and tilted at most PLANE_TILT_MAX."""
        return self.normal_std <= PLANE_STD_MAX and self.tilt <= PLANE_TILT_MAX


def fit_plane(points) -> PlaneFit | None:
    """The least-squares plane through the finite rows of `points` (n, 3), or None under four: three always fit."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 4:
        return None
    centroid = points.mean(axis=0)
    _, singular, rows = np.linalg.svd(points - centroid)
    normal = rows[2] if rows[2][2] >= 0 else -rows[2]
    # * Noise per point left after the fit, then how much it tips the normal across the points' narrower spread.
    spare = len(points) - 3
    normal_std = float(singular[2] / np.sqrt(spare) / singular[1]) if singular[1] > 1e-9 else np.inf
    return PlaneFit(centroid, normal, (points - centroid) @ normal, normal_std)


def floor_from_raw(plane: PlaneFit, origin) -> np.ndarray:
    """The 4x4 transform from raw to floor coordinates, before marks or anchors are stored.

    The lab frame's z is the plane's normal, its origin is `origin` dropped onto the plane, and its x is Motive's x
    laid onto the plane.
    """
    rotation = plane_axes(plane.normal)  # rows: the floor axes in mocap coordinates
    on_plane = np.asarray(origin, dtype=float) - plane.height(origin) * plane.normal
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = -rotation @ on_plane
    return transform


def to_raw(lab_from_raw: tuple | None, position) -> np.ndarray:
    """A position the relay published, taken back to raw (Motive's, Z-up): its `lab_from_raw` undone."""
    if lab_from_raw is None:
        return np.asarray(position, dtype=float)
    shift, quaternion = lab_from_raw
    return Rotation.from_quat(quaternion).inv().apply(np.subtract(position, shift))


def from_raw(lab_from_raw: tuple | None, position) -> np.ndarray:
    """A raw position (Motive's, Z-up) in the frame the relay publishes: its `lab_from_raw` done."""
    if lab_from_raw is None:
        return np.asarray(position, dtype=float)
    shift, quaternion = lab_from_raw
    return Rotation.from_quat(quaternion).apply(position) + np.asarray(shift, dtype=float)


def transform_json(matrix: np.ndarray) -> dict:
    """A 4x4 transform as the export writes it: the matrix, and its position and quaternion (x, y, z, w)."""
    return {"matrix": matrix.tolist(), "position": matrix[:3, 3].tolist(),
            "orientation": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()}


@dataclass(frozen=True)
class FloorSolution:
    """The floor calibration from the probed samples: the plane, and the lab frame on it once it can be placed.

    Attributes:
        plane: Fit of every floor and mark sample, or None under four.
        samples: Those samples, and the anchor samples.
        measured: Each probed mark's mean position, raw, metres.
        transform: floor_from_raw, or None while the frame cannot be placed (`problem` says why).
        fit: The fit used, to the marks and anchors together; None while none are stored and the origin and Motive's
            x place it.
        problem: Why there is no transform, or "".
        anchors: Each anchor read, its marker's raw position, by name.
        fits: The fits by what they used: "marks", "anchors", "both", each where two points were there for it.
        anchor_heights: Each anchor seen above the plane less its stored z, metres.
        moved: Anchors left out of the fit used, over MARK_WARN from their stored place, and how far under it, metres:
            moved, or seen elsewhere by Motive. Still exported for the relay to follow, at their place in this frame.
    """

    plane: PlaneFit | None
    samples: list[Sample]
    measured: dict[str, np.ndarray]
    transform: np.ndarray | None = None
    fit: FloorPointsFit | None = None
    problem: str = ""
    anchors: dict[str, np.ndarray] = field(default_factory=dict)
    fits: dict[str, FloorPointsFit] = field(default_factory=dict)
    anchor_heights: dict[str, float] = field(default_factory=dict)
    moved: dict[str, float] = field(default_factory=dict)

    @property
    def apart(self) -> float | None:
        """How far the marks-only and anchors-only fits place the lab's volume apart, metres; None without both."""
        if "marks" not in self.fits or "anchors" not in self.fits:
            return None
        return volume_moved(Pose.from_matrix(self.fits["marks"].floor_from_raw
                                             @ np.linalg.inv(self.fits["anchors"].floor_from_raw)))

    def marks_off(self, lab_from_raw: Pose) -> float | None:
        """How far the marks-only fit and `lab_from_raw` place the lab's volume apart, metres; None without that fit.

        The marks are probed on the floor, apart from the anchors the relay follows: a check of a frame the relay
        applies or estimates.
        """
        if "marks" not in self.fits:
            return None
        return volume_moved(compose(Pose.from_matrix(self.fits["marks"].floor_from_raw), invert(lab_from_raw)))

    @property
    def offsets(self) -> dict[str, float]:
        """dict[str, float]: How far each stored mark and anchor lands from its stored place, metres.

        A mark on the floor, an anchor in 3D (its height above the plane against its stored z too).
        """
        if self.fit is None:
            return {}
        return {name: float(np.hypot(off, self.anchor_heights.get(name, 0.0)))
                for name, off in self.fit.residuals.items()}

    @property
    def worst(self) -> tuple[str, float] | None:
        """The mark or anchor farthest from its stored place, and how far, metres; None without stored ones."""
        offsets = self.offsets
        return max(offsets.items(), key=lambda item: item[1]) if offsets else None


def anchor_name(sample: Sample) -> str:
    """The anchor an anchor sample is of: its name without the running number, "A3_2" -> "A3"."""
    return sample.name.rpartition("_")[0]


def pointed_anchors(samples: list[Sample]) -> dict[str, np.ndarray]:
    """Each anchor pointed at (ANCHOR_LABEL samples), its tips' mean raw position, in the order first pointed at.

    ! Only a hint where the anchor's marker is: the tip stops about a centimetre from it.
    """
    pointed = [sample for sample in samples if sample.label == ANCHOR_LABEL]
    return {name: np.mean([sample.raw for sample in pointed if anchor_name(sample) == name], axis=0)
            for name in dict.fromkeys(map(anchor_name, pointed))}


def solve_floor(samples: list[Sample], known: dict[str, np.ndarray], ideal_anchors: dict[str, np.ndarray] | None = None,
                anchors_now: dict[str, np.ndarray] | None = None) -> FloorSolution:
    """Fit the plane to the floor and mark samples, raw, then place the lab frame on it.

    With marks (`known`) or anchors (`ideal_anchors`, (x, y, z)) stored, the frame's position and turn on the plane are
    a least-squares fit of the probed marks and the anchors read to their stored (x, y), all alike, two at least; marks
    only and anchors only are fitted too, to compare. Without any stored, the origin point and Motive's x place it, as
    for the first calibration. Anchors are not fitted into the plane: they are the markers of `anchors_now` (raw, one
    reading of them all), pointed at or found near their stored place. An anchor over
    MARK_WARN from its stored place is left out of the fits too (`FloorSolution.moved`); a mark that far is not.
    """
    ideal_anchors = ideal_anchors or {}
    used = [sample for sample in samples if sample.label == FLOOR_LABEL or sample.label in MARKS]
    pointed = [sample for sample in samples if sample.label == ANCHOR_LABEL]
    plane = fit_plane([sample.raw for sample in used])
    measured = {name: np.mean([sample.raw for sample in used if sample.label == name], axis=0)
                for name in MARKS if any(sample.label == name for sample in used)}
    anchors = {name: np.asarray(position, dtype=float) for name, position in (anchors_now or {}).items()}
    solution = FloorSolution(plane, used + pointed, measured, anchors=anchors)
    if plane is None:
        return replace(solution, problem=f"the floor needs four floor or mark points, got {len(used)}")
    if not plane.is_usable():
        return replace(solution, problem=f"the floor fit is unusable (tilt {np.degrees(plane.tilt):.1f}°, normal ± "
                                         f"{np.degrees(plane.normal_std):.2f}° per axis): probe points spread over the "
                                         f"floor")
    if known or ideal_anchors:
        ideal_xy = {name: xyz[:2] for name, xyz in ideal_anchors.items()}
        fits = {kind: fit for kind, fit in (
            ("marks", fit_floor_points(measured, known, plane.normal, plane.centroid)),
            ("anchors", fit_floor_points(anchors, ideal_xy, plane.normal, plane.centroid)),
            ("both", fit_floor_points({**measured, **anchors}, {**known, **ideal_xy}, plane.normal, plane.centroid)))
            if fit is not None}
        heights = {name: plane.height(anchors[name]) - float(ideal_anchors[name][2]) for name in anchors
                   if name in ideal_anchors}

        def off(fit: FloorPointsFit, name: str) -> float:
            """How far anchor `name` lands from its stored place in 3D under `fit`, metres."""
            on_floor = np.linalg.norm(floor_xy(fit.floor_from_raw, anchors[name]) - ideal_xy[name])
            return float(np.hypot(on_floor, heights.get(name, 0.0)))

        # * The anchors-only fit keeps every anchor, to compare with the marks. The one used leaves out the anchor
        #   farthest off while it is over MARK_WARN, one at a time: a moved anchor pulls the others off too.
        kept, moved, both = dict(anchors), {}, fits.get("both")
        while both is not None:
            worst = max((name for name in kept if name in ideal_xy), key=lambda name: off(both, name), default=None)
            if worst is None or off(both, worst) <= MARK_WARN:
                break
            moved[worst] = off(both, worst)
            del kept[worst]
            both = fit_floor_points({**measured, **kept}, {**known, **ideal_xy}, plane.normal, plane.centroid)
        moved = {name: off(both, name) for name in moved} if both is not None else moved
        fits = {kind: fit for kind, fit in {**fits, "both": both}.items() if fit is not None}
        solution = replace(solution, fits=fits, anchor_heights=heights, moved=moved)
        if "both" not in fits:
            stored = ', '.join(ideal_anchors) or 'none stored'
            return replace(solution, problem=f"probe at least two of the marks ({', '.join(known)}), or point "
                                             f"at two of the anchors ({stored})")
        return replace(solution, transform=fits["both"].floor_from_raw, fit=fits["both"])
    if ORIGIN_LABEL not in measured:
        return replace(solution, problem="probe the origin: no marks are stored yet")
    return replace(solution, transform=floor_from_raw(plane, measured[ORIGIN_LABEL]))


def floor_calibration_json(solution: FloorSolution, lab_frame_file: str) -> dict:
    """The floor calibration as written by "Export floor calibration", with the samples it came from.

    Args:
        solution: The solved floor calibration.
        lab_frame_file: Name of the lab frame file whose marks and anchors the frame was fitted to.

    Raises:
        ValueError: If the frame cannot be placed (`solution.problem`).
    """
    if solution.transform is None:
        raise ValueError(solution.problem)
    plane, transform, fit = solution.plane, solution.transform, solution.fit
    placed = (f"x, y and the origin fitted to the marks and anchors of {lab_frame_file}" if fit else
              "origin the origin point dropped onto the floor, x Motive's x laid onto the floor (no marks or anchors "
              "stored yet)")

    def fit_summary(kind: str) -> dict | None:
        """One fit's points and residuals, metres."""
        found = solution.fits.get(kind)
        return None if found is None else {"residuals": found.residuals, "rms": float(np.sqrt(np.mean(
            np.square(list(found.residuals.values()))))), "worst": list(found.worst)}

    return {
        "kind": "mocap_floor_calibration",
        "frame": f"floor_from_raw maps raw coordinates (Motive's, Z-up, before the relay applies any floor "
                 f"calibration: the samples' raw_position, the marks', the anchors', the cameras') to the lab frame: z "
                 f"the floor's normal, {placed}; the anchors are one reading of the relay's markers, at export; an "
                 f"anchor's moved (metres off its stored place) means it is left out of the fit, its floor_xyz its "
                 f"new place; "
                 f"metres, quaternions are (x, y, z, w)",
        "exported_at": started_at(),
        "floor_from_raw": transform_json(transform),
        "plane": {"points": len(plane.residuals), "normal": plane.normal.tolist(), "centroid": plane.centroid.tolist(),
                  "rms": plane.rms, "max_error": plane.max_error, "tilt_deg": float(np.degrees(plane.tilt)),
                  "normal_std_deg": float(np.degrees(plane.normal_std))},
        "lab_frame_file": lab_frame_file if fit else None,
        "marks": {name: {"position": position.tolist(), "height": plane.height(position),
                         "floor_xy": floor_xy(transform, position).tolist(),
                         "samples": sum(sample.label == name for sample in solution.samples),
                         "residual": fit.residuals.get(name) if fit else None}
                  for name, position in solution.measured.items()},
        "anchors": {name: position.tolist() for name, position in solution.anchors.items()},
        "anchors_in_floor": {name: {"floor_xyz": floor_point(transform, position).tolist(),
                                    "residual": fit.residuals.get(name) if fit else None,
                                    "height_off": solution.anchor_heights.get(name),
                                    "moved": solution.moved.get(name)}
                             for name, position in solution.anchors.items()},
        "fits": {kind: fit_summary(kind) for kind in ("marks", "anchors", "both")},
        "marks_vs_anchors": solution.apart,
        "samples": [asdict(sample) for sample in solution.samples],
    }
