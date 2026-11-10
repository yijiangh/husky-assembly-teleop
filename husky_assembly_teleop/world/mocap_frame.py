"""
The floor calibration the relay applies, and how Motive's frame moved since: fits to its anchors and truss cameras.

Layers: the relay turns Motive's poses Z-up ("raw"), then puts them in the lab frame (`world/lab_frame.py`) by
`lab_from_raw`: the floor calibration's `floor_from_raw`, then the refit in use; robot base calibrations go on top. A
recalibration or Motive's continuous calibration moves Motive's frame and every raw pose with it, while the anchors and
the truss cameras stay put: a fit (turn, shift and scale) of where Motive puts them now against where the floor
calibration stored them (raw) gives `raw_cal_from_raw_now` and `scale` (`FrameFit.motion`), which a refit composes onto
floor_from_raw. Every fit is against the floor calibration's anchors and cameras; a refit is kept only in memory
(`frame_follower.py`).

- A camera or anchor moved on its own does not fit the others: it is left out and named. The largest residual among
  those that fit is Motive's calibration warping the volume.
- A refit follows a change of Motive's scale too: poses come out at the floor calibration's scale. The anchors'
  distances alone tell the change since the floor calibration (`fit_scale`), to report it: a recalibration happened.
- Motive streams anchors as unlabeled markers, with no name: each is the marker nearest where the floor calibration
  measured it (`find_anchors`), or, after Motive's frame moved further, where those found put the others
  (`follow_anchors`) or their shape puts them (`match_anchors`).
- Floor calibrations are written by the probe only, in the dated calibration session folders below the Drive root
  (a simulation's in its own, which the robots' relay never reads); a newer one is applied only on command.
- Thresholds, side by side: a camera moved between two readings over CAMERA_MOVED (1 mm) or CAMERA_TURNED (0.1°) is
  warned about (`mocap_definitions.py`); within one fit, a camera off the others by BUMP_POSITION (2 mm) or BUMP_TURN
  (0.1°), or an anchor by ANCHOR_BUMP (2 mm), is left out, and an anchor left out comes back within BACK_SHARE of it
  (1.5 mm); a fit is trusted within UNCERTAIN_MAX (2 mm); a refit is pending beyond `off_limit` (VOLUME_MOVED, 1 mm,
  or twice the fit's uncertainty) and the frame held again within BACK_SHARE of it; Motive's scale changed beyond
  SCALE_CHANGED (about 0.39 mm/m, 2 mm across the lab's volume) is reported; the shape match allows ANCHOR_STRETCH
  (0.5 %) of scale.

Pure functions of files and numbers; no ROS, any thread.
"""

from __future__ import annotations

import itertools
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Collection, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert

from ..robot_interface.mocap_definitions import MocapAnchor, MocapCamera, ScaledPose, read_cameras, scaled
from .calibration_sessions import SESSION_NAME, misplaced_problem

#: The lab's names for its cameras, by serial: Top/Bottom, Front/Middle/Back (+y is back), Left/Right (+x, facing the
#: front). Motive names them by model and serial ("PrimeX 22 #72318"); logs show both (`camera_label`).
CAMERA_NAMES = {"31321": "TFL", "66076": "TFR", "31330": "TBL", "31322": "TBR",
                "72657": "BFL", "72652": "BML", "72654": "BBL", "72708": "BFR", "72318": "BMR", "72653": "BBR"}
#: The cameras bolted to the truss, by serial: the four on top and the two lower ones in the far corners.
TRUSS_CAMERAS = ("31321", "31322", "31330", "66076", "72657", "72654")
#: Left without a camera, a fit must place it within this, metres, and turn it within this, else it was bumped.
BUMP_POSITION, BUMP_TURN = 2e-3, np.radians(0.1)
#: Left without an anchor, a fit must place it within this, metres, else it moved on its own. A bumped anchor moves
#: centimetres; a millimetre is how well its position was recorded.
ANCHOR_BUMP = 2e-3
#: A point left out, or a pending refit, comes back only within this share of its limit: no flips on noise at it.
BACK_SHARE = 0.75
#: Fewest cameras or anchors a fit is used with: a bumped one among them can still be told from the others.
MIN_POINTS = 4
#: The least scatter a fit is taken to have, metres, per anchor and per camera; the scatter it shows when larger.
#: ? Anchors: one reading spreads 0.05 mm, Motive's continuous calibration moves them more; cameras: 0.5 mm seen 27 Oct.
ANCHOR_NOISE, CAMERA_NOISE = 1e-4, 3e-4
#: A fit is trusted only while its uncertainty at the lab volume's worst corner (`FrameFit.uncertainty`) is within this.
UNCERTAIN_MAX = 2e-3
#: The most points a fit leaves out: subsets of this many fewer are tried, at most.
MOST_LEFT_OUT = 3
#: What a fit was made from.
TRUSS, ANCHORS = "truss cameras", "anchors"
#: How far a marker may be from where an anchor was seen (stored with the floor calibration, or pointed at in the
#: probe) and still be taken for it, metres: under half the distance between two markers, over how far Motive's
#: continuous calibration moves the frame.
ANCHOR_RADIUS = 0.02
#: How much Motive's scale may differ between two calibrations and the anchors still be found by their shape, a share
#: of each distance between them (`match_anchors`); a recalibration was seen to change it by 0.16 %.
ANCHOR_STRETCH = 0.005
#: The smallest triangle of anchors the shape match starts from, square metres: a thinner one pins no turn.
MIN_TRIANGLE = 0.1
#: The lab's volume a frame move is judged in, metres, (min, max) per axis: the central 3 m cube where the robots
#: work, one corner at the lab frame's origin, along x, -y and z.
VOLUME = ((0.0, 3.0), (-3.0, 0.0), (0.0, 3.0))
#: Two motions of the frame differ once they put a point of VOLUME this far apart, metres: a shift and a turn alike.
#: Smaller moves are Motive's continuous calibration: no warning, no refit pending.
VOLUME_MOVED = 1e-3
#: The lab volume's diagonal, metres.
VOLUME_DIAGONAL = float(np.linalg.norm(np.ptp(VOLUME, axis=1)))
#: Motive's scale counts as changed since the floor calibration once it changes the lab volume's diagonal by over
#: UNCERTAIN_MAX: about 0.39 mm/m (`fit_scale`); reported, as a recalibration happened, while a refit follows it.
#: Motive's continuous calibration sways it by about 0.2 mm/m; a recalibration was seen to change it by 1.55 mm/m.
SCALE_CHANGED = UNCERTAIN_MAX / VOLUME_DIAGONAL
#: Only pairs of anchors at least this far apart, metres, tell the scale (`fit_scale`): 0.5 mm of noise is 0.25 mm/m.
SCALE_PAIR_MIN = 2.0
#: A fit is trusted only while it changes Motive's scale by at most this share (`FrameFit.scale_change`): about twice
#: the largest recalibration seen (1.55 mm/m). More is a wrong match, or a moved anchor pair taken for a scale.
SCALE_MAX = 3e-3
#: A floor calibration's name, in a dated calibration session folder (`calibration_sessions.sessions_folder`):
#: "<date>_<time>_mocap_floor_calibration.json".
FLOOR_PREFIX = "mocap_floor_calibration"
FLOOR_SUFFIX = f"_{FLOOR_PREFIX}.json"


def moved_camera(motion: Pose | ScaledPose, camera: MocapCamera) -> MocapCamera:
    """A camera's pose moved by `motion` (applied on the world side), e.g. a correction."""
    motion = scaled(motion)
    return MocapCamera(camera.name, tuple(float(v) for v in motion.apply(camera.position)),
                       motion.turn(camera.orientation))


def camera_serial(name: str) -> str:
    """A camera's serial from Motive's name for it: "PrimeX 22 #72318" -> "72318"."""
    return name.rpartition("#")[2].strip()


def camera_label(name: str) -> str:
    """A camera as logs show it: the lab's name and the serial, "BMR #72318"; Motive's name if the lab has none."""
    serial = camera_serial(name)
    return f"{CAMERA_NAMES[serial]} #{serial}" if serial in CAMERA_NAMES else name


def truss_names(cameras: dict[str, MocapCamera], serials: Sequence[str] = TRUSS_CAMERAS) -> list[str]:
    """The cameras among `cameras` that are truss cameras, by name ("PrimeX 22 #72657" is serial 72657)."""
    return sorted(name for name in cameras if camera_serial(name) in serials)


@dataclass(frozen=True)
class FrameFit:
    """How Motive's frame moved since the floor calibration, from its truss cameras or anchors then and now.

    Attributes:
        raw_cal_from_raw_now: Raw poses now taken back to raw as at the floor calibration: what undoes the move (with
            `scale`).
        used: The cameras or anchors it was fitted to.
        bumped: Those left out, each with why.
        missing: Those only one of the two readings has.
        residual: Largest distance left between a used camera and where the fit puts it, metres.
        source: What it was fitted to: TRUSS or ANCHORS.
        uncertainty: How far the fit may be off at the lab volume's worst corner, metres (one standard deviation):
            the layout's leverage (`corner_leverage`) times the scatter of those used, at least ANCHOR_NOISE or
            CAMERA_NOISE.
        consistent: Whether all those used agree with the fit within their bump limits.
        residuals: Each one in both readings, used or left out: its distance from where this fit puts it, metres.
        left_out_errors: Each one's distance from where a fit of the others puts it, metres: for those used, the fit
            the agreement check made without it; for those left out, this fit. A large one is a bad point.
        scale: How much longer the fit makes distances: Motive's scale at the floor calibration over now's, 1.0 for
            no change.
    """

    raw_cal_from_raw_now: Pose
    used: tuple[str, ...]
    bumped: dict[str, str] = field(default_factory=dict)
    missing: tuple[str, ...] = ()
    residual: float = 0.0
    source: str = TRUSS
    uncertainty: float = 0.0
    consistent: bool = True
    residuals: dict[str, float] = field(default_factory=dict)
    left_out_errors: dict[str, float] = field(default_factory=dict)
    scale: float = 1.0

    @property
    def motion(self) -> ScaledPose:
        """ScaledPose: What undoes the move: raw_cal_from_raw_now with the scale."""
        return ScaledPose(self.raw_cal_from_raw_now, self.scale)

    @property
    def scale_change(self) -> float:
        """float: How much longer Motive makes distances now than at the floor calibration, a share: `scale` undone."""
        return 1.0 / self.scale - 1.0

    @property
    def trusted(self) -> bool:
        """bool: Whether the fit may be used: MIN_POINTS or more agree, UNCERTAIN_MAX at any corner, SCALE_MAX."""
        return (len(self.used) >= MIN_POINTS and self.consistent and self.uncertainty <= UNCERTAIN_MAX
                and abs(self.scale_change) <= SCALE_MAX)

    @property
    def moved(self) -> tuple[float, float]:
        """tuple[float, float]: How far Motive's frame moved: metres at its origin, and radians."""
        pose = self.raw_cal_from_raw_now
        return float(np.linalg.norm(pose.position)), float(Rotation.from_quat(pose.orientation).magnitude())

    def describe(self, in_use: Pose | ScaledPose = Pose()) -> str:
        """One line for people: poses off now against the transform `in_use`, the move, the points, why not trusted.

        `in_use` is the refit in use (its motion), or none.
        """
        metres, turned = self.moved
        if len(self.used) < 3:
            return (f"NO FIT: only {len(self.used)} {self.source} in both"
                    + (f"; missing: {', '.join(map(camera_label, self.missing))}" if self.missing else ""))
        text = (f"poses off now {volume_moved(self.motion, in_use) * 1e3:.1f} mm; since the floor calibration moved "
                f"{metres * 1e3:.1f} mm, turned {np.degrees(turned):.2f}°, "
                f"scaled {self.scale_change * 1e3:+.2f} mm/m, the lab's volume up to "
                f"{volume_moved(self.motion) * 1e3:.1f} ± {self.uncertainty * 1e3:.1f} mm; from "
                f"{len(self.used)} {self.source}, largest residual {self.residual * 1e3:.1f} mm")
        if self.bumped:
            text += "; left out: " + "; ".join(sorted(f"{camera_label(name)} {why}"
                                                      for name, why in self.bumped.items()))
        if self.missing:
            text += "; missing: " + ", ".join(map(camera_label, self.missing))
        if len(self.used) < MIN_POINTS:
            text += f"; NOT TRUSTED: under {MIN_POINTS} {self.source} fit"
        elif not self.consistent:
            text += f"; NOT TRUSTED: no {MIN_POINTS} {self.source} agree with one fit"
        elif self.uncertainty > UNCERTAIN_MAX:
            text += (f"; NOT TRUSTED: ± {self.uncertainty * 1e3:.1f} mm, over {UNCERTAIN_MAX * 1e3:.0f} mm (those left "
                     f"do not pin a turn, or scatter)")
        elif abs(self.scale_change) > SCALE_MAX:
            text += (f"; NOT TRUSTED: scaled {self.scale_change * 1e3:+.2f} mm/m, over {SCALE_MAX * 1e3:.0f} mm/m "
                     f"(a wrong match, or points moved together)")
        return text

    def as_dict(self, in_use: Pose | ScaledPose = Pose()) -> dict:
        """For the relay's announcements: `off_now` and `describe` against the transform `in_use`; metres."""
        pose = self.raw_cal_from_raw_now
        return {"raw_cal_from_raw_now": [list(pose.position), list(pose.orientation)], "scale": self.scale,
                "used": list(self.used),
                "bumped": dict(self.bumped), "missing": list(self.missing), "residual": self.residual,
                "trusted": bool(self.trusted), "source": self.source, "uncertainty": float(self.uncertainty),
                "consistent": bool(self.consistent), "moves_volume": volume_moved(self.motion),
                "off_now": volume_moved(self.motion, in_use), "describe": self.describe(in_use)}


def volume_moved(a: Pose | ScaledPose, b: Pose | ScaledPose = Pose()) -> float:
    """How far apart two motions put a point of VOLUME at most, metres: at a corner, as the volume is a box."""
    corners = np.array(list(itertools.product(*VOLUME)))
    return float(np.max(np.linalg.norm(scaled(a).apply(corners) - scaled(b).apply(corners), axis=1)))


def _fit(now: np.ndarray, stored: np.ndarray) -> tuple[Rotation, np.ndarray, float]:
    """The turn, shift and scale taking the points `now` onto the `stored` ones best (Kabsch, then the scale)."""
    now_mean, stored_mean = now.mean(axis=0), stored.mean(axis=0)
    rotation, _ = Rotation.align_vectors(stored - stored_mean, now - now_mean)
    turned = rotation.apply(now - now_mean)
    scale = float(np.sum((stored - stored_mean) * turned) / np.sum(turned ** 2))
    return rotation, stored_mean - scale * rotation.apply(now_mean), scale


def _misfit(rotation: Rotation, shift: np.ndarray, scale: float, stored: MocapCamera | MocapAnchor,
            now: MocapCamera | MocapAnchor) -> tuple[float, str, float]:
    """How badly a camera or anchor fits a motion of the frame: share of its bump limits, why if over 1, metres off."""
    off = float(np.linalg.norm(scale * rotation.apply(now.position) + shift - stored.position))
    if isinstance(stored, MocapAnchor):
        return off / ANCHOR_BUMP, f"moved {off * 1e3:.1f} mm against the others" if off > ANCHOR_BUMP else "", off
    turned = (Rotation.from_quat(stored.orientation).inv() * rotation
              * Rotation.from_quat(now.orientation)).magnitude()
    score = max(off / BUMP_POSITION, turned / BUMP_TURN)
    why = f"bumped ({off * 1e3:.1f} mm, {np.degrees(turned):.2f}° off the others)" if score > 1.0 else ""
    return score, why, off


def _skew(vector: np.ndarray) -> np.ndarray:
    """The cross product with `vector` as a matrix."""
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def corner_leverage(points: np.ndarray) -> float:
    """How far a fit to `points` may move the lab volume's worst corner, per metre of scatter at each point.

    The fit linearised (a small turn, shift and scale): its covariance for unit, isotropic scatter, carried to each
    corner of VOLUME; the root of the largest trace. Points in two tight pairs, or on a line, pin no turn about it: it
    grows; a corner far from the points' centre carries the scale's uncertainty further.
    """
    centre = points.mean(axis=0)

    def row(point) -> np.ndarray:
        """How a point moves with a small turn, shift and scale about the centre."""
        arm = np.subtract(point, centre)
        return np.hstack([-_skew(arm), np.eye(3), arm[:, None]])

    jacobian = np.vstack([row(point) for point in points])
    covariance = np.linalg.pinv(jacobian.T @ jacobian)
    leverage = 0.0
    for corner in itertools.product(*VOLUME):
        carry = row(corner)
        leverage = max(leverage, float(np.sqrt(np.trace(carry @ covariance @ carry.T))))
    return leverage


def fit_frame(stored: dict[str, MocapCamera] | dict[str, MocapAnchor],
              now: dict[str, MocapCamera] | dict[str, MocapAnchor], names: Sequence[str],
              held_out: Collection[str] = ()) -> FrameFit:
    """Fit how Motive's frame moved from the points `names`: as `stored` (the floor calibration's), and `now`.

    Every subset, from all of them down to MIN_POINTS (at most MOST_LEFT_OUT fewer), is fitted; the largest whose
    members each agree with a fit of the others (`_misfit`: a fit through one that moved would hide it) is used, the
    tightest among equal sizes, and the rest are left out as bumped. If no subset agrees, the tightest of the smallest
    is used and the fit is not consistent. Each fit carries its uncertainty at the lab volume's worst corner.
    Those `held_out` (left out at the last fit) agree only within BACK_SHARE of their limits.
    """
    names = sorted(set(names))
    missing = tuple(name for name in names if (name in stored) != (name in now))
    names = [name for name in names if name in stored and name in now]
    source = ANCHORS if any(isinstance(item, MocapAnchor) for item in stored.values()) else TRUSS
    if len(names) < 3:
        return FrameFit(Pose(), tuple(names), {}, missing, source=source)
    best = None
    for size in range(len(names), max(min(len(names), MIN_POINTS), len(names) - MOST_LEFT_OUT) - 1, -1):
        for subset in itertools.combinations(names, size):
            rotation, shift, scale = _fit(np.array([now[n].position for n in subset]),
                                          np.array([stored[n].position for n in subset]))
            offs = [float(np.linalg.norm(scale * rotation.apply(now[n].position) + shift - stored[n].position))
                    for n in subset]
            scores, left_out = [], {}
            for name in subset:
                others = [n for n in subset if n != name]
                without = _fit(np.array([now[n].position for n in others]),
                               np.array([stored[n].position for n in others]))
                score, _, left_out[name] = _misfit(*without, stored[name], now[name])
                scores.append(score / BACK_SHARE if name in held_out else score)
            agree = bool(max(scores) <= 1.0)
            # * Agreement first, then the tightest: the largest residual, then the worst share of a bump limit.
            key = (agree, -max(offs), -max(scores))
            if best is None or key > best[0]:
                best = (key, subset, rotation, shift, scale, left_out)
        if best[0][0]:
            break
    (consistent, _, _), used, rotation, shift, scale, left_out = best
    residuals = {n: _misfit(rotation, shift, scale, stored[n], now[n])[2] for n in names}
    bumped = {n: _misfit(rotation, shift, scale, stored[n], now[n])[1]
              or _why_kept_out(n in held_out, residuals[n], source) for n in names if n not in used}
    seen_now, seen_then = np.array([now[n].position for n in used]), np.array([stored[n].position for n in used])
    offs = np.linalg.norm(scale * rotation.apply(seen_now) + shift - seen_then, axis=1)
    # * The scatter of those used, per point and axis (seven of 3 n numbers go to the fit), at least the noise floor.
    dof = max(3 * len(used) - 7, 1)
    scatter = max(float(np.sqrt(np.sum(offs ** 2) / dof)), ANCHOR_NOISE if source == ANCHORS else CAMERA_NOISE)
    return FrameFit(Pose(tuple(float(v) for v in shift), tuple(float(v) for v in rotation.as_quat())), tuple(used),
                    bumped, missing, float(offs.max()), source, corner_leverage(seen_then) * scatter, consistent,
                    residuals, {**{n: residuals[n] for n in bumped}, **left_out}, scale)


def _why_kept_out(held_out: bool, off: float, source: str) -> str:
    """Why a point within its limit is left out: held out since the last fit and not back within BACK_SHARE yet."""
    if not held_out:
        return "left out of the fit"
    back = BACK_SHARE * (ANCHOR_BUMP if source == ANCHORS else BUMP_POSITION)
    return f"{off * 1e3:.1f} mm off the others, kept out until back within {back * 1e3:.1f} mm"


def fit_cameras(stored: dict[str, MocapCamera], now: dict[str, MocapCamera],
                serials: Sequence[str] = TRUSS_CAMERAS) -> FrameFit:
    """Fit how Motive's frame moved from the truss cameras (by serial): as `stored`, and `now`."""
    return fit_frame(stored, now, truss_names({**stored, **now}, serials))


def fit_anchors(stored: dict[str, MocapAnchor], now: dict[str, MocapAnchor],
                held_out: Collection[str] = ()) -> FrameFit:
    """Fit how Motive's frame moved from the anchors: as `stored` (the floor calibration's), and `now`.

    Those `held_out` (left out at the last fit) come back only within BACK_SHARE of ANCHOR_BUMP.
    """
    return fit_frame(stored, now, tuple(stored), held_out)


def find_anchors(lab: dict[str, MocapAnchor], markers: Sequence[tuple[float, float, float]],
                 radius: float = ANCHOR_RADIUS) -> dict[str, int]:
    """Each lab anchor's marker, by index: the nearest within `radius`; a marker taken by two is neither's."""
    if not lab or not len(markers):
        return {}
    points = np.asarray(markers, dtype=float)
    found = {}
    for name, anchor in lab.items():
        distances = np.linalg.norm(points - anchor.position, axis=1)
        nearest = int(np.argmin(distances))
        if distances[nearest] <= radius:
            found[name] = nearest
    taken = [index for index in found.values()]
    return {name: index for name, index in found.items() if taken.count(index) == 1}


def follow_anchors(lab: dict[str, MocapAnchor], markers: Sequence[tuple[float, float, float]],
                   found: dict[str, int], radius: float = ANCHOR_RADIUS) -> dict[str, int]:
    """The lab anchors found again, by index, where the turn, shift and scale fitting those `found` put them.

    A quick way to the others after Motive's frame moved; none under three found.
    """
    names = [name for name in lab if name in found]
    if len(names) < 3:
        return {}
    points = np.asarray(markers, dtype=float)
    carried, _ = _carry(np.array([lab[name].position for name in names]), points[[found[name] for name in names]],
                        np.array([anchor.position for anchor in lab.values()]))
    return find_anchors({name: MocapAnchor(name, tuple(point)) for name, point in zip(lab, carried)}, markers, radius)


def match_anchors(lab: dict[str, MocapAnchor], markers: Sequence[tuple[float, float, float]],
                  radius: float = ANCHOR_RADIUS) -> dict[str, int]:
    """Each lab anchor's marker, by index, found by the anchors' shape wherever Motive's frame put them.

    Three anchors are matched to three markers by the distances between them (within `radius` and ANCHOR_STRETCH);
    the anchors carried by that match (turned, shifted and scaled) are looked for with `find_anchors`. The match
    finding the most wins, and only with MIN_POINTS: three stray markers can fit any triangle.
    """
    names = list(lab)
    if len(names) < MIN_POINTS or len(markers) < MIN_POINTS:
        return {}
    stored, points = np.array([lab[name].position for name in names]), np.asarray(markers, dtype=float)
    apart_stored = np.linalg.norm(stored[:, None] - stored[None], axis=2)
    apart = np.linalg.norm(points[:, None] - points[None], axis=2)
    tolerance = radius + ANCHOR_STRETCH * apart_stored

    def area(triangle: tuple[int, ...]) -> float:
        first, second, third = stored[list(triangle)]
        return float(np.linalg.norm(np.cross(second - first, third - first))) / 2

    best: dict[str, int] = {}
    # * The widest triangles first, as they place the others best; a thin one pins no turn.
    for a, b, c in sorted(itertools.combinations(range(len(names)), 3), key=area, reverse=True):
        if area((a, b, c)) < MIN_TRIANGLE:
            break
        for i, j in zip(*np.nonzero(np.abs(apart - apart_stored[a, b]) <= tolerance[a, b])):
            thirds = (np.abs(apart[i] - apart_stored[a, c]) <= tolerance[a, c]) & \
                     (np.abs(apart[j] - apart_stored[b, c]) <= tolerance[b, c])
            for k in np.nonzero(thirds)[0]:
                if k in (i, j):
                    continue
                carried, _ = _carry(stored[[a, b, c]], points[[i, j, k]], stored)
                found = find_anchors({name: MocapAnchor(name, tuple(point)) for name, point in zip(names, carried)},
                                     markers, radius)
                if len(found) > len(best):
                    best = found
                    if len(best) == len(names):
                        return best
    return best if len(best) >= MIN_POINTS else {}


def _carry(source: np.ndarray, target: np.ndarray, points: np.ndarray) -> tuple[np.ndarray, float]:
    """`points` carried by the turn, shift and scale that take the `source` points onto the `target` ones best."""
    source_mean, target_mean = source.mean(axis=0), target.mean(axis=0)
    scale = float(np.sqrt(np.sum((target - target_mean) ** 2) / np.sum((source - source_mean) ** 2)))
    rotation, _ = Rotation.align_vectors(target - target_mean, source - source_mean)
    return scale * rotation.apply(points - source_mean) + target_mean, scale


@dataclass(frozen=True)
class ScaleFit:
    """How Motive's scale changed since the floor calibration, from its anchors then and now.

    Attributes:
        scale: How much longer Motive puts distances now, a share (-0.0015: 1.5 mm per metre shorter).
        used: The anchors it was found from.
        pairs: How many pairs of them are at least SCALE_PAIR_MIN apart.
    """

    scale: float
    used: tuple[str, ...]
    pairs: int

    @property
    def volume_change(self) -> float:
        """How much the scale changes the lab volume's diagonal, metres."""
        return abs(self.scale) * VOLUME_DIAGONAL

    def describe(self) -> str:
        """One line for people: the change, and from what."""
        return (f"distances {self.scale * 1e3:+.2f} mm/m ({self.volume_change * 1e3:.1f} mm across the lab's volume), "
                f"from {self.pairs} pairs of {len(self.used)} anchors")

    def as_dict(self) -> dict:
        """For the relay's announcements and metrics; metres."""
        return {"scale": self.scale, "mm_per_m": self.scale * 1e3, "used": list(self.used), "pairs": self.pairs,
                "volume_change": self.volume_change, "describe": self.describe()}


def fit_scale(stored: dict[str, MocapAnchor], now: dict[str, MocapAnchor]) -> ScaleFit | None:
    """How Motive's scale changed, from the anchors as `stored` (the floor calibration's) and `now`, raw.

    The median of how much longer each pair of anchors at least SCALE_PAIR_MIN apart is now, taken with every anchor
    and with each one left out; the smallest change wins, so one anchor moved, or Motive's solution warped in a corner,
    reads as none. None under MIN_POINTS anchors, or when one left out leaves under three such pairs.
    """
    names = sorted(name for name in stored if name in now)
    if len(names) < MIN_POINTS:
        return None
    changes = {}
    for first, second in itertools.combinations(names, 2):
        then = float(np.linalg.norm(np.subtract(stored[first].position, stored[second].position)))
        if then >= SCALE_PAIR_MIN:
            changes[first, second] = float(np.linalg.norm(np.subtract(now[first].position,
                                                                      now[second].position))) / then - 1.0
    sets = [list(changes.values())] + [[change for pair, change in changes.items() if name not in pair]
                                       for name in names]
    if min(map(len, sets)) < 3:
        return None
    return ScaleFit(min((float(np.median(values)) for values in sets), key=abs), tuple(names), len(changes))


def frame_and_cameras(before: dict[str, MocapCamera], now: dict[str, MocapCamera],
                      serials: Sequence[str] = TRUSS_CAMERAS) -> tuple[FrameFit | None, dict[str, MocapCamera]]:
    """How Motive's frame moved between two readings, from the truss cameras, and every camera now seen from before.

    What is left between `before` and the cameras returned is each camera's own move, against the frame. Without a
    trusted fit, None and the cameras as they are.
    """
    fit = fit_cameras(before, now, serials)
    if not fit.trusted:
        return None, now
    return fit, {name: moved_camera(fit.motion, camera) for name, camera in now.items()}


def off_limit(fit: FrameFit) -> float:
    """How far a fit may place the lab's volume from the transform in use before a refit is pending, metres.

    VOLUME_MOVED, or twice the fit's uncertainty if larger.
    """
    return max(VOLUME_MOVED, 2.0 * fit.uncertainty)


def frame_moved(fit: FrameFit | None, since: Pose | ScaledPose = Pose()) -> bool:
    """Whether a fit moves a point of VOLUME beyond its `off_limit`, against `since`."""
    return fit is not None and volume_moved(fit.motion, since) > off_limit(fit)


def deciding_fit(anchor_fit: FrameFit | None, camera_fit: FrameFit | None) -> FrameFit | None:
    """The fit that decides: the anchors' when trusted, else the truss cameras', else the anchors' untrusted one."""
    if anchor_fit is not None and (anchor_fit.trusted or camera_fit is None):
        return anchor_fit
    return camera_fit


def frame_metrics(when: float, floor_file: str, status: str, anchor_fit: FrameFit | None,
                  camera_fit: FrameFit | None, anchors_missing: Sequence[str] = (),
                  in_use: FrameFit | None = None) -> dict:
    """One sample of how Motive's frame moved since the floor calibration, and how well each fit knows it.

    The relay publishes it every look on `~/frame_metrics` (JSON); `scripts/mocap_frame_plot.py` plots it. Each fit
    as `FrameFit.as_dict`, all against the floor calibration's anchors and cameras; `refit`: the refit in use, or None;
    `off`: how far the deciding fit and the transform in use place the lab's volume apart (poses are off by up to
    that), and `off_limit`, beyond which a refit is pending; `apart`: the two fits, when both are trusted; metres.
    """
    decides = deciding_fit(anchor_fit, camera_fit)
    both = anchor_fit is not None and camera_fit is not None and anchor_fit.trusted and camera_fit.trusted
    used = in_use.motion if in_use is not None else Pose()
    return {"time": when, "floor_calibration": floor_file, "status": status,
            "decides": decides.source if decides is not None else None,
            "anchors": anchor_fit.as_dict(used) if anchor_fit is not None else None,
            "truss_cameras": camera_fit.as_dict(used) if camera_fit is not None else None,
            "refit": in_use.as_dict() if in_use is not None else None,
            "off": volume_moved(decides.motion, used) if decides is not None else None,
            "off_limit": off_limit(decides) if decides is not None else None,
            "apart": volume_moved(anchor_fit.motion, camera_fit.motion) if both else None,
            "anchors_missing": list(anchors_missing)}


# --- --- --- --- --- FILES --- --- --- --- ---

def pose_entry(entry) -> Pose:
    """A pose stored as {"position", "orientation"} or [[x, y, z], [qx, qy, qz, qw]]."""
    position, orientation = (entry["position"], entry["orientation"]) if isinstance(entry, dict) else entry
    return Pose(tuple(float(v) for v in position), tuple(float(v) for v in orientation))


@dataclass(frozen=True)
class FloorCalibration:
    """A floor calibration: its floor_from_raw, and Motive's anchors and cameras when it was made.

    Attributes:
        floor_file: The floor calibration file.
        floor_from_raw: The lab frame from raw (Z-up) poses when it was made: the relay's starting guess.
        cameras: Every camera when it was made, raw, by name: what refits are fitted against.
        anchors: The anchors seen when it was made, raw, by name: the same.
    """

    floor_file: Path
    floor_from_raw: Pose
    cameras: dict[str, MocapCamera]
    anchors: dict[str, MocapAnchor] = field(default_factory=dict)


def _anchors(entries: dict, pose: Pose = Pose()) -> dict[str, MocapAnchor]:
    """Anchors stored as {name: [x, y, z]}, moved by `pose`."""
    return {str(name): MocapAnchor(str(name), compose(pose, Pose(tuple(float(v) for v in position))).position)
            for name, position in (entries or {}).items()}


def read_floor_calibration(floor_file: Path) -> FloorCalibration | None:
    """A floor calibration's floor_from_raw, cameras and anchors (raw); None if it names no cameras.

    Older ones stored floor_from_mocap with the cameras and anchors as the relay then published them, corrected by
    relay_correction: taken back to raw.

    Raises:
        ValueError: If it does not read, or was made in a simulation and sits in the robots' folder.
    """
    content = _read(floor_file)
    problem = misplaced_problem(content, floor_file)
    if problem is not None:
        raise ValueError(problem)
    if not content.get("cameras"):
        return None
    try:
        cameras = read_cameras(json.dumps({"cameras": content["cameras"]}))
        if "floor_from_raw" in content:
            return FloorCalibration(floor_file, pose_entry(content["floor_from_raw"]), cameras,
                                    _anchors(content.get("anchors")))
        published = pose_entry(content.get("relay_correction") or ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
        back = invert(published)
        return FloorCalibration(floor_file, compose(pose_entry(content["floor_from_mocap"]), published),
                                {name: moved_camera(back, camera) for name, camera in cameras.items()},
                                _anchors(content.get("anchors"), back))
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        raise ValueError(f"{floor_file.name}: its floor_from_raw, cameras or anchors do not read ({error!r})") from None


def newest_floor_calibration(sessions: Path) -> tuple[FloorCalibration | None, list[str]]:
    """The newest floor calibration in the dated session folders of `sessions` that reads and names its cameras.

    Returns:
        tuple[FloorCalibration | None, list[str]]: It, or None if none; and why each newer one was passed over
            (half-synced, made in a simulation and in the robots' folder), newest first.
    """
    skipped: list[str] = []
    if not sessions.is_dir():
        return None, skipped
    floor_files = (path for path in sessions.glob(f"*/*{FLOOR_SUFFIX}") if SESSION_NAME.fullmatch(path.parent.name))
    for floor_file in sorted(floor_files, key=lambda path: path.name, reverse=True):
        try:
            frame = read_floor_calibration(floor_file)
        except ValueError as error:
            skipped.append(str(error))
            continue
        if frame is not None:
            return frame, skipped
    return None, skipped


def _read(path: Path) -> dict:
    """A JSON object from a file.

    Raises:
        ValueError: If it does not read or is no object.
    """
    try:
        content = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"{path.name}: cannot read it ({error}); half-synced?") from None
    if not isinstance(content, dict):
        raise ValueError(f"{path.name}: not a JSON object")
    return content
