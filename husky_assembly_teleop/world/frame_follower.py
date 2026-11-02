"""
Following Motive's frame for the relay: the frame's state, the refit in use, and what to log and announce.

The relay hands it the floor calibration, Motive's cameras and each look's unlabeled markers (raw, Z-up); it gives back
`lab_from_raw` (what every raw pose is put in the lab frame by), the frame's state (`~/frame`), a metrics sample
(`~/frame_metrics`) and log events. The fits themselves are in `mocap_frame.py`.

- States (`status`): FRAME_NONE without a floor calibration (FRAME_ERROR if none could be read), FRAME_UNCHECKED until a
  fit decides, then FRAME_HELD, FRAME_REFIT_PENDING or FRAME_UNTRUSTED.
- ! Pure: no ROS, no clock of its own (times come in); one thread only.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose

from ..robot_interface.mocap_definitions import (FRAME_ERROR, FRAME_HELD, FRAME_NONE, FRAME_REFIT_PENDING,
                                                 FRAME_UNCHECKED, FRAME_UNTRUSTED, MocapAnchor, MocapCamera)
from .mocap_frame import (BACK_SHARE, MIN_POINTS, TRUSS_CAMERAS, VOLUME_MOVED, FloorCalibration, FrameFit,
                          camera_label, deciding_fit, find_anchors, fit_anchors, fit_cameras, frame_metrics,
                          off_limit, truss_names, volume_moved)

#: An anchor not seen for this long, seconds, is reported missing; one that flickers is not.
ANCHOR_GONE_SEC = 10.0
#: Decimals kept of the raw inputs in a metrics sample: positions to 0.1 µm, turns to 1e-8, noise to 0.1 µm (in mm).
POSITION_DECIMALS, TURN_DECIMALS, NOISE_DECIMALS = 7, 8, 4

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class FrameEvent:
    """Something for the relay to log.

    Attributes:
        level: "info", "warn" or "error".
        title: The line that says what happened.
        lines: Lines below it (a table of the fits), boxed for a warning or an error.
    """

    level: str
    title: str
    lines: tuple[str, ...] = ()


def why_untrusted(fit: FrameFit) -> str:
    """Why a fit is not trusted, in `FrameFit.describe`'s words."""
    text = fit.describe()
    return text.partition("NOT TRUSTED: ")[2] or text


def fit_table(fits: Sequence[FrameFit], since: str, in_use: Pose = Pose(), deciding: FrameFit | None = None,
              mark: str = "decides") -> list[str]:
    """Lines for people: one row per fit, its trust and the deciding one marked with `mark`, then what each left out.

    Moved, turned and the lab volume (how far the fit moves a point of the lab's volume, ± its uncertainty there) are
    since the floor calibration `since`; "off now" is against the transform in use (`in_use`): how far poses are off.
    """
    lines = [f"since {since}: moved, turned, lab volume; off now: against the transform in use",
             f"{'fit':<13} {'trust':<11} {'used':>5}  {'moved':>8}  {'turned':>6}  {'lab volume':>13}  {'off now':>8}  "
             f"{'largest residual':>16}"]
    notes = []
    for fit in fits:
        used = f"{len(fit.used)}/{len(fit.used) + len(fit.bumped) + len(fit.missing)}"
        # * The trust always in the same column, so "NOT TRUSTED" is found by where it is; the reason goes below.
        head = f"{fit.source:<13} {'trusted' if fit.trusted else 'NOT TRUSTED':<11}"
        if len(fit.used) < 3:
            lines.append(f"{head} {used:>5}  no fit")
        else:
            metres, turned = fit.moved
            lines.append(f"{head} {used:>5}  {metres * 1e3:5.1f} mm  {np.degrees(turned):5.2f}°  "
                         f"{volume_moved(fit.raw_cal_from_raw_now) * 1e3:5.1f} ± {fit.uncertainty * 1e3:3.1f} mm  "
                         f"{volume_moved(fit.raw_cal_from_raw_now, in_use) * 1e3:5.1f} mm  "
                         f"{fit.residual * 1e3:13.1f} mm"
                         + (f"  ← {mark}" if fit is deciding and len(fits) > 1 else ""))
        if fit.bumped:
            notes.append(f"{fit.source} left out: " + "; ".join(sorted(f"{camera_label(name)} {why}"
                                                                       for name, why in fit.bumped.items())))
        if fit.missing:
            notes.append(f"{fit.source} missing: " + ", ".join(map(camera_label, fit.missing)))
        if not fit.trusted:
            notes.append(f"{fit.source}, why not trusted: {why_untrusted(fit)}")
    return lines + notes


class FrameFollower:
    """Follows Motive's frame against one floor calibration: which fit decides, the state, the refit in use.

    Attributes:
        floor: The floor calibration applied, or None.
        error: Why the floor calibration could not be read at the last start or reload, or "".
        newer: A newer floor calibration on the Drive, not applied: (its file name, how far it would move the lab's
            volume, metres), or None.
        cameras: Motive's cameras now, raw, by name.
        markers: The last look's unlabeled markers, raw, each averaged over the look.
        marker_noise: Each of those markers' (RMS distance from its mean, metres; frames seen in), if given.
        refit: The refit in use, against the floor calibration's anchors and cameras (kept in memory only), or None.
        anchor_fit: The anchors' newest fit, or None before they were looked for.
        camera_fit: The truss cameras' newest fit, or None before any truss camera was heard.
        anchors: The floor calibration's anchors found at the last look, raw, by name.
        anchors_unseen: Those not found at the last look.
        anchors_missing: Those not seen for ANCHOR_GONE_SEC: what is reported.
    """

    def __init__(self, truss_serials: Sequence[str] = TRUSS_CAMERAS, use_refit_hint: str = "") -> None:
        """Start without a floor calibration.

        Args:
            truss_serials: The truss cameras, by serial.
            use_refit_hint: How to use a pending refit, for the log.
        """
        self.truss_serials = tuple(truss_serials)
        self.use_refit_hint = use_refit_hint
        self.floor: FloorCalibration | None = None
        self.error = ""
        self.newer: tuple[str, float] | None = None
        self.cameras: dict[str, MocapCamera] = {}
        self.markers: list[Vec3] = []
        self.marker_noise: list[tuple[float, int]] = []
        self._announced: tuple | None = None
        self._announced_pending: FrameFit | None = None
        self._restart()

    def _restart(self) -> None:
        """Forget every fit: a floor calibration starts from its own anchors and cameras, unchecked."""
        self.refit: FrameFit | None = None
        self.anchor_fit: FrameFit | None = None
        self.camera_fit: FrameFit | None = None
        self.anchors: dict[str, MocapAnchor] = {}
        # Each anchor found at the last look: its index among the look's markers.
        self._anchor_markers: dict[str, int] = {}
        self.anchors_unseen: tuple[str, ...] = ()
        self.anchors_missing: tuple[str, ...] = ()
        self._unseen_since: dict[str, float] = {}
        self._anchors_bumped: tuple[str, ...] = ()
        self._checked = FRAME_UNCHECKED
        # The state of the last trusted look, held or a refit pending: an untrusted look in between keeps it.
        self._trusted_state = FRAME_HELD
        # The state, deciding source and deciding fit as last logged.
        self._logged: tuple[str, str, FrameFit] | None = None

    # --- --- --- --- --- INPUTS --- --- --- --- ---

    def use_floor_calibration(self, floor: FloorCalibration | None) -> None:
        """Apply a floor calibration (or none), without a refit: unchecked until a fit decides."""
        self.floor, self.error, self.newer = floor, "", None
        self._restart()

    def look(self, markers: Sequence[Vec3], now: float,
             noise: Sequence[tuple[float, int]] = ()) -> list[FrameEvent]:
        """Fit the frame by this look's unlabeled markers (raw, each averaged since the last look) and the cameras.

        A look without any marker (Motive streams none) fits no anchors: every anchor counts as unseen.

        Args:
            markers: Motive's unlabeled markers, raw.
            now: The time of the look, seconds.
            noise: Each marker's RMS distance from its mean over the look, metres, and how many frames it was seen in.

        Returns:
            list[FrameEvent]: What to log.
        """
        self.markers, self.marker_noise = [tuple(marker) for marker in markers], list(noise)
        if self.floor is None:
            return []
        events = self._find_anchors(markers, now) if self.floor.anchors else []
        self.camera_fit = (fit_cameras(self.floor.cameras, self.cameras, self.truss_serials)
                           if truss_names(self.cameras, self.truss_serials) else None)
        return events + self._check()

    def use_refit(self) -> tuple[bool, str, list[FrameEvent]]:
        """Use the pending refit for every pose from now on, in memory only.

        Returns:
            tuple[bool, str, list[FrameEvent]]: Whether it was used, why (for the caller), what to log.
        """
        fit = self.pending
        if self.floor is None or fit is None:
            return False, (f"no refit pending: Motive's frame is within the limit of the transform in use (over "
                           f"{VOLUME_MOVED * 1e3:.1f} mm and twice its fit's uncertainty)"), []
        if not fit.trusted:
            return False, f"refit not trusted, not used: {fit.describe(self._in_use())}", []
        table = fit_table(self._fits() or [fit], self.floor.floor_file.name, self._in_use(), fit, "used")
        moves = volume_moved(fit.raw_cal_from_raw_now, self._in_use())
        self.refit, self._checked, self._trusted_state = fit, FRAME_HELD, FRAME_HELD
        self._logged = (FRAME_HELD, fit.source, fit)
        event = FrameEvent("info", f"REFIT USED: every pose moved up to {moves * 1e3:.1f} mm, back into the lab frame",
                           (*table, "kept in memory until a restart or another floor calibration"))
        return True, f"refit used, in memory: {fit.describe()}", [event]

    # --- --- --- --- --- STATE --- --- --- --- ---

    @property
    def status(self) -> str:
        """str: The frame's state, one of the FRAME_* states."""
        if self.floor is None:
            return FRAME_ERROR if self.error else FRAME_NONE
        return self._checked

    @property
    def deciding(self) -> FrameFit | None:
        """The fit that decides (`deciding_fit`), or None before any."""
        return deciding_fit(self.anchor_fit, self.camera_fit)

    @property
    def anchors_hold(self) -> bool:
        """bool: Whether the anchors' trusted fit decides (held or pending): camera moves then move no pose."""
        return (self.status in (FRAME_HELD, FRAME_REFIT_PENDING) and self.anchor_fit is not None
                and self.deciding is self.anchor_fit)

    @property
    def pending(self) -> FrameFit | None:
        """The deciding fit while the frame is not held (the refit pending, or the untrusted fit), else None."""
        return self.deciding if self.status in (FRAME_REFIT_PENDING, FRAME_UNTRUSTED) else None

    def lab_from_raw(self) -> Pose | None:
        """Raw poses into the lab frame: floor_from_raw, then the refit in use; None with no floor calibration."""
        if self.floor is None:
            return None
        if self.refit is None:
            return self.floor.floor_from_raw
        return compose(self.floor.floor_from_raw, self.refit.raw_cal_from_raw_now)

    def news(self) -> bool:
        """Whether `announcement` says something new since the last call that returned True.

        New: the state, the deciding fit's source, the anchors missing, the refit in use, a newer floor calibration,
        an error, or the pending fit moving the lab's volume over VOLUME_MOVED; not every look's numbers.
        """
        pending = self.pending
        key = (self.status, self.floor.floor_file.name if self.floor else None, self.error, self.newer,
               self.deciding.source if self.deciding else None, self.anchors_missing, self.refit)
        moved = (pending is None) != (self._announced_pending is None) or pending is not None and volume_moved(
            pending.raw_cal_from_raw_now, self._announced_pending.raw_cal_from_raw_now) > VOLUME_MOVED
        if key == self._announced and not moved:
            return False
        self._announced, self._announced_pending = key, pending
        return True

    def announcement(self, now: float) -> dict:
        """The frame's state as `~/frame` announces it (JSON keys; `robot_interface/mocap_relay.py` reads it)."""
        lab_from_raw, in_use, decides = self.lab_from_raw(), self._in_use(), self.deciding
        pending = self.pending
        return {
            "time": now, "status": self.status, "error": self.error,
            "lab_from_raw": [list(lab_from_raw.position), list(lab_from_raw.orientation)] if lab_from_raw else None,
            "floor_calibration": self.floor.floor_file.name if self.floor else None,
            "refit": self.refit.as_dict() if self.refit is not None else None,
            "pending": pending.as_dict(in_use) if pending is not None else None,
            "newer": ({"floor_calibration": self.newer[0], "moves_volume": self.newer[1],
                       "moves_poses": self.floor is None or self.newer[1] > VOLUME_MOVED}
                      if self.newer is not None else None),
            "decides": decides.source if decides is not None else None,
            "anchors_missing": list(self.anchors_missing),
            "anchor_fit": self.anchor_fit.as_dict(in_use) if self.anchor_fit is not None else None,
            "camera_fit": self.camera_fit.as_dict(in_use) if self.camera_fit is not None else None,
            "truss_cameras": truss_names(self.cameras, self.truss_serials)}

    def metrics(self, now: float) -> dict:
        """One `~/frame_metrics` sample: `frame_metrics`, then the last look's raw inputs and each anchor's quality.

        Also without a floor calibration or any fit (their keys None, `anchor_quality` empty). Added keys, raw (Z-up,
        Motive's frame), metres unless named `_mm`:

        - `frames`: the most frames any unlabeled marker was seen in this look (a marker needs half to count).
        - `markers`: every unlabeled marker counted, as columns `raw` ([x, y, z], its mean), `noise_mm` (RMS distance
          from that mean over the frames) and `frames`.
        - `anchor_quality`: each of the floor calibration's anchors by name: `raw`, `noise_mm`, `frames` (null when
          not found), `residual_mm` against the anchors' fit, `left_out_mm` against a fit of the others
          (`FrameFit.left_out_errors`), and `status`: used, bumped (left out), seen (under 3 found: no fit) or missing.
        - `truss_camera_poses`: each truss camera's [position, quaternion]: with the floor calibration's file, enough
          to fit both again offline.
        """
        sample = frame_metrics(now, self.floor.floor_file.name if self.floor else None, self.status, self.anchor_fit,
                               self.camera_fit, self.anchors_missing, self.refit)
        noise = self.marker_noise if len(self.marker_noise) == len(self.markers) else [(None, None)] * len(self.markers)
        sample["frames"] = max((frames for _, frames in noise if frames is not None), default=0)
        sample["markers"] = {"raw": [_rounded(marker, POSITION_DECIMALS) for marker in self.markers],
                             "noise_mm": [_mm(spread, NOISE_DECIMALS) for spread, _ in noise],
                             "frames": [frames for _, frames in noise]}
        sample["anchor_quality"] = {name: self._anchor_quality(name, noise) for name in
                                    sorted(self.floor.anchors if self.floor else ())}
        sample["truss_camera_poses"] = {
            name: [_rounded(self.cameras[name].position, POSITION_DECIMALS),
                   _rounded(self.cameras[name].orientation, TURN_DECIMALS)]
            for name in truss_names(self.cameras, self.truss_serials)}
        return sample

    # --- --- --- --- --- INSIDE --- --- --- --- ---

    def _anchor_quality(self, name: str, noise: Sequence[tuple[float | None, int | None]]) -> dict:
        """One anchor's entry in a metrics sample's `anchor_quality` (`metrics`)."""
        fit, index = self.anchor_fit, self._anchor_markers.get(name)
        if index is None or index >= len(self.markers):
            return {"raw": None, "noise_mm": None, "frames": None, "residual_mm": None, "left_out_mm": None,
                    "status": "missing"}
        spread, frames = noise[index]
        fitted = fit is not None and len(fit.used) >= 3
        return {"raw": _rounded(self.markers[index], POSITION_DECIMALS), "noise_mm": _mm(spread, NOISE_DECIMALS),
                "frames": frames, "residual_mm": _mm(fit.residuals.get(name) if fitted else None, NOISE_DECIMALS),
                "left_out_mm": _mm(fit.left_out_errors.get(name) if fitted else None, NOISE_DECIMALS),
                "status": ("seen" if not fitted else "bumped" if name in fit.bumped else
                           "used" if name in fit.used else "missing")}

    def _in_use(self) -> Pose:
        """The refit in use as a motion of Motive's frame, or none."""
        return self.refit.raw_cal_from_raw_now if self.refit is not None else Pose()

    def _fits(self) -> list[FrameFit]:
        """The anchors' and the truss cameras' fits, those there are."""
        return [fit for fit in (self.anchor_fit, self.camera_fit) if fit is not None]

    def _find_anchors(self, markers: Sequence[Vec3], now: float) -> list[FrameEvent]:
        """Find the floor calibration's anchors among the markers, fit them, and report those gone or moved.

        They are looked for where the floor calibration saw them, raw: the markers taken back by the refit in use, or,
        after a recalibration moved them further, by the truss cameras' fit.
        """
        expected, back = self.floor.anchors, self._in_use()
        found = find_anchors(expected, [compose(back, Pose(tuple(marker))).position for marker in markers])
        if len(markers) and len(found) < len(expected) and truss_names(self.cameras, self.truss_serials):
            refit = fit_cameras(self.floor.cameras, self.cameras, self.truss_serials)
            refound = find_anchors(expected, [compose(refit.raw_cal_from_raw_now, Pose(tuple(marker))).position
                                              for marker in markers])
            found = refound if refit.trusted and len(refound) > len(found) else found
        self._anchor_markers = dict(found)
        self.anchors = {name: MocapAnchor(name, tuple(markers[index])) for name, index in found.items()}
        self.anchors_unseen = tuple(sorted(set(expected) - set(found)))
        for name in expected:
            if name in found:
                self._unseen_since.pop(name, None)
            else:
                self._unseen_since.setdefault(name, now)
        held_out = self.anchor_fit.bumped if self.anchor_fit is not None else ()
        fit = self.anchor_fit = fit_anchors(expected, self.anchors, held_out) if len(markers) else None
        events = []
        missing = tuple(sorted(name for name, since in self._unseen_since.items() if now - since >= ANCHOR_GONE_SEC))
        if missing != self.anchors_missing:
            self.anchors_missing = missing
            events.append(self._missing_event(len(expected)))
        bumped = tuple(sorted(fit.bumped)) if fit is not None else ()
        if bumped != self._anchors_bumped:
            self._anchors_bumped = bumped
            if bumped:
                events.append(FrameEvent("warn", "ANCHORS MOVED against the others since the floor calibration: "
                                                 "bumped, or Motive lost and found them elsewhere",
                                         tuple(f"{name}: {fit.bumped[name]}" for name in bumped)))
        return events

    def _missing_event(self, total: int) -> FrameEvent:
        """The anchors reported missing changed: say which, and what holds the frame."""
        missing = self.anchors_missing
        if not missing:
            return FrameEvent("info", f"all {total} anchors seen again")
        left = total - len(missing)
        names = f"anchor{'s' if len(missing) > 1 else ''} {', '.join(missing)}"
        if left >= MIN_POINTS and self.anchor_fit is not None and self.anchor_fit.trusted:
            return FrameEvent("warn", f"{names} not seen for {ANCHOR_GONE_SEC:.0f} s; the other {left} hold the frame")
        holder = ("the truss cameras hold the frame" if self.camera_fit is not None and self.camera_fit.trusted else
                  "nothing holds the frame")
        return FrameEvent("warn", f"only {left} of {total} anchors seen ({names} not for {ANCHOR_GONE_SEC:.0f} s): "
                                  f"{holder}")

    def _check(self) -> list[FrameEvent]:
        """Decide the state from the newest fits; one event when the state, the deciding source or its move changed.

        The deciding fit moving the lab's volume within `off_limit` of the transform in use is the frame held; beyond,
        a refit is pending, held again only within BACK_SHARE of the limit; a deciding fit not trusted is the frame
        untrusted. Without any fit the state stays.
        """
        fit, in_use = self.deciding, self._in_use()
        if fit is None:
            return []
        off, limit = volume_moved(fit.raw_cal_from_raw_now, in_use), off_limit(fit)
        back = BACK_SHARE * limit
        if not fit.trusted:
            status = FRAME_UNTRUSTED
        else:
            pending = off > (back if self._trusted_state == FRAME_REFIT_PENDING else limit)
            status = self._trusted_state = FRAME_REFIT_PENDING if pending else FRAME_HELD
        self._checked = status
        logged = self._logged
        if logged is not None and logged[:2] == (status, fit.source) and volume_moved(
                fit.raw_cal_from_raw_now, logged[2].raw_cal_from_raw_now) <= VOLUME_MOVED:
            return []
        self._logged = (status, fit.source, fit)
        lines = fit_table(self._fits(), self.floor.floor_file.name, in_use, fit)
        if self.anchor_fit is not None and self.camera_fit is not None and self.anchor_fit.trusted \
                and self.camera_fit.trusted:
            apart = volume_moved(self.anchor_fit.raw_cal_from_raw_now, self.camera_fit.raw_cal_from_raw_now)
            lines.append(f"the two fits place the lab's volume up to {apart * 1e3:.1f} mm apart")
        if self.refit is not None:
            lines.append("a refit is in use")
        if status == FRAME_HELD:
            return [FrameEvent("info", f"frame held by the {fit.source}: poses off now {off * 1e3:.1f} mm, within the "
                                       f"{limit * 1e3:.1f} mm limit", tuple(lines))]
        if status == FRAME_REFIT_PENDING:
            lines.append(f"held again once poses are off under {back * 1e3:.1f} mm")
            if self.use_refit_hint:
                lines.append(f"use it with {self.use_refit_hint}")
            return [FrameEvent("warn", f"REFIT PENDING by the {fit.source}: poses are off by up to {off * 1e3:.1f} mm "
                                       f"until the refit is used", tuple(lines))]
        return [FrameEvent("error", f"FRAME UNTRUSTED: no fit tells where Motive's frame is; poses may be off (by the "
                                    f"{fit.source}: {why_untrusted(fit)})", tuple(lines))]


def _rounded(values: Sequence[float], decimals: int) -> list[float]:
    """Numbers rounded for a metrics sample, well below what Motive resolves: shorter lines in the frame log."""
    return [round(float(value), decimals) for value in values]


def _mm(metres: float | None, decimals: int) -> float | None:
    """Metres in millimetres, rounded; None stays None."""
    return None if metres is None else round(float(metres) * 1e3, decimals)
