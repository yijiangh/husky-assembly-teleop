"""
What the relay announces about Motive's setup: its rigid body definitions and its cameras, and how either changed.

A body is its markers: their offsets from the pivot in the body's frame. A refine moves the markers a little; a pivot or
orientation edit moves them all together; another body on the same streaming id brings other markers. Names and
streaming ids are not part of a body: the same markers under another name or id are the same body.

The relay reads them from NatNet (`mocap_relay/body_definitions.py`) and announces them as its poses: offsets in the
body's frame as its poses turn it, cameras in the lab frame.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from functools import lru_cache
from itertools import permutations
from typing import Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation

#: Offsets this close, metres, count as identical: float32 rounding of a 0.1 m offset is about 1e-8 m.
IDENTICAL = 1e-7
#: After the best rigid fit, markers moved less than this, metres, are the same shape ...
SAME_SHAPE = 0.05e-3
#: ... and less than this the same physical markers (a refine or a redefinition); more is another body.
SAME_MARKERS = 3e-3
#: A pivot moved less than this, metres, and turned less than this, radians, counts as unmoved.
SAME_FRAME, SAME_TURN = 0.05e-3, np.radians(0.05)
#: A frame moved or turned more than this alongside a refine looks like a body made again from the same markers.
REDEFINED_FRAME, REDEFINED_TURN = 1e-3, np.radians(1.0)
#: Change kinds that put data measured with the body out of date; the others do not.
DATA_CHANGES = ("pivot moved", "refined", "redefined", "other body")
#: How a calibration's body definition compares with Motive's now (`definition_check`).
MATCHES, OUT_OF_DATE, UNKNOWN = "matches", "out_of_date", "unknown"
#: A camera moved more than this, metres, or turned more than this, radians, between two readings has moved (Motive's
#: continuous calibration moves them by less all the time). Not the frame fits' bump limits (`world/mocap_frame.py`).
CAMERA_MOVED, CAMERA_TURNED = 1e-3, np.radians(0.1)
#: The states of the relay's floor calibration (`~/frame` status): none applied; applied but not checked yet; held
#: (a fit confirmed it); a trusted refit pending (poses off by it); no fit trusted; not read at the relay's start.
FRAME_NONE, FRAME_UNCHECKED, FRAME_HELD, FRAME_REFIT_PENDING, FRAME_UNTRUSTED, FRAME_ERROR = (
    "none", "unchecked", "held", "refit_pending", "untrusted", "error")


@dataclass(frozen=True)
class BodyDefinition:
    """One rigid body as Motive defines it.

    Attributes:
        streaming_id: Its streaming id; not part of the body.
        name: Its name in Motive; not part of the body.
        offsets: Each marker's offset from the pivot in the body's frame, metres, in NatNet's order.
        labels: Each marker's active label (0 for passive markers).
    """

    streaming_id: int
    name: str
    offsets: tuple[tuple[float, float, float], ...]
    labels: tuple[int, ...] = ()

    def array(self) -> np.ndarray:
        """The offsets as an (n, 3) array."""
        return np.array(self.offsets, dtype=float).reshape(-1, 3)


@dataclass(frozen=True)
class Change:
    """How one body's definition differs from another's.

    Attributes:
        kind: "unchanged", "renamed" (the same markers, frame and all), "pivot moved" (a pivot or orientation edit),
            "refined" (markers moved a little), "redefined" (markers moved a little and the frame a lot: made again
            from the same markers) or "other body".
        markers_moved: Largest marker move left after the best rigid fit, metres: how much the shape changed.
        pivot_moved: How far the pivot moved, metres, by that fit.
        turned: How far the body's frame turned, radians, by that fit.
        detail: The numbers, for people.
    """

    kind: str
    markers_moved: float = 0.0
    pivot_moved: float = 0.0
    turned: float = 0.0
    detail: str = ""

    @property
    def data_change(self) -> bool:
        """bool: Whether data measured with the old definition is out of date under the new one."""
        return self.kind in DATA_CHANGES


@dataclass(frozen=True)
class Event:
    """What happened to a body between two readings of the definitions.

    Attributes:
        before: Its streaming id before, None for a body that is new.
        after: Its streaming id after, None for a body that went.
        change: How it changed; None when it came or went.
    """

    before: int | None
    after: int | None
    change: Change | None

    def describe(self) -> str:
        """One line for people."""
        if self.change is None:
            return f"id {self.after}: new body" if self.before is None else f"id {self.before}: body gone"
        where = f"id {self.before}" if self.before == self.after else f"id {self.before} -> {self.after}"
        return f"{where}: {self.change.kind}" + (f" ({self.change.detail})" if self.change.detail else "")


@dataclass(frozen=True)
class MocapCamera:
    """One of Motive's cameras, where its calibration puts it.

    Attributes:
        name: Its name in Motive, model and serial, e.g. "PrimeX 22 #72318".
        position: Its position in the Z-up world, metres.
        orientation: Its orientation (x, y, z, w), Motive's camera frame turned as the relay turns rigid bodies.
    """

    name: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


def camera_problem(camera: MocapCamera) -> str:
    """Why a camera's pose is none (not finite, or its quaternion not of length 1), or "": such a camera is left out."""
    if not np.all(np.isfinite([*camera.position, *camera.orientation])):
        return "its pose is not finite"
    length = float(np.linalg.norm(camera.orientation))
    return f"its orientation is no rotation (quaternion of length {length:.3g})" if abs(length - 1.0) > 0.01 else ""


@dataclass(frozen=True)
class MocapAnchor:
    """An anchor marker of Motive's continuous calibration, fixed in the lab, where Motive puts it.

    Attributes:
        name: The lab's name for it, e.g. "A1" (Motive streams it as an unlabeled marker, with no name).
        position: Its position in the Z-up world, metres.
    """

    name: str
    position: tuple[float, float, float]


# --- --- --- --- --- COMPARING --- --- --- --- ---

def _fit(before: np.ndarray, after: np.ndarray) -> tuple[Rotation, np.ndarray, np.ndarray]:
    """The rotation and shift taking `before` onto `after` best (Kabsch), and each marker's distance left over."""
    centre_b, centre_a = before.mean(axis=0), after.mean(axis=0)
    rotation, _ = Rotation.align_vectors(after - centre_a, before - centre_b)
    shift = centre_a - rotation.apply(centre_b)
    return rotation, shift, np.linalg.norm(rotation.apply(before) + shift - after, axis=1)


def _profiles(points: np.ndarray) -> np.ndarray:
    """Each marker's sorted distances to the others: the same whatever the frame or marker order."""
    distances = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    return np.sort(distances, axis=1)


def _orders(before: np.ndarray, after: np.ndarray) -> np.ndarray:
    """Marker orders of `after` to try against `before`, (k, n): NatNet's, by distance profile, all up to six markers.

    Trying every order catches near-symmetric marker layouts, where the distance profiles mislead.
    """
    count = len(before)
    costs = np.linalg.norm(_profiles(before)[:, None, :] - _profiles(after)[None, :, :], axis=2)
    orders = [list(range(count)), list(linear_sum_assignment(costs)[1])]
    if count <= 6:
        orders += [list(order) for order in permutations(range(count))]
    return np.array(orders)


def _best_order(before: np.ndarray, after: np.ndarray) -> np.ndarray:
    """The order of `after` that fits `before` best by its largest marker left over, every order fitted at once."""
    orders = _orders(before, after)
    moved = after[orders]                                    # (k, n, 3)
    old_c = before - before.mean(axis=0)
    new_c = moved - moved.mean(axis=1, keepdims=True)
    u, _, vt = np.linalg.svd(np.einsum("ni,kno->kio", old_c, new_c))
    # * Kabsch per order: a reflection is turned into the closest rotation.
    sign = np.sign(np.linalg.det(np.einsum("kij,kjl->kil", u, vt)))
    vt[:, -1, :] *= sign[:, None]
    rotations = np.einsum("kij,kjl->kil", u, vt).transpose(0, 2, 1)
    left = np.linalg.norm(np.einsum("kij,nj->kni", rotations, old_c) - new_c, axis=2).max(axis=1)
    return orders[int(np.argmin(left))]


@lru_cache(maxsize=4096)
def compare(before: BodyDefinition, after: BodyDefinition) -> Change:
    """How `after` differs from `before`, whatever their names, streaming ids and marker order.

    Remembered per pair (definitions never change), so judging every measurement on each tick stays cheap.
    """
    old, new = before.array(), after.array()
    renamed = f"; renamed {before.name!r} -> {after.name!r}" if before.name != after.name else ""
    if len(old) != len(new):
        return Change("other body", detail=f"{len(old)} markers -> {len(new)}{renamed}")
    if len(old) and float(np.max(np.abs(old - new))) <= IDENTICAL:
        return Change("renamed", detail=renamed[2:]) if renamed else Change("unchanged")
    if len(old) < 3:
        return Change("other body", detail=f"only {len(old)} markers: no frame to compare{renamed}")
    rotation, shift, left = _fit(old, new[_best_order(old, new)])
    moved, pivot, turned = float(left.max()), float(np.linalg.norm(shift)), float(rotation.magnitude())
    detail = (f"markers moved up to {moved * 1e3:.3f} mm, pivot {pivot * 1e3:.3f} mm, turned "
              f"{np.degrees(turned):.3f}°{renamed}")
    if moved > SAME_MARKERS:
        kind = "other body"
    elif moved <= SAME_SHAPE:
        kind = "pivot moved" if pivot > SAME_FRAME or turned > SAME_TURN else ("renamed" if renamed else "unchanged")
    else:
        kind = "redefined" if pivot > REDEFINED_FRAME or turned > REDEFINED_TURN else "refined"
    return Change(kind, moved, pivot, turned, detail)


def match(before: dict[int, BodyDefinition], after: dict[int, BodyDefinition]) -> list[Event]:
    """What happened to each body between two readings, by streaming id.

    A body that went and one that came with the same markers are the same body under a new id.
    """
    events = [Event(key, key, compare(before[key], after[key])) for key in sorted(before.keys() & after.keys())]
    gone, came = sorted(before.keys() - after.keys()), sorted(after.keys() - before.keys())
    for new_id in came:
        changes = [(compare(before[old_id], after[new_id]), old_id) for old_id in gone]
        same = [(change.markers_moved, old_id, change) for change, old_id in changes if change.kind != "other body"]
        if same:
            _, old_id, change = min(same, key=lambda entry: entry[0])
            gone.remove(old_id)
            events.append(Event(old_id, new_id, change))
        else:
            events.append(Event(None, new_id, None))
    events += [Event(old_id, None, None) for old_id in gone]
    return events


def definition_check(built_for: BodyDefinition | None, mocap_id: int,
                     current: dict[int, BodyDefinition] | None) -> tuple[str, str]:
    """Whether a calibration still fits its rigid body: MATCHES, OUT_OF_DATE or UNKNOWN, and why, for people.

    OUT_OF_DATE when Motive's definition now is a data change from the one it was built for (refined, redefined, pivot
    moved, another body); a rename or a new streaming id still matches.

    Args:
        built_for: The body definition the calibration was built for, or None if it names none.
        mocap_id: The rigid body it is applied to.
        current: Motive's definitions now, by streaming id, or None before the relay heard them.
    """
    if built_for is None:
        return UNKNOWN, "the calibration names no body definition (crl-husky's default, or an older session)"
    if current is None:
        return UNKNOWN, "no body definitions from Motive yet"
    if mocap_id not in current:
        return UNKNOWN, f"Motive defines no rigid body {mocap_id} now"
    change = compare(built_for, current[mocap_id])
    if change.data_change:
        return OUT_OF_DATE, f"made for an earlier body: {change.kind}" + (f" ({change.detail})" if change.detail
                                                                          else "")
    return MATCHES, change.kind


def camera_changes(before: dict[str, MocapCamera], after: dict[str, MocapCamera]) -> dict[str, str]:
    """The cameras that came, went, or moved over CAMERA_MOVED or CAMERA_TURNED, by name, with a line for people."""
    changes = {name: "camera gone" for name in before.keys() - after.keys()}
    changes.update({name: "new camera" for name in after.keys() - before.keys()})
    for name in before.keys() & after.keys():
        old, new = before[name], after[name]
        moved = float(np.linalg.norm(np.subtract(new.position, old.position)))
        turned = (Rotation.from_quat(old.orientation).inv() * Rotation.from_quat(new.orientation)).magnitude()
        if moved > CAMERA_MOVED or turned > CAMERA_TURNED:
            changes[name] = f"camera moved {moved * 1e3:.1f} mm, turned {np.degrees(turned):.2f}°"
    return changes


# --- --- --- --- --- ANNOUNCEMENTS --- --- --- --- ---

def body_dict(body: BodyDefinition) -> dict:
    """A body as JSON keeps it, in announcements, calibration files and run files."""
    return {"id": body.streaming_id, "name": body.name, "offsets": [list(xyz) for xyz in body.offsets],
            "labels": list(body.labels)}


def body_from_dict(entry: dict) -> BodyDefinition:
    """A body from `body_dict`.

    Raises:
        KeyError, TypeError, ValueError: If the entry is not one.
    """
    return BodyDefinition(int(entry["id"]), str(entry["name"]),
                          tuple(tuple(float(v) for v in xyz) for xyz in entry["offsets"]),
                          tuple(int(label) for label in entry["labels"]))


def definitions_json(bodies: Sequence[BodyDefinition], problems: Sequence[str], time: float) -> str:
    """The relay's announcement of Motive's rigid bodies, with what it could not read and when (Unix time)."""
    return json.dumps({"time": time, "problems": list(problems), "bodies": [body_dict(body) for body in bodies]})


def read_definitions(text: str) -> dict[int, BodyDefinition]:
    """The rigid bodies of the relay's announcement, by streaming id.

    Raises:
        ValueError: If the text is not the relay's format.
    """
    try:
        return {body.streaming_id: body for body in map(body_from_dict, json.loads(text)["bodies"])}
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"not the relay's body definitions: {error!r}") from None


def cameras_json(cameras: Sequence[MocapCamera], time: float) -> str:
    """The relay's announcement of Motive's cameras, and when (Unix time)."""
    return json.dumps({"time": time, "cameras": [{"name": camera.name, "position": camera.position,
                                                  "orientation": camera.orientation} for camera in cameras]})


def anchors_json(anchors: Sequence[MocapAnchor], missing: Sequence[str], time: float) -> str:
    """The relay's announcement of the anchors it sees, the lab's anchors it does not, and when (Unix time)."""
    return json.dumps({"time": time, "anchors": {anchor.name: list(anchor.position) for anchor in anchors},
                       "missing": list(missing)})


def read_anchors(text: str) -> tuple[dict[str, MocapAnchor], tuple[str, ...]]:
    """The anchors of the relay's announcement, by name, and the names of those not seen.

    Raises:
        ValueError: If the text is not the relay's format.
    """
    try:
        entry = json.loads(text)
        return ({str(name): MocapAnchor(str(name), tuple(float(v) for v in position))
                 for name, position in entry["anchors"].items()}, tuple(map(str, entry.get("missing", ()))))
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        raise ValueError(f"not the relay's anchors: {error!r}") from None


def read_cameras(text: str) -> dict[str, MocapCamera]:
    """The cameras of the relay's announcement, by name.

    Raises:
        ValueError: If the text is not the relay's format, or a camera's pose is none (`camera_problem`).
    """
    try:
        cameras = {str(entry["name"]): MocapCamera(str(entry["name"]), tuple(float(v) for v in entry["position"]),
                                                   tuple(float(v) for v in entry["orientation"]))
                   for entry in json.loads(text)["cameras"]}
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"not the relay's cameras: {error!r}") from None
    for camera in cameras.values():
        if problem := camera_problem(camera):
            raise ValueError(f"camera {camera.name!r}: {problem}")
    return cameras
