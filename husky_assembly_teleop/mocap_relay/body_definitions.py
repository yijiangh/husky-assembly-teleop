"""
Motive's rigid body definitions from NatNet's model descriptions: read them, and tell how a body's definition changed.

A body is its markers: their offsets from the pivot in the body's frame. A refine moves the markers a little; a pivot or
orientation edit moves them all together; another body on the same streaming id brings other markers. Names and
streaming ids are not part of a body: the same markers under another name or id are the same body.

! Not used by the relay yet: `scripts/mocap_body_definitions.py` tries it against Motive.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from itertools import permutations
from typing import Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation

from .natnet.DataDescriptions import DataDescriptions
from .natnet.NatNetClient import NatNetClient

#: NatNet message ids used here.
NAT_REQUEST_MODELDEF, NAT_MODELDEF = 4, 5
#: NatNet description types.
MARKER_SET, RIGID_BODY = 0, 1

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
DATA_CHANGES = ("frame moved", "refined", "redefined", "other body")


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
        kind: "unchanged", "renamed" (the same markers, frame and all), "frame moved" (a pivot or orientation edit),
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


def _orders(before: np.ndarray, after: np.ndarray) -> list[Sequence[int]]:
    """Marker orders of `after` to try against `before`: NatNet's, the closest by distance profile, all up to six.

    Trying every order catches near-symmetric marker layouts, where the distance profiles mislead.
    """
    count = len(before)
    costs = np.linalg.norm(_profiles(before)[:, None, :] - _profiles(after)[None, :, :], axis=2)
    orders = [list(range(count)), list(linear_sum_assignment(costs)[1])]
    if count <= 6:
        orders += [list(order) for order in permutations(range(count))]
    return orders


def compare(before: BodyDefinition, after: BodyDefinition) -> Change:
    """How `after` differs from `before`, whatever their names, streaming ids and marker order."""
    old, new = before.array(), after.array()
    renamed = f"; renamed {before.name!r} -> {after.name!r}" if before.name != after.name else ""
    if len(old) != len(new):
        return Change("other body", detail=f"{len(old)} markers -> {len(new)}{renamed}")
    if len(old) and float(np.max(np.abs(old - new))) <= IDENTICAL:
        return Change("renamed", detail=renamed[2:]) if renamed else Change("unchanged")
    if len(old) < 3:
        return Change("other body", detail=f"only {len(old)} markers: no frame to compare{renamed}")
    best = None
    for order in _orders(old, new):
        rotation, shift, left = _fit(old, new[list(order)])
        if best is None or left.max() < best[2].max():
            best = rotation, shift, left
    rotation, shift, left = best
    moved, pivot, turned = float(left.max()), float(np.linalg.norm(shift)), float(rotation.magnitude())
    detail = (f"markers moved up to {moved * 1e3:.3f} mm, pivot {pivot * 1e3:.3f} mm, turned "
              f"{np.degrees(turned):.3f} deg{renamed}")
    if moved > SAME_MARKERS:
        kind = "other body"
    elif moved <= SAME_SHAPE:
        kind = "frame moved" if pivot > SAME_FRAME or turned > SAME_TURN else ("renamed" if renamed else "unchanged")
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


# --- --- --- --- --- READING NATNET --- --- --- --- ---

def _string(data: bytes, offset: int) -> tuple[str, int]:
    """A zero-terminated string at `offset`, and the offset after it."""
    end = data.index(b"\0", offset)
    return data[offset:end].decode("utf-8", "replace"), end + 1


def has_sizes(major: int, minor: int) -> bool:
    """Whether NatNet `major.minor` puts each description's size after its type (4.1 and later)."""
    return (major, minor) >= (4, 1)


def parse_rigid_body(data: bytes, offset: int, major: int) -> tuple[BodyDefinition, int]:
    """One rigid body description at `offset`, and the offset after it.

    Raises:
        ValueError: Before NatNet 3, whose descriptions hold no marker offsets.
    """
    if major < 3:
        raise ValueError(f"NatNet {major} sends no marker offsets: Motive must stream NatNet 3 or later")
    name, offset = _string(data, offset)
    streaming_id, _parent = struct.unpack_from("<ii", data, offset)
    offset += 8 + 12  # the parent's id, then the offset from the parent
    count, = struct.unpack_from("<i", data, offset)
    offset += 4
    offsets = [struct.unpack_from("<fff", data, offset + 12 * index) for index in range(count)]
    offset += 12 * count
    labels = struct.unpack_from(f"<{count}i", data, offset)
    offset += 4 * count
    if major >= 4:
        for _ in range(count):  # marker names
            _name, offset = _string(data, offset)
    return BodyDefinition(streaming_id, name, tuple(tuple(map(float, xyz)) for xyz in offsets), tuple(labels)), offset


def parse_descriptions(data: bytes, major: int, minor: int) -> tuple[list[BodyDefinition], list[str]]:
    """The rigid bodies of a NAT_MODELDEF message's payload (after its 4-byte header), and what could not be read.

    From NatNet 4.1 each description carries its size, so other kinds are skipped and each rigid body is checked
    against its size; before that only marker sets and rigid bodies can be passed, and reading stops at another kind.
    """
    bodies, problems = [], []
    count, = struct.unpack_from("<i", data, 0)
    offset = 4
    for index in range(count):
        kind, = struct.unpack_from("<i", data, offset)
        offset += 4
        size = None
        if has_sizes(major, minor):
            size, = struct.unpack_from("<i", data, offset)
            offset += 4
        end = offset + size if size is not None else None
        if kind == RIGID_BODY:
            body, offset = parse_rigid_body(data, offset, major)
            bodies.append(body)
            if end is not None and offset != end:
                problems.append(f"rigid body {body.streaming_id}: read {offset - (end - size)} of {size} bytes")
                offset = end
        elif end is not None:
            offset = end
        elif kind == MARKER_SET:
            _name, offset = _string(data, offset)
            markers, = struct.unpack_from("<i", data, offset)
            offset += 4
            for _ in range(markers):
                _name, offset = _string(data, offset)
        else:
            problems.append(f"description {index + 1} of {count} is of kind {kind}, which NatNet {major}.{minor} "
                            f"gives no size for: stopped reading there")
            break
    return bodies, problems


def encode_descriptions(bodies: Sequence[BodyDefinition], major: int = 4, minor: int = 1) -> bytes:
    """A NAT_MODELDEF message holding `bodies`, as Motive sends it; for tests and fake servers."""
    payload = struct.pack("<i", len(bodies))
    for body in bodies:
        count = len(body.offsets)
        labels = body.labels or (0,) * count
        entry = body.name.encode() + b"\0" + struct.pack("<ii3f", body.streaming_id, -1, 0.0, 0.0, 0.0)
        entry += struct.pack("<i", count) + b"".join(struct.pack("<3f", *xyz) for xyz in body.offsets)
        entry += struct.pack(f"<{count}i", *labels)
        if major >= 4:
            entry += b"".join(f"Marker{index + 1}".encode() + b"\0" for index in range(count))
        payload += struct.pack("<i", RIGID_BODY) + (struct.pack("<i", len(entry)) if has_sizes(major, minor)
                                                    else b"") + entry
    return struct.pack("<HH", NAT_MODELDEF, len(payload)) + payload


# --- --- --- --- --- ASKING MOTIVE --- --- --- --- ---

class DescriptionClient(NatNetClient):
    """A NatNet client, as the relay's, that asks Motive for its rigid body definitions and keeps each reply.

    Replies are read with `parse_descriptions`, which handles NatNet 4.1's sizes; NatNetClient's own reader does not.
    ! Callbacks run on NatNetClient's threads: read the replies with `take_replies`.
    """

    def __init__(self):
        """No reply yet."""
        super().__init__()
        self.print_level = 0
        self.models_changed = 0
        self._replies: list[tuple[float, list[BodyDefinition], list[str]]] = []
        self._lock = threading.Lock()
        self.new_frame_listener = self._on_frame

    def _on_frame(self, data: dict) -> None:
        """Count the frames that set Motive's "tracked models changed" bit."""
        if data.get("tracked_models_changed"):
            self.models_changed += 1

    def _NatNetClient__unpack_data_descriptions(self, data, packet_size, major, minor):  # noqa: N802
        """Keep a NAT_MODELDEF reply (replaces NatNetClient's reader); NatNetClient wants (bytes read, descriptions)."""
        try:
            bodies, problems = parse_descriptions(bytes(data), major, minor)
        except (ValueError, struct.error) as error:
            bodies, problems = [], [f"could not read the descriptions: {error}"]
        with self._lock:
            self._replies.append((time.time(), bodies, problems))
        return len(data), DataDescriptions()

    def request(self) -> None:
        """Ask Motive for its definitions; the reply comes to `take_replies`."""
        self.send_request(self.command_socket, self.NAT_REQUEST_MODELDEF, "", (self.server_ip_address,
                                                                               self.command_port))

    def take_replies(self) -> list[tuple[float, list[BodyDefinition], list[str]]]:
        """The replies since the last call: (Unix time, bodies, what could not be read)."""
        with self._lock:
            replies, self._replies = self._replies, []
        return replies
