"""
Motive's rigid body definitions and cameras, read from NatNet's model descriptions as Motive gives them (Y-up).

What they are and how a body changed: `robot_interface/mocap_definitions.py`. The relay asks for them every few
seconds (`DescriptionClient`); `scripts/mocap_body_definitions.py` watches them by hand.
- Reads NatNet 4.0 (the lab's Motive 3.0.3.1) and 4.1; lab-tested on 4.0 for a refine, another body on an id and an id
  change.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Sequence

from ..robot_interface.mocap_definitions import BodyDefinition, MocapCamera, camera_problem
from .natnet.DataDescriptions import DataDescriptions
from .natnet.NatNetClient import NatNetClient

#: NatNet message ids used here.
NAT_REQUEST_MODELDEF, NAT_MODELDEF = 4, 5
#: NatNet description types.
MARKER_SET, RIGID_BODY, CAMERA = 0, 1, 5
#: NatNetClient's reader of each other description kind, for NatNet before 4.1, which gives no size to skip by.
SDK_READERS = {2: "skeleton", 3: "force_plate", 4: "device", CAMERA: "camera"}


@dataclass(frozen=True)
class Reply:
    """Motive's answer to one request for its model descriptions, in its own Y-up frame.

    Attributes:
        time: Unix time it arrived.
        bodies: Its rigid bodies.
        cameras: Its cameras (Motive 3.0 and later).
        problems: What could not be read.
    """

    time: float
    bodies: list[BodyDefinition] = field(default_factory=list)
    cameras: list[MocapCamera] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)


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


def parse_camera(data: bytes, offset: int) -> tuple[MocapCamera, int]:
    """One camera description at `offset` (its name, position and orientation), and the offset after it."""
    name, offset = _string(data, offset)
    values = struct.unpack_from("<7f", data, offset)
    return MocapCamera(name, tuple(map(float, values[:3])), tuple(map(float, values[3:]))), offset + 28


def parse_descriptions(data: bytes, major: int,
                       minor: int) -> tuple[list[BodyDefinition], list[MocapCamera], list[str]]:
    """The rigid bodies, cameras and reading problems of a NAT_MODELDEF payload (after its 4-byte header).

    From NatNet 4.1 each description carries its size, so other kinds are skipped and each rigid body and camera is
    checked against its size. Before that, skeletons, force plates and devices are passed with NatNetClient's own
    readers, and reading stops at an unknown kind.
    """
    bodies, cameras, problems, sdk = [], [], [], None
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
        if kind in (RIGID_BODY, CAMERA):
            if kind == RIGID_BODY:
                body, offset = parse_rigid_body(data, offset, major)
                bodies.append(body)
                what = f"rigid body {body.streaming_id}"
            else:
                camera, offset = parse_camera(data, offset)
                what = f"camera {camera.name!r}"
                if problem := camera_problem(camera):
                    problems.append(f"{what}: {problem}: left out")
                else:
                    cameras.append(camera)
            if end is not None and offset != end:
                problems.append(f"{what}: read {offset - (end - size)} of {size} bytes")
                offset = end
        elif end is not None:
            offset = end
        elif kind == MARKER_SET:
            _name, offset = _string(data, offset)
            markers, = struct.unpack_from("<i", data, offset)
            offset += 4
            for _ in range(markers):
                _name, offset = _string(data, offset)
        elif kind in SDK_READERS:
            sdk = sdk or NatNetClient()
            read = getattr(sdk, f"_NatNetClient__unpack_{SDK_READERS[kind]}_description")
            length, _description = read(data[offset:], major, minor)
            offset += length
        else:
            problems.append(f"description {index + 1} of {count} is of kind {kind}, which NatNet {major}.{minor} "
                            f"gives no size for: stopped reading there")
            break
    return bodies, cameras, problems


def encode_descriptions(bodies: Sequence[BodyDefinition], major: int = 4, minor: int = 1,
                        cameras: Sequence[MocapCamera] = ()) -> bytes:
    """A NAT_MODELDEF message holding `bodies`, then `cameras`, as Motive sends it; for tests and fake servers."""
    payload = struct.pack("<i", len(bodies) + len(cameras))
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
    for camera in cameras:
        entry = camera.name.encode() + b"\0" + struct.pack("<7f", *camera.position, *camera.orientation)
        payload += struct.pack("<i", CAMERA) + (struct.pack("<i", len(entry)) if has_sizes(major, minor)
                                                else b"") + entry
    return struct.pack("<HH", NAT_MODELDEF, len(payload)) + payload


# --- --- --- --- --- ASKING MOTIVE --- --- --- --- ---

class DescriptionClient(NatNetClient):
    """A NatNet client, as the relay's, that asks Motive for its body definitions and cameras and keeps each reply.

    Replies are read with `parse_descriptions`, which handles NatNet 4.1's sizes; NatNetClient's own reader does not.
    It counts frames flagged "tracked models changed" (`models_changed`) only while no one else sets
    `new_frame_listener`: the relay sets its own.
    ! Callbacks run on NatNetClient's threads: read the replies with `take_replies`.
    """

    def __init__(self):
        """No reply yet."""
        super().__init__()
        self.print_level = 0
        self.models_changed = 0
        self._replies: list[Reply] = []
        self._lock = threading.Lock()
        self.new_frame_listener = self._on_frame

    def _on_frame(self, data: dict) -> None:
        """Count the frames that set Motive's "tracked models changed" bit."""
        if data.get("tracked_models_changed"):
            self.models_changed += 1

    def _NatNetClient__unpack_data_descriptions(self, data, packet_size, major, minor):  # noqa: N802
        """Keep a NAT_MODELDEF reply (replaces NatNetClient's reader); NatNetClient wants (bytes read, descriptions)."""
        try:
            reply = Reply(time.time(), *parse_descriptions(bytes(data), major, minor))
        except (ValueError, struct.error) as error:
            reply = Reply(time.time(), problems=[f"could not read the descriptions: {error}"])
        with self._lock:
            self._replies.append(reply)
        return len(data), DataDescriptions()

    def request(self) -> None:
        """Ask Motive for its definitions; the reply comes to `take_replies`."""
        self.send_request(self.command_socket, self.NAT_REQUEST_MODELDEF, "", (self.server_ip_address,
                                                                               self.command_port))

    def take_replies(self) -> list[Reply]:
        """The replies since the last call, oldest first."""
        with self._lock:
            replies, self._replies = self._replies, []
        return replies
