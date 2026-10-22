"""Motive's rigid body definitions: read from NatNet, each kind of change told apart; names and ids are not a body."""

from __future__ import annotations

import socket
import struct
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.mocap_relay.body_definitions import (NAT_MODELDEF, RIGID_BODY, BodyDefinition,
                                                                DescriptionClient, compare, encode_descriptions,
                                                                match, parse_descriptions)

#: A marker layout like a calibration marker plate: five markers, no symmetry, metres from the pivot.
MARKERS = np.array([[0.0, 0.0, 0.0], [0.061, 0.004, 0.0], [0.013, 0.072, 0.002], [-0.044, 0.031, 0.011],
                    [0.027, -0.038, 0.019]]) - [0.01, 0.014, 0.006]


def body(points=MARKERS, streaming_id: int = 1866, name: str = "calibration_tool") -> BodyDefinition:
    """A body with these marker offsets, as float32 like NatNet sends them."""
    points = np.asarray(points, dtype=np.float32).astype(float)
    return BodyDefinition(streaming_id, name, tuple(tuple(xyz) for xyz in points), (0,) * len(points))


def moved(points=MARKERS, shift=(0.0, 0.0, 0.0), turn_degrees=(0.0, 0.0, 0.0), noise: float = 0.0,
          order=None, seed: int = 0) -> np.ndarray:
    """The markers as a new definition sees them: the frame shifted and turned, each marker off by up to `noise`."""
    rotation = Rotation.from_euler("xyz", turn_degrees, degrees=True)
    points = rotation.apply(points) + shift
    points += np.random.default_rng(seed).uniform(-noise, noise, points.shape)
    return points[list(order)] if order is not None else points


@pytest.mark.parametrize("name", ["calibration_tool", "tool body renamed"])
def test_the_same_markers_are_unchanged_whatever_the_name(name):
    """A rename is reported, but it is no data change."""
    change = compare(body(), body(name=name))
    assert change.kind == ("unchanged" if name == "calibration_tool" else "renamed") and not change.data_change


def test_the_same_markers_in_another_order_are_unchanged():
    """The order NatNet lists the markers in is not part of the body."""
    assert compare(body(), body(moved(order=[3, 0, 4, 1, 2]))).kind == "unchanged"


@pytest.mark.parametrize("shift, turn", [((0.002, 0.0, 0.0), (0.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (0.0, 0.0, 3.0))])
def test_a_pivot_or_orientation_edit_moves_the_frame(shift, turn):
    """The same layout in a moved or turned frame: a data change, with the pivot's move and the turn measured."""
    change = compare(body(), body(moved(shift=shift, turn_degrees=turn)))
    assert change.kind == "frame moved" and change.data_change and change.markers_moved < 1e-6
    assert change.pivot_moved == pytest.approx(np.linalg.norm(shift), abs=1e-4)
    assert np.degrees(change.turned) == pytest.approx(max(turn), abs=1e-3)


def test_a_refine_moves_markers_a_little_in_the_same_frame():
    """Markers moved by a few tenths of a millimetre around the same pivot: refined."""
    change = compare(body(), body(moved(noise=0.3e-3)))
    assert change.kind == "refined" and change.data_change and 0.05e-3 < change.markers_moved < 0.6e-3


def test_a_body_made_again_from_the_same_markers_is_redefined():
    """A new pivot and orientation, and the markers measured afresh: redefined, not another body."""
    change = compare(body(), body(moved(shift=(0.012, -0.004, 0.003), turn_degrees=(10.0, -25.0, 40.0), noise=0.2e-3,
                                        order=[1, 0, 2, 4, 3])))
    assert change.kind == "redefined" and change.data_change and change.markers_moved < 0.5e-3


@pytest.mark.parametrize("other", [MARKERS * 1.3, MARKERS[:4]])
def test_other_markers_on_the_same_id_are_another_body(other):
    """Another layout, or another number of markers."""
    assert compare(body(), body(other)).kind == "other body"


def test_the_same_markers_under_a_new_id_are_the_same_body():
    """A body that went and one that came with its markers: one event, the id changed; others come and go."""
    other = body(MARKERS * 1.5, streaming_id=1850, name="base")
    after = {1900: body(moved(order=[4, 3, 2, 1, 0]), streaming_id=1900), 1950: body(MARKERS * 2.0, streaming_id=1950)}
    events = match({1866: body(), 1850: other}, after)
    assert {(e.before, e.after, e.change.kind if e.change else None) for e in events} == {
        (1866, 1900, "unchanged"), (None, 1950, None), (1850, None, None)}


@pytest.mark.parametrize("major, minor", [(3, 0), (4, 0), (4, 1)])
def test_descriptions_read_back_as_written(major, minor):
    """Each NatNet layout: names, ids and float32 offsets come back; from 4.1 each description carries its size."""
    bodies = [body(), body(MARKERS * 1.5, streaming_id=1850, name="base")]
    message = encode_descriptions(bodies, major, minor)
    assert struct.unpack_from("<H", message)[0] == NAT_MODELDEF
    read, problems = parse_descriptions(message[4:], major, minor)
    assert problems == [] and read == bodies


def test_descriptions_of_other_kinds_are_skipped_by_their_size():
    """From NatNet 4.1 a camera or force plate between rigid bodies is passed over by its size."""
    message = encode_descriptions([body()], 4, 1)[4:]
    camera = struct.pack("<ii", 5, 12) + b"\x01" * 12
    payload = struct.pack("<i", 2) + camera + message[4:]
    read, problems = parse_descriptions(payload, 4, 1)
    assert problems == [] and read == [body()]


def test_natnet_before_3_has_no_offsets():
    """Its descriptions hold no marker offsets: reading them says so."""
    payload = struct.pack("<ii", 1, RIGID_BODY) + b"tool\0" + struct.pack("<ii3f", 1866, -1, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="NatNet 3 or later"):
        parse_descriptions(payload, 2, 9)


def test_the_client_asks_a_motive_for_its_definitions():
    """Over UDP on this PC: the handshake, then a request, read with NatNet 4.1's sizes."""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    server.settimeout(0.2)
    bodies = [body(), body(MARKERS * 1.5, streaming_id=1850, name="base")]
    stop = threading.Event()

    def motive():
        while not stop.is_set():
            try:
                data, address = server.recvfrom(4096)
            except socket.timeout:
                continue
            message = int.from_bytes(data[:2], "little")
            if message == 0:  # NAT_CONNECT: server info, Motive 3.1 streaming NatNet 4.1
                info = b"Motive".ljust(256, b"\0") + bytes([3, 1, 0, 0]) + bytes([4, 1, 0, 0]) + struct.pack(
                    "<Q", 10_000_000)
                server.sendto(struct.pack("<HH", 1, len(info)) + info, address)
            elif message == 4:  # NAT_REQUEST_MODELDEF
                server.sendto(encode_descriptions(bodies, 4, 1), address)

    thread = threading.Thread(target=motive, daemon=True)
    thread.start()
    client = DescriptionClient()
    client.set_client_address("127.0.0.1")
    client.set_server_address("127.0.0.1")
    client.set_use_multicast(False)
    client.multicast_address = "255.255.255.255"  # no multicast group on the loopback
    client.command_port = server.getsockname()[1]
    try:
        assert client.run()
        deadline = time.monotonic() + 3.0
        while not client.connected() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert client.connected() and (client.get_major(), client.get_minor()) == (4, 1)
        client.request()
        replies = []
        while not replies and time.monotonic() < deadline + 3.0:
            time.sleep(0.01)
            replies = client.take_replies()
        assert len(replies) == 1 and replies[0][1:] == (bodies, [])
    finally:
        stop.set()
        client.shutdown()
        thread.join(1.0)
        server.close()
