"""Motive's rigid body definitions and cameras read from NatNet's model descriptions, 3.0 to 4.1, and asked for."""

from __future__ import annotations

import socket
import struct
import threading
import time
from pathlib import Path

import pytest

from body_definition_fixtures import MARKERS, body
from husky_assembly_teleop.mocap_relay.body_definitions import (NAT_MODELDEF, RIGID_BODY, DescriptionClient,
                                                                encode_descriptions, parse_descriptions)
from husky_assembly_teleop.robot_interface.mocap_definitions import MocapCamera

#: The lab's Motive 3.0.3.1 (NatNet 4.0) answering NAT_REQUEST_MODELDEF, 23 Oct 2026: the "all" marker set and 10
#: cameras, no rigid bodies; the payload after the message header.
MOTIVE_3_0_REPLY = (Path(__file__).parent / "data" / "modeldef_motive_3_0_3_1.bin").read_bytes()
#: A camera as Motive gives it (Y-up), the first of that reply's.
CAMERA = MocapCamera("Prime 13 #31321", (3.823, 5.656, 5.158), (-0.346, 0.218, 0.141, 0.902))


@pytest.mark.parametrize("major, minor", [(3, 0), (4, 0), (4, 1)])
def test_descriptions_read_back_as_written(major, minor):
    """Each NatNet layout: names, ids and float32 offsets come back; from 4.1 each description carries its size."""
    bodies = [body(), body(MARKERS * 1.5, streaming_id=1850, name="base")]
    message = encode_descriptions(bodies, major, minor)
    assert struct.unpack_from("<H", message)[0] == NAT_MODELDEF
    read, cameras, problems = parse_descriptions(message[4:], major, minor)
    assert problems == [] and read == bodies and cameras == []


def test_descriptions_of_other_kinds_are_skipped_by_their_size():
    """From NatNet 4.1 a force plate between rigid bodies is passed over by its size."""
    message = encode_descriptions([body()], 4, 1)[4:]
    force_plate = struct.pack("<ii", 3, 12) + b"\x01" * 12
    payload = struct.pack("<i", 2) + force_plate + message[4:]
    read, cameras, problems = parse_descriptions(payload, 4, 1)
    assert problems == [] and read == [body()] and cameras == []


@pytest.mark.parametrize("major, minor", [(4, 0), (4, 1)])
def test_cameras_read_back_as_written(major, minor):
    """Name, position and float32 orientation, after the rigid bodies; from 4.1 checked against their size."""
    message = encode_descriptions([body()], major, minor, [CAMERA])
    read, cameras, problems = parse_descriptions(message[4:], major, minor)
    assert problems == [] and read == [body()] and len(cameras) == 1 and cameras[0].name == CAMERA.name
    assert cameras[0].position == pytest.approx(CAMERA.position) and \
        cameras[0].orientation == pytest.approx(CAMERA.orientation)


@pytest.mark.parametrize("orientation", [(0.0, 0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0, 1.0)])
def test_a_camera_without_a_rotation_is_left_out_and_said(orientation):
    """A zero or NaN quaternion would end the relay where it is turned: the camera is left out, the others kept."""
    bad = MocapCamera("PrimeX 22 #72318", (1.0, 2.0, 3.0), orientation)
    message = encode_descriptions([body()], 4, 1, [bad, CAMERA])
    read, cameras, problems = parse_descriptions(message[4:], 4, 1)
    assert [camera.name for camera in cameras] == [CAMERA.name] and read == [body()]
    assert len(problems) == 1 and problems[0].startswith("camera 'PrimeX 22 #72318'") and "left out" in problems[0]


def test_motive_3_0s_cameras_are_read():
    """Motive 3.0's reply (NatNet 4.0, no sizes): its marker set passed, its ten cameras read to the end."""
    read, cameras, problems = parse_descriptions(MOTIVE_3_0_REPLY, 4, 0)
    assert problems == [] and read == [] and len(cameras) == 10
    assert cameras[0].name == CAMERA.name and cameras[0].position == pytest.approx(CAMERA.position, abs=1e-3)
    assert cameras[-1].name == "PrimeX 22 #72653"


def test_natnet_4_0_rigid_bodies_after_the_cameras_are_read():
    """Rigid bodies behind Motive 3.0's cameras come back as written."""
    bodies = [body(), body(MARKERS * 1.5, streaming_id=1850, name="base")]
    count, = struct.unpack_from("<i", MOTIVE_3_0_REPLY)
    payload = struct.pack("<i", count + 2) + MOTIVE_3_0_REPLY[4:] + encode_descriptions(bodies, 4, 0)[8:]
    read, cameras, problems = parse_descriptions(payload, 4, 0)
    assert problems == [] and read == bodies and len(cameras) == 10


def test_an_unknown_kind_stops_reading_before_4_1():
    """Without a size, a kind NatNetClient cannot read either ends the reading, with the bodies read so far."""
    message = encode_descriptions([body()], 4, 0)[4:]
    payload = struct.pack("<i", 2) + message[4:] + struct.pack("<i", 9) + b"\x01" * 8
    read, _cameras, problems = parse_descriptions(payload, 4, 0)
    assert read == [body()] and problems == ["description 2 of 2 is of kind 9, which NatNet 4.0 gives no size for: "
                                             "stopped reading there"]


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
                server.sendto(encode_descriptions(bodies, 4, 1, [CAMERA]), address)

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
        assert len(replies) == 1 and (replies[0].bodies, replies[0].problems) == (bodies, [])
        assert [camera.name for camera in replies[0].cameras] == [CAMERA.name]
    finally:
        stop.set()
        client.shutdown()
        thread.join(1.0)
        server.close()
