"""Tests for the marker cloud: the relay keeps every visible marker, the monitor stores the latest cloud."""

from __future__ import annotations

import numpy as np
from crl_husky_msgs.msg import MocapMarkers

from husky_assembly_teleop.mocap_relay.node import MARKER_OCCLUDED, MARKER_POINT_CLOUD_SOLVED, MocapRelay
from husky_assembly_teleop.robot_interface.mocap import store_markers
from husky_assembly_teleop.world.measured import WorldState


def test_relay_keeps_every_visible_marker_of_any_asset_or_none():
    """Markers of any asset and unlabeled ones go into the cloud, Z-up; occluded ones do not."""
    # * Only the NatNet callback's own state: no node, no NatNet client.
    relay = MocapRelay.__new__(MocapRelay)
    relay._pending_samples = {}
    relay._receive_labeled_markers({
        1020: {1: {"pos": (1.0, 2.0, 3.0), "error": 0.001, "param": MARKER_POINT_CLOUD_SOLVED},
               2: {"pos": (9.0, 9.0, 9.0), "error": 0.0, "param": MARKER_OCCLUDED}},
        0: {7: {"pos": (4.0, 5.0, 6.0), "error": 0.002, "param": MARKER_POINT_CLOUD_SOLVED}},
    })
    cloud = relay._pending_cloud
    # * Y-up to Z-up, as for poses: (x, y, z) -> (x, -z, y).
    assert cloud.positions == (1.0, -3.0, 2.0, 4.0, -6.0, 5.0)
    assert cloud.asset_ids == (1020, 0)
    assert cloud.member_ids == (1, 7)
    assert cloud.residuals == (0.001, 0.002)


def test_monitor_keeps_the_latest_cloud_as_arrays():
    """The cloud lands in `world.markers` as (n, 3) positions, stamped with its capture and arrival times."""
    world = WorldState()
    message = MocapMarkers(positions=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], asset_ids=[0, 1020], member_ids=[7, 1],
                           residuals=[0.0, 0.0])
    message.header.stamp.sec = 100
    store_markers(world, message, now=100.5)
    np.testing.assert_array_equal(world.markers.positions, [[1, 2, 3], [4, 5, 6]])
    assert list(world.markers.asset_ids) == [0, 1020]
    assert list(world.markers.member_ids) == [7, 1]
    assert (world.markers.capture_time, world.markers.arrival_time) == (100.0, 100.5)
