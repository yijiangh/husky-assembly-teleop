"""The relay's marker cloud as the monitor stores it."""

from __future__ import annotations

import numpy as np
from crl_husky_msgs.msg import MocapMarkers

from husky_assembly_teleop.robot_interface.mocap import store_markers
from husky_assembly_teleop.world.measured import WorldState


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
