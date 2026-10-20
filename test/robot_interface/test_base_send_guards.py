"""The base's own guards: no command that is not finite, no path it may not follow, no twist on a stale controller."""

import itertools
import math
from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import Controllers, Node, Publisher

from husky_assembly_teleop.robot_interface.base import (FOLLOWER_SILENT_AFTER, PLATFORM_VELOCITY_CONTROLLER,
                                                        BaseInterface, BaseState, FollowerState)
from husky_assembly_teleop.robot_interface.controller_manager import CONTROLLER_STATE_MAX_AGE
from husky_assembly_teleop.robot_interface.event import Event
from husky_assembly_teleop.robot_interface.mocap_state import POSE_TIMEOUT
from husky_assembly_teleop.robot_interface.soft_stop import RobotStopped, SoftStopLatch

pytestmark = pytest.mark.safety

NOW = 100.0
#: A path every guard lets through.
PATH = ([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [0.0, 2.0], 0.3, 0.5)


def _base() -> BaseInterface:
    """A base ready to follow at ROS time NOW: a live mocap pose, a live follower, the velocity controller running."""
    base = BaseInterface.__new__(BaseInterface)
    base._node, base._config, base._latch = Node(NOW), SimpleNamespace(mocap_id=1), SoftStopLatch("0804")
    base.state = BaseState(tracked=True, tracking_valid=True, marker_errors=deque([(NOW, 0.0)]),
                           last_update_time=NOW, last_fix_time=NOW, estopped=False)
    base.state.follower = FollowerState(
        controller="pure_pursuit", state="idle", reason="", path_id=0, timed=False, piece=0, piece_count=0,
        turning=False, progress=0.0, reference=np.zeros(3), lookahead=np.zeros(2), cross_track_error=0.0,
        along_track_error=math.nan, yaw_error=0.0, command=np.zeros(2), update_time=NOW, stamp=NOW)
    base.state.controllers.active, base.state.controllers.last_update_time = PLATFORM_VELOCITY_CONTROLLER, NOW
    base.controllers = Controllers(base.state.controllers, "/a200_0804", base._node)
    base._cmd_vel, base._base_path, base._path_ids = Publisher(), Publisher(), itertools.count(1)
    base.twists_sent, base.paths_sent = Event("x_twist"), Event("x_path")
    return base


@pytest.mark.parametrize("twist", [(math.nan, 0.0), (0.0, math.inf), (-math.inf, math.nan)])
def test_twist_not_finite_is_refused(twist):
    """send_twist returns False, publishes nothing and fires no `twists_sent`."""
    base = _base()
    heard = []
    base.twists_sent.attach(heard.append)
    assert not base.send_twist(*twist)
    assert not base._cmd_vel.sent and not heard


def test_twist_refused_while_the_controller_state_is_stale():
    """A controller manager silent for over CONTROLLER_STATE_MAX_AGE keeps its last answer: no twist goes out."""
    base = _base()
    base._node.now += CONTROLLER_STATE_MAX_AGE + 0.1
    assert not base.send_twist(0.2, 0.0)
    assert base.send_twist(0.0, 0.0, require_controller=False), "a stop always goes out"
    assert len(base._cmd_vel.sent) == 1


@pytest.mark.parametrize("path, why", [
    (([0.0, math.nan], [0.0, 0.0], [0.0, 0.0], None, 0.3, 0.5), "not finite"),
    (([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [0.0, math.inf], 0.3, 0.5), "not finite"),
    (([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], None, math.nan, 0.5), "not finite"),
    (([0.0, 1.0], [0.0], [0.0, 0.0], None, 0.3, 0.5), "differ in length"),
    (([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [0.0], 0.3, 0.5), "differ in length"),
])
def test_path_not_finite_or_ragged_is_refused(path, why):
    """send_path raises before it takes a path id or publishes."""
    base = _base()
    with pytest.raises(ValueError, match=why):
        base.send_path(*path)
    assert not base._base_path.sent and next(base._path_ids) == 1


@pytest.mark.parametrize("change, why", [
    (lambda base: setattr(base.state, "estopped", True), "e-stopped"),
    (lambda base: setattr(base._node, "now", NOW + POSE_TIMEOUT + 0.01), "no mocap pose"),
    (lambda base: setattr(base.state, "follower", None), "no follower"),
])
def test_path_refused_while_the_base_may_not_follow(change, why):
    """An e-stopped base, a lost pose or no follower: `follow_problem` says why and send_path sends nothing.

    * An e-stopped base matters most: a path taken now would drive as soon as the e-stop is released.
    """
    base = _base()
    change(base)
    assert why in base.follow_problem()
    with pytest.raises(ValueError, match=why):
        base.send_path(*PATH)
    assert not base._base_path.sent


def test_a_silent_follower_counts_as_none():
    """A follower report older than FOLLOWER_SILENT_AFTER means no follower runs."""
    base = _base()
    base.state.follower.update_time -= FOLLOWER_SILENT_AFTER
    base.state.marker_errors.append((NOW, 0.0))
    assert "no follower" in base.follow_problem()


def test_finite_commands_go_out():
    """A finite twist and a finite path are published while the base may drive, and fired as sent; so is a stop."""
    base = _base()
    twists, paths = [], []
    base.twists_sent.attach(twists.append)
    base.paths_sent.attach(paths.append)
    assert base.follow_problem() == ""
    assert base.send_twist(0.2, -0.1)
    assert base.send_path(*PATH) == 1
    assert len(base._cmd_vel.sent) == 1 and list(base._base_path.sent[0].t) == [0.0, 2.0]
    base.stop_path()
    twist, = twists
    assert twist.command.tolist() == [0.2, -0.1] and twist.update_time == twist.stamp == NOW
    sent, stopped = paths
    assert (sent.path_id, sent.waypoints, sent.timed, sent.speed_caps.tolist()) == (1, 2, 1, [0.3, 0.5])
    assert (stopped.path_id, stopped.waypoints) == (1, 0)


def test_a_soft_stopped_base_refuses_to_drive_but_still_stops():
    """Twist and path raise RobotStopped and send nothing, the velocity controller may not start; stops still go out."""
    base = _base()
    base._latch.stopped, base._path_id = True, 0
    with pytest.raises(RobotStopped, match="soft stopped, a twist not sent"):
        base.send_twist(0.2, 0.0)
    with pytest.raises(RobotStopped, match="a path not sent"):
        base.send_path(*PATH)
    assert not base._cmd_vel.sent and not base._base_path.sent
    assert "soft stopped" in base._switch_refusal(PLATFORM_VELOCITY_CONTROLLER)
    assert base.send_twist(0.0, 0.0)
    base.stop_path()
    assert len(base._cmd_vel.sent) == 1 and len(base._base_path.sent) == 1
    base._latch.stopped = False
    assert base._switch_refusal(PLATFORM_VELOCITY_CONTROLLER) is None and base.send_twist(0.2, 0.0)
