"""Robot interfaces without ROS: a node with a settable clock and a log, publishers that keep what they send."""

from __future__ import annotations

import asyncio
from contextlib import contextmanager
from types import SimpleNamespace
from typing import Iterator

import numpy as np
from builtin_interfaces.msg import Time
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.config import ArmConfig
from husky_assembly_teleop.robot_interface.arm import (SCALED_JOINT_TRAJECTORY_CONTROLLER, UR_JOINT_NAMES, ArmInterface,
                                                       ArmState)
from husky_assembly_teleop.robot_interface.controller_manager import ControllerManagerInterface
from husky_assembly_teleop.robot_interface.event import Event
from husky_assembly_teleop.robot_interface.simulated_parts import SimulatedPart
from husky_assembly_teleop.robot_interface.robot_lease import Lease, LeaseSlot
from husky_assembly_teleop.robot_interface.soft_stop import SoftStopLatch
from husky_assembly_teleop.robot_interface.ur_frames import BASE_LINK_FROM_UR_BASE
from scaffolding_assembly_core.geometry import Pose

#: The lease every test holds (conftest.py) and every fake robot part is leased to.
TEST_LEASE = Lease("a test", ())

#: Where the fake arm's tool0 is, in its base_link: well inside the UR5e's reach.
TCP_POSITION = np.array([0.4, 0.0, 0.4])
TCP_ORIENTATION = np.array([0.0, 0.0, 0.0, 1.0])


def leased_slot(serial: str) -> LeaseSlot:
    """A robot's lease slot, leased to TEST_LEASE."""
    guard = LeaseSlot(serial)
    guard.lease = TEST_LEASE
    return guard


def take_test_lease(_serials, _label) -> Lease:
    """`ctx.take_lease` of a fake context: the lease every test holds."""
    return TEST_LEASE


@contextmanager
def lease_test_robots(_serials, _label) -> Iterator[Lease]:
    """`ctx.lease_robots` of a fake context: the lease every test holds."""
    yield TEST_LEASE


async def wait_ticks_until(predicate, timeout_s=None, description="") -> None:
    """`ctx.wait_until` of a fake context: yield to the loop until `predicate` holds."""
    while not predicate():
        await asyncio.sleep(0)


class FreeRobots(dict):
    """`world.robots` of a fake context: any serial is a robot whose lease is free."""

    def __missing__(self, serial: str):
        """A robot with only a free lease slot."""
        robot = self[serial] = SimpleNamespace(lease_slot=LeaseSlot(serial))
        return robot


def lease_context(**fields) -> SimpleNamespace:
    """A fake plugin context with `take_lease` and free robots, and `fields`."""
    return SimpleNamespace(take_lease=take_test_lease, world=SimpleNamespace(robots=FreeRobots()), **fields)


class Publisher:
    """Keeps what is published."""

    def __init__(self):
        """Nothing published yet."""
        self.sent = []

    def publish(self, message):
        """Keep the message."""
        self.sent.append(message)


class Future:
    """A service call's future that the test answers with `answer` or `fail`."""

    def __init__(self, request):
        """Unanswered."""
        self.request, self._callbacks, self._result, self._error = request, [], None, None

    def add_done_callback(self, callback) -> None:
        """Run `callback(self)` once answered."""
        self._callbacks.append(callback)

    def result(self):
        """The answer, or raise what the call raised."""
        if self._error is not None:
            raise self._error
        return self._result

    def answer(self, response) -> None:
        """Answer with `response` and run the callbacks."""
        self._result = response
        for callback in self._callbacks:
            callback(self)

    def fail(self, error: Exception) -> None:
        """Make the call raise `error` and run the callbacks."""
        self._error = error
        for callback in self._callbacks:
            callback(self)


class Client:
    """A service client that keeps every call; `ready` says whether the service is up."""

    def __init__(self, ready: bool = True):
        """No calls yet."""
        self.ready, self.calls = ready, []

    def service_is_ready(self) -> bool:
        """Whether the service is up."""
        return self.ready

    def call_async(self, request) -> Future:
        """Keep the call and return its future."""
        self.calls.append(Future(request))
        return self.calls[-1]


class Log:
    """A node logger that keeps (level, text) of every line."""

    def __init__(self):
        """Nothing logged yet."""
        self.lines: list[tuple[str, str]] = []

    def _keep(self, level: str):
        return lambda text, **_options: self.lines.append((level, text))

    def __getattr__(self, level: str):
        """debug, info, warning, warn, error and fatal all keep their line."""
        return self._keep(level)

    def text(self, level: str = "") -> str:
        """Every kept line of `level` (all levels by default), one per line."""
        return "\n".join(text for kept, text in self.lines if not level or kept == level)


class Node:
    """A node whose ROS time is `now`, seconds; set it to move time on."""

    def __init__(self, now: float = 100.0):
        """A node at ROS time `now`."""
        self.now = now
        self.log = Log()

    def get_logger(self) -> Log:
        """The kept log."""
        return self.log

    def get_clock(self):
        """A clock that reads `now`."""
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(round(self.now * 1e9)),
                                                           to_msg=lambda: Time(sec=int(self.now))))


class Controllers:
    """Stands in for ControllerManagerInterface: `state`, judged by the real `is_active` and `running_problem`."""

    is_active = ControllerManagerInterface.is_active
    running_problem = ControllerManagerInterface.running_problem

    def __init__(self, state, namespace: str, node: Node | None = None):
        """Read `state`, a ControllerManagerState, at the time of `node`."""
        self.state, self.namespace, self._node = state, namespace, node or Node()


def fake_arm(controller: str | None = SCALED_JOINT_TRAJECTORY_CONTROLLER, joints=None, now: float = 100.0,
             simulated: SimulatedPart | None = None, **config) -> ArmInterface:
    """An operational arm at rest at `joints` (all zero by default), measured at `now`, with `controller` running.

    Its tool0 is at TCP_POSITION, TCP_ORIENTATION; `config` goes to its ArmConfig.
    """
    arm = ArmInterface.__new__(ArmInterface)
    arm._node, arm.config = Node(now), ArmConfig(name="ur_arm", ros_namespace="ur5e", **config)
    arm.base_in_husky, arm.pendant_tcp, arm.urdf_problem, arm.end_effector = Pose(), Pose(), None, None
    arm.state, arm._test_mode_logged, arm._motion_reference, arm._free_drive_on = ArmState(), {}, {}, False
    arm._free_drive_until, arm._free_drive_leases = float("-inf"), frozenset()
    arm._simulated, arm._latch, arm._lease_slot = (lambda: simulated), SoftStopLatch("0804"), leased_slot("0804")
    arm.controllers = Controllers(arm.state.controllers, "/a200_0804/ur5e", arm._node)
    arm._trajectory, arm._target_frame, arm._target_wrench = Publisher(), Publisher(), Publisher()
    arm.trajectories_sent, arm.targets_sent, arm.wrenches_sent = Event("ur_arm_trajectory"), Event("ur_arm_target"), \
        Event("ur_arm_wrench")
    state = arm.state
    state.controllers.active, state.controllers.last_update_time = controller, now
    joints = np.zeros(len(UR_JOINT_NAMES)) if joints is None else np.asarray(joints, dtype=float)
    state.joint_positions = {f"ur_arm_{name}": float(value) for name, value in zip(UR_JOINT_NAMES, joints)}
    state.last_update_time = now
    state.status_update_time, state.dashboard_up, state.operational, state.program_playing = now, True, True, True
    state.tcp_position, state.tcp_orientation, state.tcp_update_time = TCP_POSITION.copy(), TCP_ORIENTATION.copy(), now
    state.tcp_raw_position = BASE_LINK_FROM_UR_BASE.inv().apply(TCP_POSITION)
    state.tcp_raw_orientation = (BASE_LINK_FROM_UR_BASE.inv() * Rotation.from_quat(TCP_ORIENTATION)).as_quat()
    return arm
