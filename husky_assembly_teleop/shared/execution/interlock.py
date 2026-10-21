"""
The one-motion-at-a-time lock: one automatic motion across every robot, and every other motion command refused then.

`take` holds a robot only while no robot is held, no robot is leased (`robot_interface/robot_lease.py`) and nothing
moves (`busy_reason`); `give_back` returns it. While any robot is held, a plugin about to command any robot calls
`refuse_while_held` first and sends nothing if it refuses; `PluginContext` gives no robot lease to anyone else.

- ! Two leases: this lock (`HuskyRobotInterface.lease`, our `Lease`) is across every robot; the robot lease
  (`HuskyRobotInterface.lease_slot`) is per robot, and every motion command needs it. The holder of this lock also
  takes the held robot's robot lease (movement_exec), so its sends pass, and the robot's other users are kept off.

- Hold and Stop are never refused: Hold calls the lease's `stop`, which cancels the holder's work and holds every arm.
- "Moving" fails safe: an arm or base whose states went stale after being live counts as moving (nobody can tell).
- ! Main thread only. The lease is the only way a robot's `lease` is set (`robot_interface.robot.Lease`).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

from ...robot_interface.arm import JOINT_STATE_MAX_AGE
from ...robot_interface.robot import Lease

if TYPE_CHECKING:
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.base import BaseState
    from ...robot_interface.robot import HuskyRobotInterface
    from ...world.measured import WorldState

#: An arm whose joints moved slightly (`ArmState.last_slight_motion_time`) within this many seconds is moving.
MOVED_WITHIN = 2.0
#: A wheel turning faster than this, rad/s, drives the base.
WHEEL_MOVING = 0.05
#: A follower report younger than this, seconds, saying "following", drives the base.
FOLLOWER_FRESH = 2.0
#: What clears a robot whose states went stale: it counts as moving for good (nobody can tell) until then.
UNTIL_BACK = "no automatic motion until they come back (check its link in Health) or the monitor restarts"


class Busy(Exception):
    """A lease refused; the message says why."""


def take(world: WorldState, serial: str, holder: str, label: str, stop: Callable[[], None], now: float) -> Lease:
    """Hold a robot for an automatic motion.

    Args:
        world: The measured world, `ctx.world`.
        serial: The robot to hold.
        holder: The plugin taking it, e.g. "movement_exec".
        label: What it runs, e.g. "B20_R_home".
        stop: Cancels the holder's work; Hold on any of the robot's arms calls it.
        now: ROS time, seconds.

    Returns:
        Lease: The lease, set on the robot.

    Raises:
        Busy: If the robot is not configured, any robot is held, or anything moves (`busy_reason`).
    """
    if serial not in world.robots:
        raise Busy(f"{serial} is not configured")
    reason = busy_reason(world, now)
    if reason:
        raise Busy(reason)
    lease = Lease(holder, label, now, stop)
    world.robots[serial].lease = lease
    return lease


def give_back(world: WorldState, lease: Lease) -> None:
    """Return a lease: cleared only on a robot that still holds this very lease."""
    for robot in world.robots.values():
        if robot.lease is lease:
            robot.lease = None


def lease_of(robot: HuskyRobotInterface) -> Lease | None:
    """The lease a robot is held by, or None."""
    return robot.lease


def held(world: WorldState) -> tuple[str, Lease] | None:
    """The first robot held, and its lease; None while no robot is held."""
    return next(((serial, robot.lease) for serial, robot in world.robots.items() if robot.lease is not None), None)


def busy_reason(world: WorldState, now: float, besides: str | None = None) -> str:
    """Why no automatic motion may start now: a robot is held or leased, or an arm or a base moves; "" if one may.

    A robot leased by any plugin (robot_control's teleop, a planner's commit, a base experiment) is busy: it may move.

    Args:
        world: The measured world.
        now: ROS time, seconds.
        besides: A robot left out (its leases, arms and base): the one an automatic motion is for, or None.
    """
    for serial, robot in world.robots.items():
        if serial != besides and lease_of(robot) is not None:
            return f"{serial} is held by {robot.lease.holder} ({robot.lease.label})"
    for serial, robot in world.robots.items():
        leased = robot.lease_slot.lease
        if serial != besides and leased is not None:
            return f"{serial} is leased by {leased.holder}"
    for serial, robot in world.robots.items():
        if serial == besides:
            continue
        for name, arm in robot.arms.items():
            why = arm_moving(arm, f"{serial} {name}", now)
            if why:
                return why
        why = base_moving(robot.base.state, f"{serial} base", now)
        if why:
            return why
    return ""


def arm_moving(arm: ArmInterface, label: str, now: float) -> str:
    """Why an arm counts as moving, or "".

    It moves while `is_executing` holds or its joints moved slightly within MOVED_WITHIN. Joint states never received
    this session are not counted (the robot is not running); states older than JOINT_STATE_MAX_AGE count as moving.

    Args:
        arm: The arm.
        label: Its name in the reason, e.g. "a200-0805 ur_arm".
        now: ROS time, seconds.
    """
    state = arm.state
    if state.last_update_time is None:
        return ""
    age = now - state.last_update_time
    if age > JOINT_STATE_MAX_AGE:
        return f"can't tell whether {label} is moving: its joint states are {age:.1f} s old; {UNTIL_BACK}"
    if state.is_executing:
        return f"{label} is running a trajectory"
    if state.last_slight_motion_time is not None and now - state.last_slight_motion_time <= MOVED_WITHIN:
        return f"{label} moved within the last {MOVED_WITHIN:.0f} s"
    return ""


def base_moving(state: BaseState, label: str, now: float) -> str:
    """Why a base counts as moving, or "": following a path, wheels turning, or wheel states gone stale.

    ? A path handed to the follower just before a lease counts only once the follower reports it (about 30 times a
      second) or the wheels turn: the monitor keeps no time for a path it sent. The watch during the automatic
      motion sees it then and holds the held robot.

    Args:
        state: The base's state.
        label: Its name in the reason, e.g. "a200-0806 base".
        now: ROS time, seconds.
    """
    follower = state.follower
    if follower is not None and follower.state == "following" and now - follower.update_time <= FOLLOWER_FRESH:
        return f"{label} is following a path"
    if state.joint_states_update_time is None:
        return ""
    age = now - state.joint_states_update_time
    if age > JOINT_STATE_MAX_AGE:
        return f"can't tell whether {label} is moving: its wheel states are {age:.1f} s old; {UNTIL_BACK}"
    if any(abs(speed) > WHEEL_MOVING for speed in state.wheel_velocities.values()):
        return f"{label} is driving"
    return ""


def refuse_while_held(world: WorldState, what: str, log: Callable[[str], None]) -> bool:
    """Refuse a motion command while any robot is held for an automatic motion, saying so in the log.

    ! Never for Hold or Stop: stopping is always allowed.

    Args:
        world: The measured world.
        what: The command, e.g. "a200-0805 ur_arm Start".
        log: Gets the refusal, e.g. `ctx.log_warn`.

    Returns:
        bool: True if refused (send nothing), False while no robot is held.
    """
    found = held(world)
    if found is None:
        return False
    serial, lease = found
    log(f"{what} refused: {serial} is held by {lease.holder} ({lease.label}), one motion at a time; Cancel it first")
    return True
