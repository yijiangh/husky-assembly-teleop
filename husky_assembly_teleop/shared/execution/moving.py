"""
Whether anything moves: an automatic motion starts only while no other arm or base moves, and stops if one starts.

It watches what the robots report, so it also sees motion this monitor never sent (the teach pendant, a node on the
robot). Who may command a robot is the robot lease's job (`robot_interface/robot_lease.py`), not this module's.

- "Moving" fails safe: an arm or base whose states went stale after being live counts as moving (nobody can tell).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from ...robot_interface.arm import JOINT_STATE_MAX_AGE

if TYPE_CHECKING:
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.base import BaseState
    from ...world.measured import WorldState

#: An arm whose joints moved slightly (`ArmState.last_slight_motion_time`) within this many seconds is moving.
MOVED_WITHIN = 2.0
#: A wheel turning faster than this, rad/s, drives the base.
WHEEL_MOVING = 0.05
#: A follower report younger than this, seconds, saying "following", drives the base.
FOLLOWER_FRESH = 2.0
#: What clears a robot whose states went stale: it counts as moving for good (nobody can tell) until then.
UNTIL_BACK = "no automatic motion until they come back (check its link in Health) or the monitor restarts"


def moving_reason(world: WorldState, now: float, besides: str | None = None) -> str:
    """Why no automatic motion may start or go on now: an arm or a base moves; "" if none does.

    Leases are not checked here: the automatic motion leases every robot first, so nothing else here can command one.

    Args:
        world: The measured world.
        now: ROS time, seconds.
        besides: A robot left out (its arms and base): the one an automatic motion is for, or None.
    """
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
      motion sees it then and holds the acting robot's arms.

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
