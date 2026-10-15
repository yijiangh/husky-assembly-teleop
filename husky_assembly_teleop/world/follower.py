"""
Judging the onboard path follower's reports: whether one runs, and how it does with a path it was sent.

Only reads stored fields, so it needs no ROS; receiving the reports is robot_interface/base.py's job.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .checks import BAD, GOOD, STALE_AFTER, WARN, Check

if TYPE_CHECKING:
    from ..robot_interface.base import FollowerState


def live_report(follower: FollowerState | None, now: float) -> FollowerState | None:
    """The follower's last report if it came within STALE_AFTER seconds, else None: no follower runs."""
    if follower is None or now - follower.update_time >= STALE_AFTER:
        return None
    return follower


def path_check(follower: FollowerState | None, path_id: int, sent_at: float, now: float) -> Check:
    """Judge how the follower does with path `path_id`: WARN while under way, GOOD done, BAD for any other end.

    Args:
        follower: The base's last follower report.
        path_id: The path's id, as `BaseInterface.send_path` returned it.
        sent_at: ROS time the path went out, seconds.
        now: Current ROS time, seconds.

    Returns:
        Check: The path's chip; its label and detail also say how a run ended.
    """
    follower = live_report(follower, now)
    if follower is None:
        return Check("follower silent", BAD, f"no report from the follower for over {STALE_AFTER:g}s")
    if follower.path_id != path_id:
        # * Until its first report on the new path, the follower may still report the old one.
        if follower.update_time < sent_at + STALE_AFTER:
            return Check("sent", WARN, "waiting for the follower to take the path")
        return Check("replaced", BAD, f"the follower is on path {follower.path_id}, not ours")
    if follower.state == "following":
        return Check(f"following {follower.progress:.0%}", WARN)
    if follower.state == "done":
        return Check("done", GOOD)
    return Check(follower.state, BAD, follower.reason)
