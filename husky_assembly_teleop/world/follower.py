"""
Judging how the onboard path follower does with a path it was sent, as a chip (`path_check`).

Whether a follower runs at all is `robot_interface.base.live_report`; receiving the reports is the base's job.
"""

from __future__ import annotations

from ..robot_interface.base import FOLLOWER_SILENT_AFTER, FollowerState, live_report
from .checks import BAD, GOOD, PENDING, WARN, Check


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
        return Check("follower silent", BAD, f"no report from the follower for over {FOLLOWER_SILENT_AFTER:g}s")
    if follower.path_id != path_id:
        # * Until its first report on the new path, the follower may still report the old one.
        if follower.update_time < sent_at + FOLLOWER_SILENT_AFTER:
            return Check("sent", WARN, "waiting for the follower to take the path")
        return Check("replaced", BAD, f"the follower is on path {follower.path_id}, not ours")
    if follower.state == "following":
        return Check(f"following {follower.progress:.0%}", WARN)
    if follower.state == "done":
        return Check("done", GOOD)
    if follower.state == "idle":
        # * No path, or stopped by an empty one: Cancel and Stop all send that.
        return Check("idle", PENDING, follower.reason)
    return Check(follower.state, BAD, follower.reason)
