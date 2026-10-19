"""The follower's verdicts: a report counts while it is fresh, and a sent path is under way, done, or lost."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from husky_assembly_teleop.robot_interface.base import FOLLOWER_SILENT_AFTER, FollowerState, live_report
from husky_assembly_teleop.world.checks import BAD, GOOD, WARN
from husky_assembly_teleop.world.follower import path_check

pytestmark = pytest.mark.safety

#: A report on path 7, received at ROS time 10 s.
REPORT = FollowerState(controller="pure_pursuit", state="following", reason="", path_id=7, timed=False, piece=0,
                       piece_count=3, turning=False, progress=0.4, reference=np.zeros(3), lookahead=np.zeros(2),
                       cross_track_error=0.0, along_track_error=np.nan, yaw_error=0.0, command=np.zeros(2),
                       update_time=10.0, stamp=10.0)


def test_report_counts_while_fresh():
    """A report is live for FOLLOWER_SILENT_AFTER seconds; none is never live."""
    assert live_report(REPORT, 10.0 + FOLLOWER_SILENT_AFTER / 2) is REPORT
    assert live_report(REPORT, 10.0 + FOLLOWER_SILENT_AFTER) is None
    assert live_report(None, 10.0) is None


@pytest.mark.parametrize("report, sent_at, label, level", [
    (REPORT, 9.0, "following 40%", WARN),
    (replace(REPORT, state="done"), 9.0, "done", GOOD),
    (replace(REPORT, state="aborted", reason="off the path"), 9.0, "aborted", BAD),
    (replace(REPORT, path_id=6), 9.5, "sent", WARN),
    (replace(REPORT, path_id=8), 8.0, "replaced", BAD),
    (None, 9.0, "follower silent", BAD),
])
def test_path_check(report, sent_at, label, level):
    """Under way is WARN (sent, or following); every way a run ends is GOOD (done) or BAD."""
    check = path_check(report, 7, sent_at, 10.5)
    assert (check.label, check.level) == (label, level)
