"""Health's soft-stop chip: shown only while a robot is soft stopped, with why."""

from types import SimpleNamespace

from husky_assembly_teleop.plugins.health import soft_stop_check
from husky_assembly_teleop.robot_interface.soft_stop import SoftStopLatch
from husky_assembly_teleop.world.checks import BAD


def test_the_chip_shows_while_stopped_with_the_reason():
    """No chip while running; a red SOFT STOPPED chip with the reason while stopped."""
    latch = SoftStopLatch("0805")
    robot = SimpleNamespace(soft_stop_latch=latch)
    assert soft_stop_check(robot) is None
    latch.stopped, latch.reason = True, "robot 0805 cannot be checked: joints not known: ur_arm_elbow_joint"
    check = soft_stop_check(robot)
    assert (check.label, check.level) == ("SOFT STOPPED", BAD) and check.detail.startswith(latch.reason)
