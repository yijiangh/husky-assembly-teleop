"""
The stops and gripper failures a run records by itself (`FaultWatch`), read from each robot's state.

! A dead gripper link is not here: only the driver's own log says so, and `experiments/incidents` reads it from the
  robots' panes.
"""

from __future__ import annotations

from typing import Any, Hashable, Mapping

from ur_dashboard_msgs.msg import SafetyMode

from ...robot_interface.end_effectors import EndEffector, RobotiqState, robotiq_failure
from ...robot_interface.robot import HuskyRobotInterface

#: The events, for `request_record`.
SAFETY_MODE, BASE_ESTOP, GRIPPER_FAILED = "safety_mode", "base_estop", "gripper_failed"
#: SafetyMode constant -> readable name, e.g. 3 -> "protective stop".
SAFETY_MODE_NAMES = {getattr(SafetyMode, name): name.lower().replace("_", " ")
                     for name in dir(SafetyMode) if name.isupper() and isinstance(getattr(SafetyMode, name), int)}


class FaultWatch:
    """What each robot showed at the last look: a change is an event, and so is a stop already there at the first."""

    def __init__(self):
        """Have seen nothing yet."""
        self._seen: dict[Hashable, Any] = {}

    def changes(self, robots: Mapping[str, HuskyRobotInterface]) -> list[tuple[str, dict[str, Any]]]:
        """The events since the last call, each with its data for the record.

        Args:
            robots: Serial -> robot (`ctx.world.robots`).

        Returns:
            list[tuple[str, dict[str, Any]]]: (event, data). Each event's return to normal is one too, except a
            gripper's: that needs no action from anyone.
        """
        found = []
        for serial, robot in robots.items():
            engaged = robot.base.state.estopped
            if engaged is not None and self._is_news((serial, BASE_ESTOP), engaged, False):
                found.append((BASE_ESTOP, {"serial": serial, "engaged": engaged}))
            for name, arm in robot.arms.items():
                mode = arm.state.safety_mode
                was = self._seen.get((serial, name, SAFETY_MODE))
                if mode is not None and self._is_news((serial, name, SAFETY_MODE), mode, SafetyMode.NORMAL):
                    found.append((SAFETY_MODE, {"serial": serial, "arm": name, "mode": _mode_name(mode),
                                                "was": None if was is None else _mode_name(was)}))
                if arm.end_effector is not None:
                    found += [(GRIPPER_FAILED, {"serial": serial, "arm": name, **data})
                              for data in self._tool_failures((serial, name), arm.end_effector)]
        return found

    def _tool_failures(self, key: tuple[str, str], tool: EndEffector) -> list[dict[str, Any]]:
        """A tool's new failures: a command refused before it was sent and, for a Robotiq, one that failed."""
        found = []
        refusal = tool.last_refusal
        # * At the first look a tool's last failure is old news.
        if self._is_news((*key, "refusal"), refusal, refusal) and refusal is not None \
                and not refusal.label.startswith("joint screw"):  # not the gripper; stalled, a joint is in
            found.append({"failure": refusal.label, "reason": refusal.reason})
        state = tool.state
        if isinstance(state, RobotiqState):
            failed = robotiq_failure(state)
            if self._is_news((*key, "result"), failed, failed) and failed is not None:
                found.append({"failure": failed[0], "reason": failed[1], "commanded": state.commanded_position,
                              "effort": state.commanded_effort, "position": state.position})
            error = state.reactivate_error
            if self._is_news((*key, "reactivate"), error, error) and error:
                found.append({"failure": "reactivate failed", "reason": error})
        return found

    def _is_news(self, key: Hashable, value: Any, quiet: Any) -> bool:
        """Whether `value` differs from the last seen, or, the first time, from `quiet`; remember it."""
        news = self._seen.get(key, quiet) != value
        self._seen[key] = value
        return news


def _mode_name(mode: int) -> str:
    """A SafetyMode's readable name."""
    return SAFETY_MODE_NAMES.get(mode, f"mode {mode}")
