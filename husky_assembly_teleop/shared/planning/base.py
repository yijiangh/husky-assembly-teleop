"""What the base planner plugin plans for (`BasePlanRequest`) and what it offers others (`BasePlannerPluginApi`)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .service import PlannerPluginApi


@dataclass(frozen=True)
class BasePlanRequest:
    """A base path for robot `serial` from where it is now to `goal`, and whether to record the attempt.

    Attributes:
        serial: The robot to plan for.
        goal: (x, y, yaw) to end at, metres and radians, world frame.
        algorithm: The algorithm's label, as in the panel's Algorithm dropdown; "" for the default.
        settings: (name, value) pairs that replace the algorithm's default settings.
        debug: Whether the search keeps a record of how it worked, for the debug view.
        task: The experiment task chosen, by name; the attempt counts for it only from its start to its target.
        record: Whether to save the attempt, and the run once committed, to the Drive.
        experiment: The experiment the recordings are saved under.
    """

    serial: str
    goal: tuple[float, float, float]
    algorithm: str = ""
    settings: tuple[tuple[str, Any], ...] = ()
    debug: bool = False
    task: str | None = None
    record: bool = False
    experiment: str = "default"


#: What the base planner plugin offers: `planner: BasePlannerPluginApi = ctx.require("base_planner")`.
BasePlannerPluginApi = PlannerPluginApi[BasePlanRequest]
