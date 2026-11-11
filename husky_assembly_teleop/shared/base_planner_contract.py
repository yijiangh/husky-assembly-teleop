"""
What the base planner plugin offers others: base paths to a floor goal (`BasePlanRequest`).

Users type `ctx.require("base_planner")` with it: `planner: BasePlannerContract = ctx.require("base_planner")`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol

from .planner.plan import PlannerContract

if TYPE_CHECKING:
    from ..plugin_api.context import PluginContext
    from ..world.checks import Check


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


class BasePlannerContract(PlannerContract[BasePlanRequest], Protocol):
    """What `ctx.require("base_planner")` offers: the generic planner, and how a robot's last commit ran."""

    def run_check(self, ctx: PluginContext, serial: str) -> Check | None:
        """The run chip of `serial`'s last commit: how it ended, else how the follower does; None before a commit.

        An ended run's detail says why it ended (e.g. the follower refused the path), cut to a hover's length.
        """
