"""
One planner search: what it is given (`Search`), how long it may take, and what it returns (`PlanResult`).

`with_guesses` lets a search plan from a robot's last known state when it is not measured now.

! A search's `work` runs on the planner's worker thread; the main thread ends it early through its abort flag.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Callable, Generic, Protocol, TypeVar

if TYPE_CHECKING:
    from scaffolding_assembly_core.scene import Scene

#: Give up searching after this long, seconds.
TIME_LIMIT = 10.0
#: Seconds to wait for the worker beyond the search's own time limit.
WAIT_MARGIN = 5.0

P = TypeVar("P")


class PlanningAborted(Exception):
    """Raised inside a search to end it early."""


@dataclass
class PlanResult(Generic[P]):
    """What a search came back with.

    Attributes:
        path: The path, or None if none was found.
        reason: Why not, when `path` is None; empty otherwise.
        seconds: How long it took, syncing the world included.
        direct: Whether the straight move was already free.
    """

    path: P | None
    reason: str = ""
    seconds: float = 0.0
    direct: bool = False


class PlanningWorld(Protocol):
    """A planner's private planning world, as far as the service needs it."""

    def close(self) -> None:
        """Disconnect the world. On the worker, once no search is running."""


@dataclass
class Search:
    """One search, built by the planner on the main thread (`make_search`).

    Attributes:
        label: Task name, e.g. "plan a200-0806".
        message: The plan's message while planning.
        work: Syncs the world and searches, on the worker; must stop soon after its `abort` is set.
        time_limit: Seconds to wait for `work`, margin included.
        accepted: Main thread, when a path was found: the plan's message and whether it is a warning.
        finished: Main thread, after `work` returned, path or not.
    """

    label: str
    message: str
    work: Callable[[], PlanResult]
    time_limit: float
    accepted: Callable[[PlanResult], tuple[str, bool]]
    finished: Callable[[], None] = lambda: None


def with_guesses(snapshot: Scene, object_id: str) -> tuple[Scene, list[str]]:
    """The snapshot with a robot's untracked base and unknown joints taken as known, and what that guessed.

    The guesses are the last mocap fix, else the default pose, and the stow joints, else 0. An absent robot is left
    as it is, so planning still refuses it.

    Returns:
        tuple[Scene, list[str]]: The snapshot, a copy only if something was guessed; the robot's problems guessed past.
    """
    robot = snapshot.robots[object_id]
    guessed = robot.acting_problems() if robot.enabled else []
    if not guessed:
        return snapshot, []
    known = replace(robot, base_tracked=True, unmeasured=frozenset())
    return replace(snapshot, robots={**snapshot.robots, object_id: known}), guessed
