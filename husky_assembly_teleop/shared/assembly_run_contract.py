"""
What the assembly_run plugin offers other plugins (`AssemblyRunContract`, `RunView`), and the rules it marks by.

A robot is live while it is in the scene with its base tracked and every joint measured, both fresh (`is_live`). A mark
records every live robot where it is; a robot not live at its own action's mark gets the export's end, once the
operator confirms (`exported_end`).

Pure: any thread.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence

from scaffolding_assembly_core.process.plan_check import end_state
from scaffolding_assembly_core.robot import RobotObject

from .assembly_planning.scene_for import RobotBelief
from .assembly_planning.process_steps import LoadedProcess
from .runs.runs import (ASSUMED, DONE, LIVE, LIVE_BASE_AGE, LIVE_JOINTS_AGE, ActionRecord, Progress, RobotRecord,
                        RunSettings)


@dataclass(frozen=True)
class RunView:
    """The assembly run as other plugins read it; replaced whenever anything in it changes, never edited.

    Attributes:
        folder: The run's folder, or None before the first write makes one.
        process: The loaded process it runs, or None.
        statuses: Action id -> PENDING or DONE, every scheduled action.
        assumed: Actions marked done with their robot's end assumed from the export.
        current: (action id, movement index) the operator is at, or None.
        beliefs: Live robot id -> where it was last recorded, for planning scenes (`scene_for` uses them only for
            robots not measured).
        settings: The run's settings.
        offer: The name of the run used last while it is offered (made on another export, or used after `match`;
            `match` itself while a newer run folder can't be read), else "".
        match: An older run made on exactly these process files, offered beside `offer`, else "".
        revision: Goes up with every change.
    """

    folder: Path | None
    process: LoadedProcess | None
    statuses: Mapping[str, str]
    assumed: frozenset[str]
    current: tuple[str, int] | None
    beliefs: Mapping[str, RobotBelief]
    settings: RunSettings
    offer: str
    match: str
    revision: int


class AssemblyRunContract(Protocol):
    """What the assembly_run plugin offers: type `ctx.require("assembly_run")` as this.

    ! Call the requests on the main thread. They never raise for the operator's mistakes: those are logged and shown.
    """

    @property
    def view(self) -> RunView:
        """RunView: The run now; the same object until something changes."""

    def request_set_current(self, action_id: str, movement_index: int | None = None) -> None:
        """Make an action (at a movement, else its first step) current and select it in the process plugin."""

    def request_mark_done(self, action_id: str | None = None) -> None:
        """Mark an action (the current one if None) done; asks to confirm when its robot is not live."""

    def request_reopen(self, action_id: str | None = None) -> None:
        """Put a done action (the current one if None) back to pending, and make it current."""

    def request_new_run(self) -> None:
        """Start a new run after the operator confirms; the old one is kept."""

    def request_continue(self, run_name: str | None = None) -> None:
        """Continue an offered run: the one used last if None; one made on another export goes on in a new run."""

    def request_settings(self, **changes: Any) -> None:
        """Change run settings (`RunSettings` fields); saved with the run."""

    def request_record(self, event: str, *, action: str | None = None, movement: str | None = None,
                       data: Mapping[str, Any] | None = None, file: Mapping[str, Any] | None = None) -> None:
        """Record an event, with `data` as its fields; `file` is written as a movement record the event names."""


def is_live(robot: RobotObject, now: float) -> bool:
    """Whether a snapshot robot is measured now: in the scene, every movable joint measured, base tracked, all fresh.

    Fresh: the base fixed within LIVE_BASE_AGE, the joints within LIVE_JOINTS_AGE.

    Args:
        robot: The robot in this tick's snapshot.
        now: ROS time now, seconds.
    """
    return (robot.enabled and robot.base_tracked and not robot.unmeasured
            and robot.base_time is not None and now - robot.base_time <= LIVE_BASE_AGE
            and robot.joints_time is not None and now - robot.joints_time <= LIVE_JOINTS_AGE)


def live_record(robot: RobotObject, at: str) -> RobotRecord:
    """Where a snapshot robot is: its base and movable joints, source LIVE, at time `at`."""
    return RobotRecord(robot.base, {name: float(robot.joints[name]) for name in robot.model.movable_joints}, LIVE, at)


def exported_end(loaded: LoadedProcess, action_id: str, at: str = "") -> RobotRecord | None:
    """Where the export leaves an action's robot: its last movement's end; None where its base or joints are null.

    Args:
        loaded: The loaded process.
        action_id: The action.
        at: The time to record.

    Returns:
        RobotRecord | None: Source ASSUMED, or None.
    """
    action = loaded.process.actions[action_id]
    if not action.robot or not action.movements:
        return None
    state = end_state(action.movements[-1]).robots.get(action.robot)
    if state is None or state.base is None or state.joints is None:
        return None
    return RobotRecord(state.base, dict(state.joints), ASSUMED, at)


def beliefs_of(progress: Progress) -> dict[str, RobotBelief]:
    """Every recorded robot as a planning belief, by live id."""
    return {live: RobotBelief(record.base, dict(record.joints)) for live, record in progress.robots.items()}


def first_pending(progress: Progress, schedule: Sequence[str]) -> str | None:
    """The first scheduled action not done, or None when all are."""
    return next((action_id for action_id in schedule
                 if progress.actions.get(action_id, ActionRecord()).status != DONE), None)
