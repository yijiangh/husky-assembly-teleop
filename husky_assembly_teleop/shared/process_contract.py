"""
What the process plugin offers others: its selected movement (`Selection`), read through `ProcessContract`.

It also says whether the process's obstacles from Rhino count in collision checks (`obstacles_collide`), and whether
the transfer workaround is on (`transfer_as_free`). Others may show each action's mark in its row (`request_marks`), and
an action's later movements with joints found live (`request_carry`, a `Carry`).

Users type `ctx.require("process")` with it: `process: ProcessContract = ctx.require("process")`. The process, its
actions and movements are the core's types (`scaffolding_assembly_core.process`).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Protocol

from scaffolding_assembly_core.ids import IdMap
from scaffolding_assembly_core.process import Action, Movement, Process


#: An action's mark in the panel's action list (`ProcessContract.request_marks`): pending, done, or done with its end
#: assumed (its robot was not live).
MARK_PENDING, MARK_DONE, MARK_ASSUMED = "pending", "done", "assumed"


@dataclass(frozen=True)
class Carry:
    """Joints found live for one robot of an action: from `movement_id`'s end on, the panel draws `robot` at `joints`.

    Every later movement of the action shows them too, as the old monitor carried a solved goal forward. Only the
    drawing: plans, runs and records never read it.

    Attributes:
        action_id: The action.
        movement_id: The movement whose end the joints are, e.g. a transfer after Live IK or a plan.
        robot: The process robot id, e.g. "robots/cindy".
        joints: Joint name -> value, radians.
        source: Where they came from, for the panel: "live IK" or "plan".
    """

    action_id: str
    movement_id: str
    robot: str
    joints: Mapping[str, float]
    source: str = "live IK"


@dataclass(frozen=True)
class Selection:
    """The process plugin's selected movement, as other plugins read it; replaced whenever any field changes, never edited.

    The run (assembly_run) follows it: a movement the operator selects in the panel becomes the run's current one,
    except while movement_exec has a robot's lease for a step.

    Attributes:
        process: The loaded process, as authored; `process.folder` is where it was read from (the converted copy of
            an old export).
        index: Position of the selected movement in the authored `process.movements()`, from 0.
        action: The action as the panel shows it: with the transfer workaround, a jointing action's movements in run
            order (`steps.as_run`). Its `robot` is "" for a `bar_placing` action.
        movement: The selected movement as shown, with its authored id: with the workaround, the transfer is the
            transit, from the action's start with empty tools.
        ids: Process id -> live scene id ("bars/B1" -> "process/bars/B1", planned robot -> real robot), or None
            until the movement's scene bodies are put.
        process_ids: Live scene id -> process id, or None likewise.
    """

    process: Process
    index: int
    action: Action
    movement: Movement
    ids: IdMap | None = None
    process_ids: IdMap | None = None


class ProcessContract(Protocol):
    """What the process plugin offers other plugins: type `ctx.require("process")` as this."""

    @property
    def selection(self) -> Selection | None:
        """Selection | None: The selected movement, or None while no process with movements is loaded."""

    @property
    def obstacles_collide(self) -> bool:
        """bool: Whether the process's obstacles (Rhino's "obstacles/<name>") count in collision checks.

        False: they are only drawn (the panel's "Rhino obstacles collide" box).
        """

    @property
    def transfer_as_free(self) -> bool:
        """bool: The transfer workaround, the panel's "Transfer workaround" box: on at start, for this session only.

        On: a jointing action's transfer runs as a free move with empty tools to the insert start, where the operator
        places the bar.
        """

    def request_select(self, index: int) -> None:
        """Select a movement by index into the authored `process.movements()`, clamped; ignored with no process loaded.

        A movement the panel does not list (the workaround's travel to load) selects its action's first one listed.

        ! Main thread only.
        """

    def request_marks(self, marks: Mapping[str, str]) -> None:
        """Show each action's mark in its row of the action list: action id -> MARK_PENDING, MARK_DONE or MARK_ASSUMED.

        Replaces the marks given before; a new process starts without any. ! Main thread only.
        """

    def request_carry(self, carry: Carry | None) -> None:
        """Draw an action's later movements with joints found live (`Carry`); None draws them as authored again.

        Replaces the carry given before; a new process starts without one. ! Main thread only.
        """
