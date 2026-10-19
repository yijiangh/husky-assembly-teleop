"""
What the process plugin offers others: its selected step (`Selection`), read through `ProcessContract`.

Users type `ctx.require("process")` with it: `process: ProcessContract = ctx.require("process")`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from scaffolding_assembly_core.ids import IdMap

from .loaded_process import LoadedProcess, Step


@dataclass(frozen=True)
class Selection:
    """The process plugin's selected step, as other plugins read it; replaced whenever any field changes, never edited.

    Attributes:
        loaded: The loaded process.
        index: Position of `step` in `loaded.steps`.
        step: The selected movement.
        ids: Process id -> live scene id ("bars/B1" -> "process/bars/B1", planned robot -> real robot), or None
            until the step's scene bodies are put.
        process_ids: Live scene id -> process id, or None likewise.
    """

    loaded: LoadedProcess
    index: int
    step: Step
    ids: IdMap | None = None
    process_ids: IdMap | None = None


class ProcessContract(Protocol):
    """What the process plugin offers other plugins: type `ctx.require("process")` as this."""

    @property
    def selection(self) -> Selection | None:
        """Selection | None: The selected step, or None while no process with steps is loaded."""
