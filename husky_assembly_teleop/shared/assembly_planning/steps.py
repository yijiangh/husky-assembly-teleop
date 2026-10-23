"""
The steps an operator walks through in one action: each runs one movement, with any tool movements folded into it.

- An ungrasp (a tool movement opening every grip it commands) directly before a two-arm linear retreat runs with
  that retreat.
- With the transfer workaround the free transfer runs first, with empty tools. The operator and tool movements
  authored before it (place, grasp) are done after its motion, in a pause at the insertion start, and the travel to
  load before them is skipped.

Pure: any thread.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from scaffolding_assembly_core.process import Action, Process

from .movement import MovementSpec, Routine, UnknownMovement, classify_action

#: Routines done in the pause after a free transfer (placing the bar by hand, closing the tools).
_PAUSE = (Routine.OPERATOR, Routine.TOOL_ONLY)


@dataclass(frozen=True)
class OperatorStep:
    """One step of an action, by movement index in the action.

    Attributes:
        primary: The movement the step loads and runs.
        with_motion: Tool movements carried out while the primary moves (the ungrasp during the retreat).
        after_motion: Movements done after the primary's motion, in this order, in a pause before the next step
            (with the transfer workaround: place and grasp the bar in the empty tools at the insertion start).
        skipped: Movements not run at all, carried by this step (with the workaround: the travel to load).
    """

    primary: int
    with_motion: tuple[int, ...] = ()
    after_motion: tuple[int, ...] = ()
    skipped: tuple[int, ...] = ()

    @property
    def indices(self) -> tuple[int, ...]:
        """tuple[int, ...]: What the step runs, in order: the primary and what runs during it, then the pause after."""
        return (self.primary, *self.with_motion, *self.after_motion)


def operator_steps(process: Process, action: Action, transfer_as_free: bool = False) -> list[OperatorStep]:
    """Group an action's movements into the steps the operator walks through.

    Every movement index appears exactly once over all steps, in `indices` or in `skipped`.

    Args:
        process: The process.
        action: The action.
        transfer_as_free: The transfer workaround: the coupled free movement runs without the bar.

    Returns:
        list[OperatorStep]: The steps, in order.

    Raises:
        UnknownMovement: If a movement matches no routine.
    """
    specs = classify_action(process, action, transfer_as_free)
    with_motion: dict[int, tuple[int, ...]] = {}
    after_motion: dict[int, tuple[int, ...]] = {}
    skipped: dict[int, tuple[int, ...]] = {}
    folded: set[int] = set()
    for index, spec in enumerate(specs):
        if _is_ungrasp(spec) and index + 1 < len(specs) and specs[index + 1].routine is Routine.RETREAT_LINEAR:
            with_motion[index + 1] = (index,)
            folded.add(index)
        if spec.routine is Routine.TRANSFER_FREE:
            first = index
            while first > 0 and specs[first - 1].routine in _PAUSE and first - 1 not in folded:
                first -= 1
            after_motion[index] = tuple(range(first, index))
            # * Nothing to load into the empty tools: the travel to load just before the pause is not run.
            if first > 0 and specs[first - 1].routine is Routine.FREE_TO_LOAD and first - 1 not in folded:
                skipped[index] = (first - 1,)
            folded.update(after_motion[index], skipped.get(index, ()))
    return [OperatorStep(index, with_motion.get(index, ()), after_motion.get(index, ()), skipped.get(index, ()))
            for index in range(len(specs)) if index not in folded]


def first_step(process: Process, action: Action, transfer_as_free: bool) -> int:
    """The movement an action starts at: its first operator step's (with the workaround, B20_J starts at the transfer).

    An action whose movements can't be classified starts at its first movement.

    Args:
        process: The process.
        action: The action.
        transfer_as_free: The transfer workaround.

    Returns:
        int: A movement index in the action.
    """
    try:
        return operator_steps(process, action, transfer_as_free)[0].primary
    except UnknownMovement:
        return 0


def step_index_of(steps: Sequence[OperatorStep], movement_index: int | None) -> int | None:
    """Which step holds a movement: as its primary, run with or after it, or skipped by it.

    Args:
        steps: `operator_steps` of the action.
        movement_index: A movement's index in the action, or None.

    Returns:
        int | None: The step's index in `steps`, or None if no step holds the movement.
    """
    for index, step in enumerate(steps):
        if movement_index in (*step.indices, *step.skipped):
            return index
    return None


def _is_ungrasp(spec: MovementSpec) -> bool:
    """Whether a movement only opens tools: a tool movement whose every commanded grip goes "open"."""
    return spec.routine is Routine.TOOL_ONLY and bool(spec.grips) and set(spec.grips.values()) == {"open"}
