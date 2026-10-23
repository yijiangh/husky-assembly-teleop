"""
The steps an operator walks through in one action: each runs one movement, with any tool movements folded into it.

- An ungrasp (a tool movement opening every grip it commands) directly before a two-arm linear retreat runs with
  that retreat.
- With the transfer workaround the free transfer runs first, with empty tools. The operator and tool movements
  authored before it (place, grasp) are done after its motion, in a pause at the insertion start, and the travel to
  load before them is skipped. `as_run` gives the process in that order, for display.

Pure: any thread.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Sequence

from scaffolding_assembly_core.process import Action, Movement, Process, RobotState, State, Target
from scaffolding_assembly_core.process.plan_check import end_state

from .movement import MovementSpec, Routine, UnknownMovement, classify_action

#: Routines done in the pause after a free transfer (placing the bar by hand, closing the tools).
_PAUSE = (Routine.OPERATOR, Routine.TOOL_ONLY)
#: The label of the transfer as the workaround runs it (`as_run`).
TRANSIT_LABEL = "Transit to the insert start, empty tools (transfer workaround)"


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


def as_run(process: Process, transfer_as_free: bool) -> Process:
    """The process with each action's movements in the order its operator steps run them, for display only.

    With the workaround a jointing action lists the transfer first, as a transit with empty tools from where the
    action starts to the insert start; then placing and grasping the bar there, then the insert. The travel to load is
    left out.
    Every movement keeps its id.

    - ! For the process panel and the planners' targets only: plans, runs and records keep the authored process.
    - An action the workaround does not change (no pause, nothing skipped) keeps its authored objects.

    Args:
        process: The authored process.
        transfer_as_free: The transfer workaround.

    Returns:
        Process: `process` itself without the workaround; else a copy with the jointing actions in run order.
    """
    if not transfer_as_free:
        return process
    return replace(process, actions={action_id: _as_run(process, action)
                                     for action_id, action in process.actions.items()})


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


def _as_run(process: Process, action: Action) -> Action:
    """One action in run order with the workaround (`as_run`); the action itself when the workaround changes nothing."""
    try:
        steps = operator_steps(process, action, True)
    except UnknownMovement:
        return action
    # ! Only the workaround pauses or skips: anything else, e.g. a release's ungrasp run with its retreat, stays as
    #   authored.
    if not any(step.after_motion or step.skipped for step in steps):
        return action
    movements = action.movements
    shown: list[Movement] = []
    for step in steps:
        if not step.after_motion:
            shown += [movements[index] for index in (step.primary, *step.with_motion)]
            continue
        # * The transfer becomes the transit; each movement of the pause starts where the one before it ends.
        transit = _transit(action, movements[step.primary], movements[0].start)
        shown += [transit, *(movements[index] for index in step.with_motion)]
        state = _end(transit, action.robot)
        for index in step.after_motion:
            shown.append(replace(movements[index], start=state))
            state = end_state(shown[-1])
    return replace(action, movements=tuple(shown))


def _transit(action: Action, transfer: Movement, start: State) -> Movement:
    """The transfer as the workaround runs it: from the action's `start` (no bar, tools open) to the insert start.

    Its target is the acting robot's joints where the transfer ends, or the transfer's own target if they are unknown.
    """
    end = _end(transfer, action.robot).robots.get(action.robot)
    joints = end.joints if end is not None else None
    target = Target.of(joints={action.robot: joints}) if joints else transfer.target
    return replace(transfer, start=start, target=target, label=TRANSIT_LABEL)


def _end(movement: Movement, robot: str) -> State:
    """The state a movement ends in (`end_state`), the robot at its target joints even when its start ones are null.

    ? `end_state` leaves the joints unknown after a start with null joints, and exports leave the transfer's start null.
    """
    state = end_state(movement)
    after = state.robots.get(robot)
    given = movement.target.joints.get(robot) if movement.target is not None else None
    if after is None or after.joints is not None or not given:
        return state
    return replace(state, robots={**state.robots, robot: RobotState(after.base, dict(given))})
