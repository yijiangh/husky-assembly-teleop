"""
Grasp offsets: a support gripper's grasp slid along its bar and rolled about it, for one hold.

A hold is a `bar_holding` action and its `bar_holding_release`. Its offset moves every target of the holding flange
from the hold's approach to its release's retreat by one rigid move about the bar (`grasp_move`), so the approach and
retreat lines keep their shape around the new grasp; the holder's grasp follows. The holding arm's joints in those
targets are dropped (the live IK solves the moved poses); the joints in the states between stay the authored guess.

Pure: any thread.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Mapping

from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.process import Process
from scaffolding_assembly_core.process.types import Action, Holder, LineSpec, Movement, RobotTarget, State, Target

from ...shared.bars.pose import bar_slide_roll

#: Action -> (slide mm, roll degrees), by its `bar_holding` action's id.
GraspOffsets = Mapping[str, tuple[float, float]]


def hold_of(process: Process, action_id: str) -> str | None:
    """The `bar_holding` action a hold's action belongs to: itself, or the one its release ends; None for others."""
    action = process.actions.get(action_id)
    if action is None or action.type not in ("bar_holding", "bar_holding_release"):
        return None
    if action.type == "bar_holding":
        return action_id
    before = process.schedule[:process.schedule.index(action_id)]
    return next((each for each in reversed(before) if process.actions[each].type == "bar_holding"
                 and (process.actions[each].robot, process.actions[each].bar) == (action.robot, action.bar)), None)


def grasp_move(process: Process, hold_id: str, slide_mm: float, roll_deg: float) -> tuple[str, Pose] | None:
    """The holding flange and the world move of its targets: the bar's frame slid and rolled, `bar S bar⁻¹`.

    Returns None when the hold has no flange target with a grasp (nothing to move).
    """
    hold = process.actions[hold_id]
    flange = next((arm for movement in hold.movements for arm in movement.arms), None)
    if flange is None:
        return None
    grasp = next((holder.grasp for movement in hold.movements if movement.target and movement.target.attached
                  for holder in movement.target.attached.get(hold.bar, ()) if holder.to == flange), None)
    at = next((movement.target.link_poses[flange] for movement in reversed(hold.movements)
               if movement.target and flange in movement.target.link_poses), None)
    if grasp is None or at is None:
        return None
    bar = compose(at, grasp)
    slide = Pose.from_matrix(bar_slide_roll(slide_mm / 1000.0, roll_deg))
    return flange, compose(compose(bar, slide), invert(bar))


def offset_grasps(process: Process, offsets: GraspOffsets) -> Process:
    """The process with each hold's grasp moved by its offset; `process` itself when none moves anything.

    Args:
        process: The process (placed or not).
        offsets: Hold action id -> (slide mm, roll degrees); unknown holds and zero offsets are ignored.
    """
    actions, moved = dict(process.actions), False
    for hold_id, (slide_mm, roll_deg) in offsets.items():
        if hold_id not in actions or actions[hold_id].type != "bar_holding" or (slide_mm, roll_deg) == (0.0, 0.0):
            continue
        found = grasp_move(process, hold_id, slide_mm, roll_deg)
        if found is None:
            continue
        flange, move = found
        hold = actions[hold_id]
        unslide = invert(Pose.from_matrix(bar_slide_roll(slide_mm / 1000.0, roll_deg)))
        mover = _Mover(hold.robot, hold.bar, flange, move, unslide)
        for action_id in _span(process, hold):
            actions[action_id] = mover.action(actions[action_id], own=actions[action_id].robot == hold.robot)
        moved = True
    return replace(process, actions=actions) if moved else process


def _span(process: Process, hold: Action) -> tuple[str, ...]:
    """The hold's action, every action up to its release, and the release (to the end without one)."""
    start = process.schedule.index(hold.id)
    for end in range(start + 1, len(process.schedule)):
        action = process.actions[process.schedule[end]]
        if action.type == "bar_holding_release" and (action.robot, action.bar) == (hold.robot, hold.bar):
            return process.schedule[start:end + 1]
    return process.schedule[start:]


class _Mover:
    """Moves one holding flange's targets and lines, and its holder's grasp."""

    def __init__(self, robot: str, bar: str, flange: str, move: Pose, unslide: Pose):
        self.robot, self.bar, self.flange, self.move, self.unslide = robot, bar, flange, move, unslide
        self._turn = Rotation.from_quat(move.orientation)

    def action(self, action: Action, own: bool) -> Action:
        return replace(action, movements=tuple(self._movement(movement, own) for movement in action.movements))

    def _movement(self, movement: Movement, own: bool) -> Movement:
        line = movement.line
        if own and self.flange in line:
            spec = line[self.flange]
            direction = tuple(float(value) for value in self._turn.apply(spec.direction))
            line = {**line, self.flange: LineSpec(direction, spec.distance)}
        target = movement.target
        if target is not None:
            target = replace(target, robots=self._robots(target) if own else target.robots,
                             attached=self._attached(target.attached))
        return replace(movement, start=self._state(movement.start), line=line, target=target)

    def _robots(self, target: Target) -> dict[str, RobotTarget]:
        part = target.robots.get(self.robot)
        if part is None or self.flange not in part.link_poses:
            return target.robots
        poses = {**part.link_poses, self.flange: compose(self.move, part.link_poses[self.flange])}
        # * The authored joints reach the authored grasp: the live IK solves the moved one.
        return {**target.robots, self.robot: RobotTarget(part.base, {}, poses)}

    def _state(self, state: State) -> State:
        attached = self._attached(state.attached)
        return state if attached is state.attached else replace(state, attached=attached)

    def _attached(self, attached):
        if not attached or not any(holder.to == self.flange for holder in attached.get(self.bar, ())):
            return attached
        holders = tuple(Holder(holder.to, compose(holder.grasp, self.unslide)) if holder.to == self.flange else holder
                        for holder in attached[self.bar])
        return {**attached, self.bar: holders}
