"""
The plan checks B1–B14 of format §9: does the plan make sense? Run on demand: before export, planning or execution.

A file can pass the file checks (`validate`) and still fail here, e.g. a half-finished plan saved from Rhino. Errors
block execution; warnings are reported. Only B13 (SRDF groups) and B14 (forward kinematics) read the robot files.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, Iterator, List, Mapping, Optional, Set, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from ..geometry import Pose, compose, invert
from ..ids import split_link_id
from ..kinematics import link_pose
from ..urdf import srdf_group_tips
from .relations import ENGAGED, bar_of, mate_status, mount_offset
from .types import BAR_PREFIX, GROUND_PREFIX, HALF_PREFIX, Action, Design, Holder, Movement, RobotState, State

#: Part -> the half's pose in the frame of the tool centre point holding it (B14), for designs without a `parts`
#: catalogue: a ground half sits turned 180° about the TCP's z axis. The design's `parts` win.
PART_SEATS: Dict[str, Pose] = {"T20/Male": Pose(), "T20/Ground": Pose(orientation=(0.0, 0.0, 1.0, 0.0))}
#: How far a grasp may be from the geometry (B14), metres and radians.
GRASP_TOLERANCE: Tuple[float, float] = (1e-4, 1e-3)
#: How far a movement's end joints may be from the next start (B11), radians or metres.
JOINT_TOLERANCE = 1e-6


@dataclass(frozen=True)
class PlanReport:
    """The result of `check_plan`: one line per problem, each starting with its check, e.g. "B7: ..."."""

    errors: Tuple[str, ...]
    warnings: Tuple[str, ...]

    @property
    def ok(self) -> bool:
        """bool: No errors (warnings allowed)."""
        return not self.errors


def check_plan(design: Design, *, geometry: bool = True, seats: Optional[Mapping[str, Pose]] = None) -> PlanReport:
    """Run the plan checks B1–B14 on a design that passed the file checks.

    Args:
        design: The design.
        geometry: Also run B13 and B14, which read the robot files.
        seats: Part -> the half's pose in its tool's TCP frame, for B14. None: the design's `parts`, then PART_SEATS.

    Returns:
        PlanReport: Errors and warnings, each check's messages in schedule order.
    """
    if seats is None:
        seats = {**PART_SEATS, **{name: part.seat for name, part in design.parts.items()}}
    checker = _PlanChecker(design, seats)
    checker.run(geometry)
    return PlanReport(tuple(checker.errors), tuple(checker.warnings))


def end_state(movement: Movement) -> State:
    """The state a movement ends in, as far as the design says: the start with the target applied.

    The target's joints, grips, tool `on`, `attached` and `built` replace the start's; the rest stays.
    ! The acting robot's joints are unknown (None) after an arm motion whose target gives none of its joints.
    """
    start, target = movement.start, movement.target
    if target is None:
        return start
    robots = dict(start.robots)
    moving = {split_link_id(arm)[0] for arm in movement.arms}
    for robot in moving | set(target.joints):
        robot_state = robots.get(robot)
        if robot_state is None:
            continue
        given = target.joints.get(robot)
        joints = None if given is None or robot_state.joints is None else {**robot_state.joints, **given}
        robots[robot] = RobotState(robot_state.base, joints)
    tools = dict(start.tools)
    for tool, grip in target.tools.items():
        if tools.get(tool) is not None:
            tools[tool] = replace(tools[tool], grip=grip)
    for tool, body in target.on.items():
        if tools.get(tool) is not None:
            tools[tool] = replace(tools[tool], on=body)
    attached = start.attached if target.attached is None else target.attached
    built = start.built if target.built is None else target.built
    return replace(start, robots=robots, tools=tools, attached=dict(attached), built=frozenset(built),
                   present=start.present | set(attached) | built)


class _PlanChecker:
    """One run of the plan checks: the design and the messages found so far."""

    def __init__(self, design: Design, seats: Mapping[str, Pose]):
        self.design, self.seats = design, seats
        self.errors: List[str] = []
        self.warnings: List[str] = []
        # Robot id -> {flange link name: tool id}.
        self.tool_at = {robot_id: dict(robot.tools) for robot_id, robot in design.robots.items()}
        self.steps: List[Tuple[Action, Movement]] = list(design.movements())

    def error(self, check: int, where: str, text: str) -> None:
        """Note one error."""
        self.errors.append(f"B{check}: {where}: {text}")

    def warn(self, check: int, where: str, text: str) -> None:
        """Note one warning."""
        self.warnings.append(f"B{check}: {where}: {text}")

    def holder_tool(self, holder: Holder) -> Optional[str]:
        """The tool mounted at a holder's link, if any."""
        robot, link = split_link_id(holder.to)
        return self.tool_at.get(robot, {}).get(link)

    def states(self) -> Iterator[Tuple[str, State]]:
        """Every start state, then the end of the last movement, with where they are for messages."""
        for _, movement in self.steps:
            yield f"{movement.id} start", movement.start
        if self.steps:
            yield f"{self.steps[-1][1].id} end", end_state(self.steps[-1][1])

    def run(self, geometry: bool) -> None:
        """Run every check."""
        engaged: Set[Tuple[str, str]] = set()
        for where, state in self.states():
            self.check_structure(state, where, engaged)
        self.check_unmated(engaged)
        for index, (action, movement) in enumerate(self.steps):
            at = f"{movement.id}"
            self.check_ending(movement, at)
            self.check_carrying(action, movement, at)
            self.check_changes(movement, at)
            if index + 1 < len(self.steps):
                self.check_chain(action, movement, *self.steps[index + 1])
        self.check_holds()
        if geometry:
            self.check_arms()
            self.check_grasps()

    # --- --- --- --- --- STRUCTURE (B1–B5) --- --- --- --- ---

    def check_structure(self, state: State, where: str, engaged: Set[Tuple[str, str]]) -> None:
        """B1–B4 in one state; collects the mates engaged in it for B5."""
        design = self.design
        for bar in sorted(state.present):
            if bar.startswith(BAR_PREFIX) and bar not in state.built and bar not in state.attached \
                    and bar not in state.poses:
                self.warn(1, where, f"{bar} is neither built nor attached and has no pose: it floats at its design "
                                    f"pose")
        # * B2: every built bar has an engaged mate, and the built bars reach the ground through engaged mates.
        links: Dict[str, Set[str]] = {}
        for mate in design.mates:
            if mate_status(design, state, mate) != ENGAGED:
                continue
            engaged.add(mate)
            a, b = (bar_of(design, side) or side for side in mate)
            links.setdefault(a, set()).add(b)
            links.setdefault(b, set()).add(a)
        reached = {node for node in links if node.startswith(GROUND_PREFIX)}
        frontier = list(reached)
        while frontier:
            for other in links.get(frontier.pop(), ()):
                if other not in reached:
                    reached.add(other)
                    frontier.append(other)
        for bar in sorted(state.built):
            if bar not in links:
                self.error(2, where, f"{bar} is built but none of its mates is engaged")
            elif bar not in reached:
                self.error(2, where, f"{bar} is built but does not reach the ground through engaged mates")
        for bar, holders in sorted(state.attached.items()):
            robots = {split_link_id(holder.to)[0] for holder in holders}
            if bar not in state.built and len(robots) > 1:
                self.error(3, where, f"{bar} is not built but held by several robots {sorted(robots)}")
            part = {bar, *design.halves_of(bar)}
            for robot in sorted(robots):
                on = {tool_state.on for tool, tool_state in state.tools.items()
                      if tool_state is not None and tool in self.tool_at.get(robot, {}).values()}
                if not on & part:
                    self.error(4, where, f"{robot} holds {bar} but none of its tools is on {bar} or its halves")

    def check_unmated(self, engaged: Set[Tuple[str, str]]) -> None:
        """B5: mates never engaged in the schedule, and halves in no mate."""
        for mate in sorted(self.design.mates - engaged):
            self.warn(5, "mates", f"{list(mate)} is never engaged in the schedule")
        mated = {side for mate in self.design.mates for side in mate}
        for half in sorted(body for body in self.design.bodies if body.startswith(HALF_PREFIX) and body not in mated):
            self.warn(5, "mates", f"{half} (on {self.design.bodies[half].mount}) is in no mate: its partner is "
                                  f"not in the design")

    # --- --- --- --- --- PROCEDURE (B6–B12) --- --- --- --- ---

    def check_ending(self, movement: Movement, at: str) -> None:
        """B6: `ends_on: tools` comes with a tool part, and with arm motion needs a compliant controller."""
        if movement.ends_on != "tools":
            return
        if not movement.has_tool_part:
            self.error(6, at, "ends_on tools needs a tool part (a grip change or a drive)")
        if movement.arms and movement.controller != "compliant":
            self.error(6, at, "ends_on tools with arm motion needs controller compliant: a position-controlled arm "
                              "never stops short of its target")

    def check_carrying(self, action: Action, movement: Movement, at: str) -> None:
        """B7–B9: what an arm motion may do given what its robot holds and where its tools are."""
        if not movement.arms:
            return  # ? Manual steps and tool moves are exempt: the operator mounts the bar before the grasp.
        state, robot = movement.start, action.robot
        moving = set(movement.arms)
        for bar, holders in sorted(state.attached.items()):
            own = [holder for holder in holders if split_link_id(holder.to)[0] == robot]
            if not own:
                continue
            if bar in state.built:
                # * B8: a robot holding a built bar is frozen, except in a compliant movement releasing it.
                releases = any(movement.grip_change.get(self.holder_tool(holder)) == "open" for holder in own) or \
                    any(direction == "loosen" and self.on_part(state, tool, bar)
                        for tool, direction in movement.drives.items())
                if movement.controller != "compliant" or not releases:
                    self.error(8, at, f"{robot} holds the built {bar}: it may move only in a compliant movement "
                                      f"that releases it")
                continue
            if not moving & {holder.to for holder in own}:
                continue
            # * B7: carrying an unbuilt bar needs closed grips, and several holders move coupled.
            for holder in own:
                tool = self.holder_tool(holder)
                tool_state = state.tools.get(tool) if tool else None
                if tool_state is not None and tool_state.grip != "closed":
                    self.error(7, at, f"moves {bar} while {tool} grip is {tool_state.grip!r}")
            if len(own) > 1 and not ({holder.to for holder in own} <= moving and movement.coupled):
                self.error(7, at, f"{bar} has {len(own)} holders: they must all move, coupled")
        # * B9: a tool on a body its arm does not hold allows only a linear retreat.
        for arm in movement.arms:
            robot_id, link = split_link_id(arm)
            tool = self.tool_at.get(robot_id, {}).get(link)
            tool_state = state.tools.get(tool) if tool else None
            if tool_state is None or tool_state.on is None:
                continue
            held = bar_of(self.design, tool_state.on)
            holds = held is not None and any(holder.to == arm for holder in state.attached.get(held, ()))
            if not holds and movement.path != "linear":
                self.error(9, at, f"{tool} is on {tool_state.on}, which {arm} does not hold: only a linear retreat "
                                  f"may move it")

    def on_part(self, state: State, tool: str, bar: str) -> bool:
        """Whether a tool is on a bar or one of its halves."""
        tool_state = state.tools.get(tool)
        return tool_state is not None and tool_state.on is not None and bar_of(self.design, tool_state.on) == bar

    def check_changes(self, movement: Movement, at: str) -> None:
        """B10: `attached` and `built` change only as the operations allow."""
        start, end = movement.start, end_state(movement)

        def held(state: State) -> Set[Tuple[str, str]]:
            return {(bar, holder.to) for bar, holders in state.attached.items() for holder in holders}

        for bar, link in sorted(held(end) - held(start)):
            tool = self.holder_tool(Holder(link, Pose()))
            if movement.ends_on != "operator" and movement.grip_change.get(tool) != "closed":
                self.error(10, at, f"{bar} becomes attached to {link} without a manual mount or a grip closing there")
        for bar, link in sorted(held(start) - held(end)):
            tool = self.holder_tool(Holder(link, Pose()))
            if movement.grip_change.get(tool) != "open":
                self.error(10, at, f"{bar} stops being attached to {link} without its grip opening (an ungrasp)")
        for bar in sorted(end.built - start.built):
            if not any(direction == "tighten" and self.on_part(start, tool, bar)
                       for tool, direction in movement.drives.items()):
                self.error(10, at, f"{bar} becomes built without an insert (a tighten drive on its part)")
        for bar in sorted(start.built - end.built):
            if not any(direction == "loosen" and self.on_part(start, tool, bar)
                       for tool, direction in movement.drives.items()):
                self.error(10, at, f"{bar} stops being built without an untighten (a loosen drive on its part)")

    def check_chain(self, action: Action, movement: Movement, next_action: Action, following: Movement) -> None:
        """B11: a movement ends where the next one starts, across actions in schedule order.

        Joints, grips, tool `on`, `present`, `attached` and `built` are compared; bases only within one action,
        because robots drive between actions. Values the design leaves open (None) are not compared.
        """
        if movement.target is None:
            return  # ? By definition: it ends where the next one starts.
        end, start = end_state(movement), following.start
        at = f"{movement.id} -> {following.id}"
        for robot, robot_state in end.robots.items():
            other = start.robots.get(robot)
            if robot_state is None or other is None:
                continue
            if robot_state.joints is not None and other.joints is not None:
                shared = set(robot_state.joints) & set(other.joints)
                gap = max((abs(robot_state.joints[name] - other.joints[name]) for name in shared), default=0.0)
                if gap > JOINT_TOLERANCE:
                    self.error(11, at, f"{robot} joints jump by {gap:.3g}")
            if action is next_action and robot_state.base is not None and other.base is not None \
                    and _distance(robot_state.base, other.base) > JOINT_TOLERANCE:
                self.error(11, at, f"{robot} base moves between two movements of one action")
        for tool, tool_state in end.tools.items():
            other = start.tools.get(tool)
            if tool_state is not None and other is not None and None not in (tool_state.grip, other.grip) \
                    and tool_state.grip != other.grip:
                self.error(11, at, f"{tool} grip changes from {tool_state.grip} to {other.grip} between movements")
            if tool_state is not None and other is not None and tool_state.on != other.on:
                self.error(11, at, f"{tool} on changes from {tool_state.on} to {other.on} between movements: a "
                                   f"target sets it")
        bars = {body for body in end.present | start.present if body.startswith(BAR_PREFIX)}
        if {bar for bar in bars if bar in end.present} != {bar for bar in bars if bar in start.present}:
            self.error(11, at, f"present bars change between movements: "
                               f"{sorted((end.present ^ start.present) & bars)}")
        if end.built != start.built:
            self.error(11, at, f"built changes between movements: {sorted(end.built ^ start.built)}")
        before = {(bar, holder.to): holder.grasp for bar, holders in end.attached.items() for holder in holders}
        after = {(bar, holder.to): holder.grasp for bar, holders in start.attached.items() for holder in holders}
        if set(before) != set(after):
            self.error(11, at, f"attached changes between movements: {sorted(set(before) ^ set(after))}")
        else:
            for key in sorted(before):
                if _distance(before[key], after[key]) > GRASP_TOLERANCE[0]:
                    self.error(11, at, f"the grasp of {key[0]} on {key[1]} changes between movements")

    def check_holds(self) -> None:
        """B12: a hold is released only after every bar in its `supports_until` is built."""
        holds: Dict[Tuple[str, str], Action] = {}
        for action_id in self.design.schedule:
            action = self.design.actions[action_id]
            if action.type == "bar_holding":
                holds[(action.robot, action.bar)] = action
            elif action.type == "bar_holding_release" and action.movements:
                hold = holds.get((action.robot, action.bar))
                if hold is None:
                    self.warn(12, action.id, f"no earlier bar_holding of {action.bar} by {action.robot}")
                    continue
                missing = sorted(set(hold.supports_until) - action.movements[0].start.built)
                if missing:
                    self.error(12, action.id, f"releases {action.bar} before {missing} are built "
                                              f"({hold.id} supports until them)")

    # --- --- --- --- --- GEOMETRY (B13, B14) --- --- --- --- ---

    def check_arms(self) -> None:
        """B13: `arms` name flange links that end an SRDF group of the acting robot."""
        tips = {robot_id: set(srdf_group_tips(robot.srdf).values()) for robot_id, robot in self.design.robots.items()}
        for action, movement in self.steps:
            for arm in movement.arms:
                robot, link = split_link_id(arm)
                if robot != action.robot or link not in tips.get(robot, ()):
                    self.error(13, movement.id, f"{arm!r} does not end an SRDF group of {action.robot}")

    def check_grasps(self) -> None:
        """B14: grasps agree with the geometry: the part seat for a tool on a half, forward kinematics otherwise."""
        design = self.design
        seen: Set[tuple] = set()
        for _, movement in self.steps:
            state, where = movement.start, f"{movement.id} start"
            for bar, holders in sorted(state.attached.items()):
                reference: Optional[Pose] = design.bodies[bar].pose if bar in state.built else None
                for holder in holders:
                    tool = self.holder_tool(holder)
                    tool_state = state.tools.get(tool) if tool else None
                    on = tool_state.on if tool_state is not None else None
                    key = (bar, holder.to, holder.grasp, on)
                    if on is not None and on.startswith(HALF_PREFIX) and bar_of(design, on) == bar and key not in seen:
                        seen.add(key)
                        part = design.bodies[on].part
                        if part not in self.seats:
                            self.warn(14, where, f"no seat known for part {part!r} of {on}: grasp not checked")
                        else:
                            tcp = design.tools[tool].tcp
                            expected = compose(compose(tcp, self.seats[part]), invert(mount_offset(design, on)))
                            what = f"the grasp of {bar} on {holder.to} (tool on {on}, part {part})"
                            self._compare(expected, holder.grasp, where, what)
                    robot, link = split_link_id(holder.to)
                    robot_state = state.robots.get(robot)
                    if robot_state is None or robot_state.base is None or robot_state.joints is None:
                        continue
                    world = compose(link_pose(design.robots[robot].urdf, robot_state.base, robot_state.joints, link),
                                    holder.grasp)
                    if reference is None:
                        reference = world
                    else:
                        self._compare(reference, world, where, f"{bar} as held by {holder.to}")

    def _compare(self, expected: Pose, found: Pose, where: str, what: str) -> None:
        """Note a B14 error if two poses differ by more than GRASP_TOLERANCE."""
        distance = float(np.linalg.norm(np.subtract(expected.position, found.position)))
        angle = _angle(expected, found)
        if distance > GRASP_TOLERANCE[0] or angle > GRASP_TOLERANCE[1]:
            self.error(14, where, f"{what} is {distance * 1000:.3g} mm and {np.degrees(angle):.3g}° from the geometry")


def _angle(a: Pose, b: Pose) -> float:
    """The rotation angle between two poses' orientations, radians."""
    return float((Rotation.from_quat(a.orientation).inv() * Rotation.from_quat(b.orientation)).magnitude())


def _distance(a: Pose, b: Pose) -> float:
    """The larger of two poses' distance (metres) and rotation angle (radians)."""
    return max(float(np.linalg.norm(np.subtract(a.position, b.position))), _angle(a, b))
