"""
Check a design against rules 2–16 of format §9 (rule 1 is the reader's).

Every problem is collected and raised in one `DesignError`, each line starting with "rule <n>: <where>".
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple
from xml.etree.ElementTree import ParseError

import numpy as np

from ..geometry import BoxShape, CylinderShape, Geometry, Pose
from ..ids import ID_PATTERN
from ..urdf import is_relative_reference, mesh_references, movable_joints, srdf_group_tips, urdf_joints, urdf_links
from .relations import members, part_of
from .types import (ACTION_TYPES, BODY_PREFIXES, CONTROLLERS, ENDS_ON, PATHS, ROBOT_PREFIX, TOOL_PREFIX, Action,
                    Design, DesignError, Movement, State)
from .vocabulary import ON, TOOL_CHANNELS

#: How far a quaternion's (or a line direction's) length may be from 1 (rule 9).
UNIT_TOLERANCE = 1e-6
#: The types a note value may have (rule 15).
NOTE_TYPES = (str, int, float, bool)


def validate(design: Design, *, check_robot_meshes: bool = True) -> None:
    """Check every rule of format §9 from 2 to 16.

    Args:
        design: The design, in memory or read from a folder.
        check_robot_meshes: Check rule 11 (URDF meshes by relative path); `write` turns it off for its source URDFs.

    Raises:
        DesignError: With every problem found, if there is any.
    """
    problems = _Checker(design, check_robot_meshes).run()
    if problems:
        raise DesignError(problems)


class _RobotFiles:
    """What the checks need from one robot's URDF and SRDF. None where a file could not be read."""

    def __init__(self, urdf: Path, srdf: Path, problems: List[str], robot: str):
        self.links: Optional[Set[str]] = None
        self.joints: Optional[Dict[str, str]] = None
        self.movable: Optional[Tuple[str, ...]] = None
        self.tips: Optional[Set[str]] = None
        self.meshes: List[str] = []
        if not Path(urdf).is_file():
            problems.append(f"rule 3: {robot}: URDF {str(urdf)!r} does not exist")
        else:
            try:
                self.links, self.joints = urdf_links(urdf), urdf_joints(urdf)
                self.meshes = mesh_references(urdf)
            except ParseError as error:
                problems.append(f"rule 3: {robot}: URDF {str(urdf)!r} is not valid XML ({error})")
        if not Path(srdf).is_file():
            problems.append(f"rule 3: {robot}: SRDF {str(srdf)!r} does not exist")
        else:
            try:
                self.tips = set(srdf_group_tips(srdf).values())
                if self.joints is not None:
                    self.movable = movable_joints(urdf, srdf)
            except ParseError as error:
                problems.append(f"rule 3: {robot}: SRDF {str(srdf)!r} is not valid XML ({error})")


class _Checker:
    """One validation run: the design, its parsed robot files and the problems found so far."""

    def __init__(self, design: Design, check_robot_meshes: bool = True):
        self.design = design
        self.check_robot_meshes = check_robot_meshes
        self.problems: List[str] = []
        self.files: Dict[str, _RobotFiles] = {}
        # ? Cache by file pair: several robots may share the same files.
        by_path: Dict[Tuple[Path, Path], _RobotFiles] = {}
        for robot_id, robot in design.robots.items():
            key = (Path(robot.urdf).resolve(), Path(robot.srdf).resolve())
            if key not in by_path:
                by_path[key] = _RobotFiles(robot.urdf, robot.srdf, self.problems, robot_id)
            self.files[robot_id] = by_path[key]

    def add(self, rule: int, where: str, text: str) -> None:
        """Note one problem."""
        self.problems.append(f"rule {rule}: {where}: {text}")

    def run(self) -> List[str]:
        """Run every check.

        Returns:
            list[str]: The problems, one line each.
        """
        self._parts = part_of(self.design)
        self.check_ids()
        self.check_robots_and_tools()
        self.check_bodies()
        self.check_schedule()
        for action_id in self.design.schedule:
            if action_id in self.design.actions:
                self.check_action(action_id)
        return self.problems

    # --- --- --- --- --- REFERENCES --- --- --- --- ---

    def link_known(self, value: str, where: str, rule: int = 4) -> Optional[Tuple[str, str]]:
        """Check a link id "robots/<robot>/<link>": robot exists, link is in its URDF (rule 4).

        Returns:
            tuple[str, str] | None: (robot id, link name), or None if it is not a valid link id.
        """
        parts = value.split("/")
        if len(parts) != 3 or f"{parts[0]}/" != ROBOT_PREFIX:
            self.add(rule, where, f"{value!r} is not a link id 'robots/<robot>/<link>'")
            return None
        robot, link = f"{parts[0]}/{parts[1]}", parts[2]
        if robot not in self.design.robots:
            self.add(rule, where, f"link {value!r}: unknown robot {robot!r}")
            return None
        links = self.files[robot].links
        if links is not None and link not in links:
            self.add(rule, where, f"link {value!r}: no link {link!r} in the URDF of {robot}")
            return None
        return robot, link

    def bodies_known(self, values: Iterable[str], where: str) -> None:
        """Check body ids (rule 4)."""
        for value in values:
            if value not in self.design.bodies:
                self.add(4, where, f"unknown body {value!r}")

    def check_joint_names(self, robot: str, joints: Dict[str, float], where: str) -> None:
        """Every joint named exists in the robot's URDF (rule 6)."""
        known = self.files[robot].joints if robot in self.files else None
        if known is None:
            return
        for name in joints:
            if name not in known:
                self.add(6, where, f"{robot} has no joint {name!r}")

    def check_pose(self, pose: Pose, where: str) -> None:
        """A pose's quaternion has unit length (rule 9)."""
        length = float(np.linalg.norm(pose.orientation))
        if abs(length - 1.0) > UNIT_TOLERANCE:
            self.add(9, where, f"quaternion {list(pose.orientation)} has length {length!r}, not 1")

    def check_geometry(self, geometry: Geometry, where: str) -> None:
        """Shape origins of a geometry have unit quaternions (rule 9)."""
        for shape in (*geometry.collision, *geometry.visual):
            if isinstance(shape, (BoxShape, CylinderShape)):
                self.check_pose(shape.origin, f"{where} shape origin")

    # --- --- --- --- --- DESIGN-LEVEL --- --- --- --- ---

    def check_ids(self) -> None:
        """Ids match the pattern, keys match the objects, prefixes are right, and nothing repeats (rule 2)."""
        design = self.design
        groups = (("robot", design.robots, (ROBOT_PREFIX,)), ("tool", design.tools, (TOOL_PREFIX,)),
                  ("body", design.bodies, BODY_PREFIXES))
        for kind, objects, prefixes in groups:
            for key, value in objects.items():
                if not ID_PATTERN.fullmatch(key):
                    self.add(2, key, f"invalid {kind} id {key!r}")
                if not key.startswith(prefixes):
                    self.add(2, key, f"{kind} id {key!r} does not start with {' or '.join(prefixes)}")
                if value.id != key:
                    self.add(2, key, f"{kind} is keyed {key!r} but has id {value.id!r}")
        for robot in design.robots:
            if robot.count("/") != 1:
                self.add(2, robot, f"robot id {robot!r} is not 'robots/<name>'")
        for key, action in design.actions.items():
            if not ID_PATTERN.fullmatch(key) or "/" in key:
                self.add(2, key, f"invalid action id {key!r}")
            if action.id != key:
                self.add(2, key, f"action is keyed {key!r} but has id {action.id!r}")
        counts = Counter(movement.id for action in design.actions.values() for movement in action.movements)
        for movement_id, count in counts.items():
            if not ID_PATTERN.fullmatch(movement_id) or "/" in movement_id:
                self.add(2, movement_id, f"invalid movement id {movement_id!r}")
            if count > 1:
                self.add(2, movement_id, f"movement id {movement_id!r} is used {count} times")

    def check_robots_and_tools(self) -> None:
        """Robot files, ground links, tool mounts and tool fields (rules 3, 4, 5, 9, 11, 13)."""
        design = self.design
        mounts: Dict[str, List[str]] = {tool: [] for tool in design.tools}
        for robot_id, robot in design.robots.items():
            files = self.files[robot_id]
            for link, tool in robot.tools.items():
                where = f"{robot_id} tools"
                if files.links is not None and link not in files.links:
                    self.add(5, where, f"mount link {link!r} of {tool} is not in the URDF")
                if tool not in design.tools:
                    self.add(4, where, f"unknown tool {tool!r}")
                else:
                    mounts[tool].append(f"{robot_id}/{link}")
            for link in robot.ground_links:
                if files.links is not None and link not in files.links:
                    self.add(5, f"{robot_id} ground_links", f"no link {link!r} in the URDF")
            # * Rule 11: robot meshes by relative path, next to the URDF.
            folder = Path(robot.urdf).parent
            for reference in (dict.fromkeys(files.meshes) if self.check_robot_meshes else ()):
                if not is_relative_reference(reference):
                    self.add(11, robot_id, f"URDF mesh {reference!r} is not a path relative to the URDF")
                elif not (folder / reference).is_file():
                    self.add(11, robot_id, f"URDF mesh {reference!r} does not exist")
        for tool_id, tool in design.tools.items():
            if len(mounts[tool_id]) != 1:
                places = ", ".join(mounts[tool_id]) or "no robot"
                self.add(5, tool_id, f"mounted on {len(mounts[tool_id])} robots ({places}), must be exactly one")
            if tool.kind not in TOOL_CHANNELS:
                self.add(13, tool_id, f"unknown tool kind {tool.kind!r}; known: {sorted(TOOL_CHANNELS)}")
            self.check_pose(tool.tcp, f"{tool_id} tcp")
            self.check_geometry(tool.geometry, tool_id)
            for value in tool.mount_contacts:
                self.link_known(value, f"{tool_id} mount_contacts", rule=5)

    def check_bodies(self) -> None:
        """Body poses and markers, and connections (rules 4, 9, 16)."""
        design = self.design
        for body_id, body in design.bodies.items():
            self.check_pose(body.pose, f"{body_id} pose")
            self.check_geometry(body.geometry, body_id)
            for name, point in body.markers.items():
                if len(point) != 3:
                    self.add(4, f"{body_id} markers", f"marker {name!r} has {len(point)} coordinates, not 3")
        for a, b in design.connections:
            self.bodies_known((a, b), "connections")
            if a == b:
                self.add(16, "connections", f"{a!r} is connected to itself")

    def check_schedule(self) -> None:
        """Every scheduled action exists, once, and every action is scheduled (rule 3)."""
        design = self.design
        for action_id, count in Counter(design.schedule).items():
            if count > 1:
                self.add(2, "schedule", f"action {action_id!r} is scheduled {count} times")
            if action_id not in design.actions:
                self.add(3, "schedule", f"action {action_id!r} has no action file")
        for action_id in design.actions:
            if action_id not in design.schedule:
                self.add(3, f"actions/{action_id}.json", "action is not in the schedule")

    # --- --- --- --- --- ACTIONS --- --- --- --- ---

    def check_action(self, action_id: str) -> None:
        """One action and its movements (rules 4, 6–15)."""
        design = self.design
        action = design.actions[action_id]
        where = f"action {action_id}"
        if action.type not in ACTION_TYPES:
            self.add(4, where, f"unknown action type {action.type!r}")
        if action.robot not in design.robots:
            self.add(4, where, f"unknown robot {action.robot!r}")
        self.bodies_known((action.bar,), f"{where} bar")
        self.bodies_known(action.ground, f"{where} ground")
        self.bodies_known(action.supports_until, f"{where} supports_until")
        self.check_notes(action.notes, where)
        for movement in action.movements:
            at = f"{where} movement {movement.id}"
            self.check_parts(action, movement, at)
            self.check_state(movement.start, f"{at} start")
            self.check_carrying(action, movement, at)
            self.check_notes(movement.notes, at)
            if movement.target is not None:
                for robot, joints in movement.target.joints.items():
                    if robot not in design.robots:
                        self.add(4, f"{at} target", f"unknown robot {robot!r}")
                    else:
                        self.check_joint_names(robot, joints, f"{at} target joints")
                for link, pose in movement.target.links.items():
                    self.link_known(link, f"{at} target links")
                    self.check_pose(pose, f"{at} target link {link}")
                for tool, channels in movement.target.tools.items():
                    self.check_channels(tool, channels, f"{at} target tools", partial=True)

    def check_parts(self, action: Action, movement: Movement, at: str) -> None:
        """A movement's parts fit together (rules 10, 12)."""
        # * Rule 10: arms are flange links of the acting robot, each the tip of an SRDF group.
        for arm in movement.arms:
            found = self.link_known(arm, f"{at} arms")
            if found is None:
                continue
            robot, link = found
            if robot != action.robot:
                self.add(10, f"{at} arms", f"{arm!r} is not a link of the acting robot {action.robot}")
                continue
            tips = self.files[robot].tips
            if tips is not None and link not in tips:
                self.add(10, f"{at} arms", f"no SRDF group of {robot} ends at {link!r}")
        # * Rule 12: path and controller exactly when arms move; a line only on a linear path, for moving arms.
        if movement.arms:
            if movement.path not in PATHS:
                self.add(12, at, f"arms move, so path must be one of {PATHS}, not {movement.path!r}")
            if movement.controller not in CONTROLLERS:
                self.add(12, at, f"arms move, so controller must be one of {CONTROLLERS}, not {movement.controller!r}")
        elif movement.path is not None or movement.controller is not None or movement.coupled:
            self.add(12, at, "no arm moves, so path, controller and coupled must be absent")
        if movement.line and movement.path != "linear":
            self.add(12, at, "a line needs path linear")
        for flange, line in movement.line.items():
            if flange not in movement.arms:
                self.add(12, f"{at} line", f"{flange!r} is not a moving arm")
            length = float(np.linalg.norm(line.direction))
            if abs(length - 1.0) > UNIT_TOLERANCE:
                self.add(9, f"{at} line {flange}", f"direction {list(line.direction)} has length {length!r}, not 1")
            if line.distance <= 0.0:
                self.add(12, f"{at} line {flange}", f"distance {line.distance!r} is not positive")
        changes = movement.tool_change
        if movement.ends_on not in ENDS_ON:
            self.add(12, at, f"ends_on must be one of {ENDS_ON}, not {movement.ends_on!r}")
        elif movement.ends_on == "tools":
            if not changes:
                self.add(12, at, "ends_on tools needs a tool change")
            if movement.arms and movement.controller != "compliant":
                self.add(12, at, "ends_on tools with arm motion needs controller compliant: a position-controlled "
                                 "arm never stops short of its target")
        elif movement.ends_on == "operator" and (movement.arms or changes):
            self.add(12, at, "ends_on operator is a manual step: no arms and no tool change")
        if not movement.arms and not changes and movement.ends_on != "operator":
            self.add(12, at, "nothing moves and nothing changes: a manual step needs ends_on operator")

    def check_state(self, state: State, where: str) -> None:
        """One State is complete and consistent (rules 4, 6–9, 13)."""
        design = self.design
        # * Rule 7: every robot listed.
        for robot in design.robots:
            if robot not in state.robots:
                self.add(7, where, f"robot {robot} is not listed")
        for robot, robot_state in state.robots.items():
            if robot not in design.robots:
                self.add(4, where, f"unknown robot {robot!r}")
                continue
            if robot_state is None:
                continue
            if robot_state.base is not None:
                self.check_pose(robot_state.base, f"{where} {robot} base")
            if robot_state.joints is None:
                continue
            # * Rule 6: known joint names, and every movable joint given.
            self.check_joint_names(robot, robot_state.joints, f"{where} {robot} joints")
            movable = self.files[robot].movable
            if movable is not None:
                missing = [name for name in movable if name not in robot_state.joints]
                if missing:
                    self.add(6, f"{where} {robot} joints", f"missing joints {missing}")
        self.bodies_known(state.present, f"{where} present")
        self.bodies_known(state.poses, f"{where} poses")
        self.bodies_known(state.carried, f"{where} carried")
        # * Rule 7: a pose for every present body that is not carried, and for no other.
        for body in state.present:
            if body not in state.poses and body not in state.carried:
                self.add(7, where, f"{body} is present but has neither a pose nor a carrier")
        for body in (*state.poses, *state.carried):
            if body not in state.present:
                self.add(7, where, f"{body} has a pose or carrier but is not present")
        for body in set(state.poses) & set(state.carried):
            self.add(7, where, f"{body} is both in poses and in carried")
        for body, pose in state.poses.items():
            self.check_pose(pose, f"{where} pose of {body}")
        for body, carried in state.carried.items():
            self.check_pose(carried.offset, f"{where} offset of {body}")
            found = self.link_known(carried.to, f"{where} carrier of {body}")
            # * Rule 8: the carrying robot is in the scene.
            if found is not None and state.robots.get(found[0]) is None:
                self.add(8, where, f"{body} is carried by {found[0]}, which is not in this state")
        # * Rule 7: every mounted tool listed; rule 13: its channels and `on`.
        for robot_id, robot in design.robots.items():
            for tool in robot.tools.values():
                if tool not in state.tools:
                    self.add(7, where, f"tool {tool} is not listed")
                elif state.tools[tool] is not None and state.robots.get(robot_id) is None:
                    self.add(7, where, f"tool {tool} has a state but its robot {robot_id} is not in this state")
        for tool, tool_state in state.tools.items():
            if tool not in design.tools:
                self.add(4, where, f"unknown tool {tool!r}")
            elif tool_state is not None:
                self.check_channels(tool, {k: v for k, v in tool_state.items() if k != ON}, f"{where} {tool}")
                on = tool_state.get(ON)
                if on is not None and on not in state.present:
                    self.add(13, f"{where} {tool}", f"on {on!r}, which is not present")

    def check_channels(self, tool: str, channels: Dict[str, Optional[str]], where: str,
                       partial: bool = False) -> None:
        """A tool's channel values come from its kind's vocabulary; all channels unless `partial` (rule 13)."""
        if tool not in self.design.tools:
            self.add(4, where, f"unknown tool {tool!r}")
            return
        vocabulary = TOOL_CHANNELS.get(self.design.tools[tool].kind)
        if vocabulary is None:
            return  # unknown kind: reported once by check_robots_and_tools
        for channel, value in channels.items():
            if channel not in vocabulary:
                self.add(13, where, f"{tool} has no channel {channel!r}; its channels: {sorted(vocabulary)}")
            elif value is not None and value not in vocabulary[channel]:
                self.add(13, where, f"{tool} {channel} must be one of {vocabulary[channel]}, not {value!r}")
        missing = [channel for channel in vocabulary if channel not in channels]
        if missing and not partial:
            self.add(13, where, f"{tool} lacks channels {missing}")

    def check_carrying(self, action: Action, movement: Movement, at: str) -> None:
        """Carried bodies hang on a tool, arms carrying them hold them closed, closed tools pin their arm (rule 14)."""
        design, state = self.design, movement.start
        parts = self._parts
        robots_tools = {robot_id: robot.tools for robot_id, robot in design.robots.items()}

        def tool_of(flange_id: str) -> Optional[str]:
            robot, _, link = flange_id.rpartition("/")
            return robots_tools.get(robot, {}).get(link)

        carried_by: Dict[str, Set[str]] = {}
        for body, carried in state.carried.items():
            carried_by.setdefault(carried.to.rsplit("/", 1)[0], set()).add(body)
        # * Carrying needs a tool of the carrying robot on the part.
        for robot, bodies in carried_by.items():
            holding = {state.tools[tool].get(ON) for tool in robots_tools.get(robot, {}).values()
                       if state.tools.get(tool)}
            covered = set().union(*(members(parts, on) for on in holding if on is not None))
            for body in sorted(bodies - covered):
                self.add(14, f"{at} start", f"{body} is carried by {robot}, but no tool of it is on its part")
        if not movement.arms:
            return  # ? Manual steps and tool moves are exempt: the operator mounts the bar before the grasp.
        carried = carried_by.get(action.robot, set())
        for flange in movement.arms:
            tool = tool_of(flange)
            tool_state = state.tools.get(tool) if tool else None
            if not tool_state or tool_state.get(ON) is None:
                continue
            part = members(parts, tool_state[ON])
            grip = tool_state.get("grip")
            if part & carried and grip != "closed":
                self.add(14, at, f"{flange} carries {sorted(part & carried)} while {tool} grip is {grip!r}")
            if grip == "closed" and not part & carried:
                opens = movement.tool_change.get(tool, {}).get("grip") == "open"
                if not (movement.controller == "compliant" and opens):
                    self.add(14, at, f"{tool} is closed on {tool_state[ON]}, which {action.robot} does not carry: "
                                     f"the arm is pinned (only a compliant move that opens it may move)")

    def check_notes(self, notes: Dict[str, object], where: str) -> None:
        """Notes are flat: strings, numbers and booleans (rule 15)."""
        for key, value in notes.items():
            if not isinstance(value, NOTE_TYPES):
                self.add(15, f"{where} notes", f"{key!r} is a {type(value).__name__}; notes are flat values only")
