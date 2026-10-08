"""
The file checks A3–A13 of format §9 on a `Design` in memory; A1 and A2 are the reader's, A14 the solutions'.

They know nothing about kinematics or the assembly procedure (that is `plan_check`), only URDF/SRDF names. Every
problem is collected and raised in one `DesignError`, each line starting with "A<n>: <where>".
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple
from xml.etree.ElementTree import ParseError

import numpy as np

from ..geometry import BoxShape, CylinderShape, Geometry, Pose
from ..ids import ID_PATTERN
from ..urdf import is_relative_reference, mesh_references, movable_joints, urdf_joints, urdf_links
from .types import (ACTION_TYPES, BAR_PREFIX, BODY_PREFIXES, CONTROLLERS, ENDS_ON, GROUND_PREFIX, HALF_PREFIX, PATHS,
                    ROBOT_PREFIX, TOOL_PREFIX, Action, Design, DesignError, Holder, Movement, State)
from .vocabulary import GRIP, TOOL_KINDS

#: How far a quaternion's (or a line direction's) length may be from 1 (A7).
UNIT_TOLERANCE = 1e-6
#: The types a note value may have (A13).
NOTE_TYPES = (str, int, float, bool)


def validate(design: Design, *, check_robot_meshes: bool = True) -> None:
    """Run the file checks A3–A13.

    Args:
        design: The design, in memory or read from a folder.
        check_robot_meshes: Check that the URDFs' meshes exist by relative path (A4); `write` turns it off for its
            source URDFs, which may use `package://`.

    Raises:
        DesignError: With every problem found, if there is any.
    """
    problems = _Checker(design, check_robot_meshes).run()
    if problems:
        raise DesignError(problems)


class _RobotFiles:
    """The names a check needs from one robot's URDF and SRDF; None where a file could not be read."""

    def __init__(self, urdf: Path, srdf: Path, problems: List[str], robot: str):
        """Read the files, noting every one that is missing or broken (A4)."""
        self.links: Optional[Set[str]] = None
        self.joints: Optional[Dict[str, str]] = None
        self.movable: Optional[Tuple[str, ...]] = None
        self.meshes: List[str] = []
        if not Path(urdf).is_file():
            problems.append(f"A4: {robot}: URDF {str(urdf)!r} does not exist")
        else:
            try:
                self.links, self.joints = urdf_links(urdf), urdf_joints(urdf)
                self.meshes = mesh_references(urdf)
            except ParseError as error:
                problems.append(f"A4: {robot}: URDF {str(urdf)!r} is not valid XML ({error})")
        if not Path(srdf).is_file():
            problems.append(f"A4: {robot}: SRDF {str(srdf)!r} does not exist")
        elif self.joints is not None:
            try:
                self.movable = movable_joints(urdf, srdf)
            except ParseError as error:
                problems.append(f"A4: {robot}: SRDF {str(srdf)!r} is not valid XML ({error})")


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
        # Tool id -> the robot it is mounted on (the first, if several).
        self.mounted: Dict[str, str] = {}
        for robot_id, robot in design.robots.items():
            for tool in robot.tools.values():
                self.mounted.setdefault(tool, robot_id)

    def add(self, check: int, where: str, text: str) -> None:
        """Note one problem."""
        self.problems.append(f"A{check}: {where}: {text}")

    def run(self) -> List[str]:
        """Run every check.

        Returns:
            list[str]: The problems, one line each.
        """
        self.check_ids()
        self.check_robots_and_tools()
        self.check_bodies()
        self.check_schedule()
        for action_id in self.design.schedule:
            if action_id in self.design.actions:
                self.check_action(self.design.actions[action_id])
        return self.problems

    # --- --- --- --- --- REFERENCES --- --- --- --- ---

    def link_known(self, value: str, where: str) -> Optional[Tuple[str, str]]:
        """Check a link id "robots/<robot>/<link>": its robot exists and the link is in its URDF (A4).

        Returns:
            tuple[str, str] | None: (robot id, link name), or None if it does not resolve.
        """
        parts = value.split("/")
        if len(parts) != 3 or f"{parts[0]}/" != ROBOT_PREFIX:
            self.add(4, where, f"{value!r} is not a link id 'robots/<robot>/<link>'")
            return None
        robot, link = f"{parts[0]}/{parts[1]}", parts[2]
        if robot not in self.design.robots:
            self.add(4, where, f"link {value!r}: unknown robot {robot!r}")
            return None
        links = self.files[robot].links
        if links is not None and link not in links:
            self.add(4, where, f"link {value!r}: no link {link!r} in the URDF of {robot}")
            return None
        return robot, link

    def bodies_known(self, values: Iterable[str], where: str, prefixes: Tuple[str, ...] = BODY_PREFIXES,
                     check: int = 4) -> None:
        """Check body ids exist (A4) and have one of `prefixes` (reported under `check`)."""
        for value in values:
            if value not in self.design.bodies:
                self.add(4, where, f"unknown body {value!r}")
            elif not value.startswith(prefixes):
                self.add(check, where, f"{value!r} is not one of {', '.join(p.rstrip('/') for p in prefixes)}")

    def tool_known(self, tool: str, where: str) -> bool:
        """Check a tool id exists (A4)."""
        if tool not in self.design.tools:
            self.add(4, where, f"unknown tool {tool!r}")
            return False
        return True

    def check_joint_names(self, robot: str, joints: Dict[str, float], where: str) -> None:
        """Every joint named exists in the robot's URDF (A8)."""
        known = self.files[robot].joints if robot in self.files else None
        if known is None:
            return
        for name in joints:
            if name not in known:
                self.add(8, where, f"{robot} has no joint {name!r}")

    def check_pose(self, pose: Pose, where: str) -> None:
        """A pose's quaternion has unit length (A7)."""
        length = float(np.linalg.norm(pose.orientation))
        if abs(length - 1.0) > UNIT_TOLERANCE:
            self.add(7, where, f"quaternion {list(pose.orientation)} has length {length!r}, not 1")

    def check_geometry(self, geometry: Geometry, where: str) -> None:
        """Shape origins of a geometry have unit quaternions (A7)."""
        for shape in (*geometry.collision, *geometry.visual):
            if isinstance(shape, (BoxShape, CylinderShape)):
                self.check_pose(shape.origin, f"{where} shape origin")

    def check_label(self, label: str, where: str) -> None:
        """A label contains no "/" (A3)."""
        if "/" in label:
            self.add(3, where, f"label {label!r} contains '/'")

    # --- --- --- --- --- DESIGN-LEVEL --- --- --- --- ---

    def check_ids(self) -> None:
        """Ids match the pattern, keys match the objects, prefixes are known, nothing repeats, labels lack "/" (A3)."""
        design = self.design
        groups = (("robot", design.robots, (ROBOT_PREFIX,)), ("tool", design.tools, (TOOL_PREFIX,)),
                  ("body", design.bodies, BODY_PREFIXES))
        for kind, objects, prefixes in groups:
            for key, value in objects.items():
                if not ID_PATTERN.fullmatch(key):
                    self.add(3, key, f"invalid {kind} id {key!r}")
                if not key.startswith(prefixes):
                    self.add(3, key, f"{kind} id {key!r} does not start with {' or '.join(prefixes)}")
                if value.id != key:
                    self.add(3, key, f"{kind} is keyed {key!r} but has id {value.id!r}")
        for robot in design.robots:
            if robot.count("/") != 1:
                self.add(3, robot, f"robot id {robot!r} is not 'robots/<name>'")
        for body_id, body in design.bodies.items():
            self.check_label(body.label, body_id)
        for key, action in design.actions.items():
            if not ID_PATTERN.fullmatch(key) or "/" in key:
                self.add(3, key, f"invalid action id {key!r}")
            self.check_label(action.label, f"action {key}")
            for movement in action.movements:
                self.check_label(movement.label, f"movement {movement.id}")
        counts = Counter(movement.id for action in design.actions.values() for movement in action.movements)
        for movement_id, count in counts.items():
            if not ID_PATTERN.fullmatch(movement_id) or "/" in movement_id:
                self.add(3, movement_id, f"invalid movement id {movement_id!r}")
            if count > 1:
                self.add(3, movement_id, f"movement id {movement_id!r} is used {count} times")

    def check_robots_and_tools(self) -> None:
        """Robot files and links, tool mounts and kinds, mount contacts (A4, A6, A7, A10)."""
        design = self.design
        mounts: Dict[str, List[str]] = {tool: [] for tool in design.tools}
        for robot_id, robot in design.robots.items():
            files = self.files[robot_id]
            for link, tool in robot.tools.items():
                if files.links is not None and link not in files.links:
                    self.add(6, f"{robot_id} tools", f"mount link {link!r} of {tool} is not in the URDF")
                if self.tool_known(tool, f"{robot_id} tools"):
                    mounts[tool].append(f"{robot_id}/{link}")
            for link in robot.ground_links:
                if files.links is not None and link not in files.links:
                    self.add(4, f"{robot_id} ground_links", f"no link {link!r} in the URDF")
            # * Robot meshes by relative path, next to the URDF.
            folder = Path(robot.urdf).parent
            for reference in (dict.fromkeys(files.meshes) if self.check_robot_meshes else ()):
                if not is_relative_reference(reference):
                    self.add(4, robot_id, f"URDF mesh {reference!r} is not a path relative to the URDF")
                elif not (folder / reference).is_file():
                    self.add(4, robot_id, f"URDF mesh {reference!r} does not exist")
        for tool_id, tool in design.tools.items():
            if len(mounts[tool_id]) != 1:
                places = ", ".join(mounts[tool_id]) or "no robot"
                self.add(6, tool_id, f"mounted on {len(mounts[tool_id])} robots ({places}), must be exactly one")
            if tool.kind not in TOOL_KINDS:
                self.add(10, tool_id, f"unknown tool kind {tool.kind!r}; known: {sorted(TOOL_KINDS)}")
            self.check_pose(tool.tcp, f"{tool_id} tcp")
            self.check_geometry(tool.geometry, tool_id)
            for value in tool.mount_contacts:
                self.link_known(value, f"{tool_id} mount_contacts")

    def check_bodies(self) -> None:
        """Body poses and markers, mounts and mates (A4, A7, A12)."""
        design = self.design
        for body_id, body in design.bodies.items():
            self.check_pose(body.pose, f"{body_id} pose")
            self.check_geometry(body.geometry, body_id)
            for name, point in body.markers.items():
                if len(point) != 3:
                    self.add(2, f"{body_id} markers", f"marker {name!r} has {len(point)} coordinates, not 3")
            if body_id.startswith(HALF_PREFIX):
                if body.mount is None:
                    self.add(12, body_id, "a connector half needs a mount")
                elif body.mount not in design.bodies or not body.mount.startswith(BAR_PREFIX):
                    self.add(12, body_id, f"mount {body.mount!r} is not a bar of the design")
            elif body.mount is not None:
                self.add(12, body_id, "only connector halves (joints/) have a mount")
        in_mates: Counter = Counter()
        for mate in sorted(design.mates):
            self.bodies_known(mate, "mates")
            kinds = sorted(side.split("/", 1)[0] + "/" for side in mate)
            if kinds not in ([HALF_PREFIX, HALF_PREFIX], [GROUND_PREFIX, HALF_PREFIX]):
                self.add(12, "mates", f"{list(mate)} pairs neither two halves nor a half and a ground body")
            in_mates.update(side for side in mate if side.startswith(HALF_PREFIX))
        for half, count in in_mates.items():
            if count > 1:
                self.add(12, "mates", f"{half} is in {count} mates, at most one allowed")

    def check_schedule(self) -> None:
        """`schedule` and the action files match one to one (A5)."""
        design = self.design
        for action_id, count in Counter(design.schedule).items():
            if count > 1:
                self.add(5, "schedule", f"action {action_id!r} is scheduled {count} times")
            if action_id not in design.actions:
                self.add(5, "schedule", f"action {action_id!r} has no action file")
        for action_id in design.actions:
            if action_id not in design.schedule:
                self.add(5, f"actions/{action_id}.json", "action is not in the schedule")

    # --- --- --- --- --- ACTIONS --- --- --- --- ---

    def check_action(self, action: Action) -> None:
        """One action and its movements (A4, A7–A13)."""
        design = self.design
        where = f"action {action.id}"
        if action.type not in ACTION_TYPES:
            self.add(2, where, f"unknown action type {action.type!r}")
        if action.robot not in design.robots:
            self.add(4, where, f"unknown robot {action.robot!r}")
        self.bodies_known((action.bar,), f"{where} bar", (BAR_PREFIX,))
        self.bodies_known(action.ground, f"{where} ground", (GROUND_PREFIX,))
        self.bodies_known(action.supports_until, f"{where} supports_until", (BAR_PREFIX,))
        self.check_notes(action.notes, where)
        for movement in action.movements:
            at = f"{where} movement {movement.id}"
            self.check_parts(action, movement, at)
            self.check_state(movement.start, f"{at} start")
            self.check_notes(movement.notes, at)
            if movement.target is not None:
                self.check_target(movement, f"{at} target")

    def check_parts(self, action: Action, movement: Movement, at: str) -> None:
        """A movement's parts fit together (A4, A7, A10, A11)."""
        for arm in movement.arms:
            found = self.link_known(arm, f"{at} arms")
            if found is not None and found[0] != action.robot:
                self.add(4, f"{at} arms", f"{arm!r} is not a link of the acting robot {action.robot}")
        # * Path and controller exactly when arms move; a line on a linear path, one per moving arm.
        if movement.arms:
            if movement.path not in PATHS:
                self.add(11, at, f"arms move, so path must be one of {PATHS}, not {movement.path!r}")
            if movement.controller not in CONTROLLERS:
                self.add(11, at, f"arms move, so controller must be one of {CONTROLLERS}, not {movement.controller!r}")
        elif movement.path is not None or movement.controller is not None or movement.coupled:
            self.add(11, at, "no arm moves, so path, controller and coupled must be absent")
        if movement.ends_on not in ENDS_ON:
            self.add(11, at, f"ends_on must be one of {ENDS_ON}, not {movement.ends_on!r}")
        if movement.path == "linear" and set(movement.line) != set(movement.arms):
            self.add(11, f"{at} line", f"a linear path needs one line per moving arm {list(movement.arms)}, "
                                       f"has {sorted(movement.line)}")
        elif movement.line and movement.path != "linear":
            self.add(11, f"{at} line", "a line needs path linear")
        for flange, line in movement.line.items():
            length = float(np.linalg.norm(line.direction))
            if len(line.direction) != 3 or abs(length - 1.0) > UNIT_TOLERANCE:
                self.add(11, f"{at} line {flange}", f"direction {list(line.direction)} is not a unit vector")
            if line.distance <= 0.0:
                self.add(11, f"{at} line {flange}", f"distance {line.distance!r} is not positive")
        for tool, direction in movement.drives.items():
            if self.tool_known(tool, f"{at} drives"):
                directions = TOOL_KINDS.get(self.design.tools[tool].kind, ())
                if direction not in directions:
                    self.add(10, f"{at} drives", f"{tool} ({self.design.tools[tool].kind}) drives "
                                                 f"{list(directions) or 'nothing'}, not {direction!r}")

    def check_state(self, state: State, where: str) -> None:
        """One State is complete and consistent (A4, A7–A10)."""
        design = self.design
        # * A8: every robot listed, known joint names, and every movable joint given.
        for robot in design.robots:
            if robot not in state.robots:
                self.add(8, where, f"robot {robot} is not listed")
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
            self.check_joint_names(robot, robot_state.joints, f"{where} {robot} joints")
            movable = self.files[robot].movable
            if movable is not None:
                missing = [name for name in movable if name not in robot_state.joints]
                if missing:
                    self.add(8, f"{where} {robot} joints", f"missing joints {missing}")
        # * A9: present lists bars and ground; attached, built and poses name present bars only.
        self.bodies_known(state.present, f"{where} present", (BAR_PREFIX, GROUND_PREFIX), check=9)
        self.check_bars(state.attached, state.present, f"{where} attached")
        self.check_bars(state.built, state.present, f"{where} built")
        self.check_bars(state.poses, state.present, f"{where} poses")
        for bar in sorted(set(state.poses) & (set(state.built) | set(state.attached))):
            self.add(9, f"{where} poses", f"{bar} is built or attached, so its pose is not written")
        for bar, pose in state.poses.items():
            self.check_pose(pose, f"{where} pose of {bar}")
        self.check_holders(state.attached, where)
        # * A10: every mounted tool listed, its grip from the vocabulary, `on` an existing body.
        for robot in design.robots.values():
            for tool in robot.tools.values():
                if tool not in state.tools:
                    self.add(10, where, f"tool {tool} is not listed")
        for tool, tool_state in state.tools.items():
            if not self.tool_known(tool, f"{where} tools") or tool_state is None:
                continue
            if tool_state.grip is not None and tool_state.grip not in GRIP:
                self.add(10, f"{where} {tool}", f"grip must be one of {GRIP}, not {tool_state.grip!r}")
            if tool_state.on is not None and tool_state.on not in design.bodies:
                self.add(10, f"{where} {tool}", f"on {tool_state.on!r}, which is not a body")

    def check_bars(self, bars: Iterable[str], present: Iterable[str], where: str) -> None:
        """Bars named in a state exist and are present (A4, A9)."""
        present = set(present)
        for bar in bars:
            if bar not in self.design.bodies:
                self.add(4, where, f"unknown body {bar!r}")
            elif not bar.startswith(BAR_PREFIX):
                self.add(9, where, f"{bar!r} is not a bar")
            elif bar not in present:
                self.add(9, where, f"{bar} is not present")

    def check_holders(self, attached: Dict[str, Tuple[Holder, ...]], where: str) -> None:
        """Every holder names a robot link and a grasp with a unit quaternion (A4, A7, A9)."""
        for bar, holders in attached.items():
            if not holders:
                self.add(9, f"{where} attached", f"{bar} has no holder")
            for holder in holders:
                self.link_known(holder.to, f"{where} holder of {bar}")
                self.check_pose(holder.grasp, f"{where} grasp of {bar}")

    def check_target(self, movement: Movement, where: str) -> None:
        """A target names known robots, joints, links, tools, grips and bars (A4, A7–A10)."""
        design, target = self.design, movement.target
        for robot, joints in target.joints.items():
            if robot not in design.robots:
                self.add(4, where, f"unknown robot {robot!r}")
            else:
                self.check_joint_names(robot, joints, f"{where} joints")
        for link, pose in target.links.items():
            self.link_known(link, f"{where} links")
            self.check_pose(pose, f"{where} link {link}")
        for tool, grip in target.tools.items():
            if self.tool_known(tool, f"{where} tools") and grip not in GRIP:
                self.add(10, f"{where} {tool}", f"grip must be one of {GRIP}, not {grip!r}")
        if target.attached is not None:
            self.bodies_known(target.attached, f"{where} attached", (BAR_PREFIX,), check=9)
            self.check_holders(target.attached, where)
        if target.built is not None:
            self.bodies_known(target.built, f"{where} built", (BAR_PREFIX,), check=9)

    def check_notes(self, notes: Dict[str, object], where: str) -> None:
        """Notes are flat: strings, numbers and booleans (A13)."""
        for key, value in notes.items():
            if not isinstance(value, NOTE_TYPES):
                self.add(13, f"{where} notes", f"{key!r} is a {type(value).__name__}; notes are flat values only")
