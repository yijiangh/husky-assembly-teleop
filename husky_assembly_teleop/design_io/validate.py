"""
Check a design against rules 2–11 of format §9 (rule 1 is the reader's).

Every problem is collected and raised in one `DesignError`, each line starting with "rule <n>: <where>".
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple
from xml.etree.ElementTree import ParseError

import numpy as np

from .geometry import BoxShape, CylinderShape, Geometry
from .pose import ID_PATTERN, Pose
from .robot_files import (is_relative_reference, mesh_references, movable_joints, srdf_group_tips, urdf_joints,
                          urdf_links)
from .types import (ACTION_TYPES, BODY_PREFIXES, CONTROLLERS, MOVEMENT_TYPES, ROBOT_PREFIX, TOOL_PREFIX, Design,
                    DesignError, State)

#: How far a quaternion's length may be from 1 (rule 9).
UNIT_TOLERANCE = 1e-6


def validate(design: Design, *, check_robot_meshes: bool = True) -> None:
    """Check every rule of format §9 from 2 to 11.

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

    def any_known(self, value: str, where: str) -> None:
        """Check an id that may name a robot, a link, a tool or a body (touches, placeholder) (rule 4)."""
        design = self.design
        if value in design.robots or value in design.tools or value in design.bodies:
            return
        if value.startswith(ROBOT_PREFIX) and value.count("/") == 2:
            self.link_known(value, where)
            return
        self.add(4, where, f"unknown id {value!r}")

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
        """Robot files, tool mounts and tool fields (rules 3, 4, 5, 9, 11)."""
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
            self.check_pose(tool.tcp, f"{tool_id} tcp")
            self.check_geometry(tool.geometry, tool_id)
            for value in tool.touches:
                self.any_known(value, f"{tool_id} touches")

    def check_bodies(self) -> None:
        """Body poses and touches (rules 4, 9)."""
        for body_id, body in self.design.bodies.items():
            self.check_pose(body.pose, f"{body_id} pose")
            self.check_geometry(body.geometry, body_id)
            for value in body.touches:
                self.any_known(value, f"{body_id} touches")

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
        """One action and its movements (rules 4, 6–10)."""
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
        for movement in action.movements:
            at = f"{where} movement {movement.id}"
            if movement.type not in MOVEMENT_TYPES:
                self.add(4, at, f"unknown movement type {movement.type!r}")
            if movement.controller not in CONTROLLERS:
                self.add(4, at, f"unknown controller {movement.controller!r}")
            if movement.type in ("free", "linear") and not movement.arms:
                self.add(4, at, f"a {movement.type} movement needs arms")
            if movement.type == "tool" and (not movement.tools or not movement.tool_action):
                self.add(4, at, "a tool movement needs tools and tool_action")
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
            for tool in movement.tools:
                if tool not in design.tools:
                    self.add(4, f"{at} tools", f"unknown tool {tool!r}")
            self.check_state(movement.start, f"{at} start")
            if movement.target is not None:
                for robot, joints in movement.target.joints.items():
                    if robot not in design.robots:
                        self.add(4, f"{at} target", f"unknown robot {robot!r}")
                    else:
                        self.check_joint_names(robot, joints, f"{at} target joints")
                for link, pose in movement.target.links.items():
                    self.link_known(link, f"{at} target links")
                    self.check_pose(pose, f"{at} target link {link}")

    def check_state(self, state: State, where: str) -> None:
        """One State (rules 4, 6–9)."""
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
        self.bodies_known(state.attached, f"{where} attached")
        # * Rule 7: poses and attachments only for present bodies, never both.
        for body in (*state.poses, *state.attached):
            if body not in state.present:
                self.add(7, where, f"{body} has a pose or attachment but is not present")
        for body in set(state.poses) & set(state.attached):
            self.add(7, where, f"{body} is both in poses and in attached")
        for body, pose in state.poses.items():
            self.check_pose(pose, f"{where} pose of {body}")
        for body, attached in state.attached.items():
            self.check_pose(attached.grasp, f"{where} grasp of {body}")
            found = self.link_known(attached.to, f"{where} attachment of {body}")
            # * Rule 8: the holding robot is in the scene.
            if found is not None and state.robots.get(found[0]) is None:
                self.add(8, where, f"{body} is attached to {found[0]}, which is not in this state")
        for pair in state.touches:
            for value in pair:
                self.any_known(value, f"{where} touches")
        for value in state.placeholder:
            self.any_known(value, f"{where} placeholder")
