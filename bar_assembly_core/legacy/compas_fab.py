"""
A design as compas_fab objects: one `RobotCell` per acting robot (format App. A), and its `RobotCellState`s.

Every other robot is a `ToolModel` of its whole URDF with its tools welded on; cell keys are our ids unless
`names` maps them. A robot absent from a state is parked at PARKED_POSITION, since compas_fab needs every tool.

- ! A test harness for the equivalence check (scripts/legacy_equivalence.py): planners build cells through the mirrors.
- ! Importing this module imports compas; the core never imports it.
"""

from __future__ import annotations

from typing import Callable, Dict, Mapping, Optional

from compas.geometry import Frame
from compas_fab.robots import RigidBodyState, RobotCell, RobotCellState, RobotSemantics, ToolState
from compas_robots import ToolModel

from ..mirrors.compas import (PARKED_POSITION, filled, frame_from_pose, load_model, planning_group, pose_from_frame,
                              rigid_body, robot_as_tool, tool_model)
from ..geometry import Pose, compose
from ..design.types import ROBOT_PREFIX, Design, State
from ..ids import split_link_id


# --- --- --- --- --- CELLS --- --- --- --- ---

def _name(names: Optional[Mapping[str, str]], our_id: str) -> str:
    """The cell's key for one of our ids."""
    return names.get(our_id, our_id) if names else our_id


def _tool_model(design: Design, tool_id: str, name: str) -> ToolModel:
    """A tool of the acting robot, keyed `name`: its shapes as one mesh, its TCP as the tool frame."""
    return tool_model(design.tools[tool_id], name)


def _robot_as_tool(design: Design, robot_id: str, name: str) -> ToolModel:
    """Another robot as one articulated tool keyed `name`: its whole URDF, each tool a link fixed at its flange."""
    robot = design.robots[robot_id]
    return robot_as_tool(robot.urdf, {link: design.tools[tool] for link, tool in robot.tools.items()}, name)


def to_robot_cell(design: Design, robot_id: str, names: Optional[Mapping[str, str]] = None,
                  include: Optional[Callable[[str], bool]] = None) -> RobotCell:
    """A compas_fab cell: the acting robot with its SRDF and tools, every other robot, and the bodies.

    Args:
        design: The design.
        robot_id: The acting robot.
        names: Our id -> key in the cell, for ids that should keep another name.
        include: Which bodies to put in the cell, by id; all by default.
    """
    robot = design.robots[robot_id]
    model = load_model(robot.urdf)
    semantics = RobotSemantics.from_srdf_file(str(robot.srdf), model)
    tools = {_name(names, tool_id): _tool_model(design, tool_id, _name(names, tool_id))
             for tool_id in robot.tools.values()}
    for other in design.robots:
        if other != robot_id:
            tools[_name(names, other)] = _robot_as_tool(design, other, _name(names, other))
    bodies = {_name(names, body_id): rigid_body(body.geometry, draw_visual=True)
              for body_id, body in design.bodies.items() if include is None or include(body_id)}
    return RobotCell(model, semantics, tool_models=tools, rigid_body_models=bodies)


def to_cell_state(design: Design, robot_id: str, state: State, cell: RobotCell,
                  names: Optional[Mapping[str, str]] = None,
                  joints: Optional[Mapping[str, float]] = None) -> RobotCellState:
    """A compas_fab state for a cell made by `to_robot_cell`.

    Args:
        design: The design.
        robot_id: The acting robot.
        state: The state.
        cell: The acting robot's cell; bodies it leaves out are left out here too.
        names: As given to `to_robot_cell`.
        joints: Acting robot joints to use where the state has none (e.g. `carry.assumed_joints`).
    """
    robot = design.robots[robot_id]
    acting = state.robots[robot_id]
    acting_joints = acting.joints if acting.joints is not None else joints
    link_pose = compas_link_pose(design)
    own_links = {link.name for link in cell.robot_model.links}

    # * Every allowed contact, both ways: design-level touches of bodies and tools, and this state's.
    contacts: Dict[str, set] = {}
    pairs = set(state.touches)
    pairs |= {(body_id, other) for body_id, body in design.bodies.items() for other in body.touches}
    pairs |= {(tool_id, other) for tool_id, tool in design.tools.items() for other in tool.touches}
    for a, b in pairs:
        contacts.setdefault(a, set()).add(b)
        contacts.setdefault(b, set()).add(a)

    def split(touching) -> tuple:
        """Allowed contacts as compas_fab's (touch_links of the acting robot, touch_bodies)."""
        links, bodies = [], []
        for other in sorted(touching):
            if other.startswith(robot_id + "/"):
                links.append(split_link_id(other)[1])
            elif other == robot_id:
                links.extend(sorted(own_links))
            elif other.startswith(ROBOT_PREFIX):
                # Another robot is one tool: any of its links means the whole robot.
                bodies.append(_name(names, "/".join(other.split("/")[:2])))
            else:
                bodies.append(_name(names, other))
        return links, bodies

    # * Tools of the acting robot: attached to the group ending at their flange.
    tool_states = {}
    for link_name, tool_id in robot.tools.items():
        touch_links, _ = split(contacts.get(tool_id, ()))
        tool_states[_name(names, tool_id)] = ToolState(
            frame=None, attached_to_group=planning_group(cell, link_name), touch_links=touch_links,
            attachment_frame=Frame.worldXY())
    # * Every other robot: a tool at its base with its joints, or parked when absent.
    for other in design.robots:
        if other == robot_id:
            continue
        key = _name(names, other)
        other_state = state.robots.get(other)
        zero = cell.tool_models[key].zero_configuration()
        if other_state is None:
            tool_states[key] = ToolState(frame=Frame(PARKED_POSITION), configuration=zero)
        else:
            tool_states[key] = ToolState(frame=frame_from_pose(other_state.base),
                                         configuration=filled(zero, other_state.joints))

    # * Bodies: hidden when absent, attached when held by the acting robot, else stationary.
    body_states = {}
    for body_id, body in design.bodies.items():
        key = _name(names, body_id)
        if key not in cell.rigid_body_models:
            continue
        touch_links, touch_bodies = split(contacts.get(body_id, ()))
        body_state = RigidBodyState(frame=frame_from_pose(body.pose), touch_links=touch_links,
                                    touch_bodies=touch_bodies)
        attached = state.attached.get(body_id)
        if body_id not in state.present:
            body_state.is_hidden = True
        elif attached is not None and attached.to.startswith(robot_id + "/"):
            body_state.attached_to_link = split_link_id(attached.to)[1]
            body_state.attachment_frame = frame_from_pose(attached.grasp)
            body_state.frame = None
        elif attached is not None:
            # Held by another robot: stationary where that robot holds it, allowed to touch it.
            holder, link = split_link_id(attached.to)
            holder_state = state.robots.get(holder)
            if holder_state is not None:
                pose = compose(link_pose(holder, link, holder_state.joints or {}, holder_state.base),
                               attached.grasp)
                body_state.frame = frame_from_pose(pose)
            body_state.touch_bodies.append(_name(names, holder))
        else:
            body_state.frame = frame_from_pose(state.poses.get(body_id, body.pose))
        body_states[key] = body_state

    configuration = filled(cell.zero_full_configuration(), acting_joints) if acting_joints is not None else None
    return RobotCellState(robot_base_frame=frame_from_pose(acting.base), robot_configuration=configuration,
                          tool_states=tool_states, rigid_body_states=body_states)


# --- --- --- --- --- KINEMATICS --- --- --- --- ---

def compas_link_pose(design: Design) -> Callable[[str, str, Mapping[str, float], Pose], Pose]:
    """Forward kinematics from each robot's URDF: (robot id, link, joints, base) -> link pose; missing joints 0."""
    def link_pose(robot_id: str, link: str, joints: Mapping[str, float], base: Pose) -> Pose:
        model = load_model(design.robots[robot_id].urdf)
        frame = model.forward_kinematics(filled(model.zero_configuration(), joints), link_name=link)
        return compose(base, pose_from_frame(frame))
    return link_pose
