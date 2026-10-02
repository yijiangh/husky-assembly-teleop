"""
The design in memory: one frozen dataclass per object of doc/design_format.md, fields named as the file keys.

! Never change the dicts inside after construction; build a new object with `dataclasses.replace`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, FrozenSet, Literal, Optional, Tuple

from .geometry import Geometry
from .pose import Pose, compose

MovementType = Literal["free", "linear", "manual", "tool"]
ActionType = Literal["bar_jointing", "bar_release", "bar_holding", "bar_holding_release"]
Controller = Literal["joint_tracking", "cartesian_compliant", "none"]

MOVEMENT_TYPES: Tuple[str, ...] = ("free", "linear", "manual", "tool")
ACTION_TYPES: Tuple[str, ...] = ("bar_jointing", "bar_release", "bar_holding", "bar_holding_release")
CONTROLLERS: Tuple[str, ...] = ("joint_tracking", "cartesian_compliant", "none")

#: Body id prefixes (format §3.1).
BODY_PREFIXES: Tuple[str, ...] = ("bars/", "joints/", "ground/", "obstacles/")
ROBOT_PREFIX, TOOL_PREFIX = "robots/", "tools/"


# --- --- --- --- --- ERRORS --- --- --- --- ---

class DesignError(ValueError):
    """A design breaks the format. `problems` lists every broken rule found, one line each."""

    def __init__(self, problems):
        """Collect `problems`, one string per broken rule."""
        self.problems = list(problems)
        super().__init__("invalid design:\n  " + "\n  ".join(self.problems))


class SchemaMismatch(DesignError):
    """A file was written with another schema. Read it with the library at `commit`."""

    def __init__(self, path: str, schema: int, commit: str, expected: int):
        """Name the file, its schema and the commit that wrote it."""
        self.schema, self.commit = schema, commit
        super().__init__([f"{path} has schema {schema}, this library reads schema {expected}; "
                          f"check out design_io at commit {commit} to read it"])


# --- --- --- --- --- DESIGN-LEVEL OBJECTS --- --- --- --- ---

@dataclass(frozen=True)
class Writer:
    """Which code wrote a file (format §7)."""

    schema: int
    library: str
    commit: str
    dirty: bool


@dataclass(frozen=True)
class RobotSpec:
    """A robot (format §4.1). `urdf`/`srdf` are absolute once read.

    Attributes:
        tools: Flange link name -> tool id.
    """

    id: str
    urdf: Path
    srdf: Path
    serial: Optional[str]
    tools: Dict[str, str]

    @property
    def name(self) -> str:
        """str: The id without its prefix, e.g. "cindy"."""
        return self.id[len(ROBOT_PREFIX):]


@dataclass(frozen=True)
class ToolSpec:
    """A tool (format §4.2). Geometry and `tcp` are in the flange link frame."""

    id: str
    geometry: Geometry
    tcp: Pose
    kind: str
    touches: Tuple[str, ...] = ()


@dataclass(frozen=True)
class BodySpec:
    """A body (format §4.3). `pose` is its design pose in world; geometry is in the body frame."""

    id: str
    pose: Pose
    geometry: Geometry
    touches: Tuple[str, ...] = ()
    label: str = ""


# --- --- --- --- --- STATES --- --- --- --- ---

@dataclass(frozen=True)
class RobotState:
    """A robot at one moment. `joints` lists every non-passive joint, or is None (not fixed by the design)."""

    base: Pose
    joints: Optional[Dict[str, float]]


@dataclass(frozen=True)
class Attached:
    """A body held by a link: `to` is "robots/<robot>/<link>", `grasp` the body pose in that link's frame."""

    to: str
    grasp: Pose


@dataclass(frozen=True)
class State:
    """One complete moment (format §5.2).

    Attributes:
        robots: Every robot; None = not in the scene.
        touches: Allowed contacts, each pair sorted.
    """

    robots: Dict[str, Optional[RobotState]]
    present: FrozenSet[str]
    poses: Dict[str, Pose]
    attached: Dict[str, Attached]
    touches: FrozenSet[Tuple[str, str]] = frozenset()
    placeholder: FrozenSet[str] = frozenset()


@dataclass(frozen=True)
class Target:
    """Where a movement should end (format §5.3).

    Attributes:
        joints: Robot id -> joint -> value; a subset of joints is allowed.
        links: Link id -> world pose.
    """

    joints: Dict[str, Dict[str, float]]
    links: Dict[str, Pose]


@dataclass(frozen=True)
class Movement:
    """One movement of an action (format §5.1). `arms` are flange link ids."""

    id: str
    type: MovementType
    controller: Controller
    start: State
    arms: Tuple[str, ...] = ()
    coupled: bool = False
    tools: Tuple[str, ...] = ()
    tool_action: Optional[str] = None
    overlaps_next: bool = False
    target: Optional[Target] = None
    label: str = ""
    #: Producer's planning hints (e.g. `lm_distance_mm`), passed through unchanged. ? To become typed fields.
    notes: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Action:
    """One scheduled action (format §5)."""

    id: str
    type: ActionType
    robot: str
    bar: str
    movements: Tuple[Movement, ...]
    ground: Tuple[str, ...] = ()
    supports_until: Tuple[str, ...] = ()
    label: str = ""


@dataclass(frozen=True)
class Design:
    """A whole design folder (format §2).

    Attributes:
        folder: Where it was read from or written to; None for a design only in memory.
        actions: Keyed by action id; `schedule` gives their order.
    """

    folder: Optional[Path]
    writer: Writer
    robots: Dict[str, RobotSpec]
    tools: Dict[str, ToolSpec]
    bodies: Dict[str, BodySpec]
    schedule: Tuple[str, ...]
    actions: Dict[str, Action]

    def movements(self):
        """Every (action, movement) in execution order.

        Yields:
            tuple[Action, Movement]: Each movement with the action it belongs to.
        """
        for action_id in self.schedule:
            action = self.actions[action_id]
            for movement in action.movements:
                yield action, movement


# --- --- --- --- --- DERIVED --- --- --- --- ---

#: Forward kinematics supplied by the caller: (robot id, link name, joints, base) -> link pose in world.
LinkPose = Callable[[str, str, Dict[str, float], Pose], Pose]


def link_id(robot: str, link: str) -> str:
    """The id of one robot link: "robots/<robot>/<link>" from a robot id and a link name."""
    return f"{robot}/{link}"


def split_link_id(value: str) -> Tuple[str, str]:
    """Split "robots/<robot>/<link>" into (robot id, link name).

    Raises:
        ValueError: If `value` is not a link id.
    """
    parts = value.split("/")
    if len(parts) != 3 or parts[0] != ROBOT_PREFIX.rstrip("/"):
        raise ValueError(f"{value!r} is not a link id 'robots/<robot>/<link>'")
    return f"{parts[0]}/{parts[1]}", parts[2]


def world_pose(design: Design, state: State, body_id: str, link_pose: LinkPose) -> Optional[Pose]:
    """A body's world pose in a state: attached, moved, or at its design pose.

    Args:
        design: The design.
        state: The state.
        body_id: A body id.
        link_pose: Forward kinematics for attached bodies; this library has none of its own.

    Returns:
        Pose | None: None if the body is absent, or held by a robot whose joints are not fixed.
    """
    if body_id not in state.present:
        return None
    if body_id in state.attached:
        attached = state.attached[body_id]
        robot, link = split_link_id(attached.to)
        robot_state = state.robots.get(robot)
        if robot_state is None or robot_state.joints is None:
            return None
        return compose(link_pose(robot, link, robot_state.joints, robot_state.base), attached.grasp)
    return state.poses.get(body_id, design.bodies[body_id].pose)
