"""
The design in memory, schema 2: one frozen dataclass per object of doc/design_format.md, fields named as the file keys.

! Never change the dicts inside after construction; build a new object with `dataclasses.replace`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Dict, FrozenSet, Literal, Optional, Tuple, Union

from ..geometry import Geometry, Pose
from ..ids import ROBOTS, TOOLS
from ..robot import Tool

if TYPE_CHECKING:
    from ..scene import Scene

ActionType = Literal["bar_jointing", "bar_release", "bar_holding", "bar_holding_release"]
Controller = Literal["position", "compliant"]
EndsOn = Literal["target", "tools", "operator"]

ACTION_TYPES: Tuple[str, ...] = ("bar_jointing", "bar_release", "bar_holding", "bar_holding_release")
PATHS: Tuple[str, ...] = ("free", "linear")
CONTROLLERS: Tuple[str, ...] = ("position", "compliant")
ENDS_ON: Tuple[str, ...] = ("target", "tools", "operator")

#: Body id prefixes (format §3.1).
BODY_PREFIXES: Tuple[str, ...] = ("bars/", "joints/", "ground/", "obstacles/")
ROBOT_PREFIX, TOOL_PREFIX = f"{ROBOTS}/", f"{TOOLS}/"

#: A tool's state: channel -> value (None: not decided), plus "on": the body the tool sits on, if any.
ToolState = Dict[str, Optional[str]]
#: A note value: flat only.
Note = Union[str, int, float, bool]


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
                          f"check out bar_assembly_core at commit {commit} to read it"])


# --- --- --- --- --- DESIGN-LEVEL OBJECTS --- --- --- --- ---

@dataclass(frozen=True)
class Writer:
    """Which library wrote a file (format §7)."""

    schema: int
    library: str
    commit: str
    dirty: bool


@dataclass(frozen=True)
class Producer:
    """Which code made the design (format §7), e.g. Rhino's `RSExportAllBarActions`."""

    repo: str
    commit: str
    dirty: bool
    command: str


@dataclass(frozen=True)
class RobotSpec:
    """A robot (format §4.1). `urdf`/`srdf` are absolute once read.

    Attributes:
        tools: Flange link name -> tool id.
        ground_links: Links the robot stands on (its wheels): they may touch any `ground/` body.
    """

    id: str
    urdf: Path
    srdf: Path
    serial: Optional[str]
    tools: Dict[str, str]
    ground_links: Tuple[str, ...] = ()

    @property
    def name(self) -> str:
        """str: The id without its prefix, e.g. "cindy"."""
        return self.id[len(ROBOT_PREFIX):]


@dataclass(frozen=True)
class BodySpec:
    """A body (format §4.3). `pose` is its design pose in world; geometry and markers are in the body frame.

    Attributes:
        part: Catalogue reference, e.g. "T20/Female"; empty if none.
        markers: Marker label -> (x, y, z), metres, for placement checks and mocap registration.
    """

    id: str
    pose: Pose
    geometry: Geometry
    label: str = ""
    part: str = ""
    markers: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)


# --- --- --- --- --- STATES --- --- --- --- ---

@dataclass(frozen=True)
class RobotState:
    """A robot at one moment; `base` and `joints` are None where the design does not decide them."""

    base: Optional[Pose]
    joints: Optional[Dict[str, float]]


@dataclass(frozen=True)
class Carried:
    """A body that moves with a robot: `to` is "robots/<robot>/<link>", `offset` the body pose in that link's frame."""

    to: str
    offset: Pose


@dataclass(frozen=True)
class State:
    """One complete moment (format §5.2): readable on its own.

    Attributes:
        robots: Every robot; None = not in the scene.
        present: Bodies in the scene.
        poses: The world pose of every present body that is not carried.
        carried: Bodies that move with a robot link.
        tools: Every mounted tool's state (`ToolState`); None = its robot is absent, or the state is unknown.
    """

    robots: Dict[str, Optional[RobotState]]
    present: FrozenSet[str]
    poses: Dict[str, Pose]
    carried: Dict[str, Carried] = field(default_factory=dict)
    tools: Dict[str, Optional[ToolState]] = field(default_factory=dict)


@dataclass(frozen=True)
class LineSpec:
    """A linear path of one flange: `direction` (unit, world frame) and `distance` (metres)."""

    direction: Tuple[float, float, float]
    distance: float


@dataclass(frozen=True)
class Target:
    """Where a movement should end (format §5.3); each part may be empty.

    Attributes:
        joints: Robot id -> joint -> value; a subset of joints is allowed.
        links: Link id -> world pose.
        tools: Tool id -> the channels that change -> their value at the end.
    """

    joints: Dict[str, Dict[str, float]] = field(default_factory=dict)
    links: Dict[str, Pose] = field(default_factory=dict)
    tools: Dict[str, Dict[str, str]] = field(default_factory=dict)


@dataclass(frozen=True)
class Movement:
    """One movement (format §5.1): a segment that runs without stopping, described by independent parts.

    Attributes:
        arms: Flange link ids that move; empty: no arm moves.
        path: "free" or "linear", when arms move.
        coupled: The arms keep their relative pose (they hold one bar).
        controller: "position" or "compliant", when arms move.
        line: Per moving flange of a linear path: direction and distance.
        ends_on: "target" (arms reached it, tools done), "tools" (tools done) or "operator" (confirmed).
        target: None: end where the next movement starts.
        notes: For people only; flat values. No program reads them.
    """

    id: str
    start: State
    arms: Tuple[str, ...] = ()
    path: Optional[str] = None
    coupled: bool = False
    controller: Optional[str] = None
    line: Dict[str, LineSpec] = field(default_factory=dict)
    ends_on: str = "target"
    target: Optional[Target] = None
    label: str = ""
    notes: Dict[str, Note] = field(default_factory=dict)

    @property
    def tool_change(self) -> Dict[str, Dict[str, str]]:
        """dict: Tool id -> the channels this movement changes, from its target (empty if none)."""
        return dict(self.target.tools) if self.target is not None else {}


@dataclass(frozen=True)
class Action:
    """One scheduled action (format §5): one robot's movements on one bar."""

    id: str
    type: ActionType
    robot: str
    bar: str
    movements: Tuple[Movement, ...]
    ground: Tuple[str, ...] = ()
    supports_until: Tuple[str, ...] = ()
    label: str = ""
    notes: Dict[str, Note] = field(default_factory=dict)


@dataclass(frozen=True)
class Design:
    """A whole design folder (format §2).

    Attributes:
        folder: Where it was read from or written to; None for a design only in memory.
        actions: Keyed by action id; `schedule` gives their order.
        connections: Pairs of bodies joined in the design (joint half and bar, mated halves, ground connector and
            ground), each sorted: an allowed contact when both are present, never a placement dependency.
        producer: The code that made the design, or None.
    """

    folder: Optional[Path]
    writer: Writer
    robots: Dict[str, RobotSpec]
    tools: Dict[str, Tool]
    bodies: Dict[str, BodySpec]
    schedule: Tuple[str, ...]
    actions: Dict[str, Action]
    connections: FrozenSet[Tuple[str, str]] = frozenset()
    producer: Optional[Producer] = None

    def movements(self):
        """Every (action, movement) in execution order.

        Yields:
            tuple[Action, Movement]: Each movement with the action it belongs to.
        """
        for action_id in self.schedule:
            action = self.actions[action_id]
            for movement in action.movements:
                yield action, movement

    def scene_at(self, movement: Movement) -> Scene:
        """The world at the start of one movement; it shares nothing mutable with the design (`scenes.scene_at`).

        ! Needs yourdfpy. Run it when the design or the movement changes, not every tick.
        """
        # ? Imported here: the scene, its robots and yourdfpy stay out of `import design`.
        from .scenes import scene_at
        return scene_at(self, movement)

    def scene_after(self, bar: str) -> Scene:
        """The world once a bar is built, following the schedule (`scenes.scene_after`).

        ! Needs yourdfpy.
        """
        from .scenes import scene_after
        return scene_after(self, bar)
