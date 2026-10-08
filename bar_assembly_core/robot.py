"""
Robots: what a robot is (`RobotModel`, what mirrors build from), and one robot at one moment (`RobotObject`).

- ! A `RobotModel` never changes: another URDF, SRDF or tool is another model. Mirrors compare models by identity
  and rebuild a robot only when its model object changes.
- ! Mirrors and planners refuse to plan for a robot with `acting_problems()`.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from pathlib import Path
from typing import Dict, FrozenSet, List, Mapping, Optional, Sequence, Tuple

from .geometry import Geometry, Pose
from .urdf import TOOL_TOUCHES_ARM_LINKS, stock_frame_problem, urdf_joints, urdf_links
from .urdf import movable_joints as srdf_movable_joints

#: Suffix of the links tools mount on (UR convention: "<arm>_tool0").
FLANGE_SUFFIX = "_tool0"


@dataclass(frozen=True)
class Tool:
    """A tool mounted on a flange (format §4.2). Geometry and `tcp` are in the flange link frame.

    Attributes:
        id: E.g. "tools/AT3L" (a design's) or "tools/a200-0806/left_ur_arm" (a live robot's).
        kind: The end effector kind, e.g. "scaffolding_v3".
        touches: Ids the tool may touch, as written in a design (e.g. "robots/cindy/left_ur_arm_wrist_2_link").
    """

    id: str
    geometry: Geometry
    tcp: Pose
    kind: str
    touches: Tuple[str, ...] = ()


@dataclass(frozen=True, eq=False)
class RobotModel:
    """What a robot is: its URDF without tools, its SRDF, and the tools mounted on its flanges.

    Attributes:
        name: For messages, e.g. "cindy" or "a200-0806".
        urdf: The robot's URDF without tools; mesh paths absolute or relative to the file.
        srdf: Its SRDF (groups, passive joints, allowed link pairs), or None.
        flanges: Links tools mount on, e.g. ("left_ur_arm_tool0", "right_ur_arm_tool0").
        tools: Flange -> mounted tool, its geometry and TCP in the flange frame. Bare flanges are left out.
        tool_touches: Flange -> links of this robot its tool may touch.
        stock_ur_frames: Whether every arm keeps the stock UR base frames. The URDF variants differ only in
            `<arm>_base_link`: compare models by the forward kinematics of named links, not by file.
    """

    name: str
    urdf: Path
    srdf: Optional[Path]
    flanges: Tuple[str, ...]
    tools: Mapping[str, Tool]
    tool_touches: Mapping[str, Tuple[str, ...]]
    stock_ur_frames: bool

    @cached_property
    def links(self) -> FrozenSet[str]:
        """frozenset[str]: Every link name of the URDF (tools are not links)."""
        return frozenset(urdf_links(self.urdf))

    @cached_property
    def movable_joints(self) -> Tuple[str, ...]:
        """tuple[str, ...]: Joints a state must give, in URDF order: not fixed, not SRDF-passive (wheels), no mimic."""
        if self.srdf is not None:
            return srdf_movable_joints(self.urdf, self.srdf)
        return tuple(name for name, kind in urdf_joints(self.urdf).items() if kind != "fixed")


def robot_model(name: str, urdf: Path, srdf: Optional[Path], tools: Optional[Mapping[str, Tool]] = None,
                touches: Optional[Mapping[str, Sequence[str]]] = None) -> RobotModel:
    """A robot model with its flanges, tool touches and frame convention read from its URDF.

    Args:
        name: For messages.
        urdf: The URDF without tools.
        srdf: The SRDF, or None.
        tools: Flange -> mounted tool.
        touches: Flange -> more links its tool may touch, beyond the arm links in `urdf.TOOL_TOUCHES_ARM_LINKS`.

    Returns:
        RobotModel: A new model. ! Build each model once and share it: a new object makes mirrors rebuild.

    Raises:
        ValueError: If a tool is mounted on a link the URDF lacks.
    """
    tools, touches = dict(tools or {}), dict(touches or {})
    links = urdf_links(urdf)
    missing = sorted(set(tools) - links)
    if missing:
        raise ValueError(f"{name}: tools mounted on {missing}, which {Path(urdf).name} lacks")
    flanges = tuple(sorted({link for link in links if link.endswith(FLANGE_SUFFIX)} | set(tools)))
    tool_touches: Dict[str, Tuple[str, ...]] = {}
    arms = []
    for flange in flanges:
        arm = flange[:-len(FLANGE_SUFFIX)] if flange.endswith(FLANGE_SUFFIX) else None
        if arm is not None:
            arms.append(arm)
        near = [f"{arm}_{suffix}" for suffix in TOOL_TOUCHES_ARM_LINKS] if arm is not None else []
        tool_touches[flange] = tuple(dict.fromkeys(link for link in [*near, *touches.get(flange, ())] if link in links))
    stock = all(stock_frame_problem(Path(urdf), arm) is None for arm in arms)
    return RobotModel(name, Path(urdf), None if srdf is None else Path(srdf), flanges, tools, tool_touches, stock)


@dataclass(eq=False)
class RobotObject:
    """One robot at one moment, planned or measured. Whoever owns the scene may change its fields.

    Attributes:
        id: "robots/<name>", e.g. "robots/cindy" (planned) or "robots/a200-0806" (real).
        model: What it is. ! Assign another model to change it; never change one.
        base: World pose of the URDF's root link.
        joints: Value per joint by URDF name; joints not given count as 0.
        enabled: False: absent. It never collides and is not drawn; compas_fab mirrors park it far away.
        base_tracked: Whether `base` was measured (always True for a planned robot).
        unmeasured: Movable joints whose value is not known (never measured, or not authored); theirs in
            `joints` is a guess.
        base_time: Time `base` was measured, or None.
        joints_time: Time of the oldest arm's latest joint state, or None.
            ! Base and arms are measured at different instants; while moving, `base_time - joints_time` is the error.
        label: Display text; empty: the id.
    """

    id: str
    model: RobotModel
    base: Pose
    joints: Dict[str, float]
    enabled: bool = True
    base_tracked: bool = True
    unmeasured: FrozenSet[str] = frozenset()
    base_time: Optional[float] = None
    joints_time: Optional[float] = None
    label: str = ""

    def copy(self) -> RobotObject:
        """A copy that later changes to this robot don't reach. Shares the model."""
        return RobotObject(self.id, self.model, self.base, dict(self.joints), self.enabled, self.base_tracked,
                           self.unmeasured, self.base_time, self.joints_time, self.label)

    def acting_problems(self) -> List[str]:
        """Why this robot can't be planned for: absent, base not tracked, or joints not known; empty if it can."""
        problems = []
        if not self.enabled:
            problems.append("it is not in the scene")
        if not self.base_tracked:
            problems.append("its base is not tracked")
        if self.unmeasured:
            shown = sorted(self.unmeasured)
            problems.append(f"joints not known: {', '.join(shown[:4])}" + (" …" if len(shown) > 4 else ""))
        return problems
