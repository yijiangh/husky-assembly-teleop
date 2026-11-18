"""
Plan margins: how far a planned arm path keeps from each pair of parts.

One place for the search, the clamp at its ends, the path's smallest margin and every text about them.

- A pair keeps `PlanMargins.default`, or the margin of the first `MarginRule` matching one of its parts (the floor's
  FLOOR_PLAN_DISTANCE in ARM_PLAN_MARGINS). A new special margin is one more rule.
- A pair its start or target is already closer for keeps a little less than it has there (`clamped_at`, a `Clamp`),
  rounded down to RESOLUTION; 0 is contact only.
- ! Plans only: what stops a moving arm is `MoveKind.stop_for(smallest)` (`world/clearance.py`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable, Sequence

from scaffolding_assembly_core.ids import link_id
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror, SearchCheck

from ...world.clearance import ARM_MOVE, FLOOR_PLAN_DISTANCE
from ...world.lab import is_floor

#: How finely `PlanMargins.clamped_at` rounds down the margin of a pair closer than usual at an end, metres.
RESOLUTION = 0.0005


@dataclass(frozen=True)
class MarginRule:
    """Every pair with one kind of part keeps its own margin, e.g. the floor's 5 cm.

    ! Meant for margins of at least `PlanMargins.default`: one below it lowers `smallest`, so the clearance, for every
      path in a scene with such a part, near it or not.

    Attributes:
        label: The parts in words, for the texts: "the floor".
        matches: Whether a part id is one of them.
        margin: What their pairs keep, metres.
    """

    label: str
    matches: Callable[[str], bool]
    margin: float


@dataclass(frozen=True)
class Clamp:
    """A pair its start or target is already closer for than usual, and what it keeps instead.

    Attributes:
        first: One part's id.
        second: The other's.
        kept: What it keeps, metres: a little below its distance at the nearer end; 0 is contact only.
        usual: What it would keep otherwise (`PlanMargins.usual`).
    """

    first: str
    second: str
    kept: float
    usual: float


@dataclass(frozen=True)
class PlanMargins:
    """What every pair keeps on a planned path: `default`, a rule's margin, or a clamp's.

    Attributes:
        default: What a pair no rule matches keeps, metres.
        rules: Special margins, the first matching one wins.
        clamps: Pairs closer than usual at the path's start or target, closest first (`clamped_at`).
    """

    default: float
    rules: tuple[MarginRule, ...] = ()
    clamps: tuple[Clamp, ...] = ()

    def rule_for(self, first: str, second: str) -> MarginRule | None:
        """The first rule matching either part, or None."""
        return next((rule for rule in self.rules if rule.matches(first) or rule.matches(second)), None)

    def usual(self, first: str, second: str) -> float:
        """What a pair keeps unless clamped: its rule's margin, else `default`."""
        rule = self.rule_for(first, second)
        return self.default if rule is None else rule.margin

    @property
    def smallest(self) -> float:
        """float: The least any pair keeps: what the clearance is set from (`MoveKind.stop_for`)."""
        return min([self.default, *(rule.margin for rule in self.rules), *(clamp.kept for clamp in self.clamps)])

    def pair_margins(self, mirror: CompasFabMirror) -> dict[frozenset[str], float]:
        """The margin of every pair not keeping `default`, by its two ids: for `search_check`'s `pair_margins`.

        A pair the check never looks at (nothing on it moves) is a harmless extra key.
        """
        parts = mirror_parts(mirror)
        margins = {frozenset((part, other)): self.usual(part, other)
                   for part in parts if any(rule.matches(part) for rule in self.rules)
                   for other in parts if other != part}
        return {**margins, **{frozenset((clamp.first, clamp.second)): clamp.kept for clamp in self.clamps}}

    def search_check(self, mirror: CompasFabMirror, names: Sequence[str]) -> SearchCheck:
        """A search check over the joints `names` with these margins."""
        return mirror.search_check(names, self.default, pair_margins=self.pair_margins(mirror))

    def clamped_at(self, mirror: CompasFabMirror, names: Sequence[str], *ends: Sequence[float]) -> PlanMargins:
        """These margins for a path between `ends` (joint values of `names`) in `mirror`'s scene.

        Keeps only the rules matching a part there, and clamps each pair closer than usual at an end to a little less
        than its distance at the nearer one, rounded down to RESOLUTION.
        """
        parts = mirror_parts(mirror)
        base = replace(self, rules=tuple(rule for rule in self.rules if any(map(rule.matches, parts))), clamps=())
        check = base.search_check(mirror, names)
        kept: dict[tuple[str, str], float] = {}
        for values in ends:
            for pair, distance in check.distances(values).items():
                # * Strictly below the distance, so that end itself stays clear (a point at its margin is a hit).
                margin = max(0.0, math.floor((distance - 1e-6) / RESOLUTION) * RESOLUTION)
                kept[pair] = min(kept.get(pair, margin), margin)
        clamps = sorted((Clamp(*pair, margin, base.usual(*pair)) for pair, margin in kept.items()),
                        key=lambda clamp: clamp.kept)
        return replace(base, clamps=tuple(clamps))

    def describe(self) -> str:
        """What the pairs keep, unclamped, in words: "1 cm, 5 cm from the floor"."""
        return ", ".join([_cm(self.default), *(f"{_cm(rule.margin)} from {rule.label}" for rule in self.rules)])

    def note(self) -> str:
        """What a path keeps, for its commit: "keeps 1 cm, 5 cm from the floor", or the closest clamp first."""
        if not self.clamps:
            return f"keeps {self.describe()}"
        closest = self.clamps[0]
        more = len(self.clamps) - 1
        between = (f"between {part_name(closest.first)} and {part_name(closest.second)}"
                   + (f" (and {more} more pair{'s' if more > 1 else ''})" if more else ""))
        kept = f"keeps only {_cm(closest.kept)}" if closest.kept > 0 else "only free of contact"
        why = "is that close" if closest.kept > 0 else "touches"
        return (f"{kept} {between}: the start or target {why} (usual {_cm(closest.usual)}); "
                f"elsewhere {self.describe()}")


#: An arm plan's margins: ARM_MOVE's plan margin (PLAN_DISTANCE), FLOOR_PLAN_DISTANCE from the floor.
ARM_PLAN_MARGINS = PlanMargins(ARM_MOVE.plan, (MarginRule("the floor", is_floor, FLOOR_PLAN_DISTANCE),))


def mirror_parts(mirror: CompasFabMirror) -> list[str]:
    """Every part a search check in `mirror` can name: the robot's links, its shown tools and bodies, by id.

    ? Follows `SearchCheck`'s naming (`link_id` of the robot's links, tools and bodies by name); the only place that
      does. A core `search_check` margin per part would make it unneeded.
    """
    state = mirror.state
    return ([link_id(mirror.robot_id, link.name) for link in mirror.cell.robot_model.links]
            + [name for name, tool in state.tool_states.items() if not tool.is_hidden]
            + [name for name, body in state.rigid_body_states.items() if not body.is_hidden])


def part_name(part_id: str) -> str:
    """A part's id in plain words, e.g. "right_ur_arm_wrist_1_link", "the right_ur_arm tool", "the floor"."""
    if is_floor(part_id):
        return "the floor"
    kind, _, rest = part_id.partition("/")
    if kind == "tools":
        return f"the {rest.split('/', 1)[-1]} tool"
    if kind == "robots":
        return rest.split("/", 1)[-1]
    return rest if kind == "process" else part_id


def _cm(metres: float) -> str:
    """A distance for the texts, in cm, e.g. "0.5 cm"."""
    return f"{round(metres * 100, 2):g} cm"
