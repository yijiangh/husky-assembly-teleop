"""
Ids: one canonical id per object, a path such as "bars/B1", "robots/cindy" or "robots/cindy/left_ur_arm_tool0".

Where two id spaces meet (a planned robot and the real one, a mocap rigid body and a design body, design ids and a
plugin's prefixed scene ids) an `IdMap` lists every pair; an id it does not list is refused, never guessed.
"""

from __future__ import annotations

import re
from typing import Dict, Mapping, Tuple

#: What an id may contain: path segments of letters, digits, `_`, `.` and `-`, joined by `/`.
ID_PATTERN = re.compile(r"[A-Za-z0-9_.\-]+(/[A-Za-z0-9_.\-]+)*")

#: First id segments of robots, mounted tools and measured objects.
ROBOTS, TOOLS, TRACKED = "robots", "tools", "tracked"


def check_id(object_id: str) -> None:
    """Refuse an id that does not match ID_PATTERN.

    Raises:
        ValueError: If the id is invalid.
    """
    if not ID_PATTERN.fullmatch(object_id):
        raise ValueError(f"invalid id {object_id!r}: use letters, digits, '_', '.', '-' and '/' only")


def robot_id(name: str) -> str:
    """The id of a robot: "robots/<name>", e.g. "robots/a200-0806"."""
    return f"{ROBOTS}/{name}"


def tracked_id(name: str) -> str:
    """The id of a measured object: "tracked/<name>"."""
    return f"{TRACKED}/{name}"


def link_id(robot: str, link: str) -> str:
    """The id of one robot link: "robots/<robot>/<link>" from a robot id and a link name."""
    return f"{robot}/{link}"


def split_link_id(value: str) -> Tuple[str, str]:
    """Split "robots/<robot>/<link>" into (robot id, link name).

    Raises:
        ValueError: If `value` is not a link id.
    """
    parts = value.split("/")
    if len(parts) != 3 or parts[0] != ROBOTS:
        raise ValueError(f"{value!r} is not a link id 'robots/<robot>/<link>'")
    return f"{parts[0]}/{parts[1]}", parts[2]


class IdMap:
    """An explicit map from one id space to another. A mapped robot maps its links too.

    Example:
        >>> robots = IdMap({"robots/cindy": "robots/a200-0806"})
        >>> robots("robots/cindy/left_ur_arm_tool0")
        'robots/a200-0806/left_ur_arm_tool0'
    """

    def __init__(self, pairs: Mapping[str, str]) -> None:
        """Map each key to its value.

        Args:
            pairs: Id -> id in the other space.
        """
        self.pairs: Dict[str, str] = dict(pairs)

    def __call__(self, object_id: str) -> str:
        """The id in the other space.

        Raises:
            KeyError: If neither the id nor, for a robot link "robots/<name>/<link>", its robot is mapped.
        """
        found = self.get(object_id)
        if found is None:
            raise KeyError(f"{object_id!r} is not mapped; map it explicitly (known: {sorted(self.pairs)[:5]} …)")
        return found

    def __contains__(self, object_id: str) -> bool:
        """Whether the id, or its robot for a link, is mapped."""
        return self.get(object_id) is not None

    def get(self, object_id: str) -> str | None:
        """The id in the other space, or None if it is not mapped."""
        if object_id in self.pairs:
            return self.pairs[object_id]
        parts = object_id.split("/")
        robot = "/".join(parts[:2])
        if parts[0] == ROBOTS and len(parts) == 3 and robot in self.pairs:
            return f"{self.pairs[robot]}/{parts[2]}"
        return None
