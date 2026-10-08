"""
Small explicit id maps at the edges, and moving planned attachments onto other robots.

The core uses one id per object ("bars/B1", "robots/cindy"). Where two id spaces meet (a planned robot and the real
one, a mocap rigid body and a design body, design ids and a plugin's prefixed scene ids) an `IdMap` lists every
pair; an id it does not list is refused, never guessed.
"""

from __future__ import annotations

from typing import Dict, Mapping

from .robot import RobotModel
from .scene import ROBOTS, Attachment


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


def retarget(attachments: Mapping[str, Attachment], robot_map: IdMap,
             models: Mapping[str, RobotModel]) -> Dict[str, Attachment]:
    """The same attachments held by other robots: only the robot id changes; link and grasp stay.

    The link names are the same in both URDF variants of a robot, so a planned grasp carries over to the real robot.
    Attachments to a body, not a robot, are kept as they are.

    Args:
        attachments: Body id -> attachment, e.g. the held bodies of a design scene.
        robot_map: Robot id -> the robot to hold it instead, e.g. {"robots/cindy": "robots/a200-0806"}.
        models: The target robots' models, by their ids.

    Returns:
        dict[str, Attachment]: Body id -> the new attachment.

    Raises:
        KeyError: If a holding robot is not in `robot_map`, or its target is not in `models`.
        ValueError: If the target model lacks the link.
    """
    result = {}
    for body_id, attachment in attachments.items():
        if not attachment.parent.startswith(f"{ROBOTS}/"):
            result[body_id] = attachment
            continue
        target = robot_map(attachment.parent)
        if attachment.link is not None and attachment.link not in models[target].links:
            raise ValueError(f"{body_id}: {target} has no link {attachment.link!r} to hold it by")
        result[body_id] = Attachment(target, attachment.link, attachment.grasp)
    return result
