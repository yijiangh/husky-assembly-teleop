"""
What a collision check hit: every colliding pair, its contacts, and the scene as it was checked (`CollisionHit`).

Planners return one where a check fails; the `collisions` plugin draws it. Ids are our scene ids, so a hit draws from
its own scene with `scene_parts`, however the world moved since.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Mapping, Optional, Tuple

from scaffolding_assembly_core.ids import TOOLS
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.scene import Scene, world_poses

Point = Tuple[float, float, float]


@dataclass(frozen=True)
class Contact:
    """The deepest contact of one colliding pair, in the world frame.

    Attributes:
        pair: The two sides as our ids, narrowed to the link touched where a side is a whole other robot.
        point_a: The point on the first side.
        point_b: The point on the second side.
        depth: Signed distance, metres: negative while the two overlap, positive for a near miss within a margin.
    """

    pair: tuple[str, str]
    point_a: Point
    point_b: Point
    depth: float


@dataclass(frozen=True)
class CollisionHit:
    """Why one configuration of a robot is refused, or came too close: every pair, and the scene as checked.

    Attributes:
        scene: The snapshot checked, with the robot at the configuration that collides. ! Read-only.
        robot: The robot checked, e.g. "robots/a200-0806".
        where: Which configuration, e.g. "waypoint 1", "the move to waypoint 3", "start", "target", or the task
            a clearance stopped.
        pairs: Every colliding pair that blocks it, as our ids; the first is the one the check stopped on. Where the
            check moved some joints, the moving side comes first.
        contacts: One per pair, None where no contact point was found.
        concave: Ids whose meshes the check tested as they are; every other mesh as its convex hull. Empty for
            compas_fab, which hulls everything.
        margin: How close counts, metres: 0 for a collision, else a clearance.
    """

    scene: Scene
    robot: str
    where: str
    pairs: tuple[tuple[str, str], ...]
    contacts: tuple[Optional[Contact], ...] = ()
    concave: FrozenSet[str] = frozenset()
    margin: float = 0.0

    @property
    def message(self) -> str:
        """str: One line naming the first pair, e.g. "waypoint 10 is in collision: ur_arm tool with ipad_rack"."""
        a, b = self.pairs[0]
        if self.margin > 0.0:
            return f"{self.where}: {self.name(a)} came within {self.margin * 100:.0f} cm of {self.name(b)}"
        return f"{self.where} is in collision: {self.name(a)} with {self.name(b)}"

    def name(self, object_id: str) -> str:
        """Short display text for one of the ids: its label, else its last segment, e.g. "ur_arm_wrist_3"."""
        text = self.scene.label(object_id)
        if text != object_id:
            return text
        # ? A mounted tool's id ends in its arm's name ("tools/a200-0805/ur_arm"), which alone reads as the arm.
        return text.rsplit("/", 1)[-1].removesuffix("_link") + (" tool" if text.startswith(f"{TOOLS}/") else "")


def scene_at(snapshot: Scene, robot: str, joints: Mapping[str, float]) -> Scene:
    """A copy of a snapshot with some of a robot's joints changed, and the bodies it holds moved with them.

    Args:
        snapshot: The scene; it is not changed.
        robot: The robot's id.
        joints: New values by joint name.

    Returns:
        Scene: The copy, `world_poses` recomputed.
    """
    scene = snapshot.copy()
    scene.robots[robot].joints.update(joints)

    def pose_of(holder, link: str):
        return link_pose(holder.model.urdf, holder.base, holder.joints, link)

    scene.world_poses = world_poses(scene.bodies, scene.robots, pose_of)
    return scene
