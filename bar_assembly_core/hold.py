"""
Hold scenes, a planning helper: the world a support robot plans its release in.

A `bar_holding` action holds its bar until the bars in `supports_until` are built. Its hold scene is the world
once the last of those is built (`Design.scene_after`), with the held bar disabled. The format has no field for it.
"""

from __future__ import annotations

from .design_io.types import Design
from .scene import SceneSnapshot


def release_bar(design: Design, action_id: str) -> str:
    """The bar whose building lets a holding action let go: the one of its `supports_until` built last.

    Args:
        design: The design.
        action_id: A `bar_holding` action.

    Returns:
        str: A bar id.

    Raises:
        ValueError: If the action is not a `bar_holding` action with `supports_until`.
    """
    action = design.actions[action_id]
    if action.type != "bar_holding" or not action.supports_until:
        raise ValueError(f"{action_id} is a {action.type} action without supports_until: it holds nothing until later")
    last_built = {design.actions[other].bar: index for index, other in enumerate(design.schedule)}
    missing = [bar for bar in action.supports_until if bar not in last_built]
    if missing:
        raise ValueError(f"{action_id}: no action builds {missing}")
    return max(action.supports_until, key=last_built.__getitem__)


def hold_scene(scene: SceneSnapshot, bar: str) -> SceneSnapshot:
    """A copy of a scene with the held bar disabled; the scene given is not changed.

    Args:
        scene: Usually `design.scene_after(release_bar(design, action_id))`.
        bar: The held bar's id.

    Raises:
        KeyError: If the scene has no such body.
    """
    held = scene.copy()
    held.bodies[bar].enabled = False
    return held


def hold_scene_for(design: Design, action_id: str) -> SceneSnapshot:
    """The hold scene of a `bar_holding` action: `hold_scene(design.scene_after(release_bar(…)), action.bar)`."""
    return hold_scene(design.scene_after(release_bar(design, action_id)), design.actions[action_id].bar)
