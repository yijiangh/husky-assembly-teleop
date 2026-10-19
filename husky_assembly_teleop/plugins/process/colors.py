"""Colours of process bodies, RGB 0-255, for the overlay and the scene bodies."""

from __future__ import annotations

#: Colours of bodies nobody holds, RGB 0-255.
BAR_COLOR = (205, 170, 110)
JOINT_COLOR = (120, 120, 140)
GROUND_COLOR = (170, 180, 170)


def body_color(body_id: str) -> tuple[int, int, int]:
    """The colour of a body nobody holds, RGB 0-255: bar, ground, or anything else (joint halves, obstacles)."""
    if body_id.startswith("bars/"):
        return BAR_COLOR
    if body_id.startswith("ground/"):
        return GROUND_COLOR
    return JOINT_COLOR
