"""
Mirrors: private copies of a scene in one collision backend each, for planners.

- ! A mirror belongs to one thread, usually a planner's worker. Backend ids never leave it.
- ! A disabled body stands at HIDDEN_POSITION in the backend: leaving it out of the checks is not enough, since a
  PyBullet window and code reading the backend directly (tamp after `lend`) would still see it.
"""

from __future__ import annotations

from os import environ

#: Where mirrors put disabled bodies and absent robots: far from everything that moves, and from PARKED_POSITION.
HIDDEN_POSITION = (-50.0, -50.0, -50.0)


def check_display() -> None:
    """Raise unless there is an X display: without one, opening a PyBullet window exits the whole process.

    Raises:
        RuntimeError: If DISPLAY is not set.
    """
    if not environ.get("DISPLAY"):
        raise RuntimeError("no X display to open a PyBullet window on")
