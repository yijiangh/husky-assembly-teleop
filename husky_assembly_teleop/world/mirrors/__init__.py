"""
Mirrors: private copies of a `SceneSnapshot` in one collision backend each, for planners.

! A mirror belongs to one thread, usually a planner's worker. Backend ids never leave it.
"""

from os import environ


def check_display() -> None:
    """Raise unless there is an X display: without one, opening a PyBullet window exits the whole process.

    Raises:
        RuntimeError: If DISPLAY is not set.
    """
    if not environ.get("DISPLAY"):
        raise RuntimeError("no X display to open a PyBullet window on")
