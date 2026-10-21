"""
A planner's compas_fab mirrors, one per robot it plans for, and the robot shown in PyBullet's own window.

! Worker thread only: make, sync, plan with and close the mirrors on the one thread that owns them.
"""

from __future__ import annotations

from typing import Callable

from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.scene import Scene


class MirrorPool:
    """One compas_fab mirror per robot planned for, and the robot shown in the window. Worker thread only."""

    def __init__(self):
        """Start empty; a robot's mirror is made on its first use (slow: loads the models)."""
        self._mirrors: dict[str, CompasFabMirror] = {}
        #: The robot used last; the PyBullet window shows its world.
        self.serial: str | None = None
        #: Whether the PyBullet window is open.
        self.gui = False
        #: The snapshot synced last: display text for our ids (`label`), and the scene of a `CollisionHit`.
        self.snapshot = Scene()
        self.label = self.snapshot.label
        #: Given to each mirror: logs each full rebuild of its cell, and why. Set by the plugin.
        self.log: Callable[[str], None] | None = None

    @property
    def serials(self) -> tuple[str, ...]:
        """tuple[str, ...]: The robots with a mirror made."""
        return tuple(self._mirrors)

    def mirror(self, serial: str) -> CompasFabMirror:
        """One robot's mirror, made on first use; the PyBullet window, if open, moves to it.

        Args:
            serial: The robot to plan for.

        Returns:
            CompasFabMirror: Its mirror, as last synced.
        """
        mirror = self._mirror(serial)
        if self.gui and self.serial is not None and serial != self.serial:
            # ? One window per process: it follows the robot being planned for.
            self._mirrors[self.serial].set_gui(False)
            mirror.set_gui(True)
        self.serial = serial
        return mirror

    def sync(self, snapshot: Scene, serial: str) -> CompasFabMirror:
        """Make one robot's planning world match a snapshot.

        Args:
            snapshot: The tick's copy of the world, taken on the main thread.
            serial: The robot to plan for.

        Returns:
            CompasFabMirror: Its mirror, synced.
        """
        mirror = self.mirror(serial)
        mirror.sync(snapshot)
        self.snapshot, self.label = snapshot, snapshot.label
        return mirror

    def set_gui(self, gui: bool, snapshot: Scene, serial: str) -> None:
        """Open or close PyBullet's own window on a robot's planning world, for debugging. Worker thread.

        Args:
            gui: Whether the window should be open.
            snapshot: The world now, shown even before the first plan.
            serial: The robot whose world to show.

        Raises:
            RuntimeError: If there is no X display.
            pybullet.error: If another PyBullet window is open in this process.
        """
        if self.serial is not None:
            self._mirrors[self.serial].set_gui(False)
        self.gui = False
        if gui:
            self._mirror(serial).set_gui(True)  # ? before sync, so the cell is built once
            self.gui = True
            self.sync(snapshot, serial)

    def _mirror(self, serial: str) -> CompasFabMirror:
        """A robot's mirror, made on first use."""
        if serial not in self._mirrors:
            self._mirrors[serial] = CompasFabMirror(robot_id(serial), log=self.log)
        return self._mirrors[serial]

    def close(self) -> None:
        """Disconnect every world. On the worker, once no search is running."""
        for mirror in self._mirrors.values():
            mirror.close()
        self._mirrors.clear()
