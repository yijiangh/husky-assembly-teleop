"""
The arm planner plugin's planning worlds, one compas_fab mirror per robot; the search is `shared.planning.arm`.

! Worker thread only: sync, search and close on the worker.
"""

from __future__ import annotations

import threading
from typing import Callable, Sequence

from ...shared.planning.arm import ArmPath, _extend
from ...shared.planning.arm import plan_arm as search_arm
from ...shared.planning.joints import arm_joint_names
from ...shared.planning.search import PlanResult
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.scene import Scene
from scaffolding_assembly_core.ids import robot_id

__all__ = ["ArmPath", "ArmPlanningWorld", "_extend", "arm_joint_names", "plan_arm"]


class ArmPlanningWorld:
    """One compas_fab mirror per robot planned for, and the robot shown in the window. Worker thread only."""

    def __init__(self):
        """Start empty; a robot's mirror is made on its first sync (slow: loads the models)."""
        self._mirrors: dict[str, CompasFabMirror] = {}
        #: The robot synced last; the PyBullet window shows its world.
        self.serial: str | None = None
        #: Whether the PyBullet window is open.
        self.gui = False
        # Display text for our ids, from the last synced snapshot.
        self.label = Scene().label
        #: Given to each mirror: logs each full rebuild of its cell, and why. Set by the plugin.
        self.log: Callable[[str], None] | None = None

    def sync(self, snapshot: Scene, serial: str) -> CompasFabMirror:
        """Make one robot's planning world match a snapshot.

        Args:
            snapshot: The tick's copy of the world, taken on the main thread.
            serial: The robot to plan for.

        Returns:
            CompasFabMirror: Its mirror, synced.
        """
        mirror = self._mirror(serial)
        if self.gui and self.serial is not None and serial != self.serial:
            # ? One window per process: it follows the robot being planned for.
            self._mirrors[self.serial].set_gui(False)
            mirror.set_gui(True)
        self.serial = serial
        mirror.sync(snapshot)
        self.label = snapshot.label
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


def plan_arm(world: ArmPlanningWorld, mirror: CompasFabMirror, arm_name: str, goal: Sequence[float],
             abort: threading.Event) -> PlanResult[ArmPath]:
    """Plan a collision-free joint path for one arm (`shared.planning.arm.plan_arm`), named with the world's labels.

    Args:
        world: The planning world, for labels.
        mirror: The robot's mirror, already synced.
        arm_name: Which arm, e.g. "ur_arm".
        goal: Six joint values, radians, UR driver order.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult[ArmPath]: The path, or why there is none.
    """
    return search_arm(mirror, arm_name, goal, abort, label=world.label)
