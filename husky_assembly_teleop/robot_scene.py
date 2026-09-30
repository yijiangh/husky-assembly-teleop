"""
The shared PyBullet scene: each real robot's URDF, posed from measurements every tick.

Plugins get the raw client id and body ids and call `p` and `pp` directly.

! Plugins remove the bodies they add in their own `teardown`. Nothing tracks them,
  and a leftover body is an obstacle in every other plugin's collision checks.
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Callable, Iterator, Mapping

import pybullet_planning as pp

from .config import RobotConfig

if TYPE_CHECKING:
    from .robot_interface import RobotState


def _as_text(name: bytes | str) -> str:
    """Decode a PyBullet joint or link name (bytes) to str.

    Args:
        name: A name as PyBullet reports it.

    Returns:
        str: The same name, comparable with the names ROS uses.
    """
    return name.decode("utf-8") if isinstance(name, bytes) else name


class RobotScene:
    """A PyBullet client with the real robots in it.

    ! Threading: PyBullet is not thread-safe, so every call here and every `p`/`pp`
      call in a plugin must run on the ROS thread. From a viser callback, go
      through PluginContext.submit.
    """

    def __init__(self, log_warn: Callable[[str], None], use_gui: bool = False):
        """Connect to PyBullet.

        Args:
            log_warn: Reports a robot whose measurements do not match its URDF.
            use_gui: Open PyBullet's own debug window. For debugging only.
        """
        self._log_warn = log_warn
        self.client_id = pp.connect(use_gui=use_gui, shadows=True, color=[0.9, 0.9, 1.0])

        #: Robot serial -> PyBullet body id.
        self.robots: dict[str, int] = {}

        # Serial -> {joint name: joint index}, built once at load.
        self._joints: dict[str, dict[str, int]] = {}

        # Unknown joint names already warned about, so each is reported once.
        self._unknown_joints: set[str] = set()

    @contextmanager
    def active(self) -> Iterator[None]:
        """Point pybullet_planning's free functions at this client.

        Yields:
            None: Inside the block, pp free functions act on this client.

        ! Plugins do not need this: the monitor runs every plugin hook inside it.
          Only bracket a second client of your own.
        """
        previous = pp.CLIENT
        pp.CLIENT = self.client_id
        try:
            yield
        finally:
            pp.CLIENT = previous

    def load_robot(self, config: RobotConfig) -> int:
        """Load one robot's URDF at its default pose, with a free (not fixed) base.

        The default pose keeps a fleet from loading as one pile at the origin,
        until the first mocap fix arrives.

        Args:
            config: Identity, URDF and default pose for this robot.

        Returns:
            int: The PyBullet body id.

        Raises:
            ValueError: If a robot with that serial is already loaded.
        """
        serial = config.serial
        if serial in self.robots:
            raise ValueError(f"robot {serial} is already in the scene")

        with self.active():
            body = pp.load_pybullet(str(config.urdf_file), fixed_base=False)
            pp.set_pose(body, (config.default_position, config.default_orientation))
            movable = pp.get_movable_joints(body)
            # ! PyBullet returns joint names as bytes; decode them, or every
            #   lookup misses and the robot silently never moves.
            self._joints[serial] = {
                _as_text(name): joint
                for name, joint in zip(pp.get_joint_names(body, movable), movable)
            }
        self.robots[serial] = body
        return body

    def sync_real(self, states: Mapping[str, "RobotState"]) -> None:
        """Pose every loaded robot from its latest measurement, once per tick.

        ! A missing measurement keeps the last value, not zero: an untracked base
          keeps its last pose, and an absent joint keeps its last position.

        Args:
            states: Measured state per serial, from WorldState.robot_states.
        """
        with self.active():
            for serial, body in self.robots.items():
                state = states.get(serial)
                if state is None:
                    continue
                if state.base.tracked:
                    pp.set_pose(body, (state.base.position, state.base.orientation))
                self._apply_joints(serial, body, state)

    def _apply_joints(self, serial: str, body: int, state: "RobotState") -> None:
        """Write one robot's measured joint positions into the scene.

        Joints missing from the URDF are skipped and warned about once.

        Args:
            serial: Robot serial, for the warning.
            body: Its PyBullet body id.
            state: Its measured state.
        """
        by_name = self._joints[serial]
        joints: list[int] = []
        values: list[float] = []
        for name, value in state.joint_positions.items():
            joint = by_name.get(name)
            if joint is None:
                if name not in self._unknown_joints:
                    self._unknown_joints.add(name)
                    self._log_warn(f"robot {serial} reports joint {name!r}, which its URDF "
                                   f"does not have; it will not move in the scene")
                continue
            joints.append(joint)
            values.append(value)
        if joints:
            pp.set_joint_positions(body, joints, values)

    def disconnect(self) -> None:
        """Close the PyBullet client.

        `pp.disconnect` closes whichever client is current, so this brackets it
        with `active()` to close only this one.
        """
        with self.active():
            pp.disconnect()
