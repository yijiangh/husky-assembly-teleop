"""
PyBullet, holding the live robots and nothing else.

Connects to PyBullet, loads each real robot's URDF once, and poses them from
measurements every tick. That is the whole job.

! No abstraction over PyBullet here.
  No scene partitions, no collision-filter type, no body-id hiding, no
  add/remove API. Plugins get the raw client id and body ids and call `p` and
  `pp` directly. The audit of old call sites that settled this is in
  doc/refactor_rationale.md. If a real shared need shows up once the plugins are
  ported, factor it out then, against actual call sites.

! Plugins clean up after themselves. Nothing tracks what a plugin loads into the
  scene; it removes its own bodies in its own `teardown`.
"""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Iterator, Mapping

import pybullet_planning as pp

from .config import RobotConfig

if TYPE_CHECKING:
    from .robot_interface import RobotState


def _as_text(name: bytes | str) -> str:
    """Decode a PyBullet name, which comes back as bytes, to str.

    Args:
        name: A joint or link name as PyBullet reports it.

    Returns:
        str: The same name, comparable with the names ROS uses.
    """
    return name.decode("utf-8") if isinstance(name, bytes) else name


class RobotScene:
    """A PyBullet client with the real robots in it.

    ! Threading. PyBullet is not thread-safe, so every call here -- and every
      `p`/`pp` call a plugin makes -- must happen on the ROS thread. A viser
      callback that wants the scene goes through PluginContext.submit.
    """

    def __init__(self, log_warn: Callable[[str], None], use_gui: bool = False):
        """Connect to PyBullet.

        Args:
            log_warn: Where to report a robot whose measurements do not match
                its URDF. Injected rather than reached for, so the scene can be
                driven from a test without standing up a node.
            use_gui: Whether to open PyBullet's own debug window. Normally False:
                viser is the user interface, and this is only for debugging the
                scene itself.
        """
        self._log_warn = log_warn
        self.client_id = pp.connect(use_gui=use_gui, shadows=True, color=[0.9, 0.9, 1.0])

        #: Robot serial -> PyBullet body id. Plugins read this directly; they
        #: need the int to call `p` and `pp` at all.
        self.robots: dict[str, int] = {}

        # Serial -> {joint name: joint index}, built once per robot at load.
        # Measurements arrive keyed by joint name, PyBullet wants indices, and
        # resolving that per tick would be a name lookup per joint per robot.
        self._joints: dict[str, dict[str, int]] = {}

        # Joint names seen in measurements that the URDF does not have, so the
        # warning is printed once rather than at 20 Hz forever.
        self._unknown_joints: set[str] = set()

    @contextmanager
    def active(self) -> Iterator[None]:
        """Point pybullet_planning's free functions at this client.

        pp reads a module global to decide which client to talk to, so calls
        into it have to be bracketed. Doing it in one place keeps the old
        scattered `saved = pp.CLIENT; pp.CLIENT = ...` dance -- which corrupted
        the global whenever something returned early -- from coming back.

        Yields:
            None: Inside the block, pp free functions act on this client.

        Example:
            >>> with ctx.scene.active():
            ...     body = ctx.scene.robots["a200-0806"]
            ...     pose = pp.get_link_pose(body, pp.link_from_name(body, "ur_arm_tool0"))
        """
        previous = pp.CLIENT
        pp.CLIENT = self.client_id
        try:
            yield
        finally:
            pp.CLIENT = previous

    def load_robot(self, config: RobotConfig) -> int:
        """Load one real robot's URDF, stand it at its default pose, remember its id.

        The base is left free rather than fixed, because a husky drives: its
        pose comes from mocap every tick, not from a weld to the world. Until
        the first fix arrives `sync_real` leaves it alone, so the default pose
        is what keeps a fleet from loading as one pile at the origin.

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
            # ! PyBullet returns joint names as bytes. Measurements arrive from
            #   ROS as str, so without decoding here every lookup misses and the
            #   robot silently never moves.
            self._joints[serial] = {
                _as_text(name): joint
                for name, joint in zip(pp.get_joint_names(body, movable), movable)
            }
        self.robots[serial] = body
        return body

    def sync_real(self, states: Mapping[str, "RobotState"]) -> None:
        """Pose every loaded robot from its latest measurement, once per tick.

        Takes the measurements rather than the whole WorldState, so this stays a
        PyBullet module and nothing here can command a robot.

        ! Only tracked poses are applied. A robot whose base is not currently
          tracked keeps its last good pose rather than snapping to the origin,
          which is what used to teleport the planning robot before the first fix
          arrived. The same reasoning covers joints: a measurement that has not
          arrived is absent from `joint_positions`, so the scene keeps the last
          value instead of assuming zero.

        Args:
            states: Measured state per serial, from WorldState.robot_states.
        """
        with self.active():
            for serial, body in self.robots.items():
                state = states.get(serial)
                if state is None:
                    continue
                if state.base_tracked:
                    pp.set_pose(body, (state.base_position, state.base_orientation))
                self._apply_joints(serial, body, state)

    def _apply_joints(self, serial: str, body: int, state: "RobotState") -> None:
        """Write one robot's measured joint positions into the scene.

        Called from inside `active()`. Joint names the URDF does not have are
        skipped and reported once: that mismatch means the robot is running a
        different configuration than the URDF describes, which is worth knowing
        and not worth crashing over.

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

        `pp.disconnect` takes no arguments -- it closes whichever client the
        module global currently points at -- so this has to be bracketed like
        every other pp call, or it would close somebody else's.
        """
        with self.active():
            pp.disconnect()
