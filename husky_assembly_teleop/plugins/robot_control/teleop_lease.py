"""
robot_control's leases: it leases a robot when the operator first commands it, and gives the lease back once idle.

- ! Every command robot_control sends that needs a lease goes through `TeleopLeases.use`.
- A robot is idle once robot_control has sent it nothing for IDLE_AFTER and the plugin's `busy` says so: no input
  held, no move or force of its own, nothing of the robot still moving.
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Callable, Iterator

from ...robot_interface.robot_lease import Lease

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext

#: Seconds without a command before an idle robot's lease is given back.
IDLE_AFTER = 1.0


class TeleopLeases:
    """The robots robot_control holds, and when it last sent each a command."""

    def __init__(self):
        """Hold nothing."""
        self._leases: dict[str, Lease] = {}
        self._used: dict[str, float] = {}

    def held(self, serial: str) -> bool:
        """Whether robot_control holds robot `serial`'s lease."""
        lease = self._leases.get(serial)
        return lease is not None and lease.active

    def taken(self, ctx: PluginContext, serial: str) -> str | None:
        """Why robot_control may not command robot `serial` now (another holder has its lease), or None."""
        lease = ctx.world.robots[serial].lease_slot.lease
        if lease is None or lease is self._leases.get(serial):
            return None
        return f"{serial} is leased by {lease.holder}"

    @contextmanager
    def use(self, ctx: PluginContext, serial: str) -> Iterator[None]:
        """Command robot `serial` in the `with` block under robot_control's lease, leasing it first if free.

        Raises:
            NotLeased: If another holder has the robot's lease.
        """
        lease = self._leases.get(serial)
        if lease is None or not lease.active:
            lease = self._leases[serial] = ctx.take_lease([serial], "teleop")
        self._used[serial] = ctx.now()
        with lease.use():
            yield

    def release_idle(self, ctx: PluginContext, busy: Callable[[str], bool]) -> None:
        """Give back the lease of every robot unused for IDLE_AFTER that `busy` says is idle.

        Args:
            ctx: The plugin's context.
            busy: Whether robot_control still uses a robot, by serial.
        """
        now = ctx.now()
        for serial, lease in list(self._leases.items()):
            if not lease.active or (now - self._used[serial] > IDLE_AFTER and not busy(serial)):
                lease.release()
                del self._leases[serial]
