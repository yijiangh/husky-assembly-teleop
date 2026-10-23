"""
Robot leases: every command that could move a robot, or switches its controllers, needs that robot's lease.

A lease keeps this monitor's own code from interfering with itself; it knows nothing of what else moves a robot. A task
that relies on other robots standing still adds a freeze (`world/freeze.py`).

- Stopping (`hold`, `stop`, a zero twist, a tool's `stop`, the soft stop and Resume) needs no lease.
- A lease takes all its robots or none, and never waits.
- ! Holding is per asyncio task: code inside `with leased(...)` or `with lease.use()` holds the lease, and so do the
  tasks it spawns there. Hand the lease to a helper that runs elsewhere and let it call `lease.use()`.
- ! Keep a lease until the motion you sent has ended or been held: given back early, other code here may command the
  robot mid-motion and nothing supervises it.
"""

from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
from typing import Iterable, Iterator

from .soft_stop import MotionRefused


class NotLeased(MotionRefused):
    """A command to a robot the running code holds no lease of: nothing was sent."""

    label = "lease refused"


class Lease:
    """Robots leased together to one holder, until `release`.

    Attributes:
        holder: Who holds them, for messages, e.g. "base_planner: commit plan 3".
        serials: The leased robots.
    """

    def __init__(self, holder: str, guards: tuple[LeaseSlot, ...]):
        """Lease `guards` to `holder`; only `take_lease` calls this, after checking they are free."""
        self.holder = holder
        self._guards = guards
        self.serials = tuple(guard.serial for guard in guards)
        for guard in guards:
            guard.lease = self

    @property
    def active(self) -> bool:
        """bool: Whether the robots are still leased to it: False once released."""
        return any(guard.lease is self for guard in self._guards)

    @contextmanager
    def use(self) -> Iterator[Lease]:
        """Hold this lease in the `with` block: commands there, and in tasks spawned there, may move its robots."""
        token = _HELD.set(_HELD.get() | {self})
        try:
            yield self
        finally:
            _HELD.reset(token)

    def release(self) -> None:
        """Give every robot of this lease back. Sends nothing: stopping a moving robot is the holder's job."""
        for guard in self._guards:
            if guard.lease is self:
                guard.lease = None


#: The leases the running code holds; asyncio gives each task its own copy.
_HELD: ContextVar[frozenset[Lease]] = ContextVar("held robot leases", default=frozenset())


def held_leases() -> frozenset[Lease]:
    """The leases the running code holds."""
    return _HELD.get()


class LeaseSlot:
    """One robot's lease, shared by its base, arms and tools, which refuse commands sent without it.

    Attributes:
        serial: The robot, for messages.
        lease: The lease it is given to, or None while free.
    """

    def __init__(self, serial: str):
        """Start free."""
        self.serial = serial
        self.lease: Lease | None = None

    def problem(self, held: frozenset[Lease] | None = None) -> str | None:
        """Why code holding `held` (by default the running code's leases) may not command this robot, or None."""
        lease = self.lease
        if lease is None:
            return f"robot {self.serial} is not leased; take its lease first"
        if lease in (held_leases() if held is None else held):
            return None
        return f"robot {self.serial} is leased by {lease.holder}"

    def check(self, command: str) -> None:
        """Raise NotLeased unless the running code holds this robot's lease.

        Args:
            command: What would be sent, for the message, e.g. "a joint trajectory to left".

        Raises:
            NotLeased: Nothing may be sent.
        """
        problem = self.problem()
        if problem is not None:
            raise NotLeased(f"{problem}, {command} not sent")


def take_lease(guards: Iterable[LeaseSlot], holder: str) -> Lease:
    """Lease every robot of `guards` to `holder` until `release`.

    Raises:
        NotLeased: If any of them is leased already, by anyone; none is leased then.
    """
    guards = tuple(dict.fromkeys(guards))
    taken = [f"robot {guard.serial} is leased by {guard.lease.holder}" for guard in guards if guard.lease is not None]
    if taken:
        raise NotLeased("; ".join(taken))
    return Lease(holder, guards)


@contextmanager
def leased(guards: Iterable[LeaseSlot], holder: str) -> Iterator[Lease]:
    """Lease every robot of `guards` to `holder` while the `with` block runs, and hold the lease in it.

    Raises:
        NotLeased: If any of them is leased already; none is leased then.
    """
    lease = take_lease(guards, holder)
    try:
        with lease.use():
            yield lease
    finally:
        lease.release()
