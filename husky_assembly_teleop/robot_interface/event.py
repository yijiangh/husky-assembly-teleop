"""An interface's message event: it hands each parsed message (a record) to whoever listens, e.g. a recording."""

from __future__ import annotations

from typing import Any, Callable

#: Messages kept between ticks on a topic with an event: a second at 50 Hz, so a late tick loses none.
MESSAGE_QUEUE = 50


class Event:
    """Fired by an interface once per message, after its state holds the message's record.

    A record carries `update_time` (ROS time the monitor handled the message, seconds) and `stamp` (time the
    sender stamped it, seconds on its clock); the rest are its own fields, e.g. `FollowerState`.

    - ! Listeners run inside the ROS callback, once per message: keep them cheap, and never let them raise.
    - ! Subscribe its topic with a queue of MESSAGE_QUEUE: the monitor reads topics only once per tick.
    """

    def __init__(self, name: str):
        """Start with no listeners.

        Args:
            name: Unique among the events a recording uses: the group name in a saved file, e.g. "0806_follower".
        """
        self.name = name
        self._listeners: list[Callable[[Any], None]] = []

    def attach(self, listener: Callable[[Any], None]) -> None:
        """Call `listener(record)` on every message from now on."""
        self._listeners.append(listener)

    def detach(self, listener: Callable[[Any], None]) -> None:
        """Stop calling `listener`. Unknown listeners are ignored."""
        if listener in self._listeners:
            self._listeners.remove(listener)

    def fire(self, record: Any) -> None:
        """Hand `record` to every listener, in the order they attached."""
        for listener in tuple(self._listeners):
            listener(record)

    def __repr__(self) -> str:
        """The event's name, for error messages."""
        return f"Event({self.name!r})"
