"""
The ROS entities one interface created, so a reconnect can destroy and recreate them.

Interfaces create every ROS entity through RosConnections, so plugins' references to the interface stay valid.
A fast topic whose messages only update state can be read newest-only: the monitor takes its messages undecoded and
decodes the newest one per tick (see `RosConnections.subscription`).
"""

from __future__ import annotations

import struct
import traceback
from typing import Callable

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.serialization import deserialize_message

from .event import Event

#: Bytes before the first field of a serialized message: the CDR encapsulation header (its 2nd byte 1 = little endian).
_CDR_HEADER = 4


class RosConnections:
    """Creates ROS entities on a node and remembers how to destroy each one."""

    def __init__(self, node: Node):
        """Start with nothing created.

        Args:
            node: The monitor node the entities are created on.
        """
        self._node = node
        #: One destroy function per entity created, in creation order.
        self._destroyers: list[Callable[[], None]] = []

    def subscription(self, message_type, topic: str, callback, qos: QoSProfile | int = 10,
                     newest_only: bool | Event = False, on_stamp: Callable[[float], None] | None = None):
        """Create a subscription, as `Node.create_subscription`, with a guarded callback.

        Decoding is most of a message's cost, so a fast topic whose callback only stores state can be newest-only:
        each tick the monitor takes all its messages undecoded and decodes and calls back only the newest.
        - ! Only for callbacks that keep the latest state: they miss whatever lasts less than a tick.

        Args:
            message_type: The message class.
            topic: The topic name.
            callback: Called with each decoded message.
            qos: Its QoS, or a queue depth.
            newest_only: True for newest-only; an Event for newest-only while nothing listens to it, every message
                while something does (a recording).
            on_stamp: Called with the `header.stamp` (seconds) of every message, decoded or not, before the
                callbacks; read straight from the bytes, so `message_type` must start with a `std_msgs/Header`.

        Raises:
            ValueError: If `on_stamp` is given for a type that does not start with a header, or without newest_only.
        """
        raw = newest_only is not False
        if on_stamp is not None and (not raw or next(iter(message_type.get_fields_and_field_types().items()))
                                     != ("header", "std_msgs/Header")):
            raise ValueError(f"on_stamp needs newest_only and a message starting with a std_msgs/Header ({topic})")
        if not raw:
            entity = self._node.create_subscription(message_type, topic, self._guarded(topic, callback), qos)
        else:
            stamp = (None if on_stamp is None
                     else self._guarded(f"{topic} stamps", lambda data: on_stamp(header_stamp(data))))
            decode = self._guarded(topic, lambda data: callback(deserialize_message(data, message_type)))

            def stamp_and_decode(data: bytes) -> None:
                """A message the executor took (it arrived after the drain): note its stamp and decode it."""
                if stamp is not None:
                    stamp(data)
                decode(data)

            entity = self._node.create_subscription(message_type, topic, stamp_and_decode, qos, raw=True)
            # * Read by the monitor's drain (`HuskyMonitor._drain_subscriptions`): whether to decode every message,
            #   and the stamp and decode steps on their own, each taking the undecoded message.
            entity.decode_every = (lambda: False) if newest_only is True else (lambda: newest_only.listened)
            entity.on_stamp, entity.decode = stamp, decode
        self._destroyers.append(lambda: self._node.destroy_subscription(entity))
        return entity

    def publisher(self, message_type, topic: str, qos: int = 10):
        """Create a publisher, as `Node.create_publisher`."""
        entity = self._node.create_publisher(message_type, topic, qos)
        self._destroyers.append(lambda: self._node.destroy_publisher(entity))
        return entity

    def client(self, service_type, name: str):
        """Create a service client, as `Node.create_client`."""
        entity = self._node.create_client(service_type, name)
        self._destroyers.append(lambda: self._node.destroy_client(entity))
        return entity

    def timer(self, period: float, callback):
        """Create a timer, as `Node.create_timer`, with a guarded callback."""
        entity = self._node.create_timer(period, self._guarded(f"timer {callback.__qualname__}", callback))
        self._destroyers.append(lambda: self._node.destroy_timer(entity))
        return entity

    def action_client(self, action_type, name: str) -> ActionClient:
        """Create an action client, as `ActionClient(node, ...)`."""
        entity = ActionClient(self._node, action_type, name)
        self._destroyers.append(entity.destroy)
        return entity

    def _guarded(self, source: str, callback: Callable) -> Callable:
        """Wrap a callback so an exception drops that message instead of killing the node.

        Only the first failure per source is logged, until the next reconnect.

        Args:
            source: Topic or timer name, for the log.
            callback: The real callback.

        Returns:
            Callable: The guarded callback.
        """
        reported = False

        def run(*args):
            nonlocal reported
            try:
                return callback(*args)
            except Exception:
                if not reported:
                    reported = True
                    self._node.get_logger().error(
                        f"callback for {source} failed; message dropped. Further failures "
                        f"from {source} are not logged until the next reconnect.\n{traceback.format_exc()}")
                return None

        return run

    def destroy_all(self) -> None:
        """Destroy everything created so far, newest first.

        ! Requests pending on destroyed clients never complete; whoever tracks them
          (`switch_in_flight`, `moving`) must clear them.
        """
        for destroy in reversed(self._destroyers):
            destroy()
        self._destroyers.clear()


def header_stamp(data: bytes) -> float:
    """The `header.stamp`, seconds, of a serialized message whose first field is a `std_msgs/Header`."""
    sec, nanosec = struct.unpack_from("<iI" if data[1] == 1 else ">iI", data, _CDR_HEADER)
    return sec + 1e-9 * nanosec
