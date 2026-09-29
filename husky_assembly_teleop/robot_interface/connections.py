"""
The ROS entities one interface created, so they can be torn down and rebuilt.

Every interface in this package creates its subscriptions, publishers, clients,
timers and action clients through a RosConnections instead of the node, then
reconnects by destroying them all and creating them again.

? Why reconnect in place rather than build a new interface.
  Plugins keep references to interfaces and their state -- a button callback
  bound to `arm.zero_ft_sensor`, a panel holding `base.controllers`. Replacing
  the objects would leave those pointing at dead ones. Rebuilding only the ROS
  side keeps every reference valid, so plugin authors cannot get this wrong.
"""

from __future__ import annotations

import traceback
from typing import Callable

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile


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

    def subscription(self, message_type, topic: str, callback, qos: QoSProfile | int = 10):
        """Create a subscription, as `Node.create_subscription`, with a guarded callback."""
        entity = self._node.create_subscription(message_type, topic, self._guarded(topic, callback), qos)
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
        """Wrap a callback so that an exception drops the message instead of the node.

        ! Without this, one malformed message -- a zero quaternion, a missing
          field -- raises out of rclpy.spin and ends the whole monitor, every
          robot and every plugin with it. A measurement callback only stores
          data, so skipping one sample is always the smaller harm.

        The first failure is logged with its traceback; later ones from the same
        source are not, so a bad topic at 500 Hz cannot bury the log. A
        reconnect creates a fresh wrapper, which reports again.

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

        ! Requests still waiting on a destroyed client or action never get an
          answer, so their done callbacks never run. Whoever tracks such a
          request (`switch_in_flight`, a gripper's `moving`) has to clear it.
        """
        for destroy in reversed(self._destroyers):
            destroy()
        self._destroyers.clear()
