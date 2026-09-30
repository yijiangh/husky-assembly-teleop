"""
The ROS entities one interface created, so they can be destroyed and recreated.

Interfaces create all their ROS entities through RosConnections, and reconnect by
destroying and recreating them, so plugins' references to the interface stay valid.
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
        """Wrap a callback so an exception drops that message instead of killing the node.

        Only the first failure per source is logged (with traceback), so a bad
        topic cannot flood the log; a reconnect resets this.

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

        ! Pending requests on destroyed clients never complete, so their done
          callbacks never run; whoever tracks them (`switch_in_flight`, `moving`)
          must clear them.
        """
        for destroy in reversed(self._destroyers):
            destroy()
        self._destroyers.clear()
