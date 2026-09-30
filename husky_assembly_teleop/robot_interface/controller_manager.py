"""
One ros2_control controller manager, and which of its controllers is running.

The base and each arm have their own controller manager; this one class handles
all of them, differing only in which controllers they may switch between.

! The running controller can be changed from outside, so it is read back from the
  controller manager, never assumed from the last request.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node

from .connections import RosConnections

#: Seconds between list_controllers polls.
REFRESH_PERIOD = 1.0


@dataclass
class ControllerManagerState:
    """What one controller manager last told us.

    Attributes:
        controllers: Reported controller name to lifecycle state. Empty until the
            first answer.
        active: The running switchable controller, or None if none runs or no
            answer has arrived (see `last_update_time`).
        switch_in_flight: Controller a switch is waiting on, or None.
        switch_error: Why the last switch failed, or None. Cleared when a new
            switch goes out.
        last_update_time: ROS time in seconds of the last answer, or None.
    """

    controllers: dict[str, str] = field(default_factory=dict)
    active: str | None = None
    switch_in_flight: str | None = None
    switch_error: str | None = None
    last_update_time: float | None = None


class ControllerManagerInterface:
    """Tracks and switches the controllers of one controller manager.

    ! Nothing blocks: answers land in `state` from the main thread (run by the tick's
      ROS pump) a few ticks later, so callers poll `state` (e.g. with concurrency.wait_until).
    """

    def __init__(self, node: Node, namespace: str, switchable: tuple[str, ...],
                 state: ControllerManagerState):
        """Create the service clients and start polling the controller list.

        Args:
            node: The monitor node.
            namespace: Namespace of the controller manager, e.g. "/a200_0806/left_ur5e".
            switchable: Controllers this part may run, one at a time.
            state: Where answers are written.
        """
        self._node = node
        self.namespace = namespace
        self.switchable = switchable
        self.state = state
        self._ros = RosConnections(node)
        # Set once the missing service is logged, so it is not repeated every poll.
        self._reported_unavailable = False
        self._connect()

    def _connect(self) -> None:
        """Create the two service clients and the poll timer."""
        self._list_client = self._ros.client(
            ListControllers, f"{self.namespace}/controller_manager/list_controllers")
        self._switch_client = self._ros.client(
            SwitchController, f"{self.namespace}/controller_manager/switch_controller")
        # * Polled because the service is often not up yet at construction and
        #   controllers can change externally. `active` is None until the first answer.
        self._ros.timer(REFRESH_PERIOD, self.refresh)

    def reconnect(self) -> None:
        """Recreate the clients and poll timer, keeping `state`.

        ! A pending switch is dropped: its answer never arrives, and it would
          block every later switch.
        """
        self._ros.destroy_all()
        self.state.switch_in_flight = None
        self._reported_unavailable = False
        self._connect()

    def is_active(self, controller: str) -> bool:
        """Whether `controller` is the one running right now, as last reported."""
        return self.state.active == controller

    def refresh(self) -> bool:
        """Ask the controller manager which controllers are running.

        Returns:
            bool: True if the request went out; False if the service is not up.
        """
        if not self._list_client.service_is_ready():
            if not self._reported_unavailable:
                self._node.get_logger().warning(
                    f"{self.namespace}: controller manager is not available; "
                    f"the active controller is unknown")
                self._reported_unavailable = True
            return False
        if self._reported_unavailable:
            self._node.get_logger().info(f"{self.namespace}: controller manager is available")
            self._reported_unavailable = False
        future = self._list_client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list_answer)
        return True

    def switch(self, controller: str) -> bool:
        """Start `controller`, stopping whichever switchable one is running.

        Asynchronous: `state.active` updates on success, `state.switch_error` on
        failure. Already running counts as success.

        Args:
            controller: One of `switchable`.

        Returns:
            bool: True if the request went out or nothing was needed. False if
                `controller` is not switchable, the service is not up, or a
                switch is still pending; `state.switch_error` says which.
        """
        if controller not in self.switchable:
            self.state.switch_error = (f"{controller!r} is not one of {self.switchable} "
                                       f"on {self.namespace}")
            self._node.get_logger().error(self.state.switch_error)
            return False
        if self.state.active == controller:
            return True
        if not self._switch_client.service_is_ready():
            self.state.switch_error = f"{self.namespace}: switch_controller is not available"
            self._node.get_logger().error(self.state.switch_error)
            return False
        if self.state.switch_in_flight is not None:
            # ! Refused, not queued: the deactivate list below comes from the last
            #   poll, which the pending switch has made stale.
            self.state.switch_error = (f"{self.namespace}: still switching to "
                                       f"{self.state.switch_in_flight!r}, ignoring {controller!r}")
            self._node.get_logger().error(self.state.switch_error)
            return False

        # ? Deactivates every other switchable controller reported active, not just
        #   `state.active`, which may be empty while another still holds the joints.
        request = SwitchController.Request()
        request.activate_controllers = [controller]
        request.deactivate_controllers = [
            name for name in self.switchable
            if name != controller and self.state.controllers.get(name) == "active"]
        request.strictness = SwitchController.Request.STRICT
        request.start_asap = True

        self.state.switch_in_flight = controller
        self.state.switch_error = None
        future = self._switch_client.call_async(request)
        future.add_done_callback(lambda done: self._on_switch_answer(controller, done))
        return True

    def deactivate_all(self) -> bool:
        """Stop every switchable controller and start none, as part of the soft stop.

        - Sent even while a switch is pending, since a stop must not wait; the
          controller manager handles requests in order, so this one wins.
        - Lists every switchable controller, as the last report may be stale.
          Best effort, so already-inactive ones do not fail the request.

        Returns:
            bool: True if the request went out; False if the service is not up
                (`state.switch_error` says so).
        """
        if not self._switch_client.service_is_ready():
            self.state.switch_error = f"{self.namespace}: switch_controller is not available, cannot stop"
            self._node.get_logger().error(self.state.switch_error)
            return False
        request = SwitchController.Request()
        request.deactivate_controllers = list(self.switchable)
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.start_asap = True
        self.state.switch_error = None
        self._switch_client.call_async(request).add_done_callback(self._on_deactivate_answer)
        return True

    # --- --- --- --- --- ANSWERS (main thread) --- --- --- --- ---

    def _on_deactivate_answer(self, future) -> None:
        """Log a failed `deactivate_all`, then read back what really runs."""
        try:
            response = future.result()
        except Exception as error:
            response, reason = None, f"switch_controller raised: {error}"
        else:
            reason = "controller manager rejected it"
        if response is None or not response.ok:
            self.state.switch_error = f"{self.namespace}: STOP FAILED, controllers not deactivated: {reason}"
            self._node.get_logger().error(self.state.switch_error)
        self.refresh()

    def _on_list_answer(self, future) -> None:
        """Record which controllers are running, from a list_controllers answer."""
        try:
            response = future.result()
        except Exception as error:
            self._node.get_logger().warning(f"{self.namespace}: list_controllers failed: {error}")
            return
        if response is None:
            return
        self.state.controllers = {c.name: c.state for c in response.controller}
        running = [name for name in self.switchable if self.state.controllers.get(name) == "active"]
        self.state.active = running[0] if running else None
        self.state.last_update_time = self._node.get_clock().now().nanoseconds * 1e-9

    def _on_switch_answer(self, controller: str, future) -> None:
        """Record the outcome of a switch request, then re-read the state."""
        self.state.switch_in_flight = None
        try:
            response = future.result()
        except Exception as error:
            response, reason = None, f"switch_controller raised: {error}"
        else:
            reason = "controller manager rejected the switch"
        if response is not None and response.ok:
            self.state.active = controller
        else:
            self.state.switch_error = f"{self.namespace}: switch to {controller!r} failed: {reason}"
            self._node.get_logger().error(self.state.switch_error)
        # Either way, re-read what is really running.
        self.refresh()
