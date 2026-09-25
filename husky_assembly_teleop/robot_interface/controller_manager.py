"""
One ros2_control controller manager, and which of its controllers is running.

Every part of a husky that moves has its own controller manager: the base at
/<robot>/controller_manager, each arm at /<robot>/<arm>/controller_manager. They
all speak the same two services, so they are all handled by this one class --
the base and the arms differ only in which controllers they may switch between.

! Which controller runs is runtime state, not configuration. It can be switched
  from here, from a terminal or by an on-board script, so it is read back from
  the controller manager rather than assumed from what was last requested.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node

#: Seconds between list_controllers polls. Controllers can be switched from
#: outside the monitor, so the answer is re-read rather than trusted forever; a
#: second is quick enough for an operator and costs one small service call.
REFRESH_PERIOD = 1.0


@dataclass
class ControllerManagerState:
    """What one controller manager last told us.

    Attributes:
        controllers: Every controller it reported, name to lifecycle state
            ("active", "inactive", ...). Empty until the first answer arrives.
        active: The running controller out of the switchable ones, or "" when
            none is running or no answer has arrived yet.
        switch_in_flight: Controller a switch request is waiting on, or None.
        switch_error: Why the last switch failed, or None. Cleared when a new
            switch goes out, so a waiter never reads an earlier failure as its own.
        last_update_time: ROS time of the last answer, seconds.
    """

    controllers: dict[str, str] = field(default_factory=dict)
    active: str = ""
    switch_in_flight: str | None = None
    switch_error: str | None = None
    last_update_time: float = 0.0


class ControllerManagerInterface:
    """Tracks and switches the controllers of one controller manager.

    ! Nothing here blocks. Requests go out with call_async and their answers
      land in `state` from the executor thread, a few ticks later. Callers poll
      `state`, typically with concurrency.wait_until.
    """

    def __init__(self, node: Node, namespace: str, switchable: tuple[str, ...],
                 state: ControllerManagerState):
        """Create the service clients and start polling the controller list.

        Args:
            node: The monitor node, used to create the clients.
            namespace: Namespace the controller manager lives in, e.g.
                "/a200_0806" for the base or "/a200_0806/left_ur5e" for an arm.
            switchable: Controllers this part may run, only one at a time. The
                `active` field in the state is always one of these, or "".
            state: Where answers are written. Owned by the caller's state
                dataclass, so it shows up in RobotState with everything else.
        """
        self._node = node
        self.namespace = namespace
        self.switchable = switchable
        self.state = state
        self._list_client = node.create_client(
            ListControllers, f"{namespace}/controller_manager/list_controllers")
        self._switch_client = node.create_client(
            SwitchController, f"{namespace}/controller_manager/switch_controller")

        # * Polled, not asked once. At construction the service is usually not
        #   discovered yet, and later someone may switch controllers behind our
        #   back. Until the first answer, `active` is "" and any command that
        #   needs a particular controller refuses.
        self._refresh_timer = node.create_timer(REFRESH_PERIOD, self.refresh)
        # Whether the missing service was already reported, so the poll above
        # says so once rather than every second.
        self._reported_unavailable = False

    def is_active(self, controller: str) -> bool:
        """Whether `controller` is the one running right now, as last reported."""
        return self.state.active == controller

    def refresh(self) -> bool:
        """Ask the controller manager which controllers are running.

        Returns:
            bool: True if the request went out. False if the service is not up,
                in which case `state` keeps what it had.
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

        Asynchronous: on success `state.active` becomes `controller`, on failure
        `state.switch_error` says why. Switching to the controller that is
        already running does nothing and counts as success.

        Args:
            controller: One of `switchable`.

        Returns:
            bool: True if the request went out (or nothing needed doing). False
                if `controller` is not switchable here or the service is not up;
                `state.switch_error` then says which.
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

        # ? Stop every other switchable controller that was reported active,
        #   rather than just `state.active`. The old code deactivated the one it
        #   believed was running, and when that belief was "" the controller
        #   manager rejected the switch because the real one still held the joints.
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

    # --- --- --- --- --- ANSWERS (executor thread) --- --- --- --- ---

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
        self.state.active = running[0] if running else ""
        self.state.last_update_time = self._node.get_clock().now().nanoseconds * 1e-9

    def _on_switch_answer(self, controller: str, future) -> None:
        """Record the outcome of a switch request, then read back the truth."""
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
        # Either way, ask what is really running now, so `controllers` is current
        # and a failed switch does not leave a stale belief behind.
        self.refresh()
