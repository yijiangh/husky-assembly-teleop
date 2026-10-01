"""
The tools mounted on an arm's flange, one class per kind.

`make_end_effector` picks the class from ArmConfig.end_effector. All tools share
open / close / stop; tool-specific actions (e.g. the screw) live on their class only.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from control_msgs.action import GripperCommand
from crl_husky_msgs.msg import ScaffoldingToolCmd, ScaffoldingToolStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

from ..config import ArmConfig
from .connections import RosConnections


# --- --- --- --- --- STATE --- --- --- --- ---

@dataclass
class RobotiqState:
    """What we know about a Robotiq gripper, from its joint states and the GripperCommand action.

    Attributes:
        position: Measured knuckle angle, radians, or None before the first
            joint state.
        last_update_time: ROS time of the last joint state, seconds.
        commanded_position: Last position sent, or None before the first command.
        commanded_effort: Force limit sent with it, or None.
        moving: Whether a command is still running.
        last_result_ok: Whether the last finished command succeeded, or None.
        reactivating: Whether a reactivation request is waiting for its answer.
        reactivate_error: Why the last reactivation failed, or "" if it did not.
    """

    position: float | None = None
    last_update_time: float | None = None
    commanded_position: float | None = None
    commanded_effort: float | None = None
    moving: bool = False
    last_result_ok: bool | None = None
    reactivating: bool = False
    reactivate_error: str = ""


@dataclass
class ScaffoldingV3State:
    """Last status the scaffolding tool driver published; fields are None until the first one.

    Attributes:
        gripper_motor: Driver's state string for the gripper motor (M1).
        joint_motor: Driver's state string for the joint screw motor (M2).
        current: Motor current, in the driver's own units.
        pwm_pct: Motor PWM duty cycle, percent.
        last_update_time: ROS time of the last status, seconds.
    """

    gripper_motor: str | None = None
    joint_motor: str | None = None
    current: int | None = None
    pwm_pct: int | None = None
    last_update_time: float | None = None


@dataclass
class ScaffoldingV1State:
    """The two UR tool outputs that drive a v1 scaffolding tool.

    ? The tool reports nothing itself; these are the outputs as the UR reports
      them, so they also show a screw switched off by the safety sync.

    Attributes:
        gripper_closed: Whether the gripper output is on, or None before the
            first io_states.
        screw_on: Whether the screw output is on, or None.
        last_update_time: ROS time of the last io_states, seconds.
        last_request_ok: Whether the last set_io request succeeded, or None.
    """

    gripper_closed: bool | None = None
    screw_on: bool | None = None
    last_update_time: float | None = None
    last_request_ok: bool | None = None


EndEffectorState = RobotiqState | ScaffoldingV3State | ScaffoldingV1State


# --- --- --- --- --- INTERFACES --- --- --- --- ---
# ! main thread only, like every command in robot_interface.py.

class EndEffector(ABC):
    """What every mounted tool can do.

    Attributes:
        state: The tool's state; the same object is in ArmState.
    """

    state: EndEffectorState

    def __init__(self, node: Node, robot_namespace: str, arm: ArmConfig):
        """Remember where the tool's driver lives; subclasses create the I/O.

        Args:
            node: The monitor node.
            robot_namespace: The robot's namespace, e.g. "a200_0806".
            arm: The arm the tool is mounted on.
        """
        self._node = node
        self._arm = arm
        #: The arm's own driver, for tools switched through the arm.
        self.arm_namespace = f"/{robot_namespace}/{arm.ros_namespace}"
        self.namespace = f"/{robot_namespace}/{arm.end_effector_namespace}"
        #: Subclasses create their ROS entities through this, in `_connect`.
        self._ros = RosConnections(node)

    def _connect(self) -> None:
        """Create this tool's topics, services and actions. None by default."""

    def reconnect(self) -> None:
        """Destroy this tool's ROS entities and create them again. Keeps `state`."""
        self._ros.destroy_all()
        self._connect()

    @abstractmethod
    def open(self) -> None:
        """Open the gripper, releasing whatever it holds."""

    @abstractmethod
    def close(self) -> None:
        """Close the gripper on whatever is between its jaws."""

    @abstractmethod
    def stop(self) -> None:
        """Stop every motor of the tool."""


class RobotiqGripper(EndEffector):
    """Robotiq 2F-85, moved through the driver's GripperCommand action.

    The driver publishes joint_states and offers reactivate_gripper, both under
    the tool's namespace.
    """

    #: Knuckle angle in radians: 0.0 is fully open, 0.8 fully closed.
    OPEN_POSITION = 0.0
    CLOSED_POSITION = 0.8
    #: Force limit as a fraction of full force. Low on purpose: a bar is light.
    DEFAULT_EFFORT = 0.1
    MAX_EFFORT = 1.0
    #: The joint the driver moves and reports; the other finger joints follow it.
    KNUCKLE_JOINT = "robotiq_85_left_knuckle_joint"

    def __init__(self, node: Node, robot_namespace: str, arm: ArmConfig):
        """Create the action client, joint state subscription and reactivate client, without waiting for them."""
        super().__init__(node, robot_namespace, arm)
        self.state = RobotiqState()
        self._connect()

    def _connect(self) -> None:
        """Create the action client, the joint state subscription and the reactivate client."""
        self._action = self._ros.action_client(
            GripperCommand, f"{self.namespace}/robotiq_gripper_controller/gripper_cmd")
        # ? Best effort: only the newest angle matters, so lost samples are not resent over wifi.
        self._ros.subscription(JointState, f"{self.namespace}/joint_states", self._on_joint_state,
                               qos_profile_sensor_data)
        self._reactivate = self._ros.client(
            Trigger, f"{self.namespace}/robotiq_activation_controller/reactivate_gripper")

    def reconnect(self) -> None:
        """Reconnect the action client, subscription and reactivate client.

        ! Clears `moving` and `reactivating`: a request sent on an old client never reports back.
        """
        super().reconnect()
        self.state.moving = False
        self.state.reactivating = False

    def open(self) -> None:
        """Open fully."""
        self.move(self.OPEN_POSITION)

    def close(self) -> None:
        """Close fully, or until the effort limit stops it."""
        self.move(self.CLOSED_POSITION)

    def stop(self) -> None:
        """Hold the fingers where they are, by commanding the measured position.

        ? GripperCommand has no stop. Before the first joint state, the last target is re-sent.
        """
        position = self.state.position if self.state.position is not None else self.state.commanded_position
        if position is not None:
            self.move(position, self.state.commanded_effort or self.DEFAULT_EFFORT)

    def server_is_ready(self) -> bool:
        """Whether the gripper's action server is discovered and reachable."""
        return self._action.server_is_ready()

    def move(self, position: float, effort: float = DEFAULT_EFFORT) -> None:
        """Move to `position`.

        Args:
            position: Knuckle angle in radians, OPEN_POSITION to CLOSED_POSITION.
                Clipped to that range.
            effort: Force limit passed to the driver, 0.0 to MAX_EFFORT. Clipped.
        """
        if not self.server_is_ready():
            self._node.get_logger().warning(f"{self.namespace}: gripper action server is not available")
            return
        position = min(max(float(position), self.OPEN_POSITION), self.CLOSED_POSITION)
        effort = min(max(float(effort), 0.0), self.MAX_EFFORT)
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        self.state.commanded_position = position
        self.state.commanded_effort = effort
        self.state.moving = True
        self._action.send_goal_async(goal).add_done_callback(self._on_goal_answer)

    def reactivate(self) -> None:
        """Ask the activation controller to reactivate the gripper.

        * Needed after a fault or power loss. The gripper opens and closes once,
          so keep its fingers clear.
        """
        if not self._reactivate.service_is_ready():
            self._node.get_logger().warning(f"{self.namespace}: reactivate_gripper service is not available")
            return
        self.state.reactivating = True
        self.state.reactivate_error = ""
        self._reactivate.call_async(Trigger.Request()).add_done_callback(self._on_reactivated)

    def _on_reactivated(self, future) -> None:
        """Record whether the reactivation worked."""
        self.state.reactivating = False
        try:
            response = future.result()
        except Exception as error:
            response, raised = None, f"reactivate_gripper raised: {error}"
        else:
            raised = ""
        if response is None or not response.success:
            answer = "no answer" if response is None else (response.message or "failed")
            self.state.reactivate_error = raised or answer
            self._node.get_logger().warning(f"{self.namespace}: reactivation failed: {self.state.reactivate_error}")

    def _on_joint_state(self, message: JointState) -> None:
        """Copy the knuckle angle into state."""
        if self.KNUCKLE_JOINT not in message.name:
            return
        self.state.position = float(message.position[message.name.index(self.KNUCKLE_JOINT)])
        self.state.last_update_time = self._node.get_clock().now().nanoseconds * 1e-9

    def _on_goal_answer(self, future) -> None:
        """Wait for the result of an accepted goal; record a rejected or failed one."""
        # ! Guarded: a raise here would leave `moving` True for good.
        try:
            handle = future.result()
        except Exception as error:
            handle = None
            self._node.get_logger().warning(f"{self.namespace}: gripper goal failed: {error}")
        if handle is None or not handle.accepted:
            self.state.moving = False
            self.state.last_result_ok = False
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        """Record that the command finished and whether it got there."""
        self.state.moving = False
        try:
            result = future.result().result
        except Exception as error:
            self.state.last_result_ok = False
            self._node.get_logger().warning(f"{self.namespace}: gripper result failed: {error}")
            return
        self.state.last_result_ok = bool(result.reached_goal or result.stalled)


class ScaffoldingV3(EndEffector):
    """Scaffolding tool v3: two motors behind an RS485 driver on the robot.

    ! The driver's M1 and M2 are not the BarAction movement roles M0..M4; use GRIPPER_MOTOR and JOINT_MOTOR.
    """

    GRIPPER_MOTOR = 1  # M1: opens and closes the gripper that holds the bar
    JOINT_MOTOR = 2    # M2: drives the screw that tightens the bar to the joint
    #: Motor states the driver reports (crl_husky onboard/protocol.md).
    #: A stalled motor will not run until `stop()` clears it.
    IDLE, TIGHTENING, LOOSENING, STALLED = "IDLE", "TIGHTENING", "LOOSENING", "STALLED"

    def __init__(self, node: Node, robot_namespace: str, arm: ArmConfig):
        """Create the command publisher and the status subscription."""
        super().__init__(node, robot_namespace, arm)
        self.state = ScaffoldingV3State()
        self._connect()

    def _connect(self) -> None:
        """Create the command publisher and the status subscription."""
        self._command = self._ros.publisher(ScaffoldingToolCmd, f"{self.namespace}/tool_cmd")
        self._ros.subscription(ScaffoldingToolStatus, f"{self.namespace}/tool_status", self._on_status)

    def open(self) -> None:
        """Run the gripper motor in the loosening direction."""
        self._drive(self.GRIPPER_MOTOR, -1)

    def close(self) -> None:
        """Run the gripper motor in the tightening direction."""
        self._drive(self.GRIPPER_MOTOR, 1)

    def stop(self) -> None:
        """Stop both motors and clear a stall."""
        self._drive(self.GRIPPER_MOTOR, 0)

    def drive_screw(self, direction: int) -> None:
        """Run the joint screw motor.

        Args:
            direction: 1 tightens, -1 loosens, 0 stops.
        """
        self._drive(self.JOINT_MOTOR, direction)

    def _drive(self, motor: int, direction: int) -> None:
        """Publish one command to the driver."""
        message = ScaffoldingToolCmd()
        message.motor = motor
        message.direction = int(direction)
        self._command.publish(message)

    def _on_status(self, message: ScaffoldingToolStatus) -> None:
        """Copy the driver's status into state."""
        self.state.gripper_motor = message.state_m1
        self.state.joint_motor = message.state_m2
        self.state.current = message.current
        self.state.pwm_pct = message.pwm_pct
        self.state.last_update_time = self._node.get_clock().now().nanoseconds * 1e-9


class ScaffoldingV1(EndEffector):
    """Scaffolding tool v1, switched through the UR's two tool digital outputs.

    Uses the arm's own set_io service and io_states, in the arm's namespace.
    Output 0 is the screw (on runs it one way); output 1 is the gripper.

    ! multi_arm_safety_sync switches output 0 off whenever it stops the arms.

    ? "Gripper on = closed" is unverified: check on the robot and flip GRIPPER_CLOSED_WHEN_ON if wrong.
    """

    SCREW_PIN = SetIO.Request.PIN_TOOL_DOUT0
    GRIPPER_PIN = SetIO.Request.PIN_TOOL_DOUT1
    GRIPPER_CLOSED_WHEN_ON = True

    def __init__(self, node: Node, robot_namespace: str, arm: ArmConfig):
        """Create the set_io client and the io_states subscription."""
        super().__init__(node, robot_namespace, arm)
        self.state = ScaffoldingV1State()
        self._connect()

    def _connect(self) -> None:
        """Create the set_io client and the io_states subscription."""
        self._set_io = self._ros.client(SetIO, f"{self.arm_namespace}/io_and_status_controller/set_io")
        # Via the arm's rate limiter, like ArmInterface.
        self._ros.subscription(IOStates, f"{self.arm_namespace}/rate_limiter/io_and_status_controller/io_states",
                               self._on_io_states, qos_profile_sensor_data)

    def service_is_ready(self) -> bool:
        """Whether the arm's set_io service is discovered and reachable."""
        return self._set_io.service_is_ready()

    def open(self) -> None:
        """Switch the gripper output to open."""
        self._set(self.GRIPPER_PIN, not self.GRIPPER_CLOSED_WHEN_ON)

    def close(self) -> None:
        """Switch the gripper output to closed."""
        self._set(self.GRIPPER_PIN, self.GRIPPER_CLOSED_WHEN_ON)

    def stop(self) -> None:
        """Switch the screw off.

        ! The gripper is left alone: it has only open and closed, and switching it would drop or grab the bar.
        """
        self._set(self.SCREW_PIN, False)

    def drive_screw(self, direction: int) -> None:
        """Run or stop the screw motor.

        Args:
            direction: 1 runs it, 0 stops it. -1 is refused: the motor runs one way only.
        """
        if direction < 0:
            self._node.get_logger().warning(f"{self.arm_namespace}: scaffolding_v1 cannot loosen; nothing sent")
            return
        self._set(self.SCREW_PIN, direction > 0)

    def _set(self, pin: int, on: bool) -> None:
        """Ask the arm to switch one tool output."""
        if not self.service_is_ready():
            self._node.get_logger().warning(f"{self.arm_namespace}: set_io service is not available")
            return
        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = pin
        request.state = float(SetIO.Request.STATE_ON if on else SetIO.Request.STATE_OFF)
        self._set_io.call_async(request).add_done_callback(self._on_set)

    def _on_set(self, future) -> None:
        """Record whether the arm switched the output."""
        try:
            response = future.result()
        except Exception as error:
            response = None
            self._node.get_logger().warning(f"{self.arm_namespace}: set_io raised: {error}")
        self.state.last_request_ok = response is not None and bool(response.success)
        if not self.state.last_request_ok:
            self._node.get_logger().warning(f"{self.arm_namespace}: set_io request failed")

    def _on_io_states(self, message: IOStates) -> None:
        """Copy the two tool outputs into state."""
        for pin in message.digital_out_states:
            if pin.pin == self.SCREW_PIN:
                self.state.screw_on = bool(pin.state)
            elif pin.pin == self.GRIPPER_PIN:
                self.state.gripper_closed = bool(pin.state) == self.GRIPPER_CLOSED_WHEN_ON
        self.state.last_update_time = self._node.get_clock().now().nanoseconds * 1e-9


# --- --- --- --- --- CONFIG -> CLASS --- --- --- --- ---

_CLASS_BY_KIND: dict[str, type[EndEffector]] = {
    "robotiq": RobotiqGripper,
    "scaffolding_v1": ScaffoldingV1,
    "scaffolding_v3": ScaffoldingV3,
}


def make_end_effector(node: Node, robot_namespace: str, arm: ArmConfig) -> EndEffector | None:
    """Build the interface for whatever `arm` has mounted.

    Args:
        node: The monitor node.
        robot_namespace: The robot's namespace, e.g. "a200_0806".
        arm: The arm, whose `end_effector` picks the class.

    Returns:
        EndEffector | None: The tool's interface, or None for a bare arm.
    """
    if arm.end_effector is None:
        return None
    return _CLASS_BY_KIND[arm.end_effector](node, robot_namespace, arm)
