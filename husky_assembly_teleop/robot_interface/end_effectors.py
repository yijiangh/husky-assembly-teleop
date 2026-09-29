"""
The tools mounted on an arm's flange, one class per kind.

Which kind an arm carries is configuration (ArmConfig.end_effector) and does not
change while running. `make_end_effector` turns that setting into the matching
class, which creates only the topics, services and actions that tool has.

All of them share open / close / stop, so code that only wants to grip does not
need to know which tool is mounted. Anything tool-specific, such as the
scaffolding tool's screw, is a method on that class alone.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from control_msgs.action import GripperCommand
from crl_husky_msgs.msg import ScaffoldingToolCmd, ScaffoldingToolStatus
from rclpy.node import Node

from ..config import ArmConfig
from .connections import RosConnections


# --- --- --- --- --- STATE --- --- --- --- ---

@dataclass
class RobotiqState:
    """What we know about a Robotiq gripper.

    ? There is no feedback subscription: the gripper only reports through its
      action result. So this is what we asked for and whether it has finished.

    Attributes:
        commanded_position: Last position sent, or None before the first command.
        moving: Whether a command is still running.
        last_result_ok: Whether the last finished command succeeded, or None.
    """

    commanded_position: float | None = None
    moving: bool = False
    last_result_ok: bool | None = None


@dataclass
class ScaffoldingV3State:
    """Last status the scaffolding tool driver published.

    Every field is None until the first status arrives.

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
    """Last outputs we set on a v1 scaffolding tool. See ScaffoldingV1.

    Attributes:
        gripper_closed: Whether the gripper output was last switched on, or
            None before we set it. The tool reports nothing back.
        screw_on: Whether the screw output was last switched on, or None.
    """

    gripper_closed: bool | None = None
    screw_on: bool | None = None


EndEffectorState = RobotiqState | ScaffoldingV3State | ScaffoldingV1State


# --- --- --- --- --- INTERFACES --- --- --- --- ---
# ! ROS thread only, like every command in robot_interface.py.

class EndEffector(ABC):
    """What every mounted tool can do.

    Attributes:
        state: The tool's own state dataclass. The arm puts the same object into
            its ArmState, so it is readable from RobotState.
    """

    state: EndEffectorState

    def __init__(self, node: Node, robot_namespace: str, arm: ArmConfig):
        """Remember where the tool's driver lives. Subclasses create its I/O.

        Args:
            node: The monitor node.
            robot_namespace: The robot's namespace, e.g. "a200_0806".
            arm: The arm the tool is mounted on.
        """
        self._node = node
        self._arm = arm
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
    """Robotiq 2F-85, driven through its GripperCommand action."""

    #: Knuckle angle in radians: 0.0 is fully open, 0.8 fully closed.
    OPEN_POSITION = 0.0
    CLOSED_POSITION = 0.8
    #: Force limit the old UI always sent. Low on purpose: a bar is light.
    DEFAULT_EFFORT = 0.1

    def __init__(self, node: Node, robot_namespace: str, arm: ArmConfig):
        """Create the action client. Does not wait for the server."""
        super().__init__(node, robot_namespace, arm)
        self.state = RobotiqState()
        self._connect()

    def _connect(self) -> None:
        """Create the action client."""
        self._action = self._ros.action_client(
            GripperCommand, f"{self.namespace}/robotiq_gripper_controller/gripper_cmd")

    def reconnect(self) -> None:
        """Reconnect the action client.

        ! A goal sent on the old client never reports back, so `moving` is
          cleared here rather than left True forever.
        """
        super().reconnect()
        self.state.moving = False

    def open(self) -> None:
        """Open fully."""
        self.move(self.OPEN_POSITION)

    def close(self) -> None:
        """Close fully, or until the effort limit stops it."""
        self.move(self.CLOSED_POSITION)

    def stop(self) -> None:
        """Hold where it is, by commanding the position it was last sent to.

        ? GripperCommand has no stop. Cancelling the goal leaves the gripper
          wherever the driver decides, so re-sending the last target is the
          predictable choice.
        """
        if self.state.commanded_position is not None:
            self.move(self.state.commanded_position)

    def server_is_ready(self) -> bool:
        """Whether the gripper's action server is discovered and reachable."""
        return self._action.server_is_ready()

    def move(self, position: float, effort: float = DEFAULT_EFFORT) -> None:
        """Move to `position`.

        Args:
            position: Knuckle angle in radians, OPEN_POSITION to CLOSED_POSITION.
            effort: Force limit passed to the driver.
        """
        if not self.server_is_ready():
            self._node.get_logger().warning(f"{self.namespace}: gripper action server is not available")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
        self.state.commanded_position = float(position)
        self.state.moving = True
        self._action.send_goal_async(goal).add_done_callback(self._on_goal_answer)

    def _on_goal_answer(self, future) -> None:
        """Wait for the result of an accepted goal; record a rejected one."""
        handle = future.result()
        if not handle.accepted:
            self.state.moving = False
            self.state.last_result_ok = False
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        """Record that the command finished and whether it got there."""
        self.state.moving = False
        result = future.result().result
        self.state.last_result_ok = bool(result.reached_goal or result.stalled)


class ScaffoldingV3(EndEffector):
    """Scaffolding tool v3: two motors behind an RS485 driver on the robot.

    ! The driver and its message call the motors M1 and M2, which is easy to
      misread as the movement roles M0..M4 of a BarAction. On the Python side
      they are named by what they do: GRIPPER_MOTOR and JOINT_MOTOR.
    """

    GRIPPER_MOTOR = 1  # M1: opens and closes the gripper that holds the bar
    JOINT_MOTOR = 2    # M2: drives the screw that tightens the bar to the joint

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
        """Stop both motors. The driver ignores the motor field when stopping."""
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
    """Scaffolding tool v1, switched through the UR's tool digital outputs.

    TODO placeholder. Not ported yet; every command raises.

    How the old code drove it (old/husky_robot.py, set_screw, and
    old/test_setio.py):
      - Service: SetIO (ur_msgs/srv/SetIO) at
        /<robot>/<arm>/io_and_status_controller/set_io -- note this is the *arm's*
        namespace (ArmConfig.ros_namespace), not end_effector_namespace.
      - Request: fun = FUN_SET_DIGITAL_OUT, state = STATE_ON / STATE_OFF.
      - Screw: pin PIN_TOOL_DOUT0.
      - Gripper: ? pin not confirmed; PIN_TOOL_DOUT1 is the likely one. Check on
        the robot before porting.
      - Feedback: none from the tool. The UR io_states topic, which ArmInterface
        already reads into ArmState.digital_in, shows the inputs only; the
        outputs we set are tracked in ScaffoldingV1State.

    ! The old SetIO code is not a safe starting point: set_screw was defined
      twice and the second copy called itself forever. Port from the protocol
      above, not from that code.
    """

    def __init__(self, node: Node, robot_namespace: str, arm: ArmConfig):
        """Record the state only; no I/O is created until this is ported."""
        super().__init__(node, robot_namespace, arm)
        self.state = ScaffoldingV1State()
        node.get_logger().warning(
            f"arm {arm.name}: scaffolding_v1 is configured but not implemented; "
            f"its commands will raise")

    def open(self) -> None:
        """Not implemented yet. See the class docstring."""
        raise NotImplementedError("scaffolding_v1 is not ported yet")

    def close(self) -> None:
        """Not implemented yet. See the class docstring."""
        raise NotImplementedError("scaffolding_v1 is not ported yet")

    def stop(self) -> None:
        """Not implemented yet. See the class docstring."""
        raise NotImplementedError("scaffolding_v1 is not ported yet")

    def drive_screw(self, direction: int) -> None:
        """Not implemented yet. See the class docstring."""
        raise NotImplementedError("scaffolding_v1 is not ported yet")


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
