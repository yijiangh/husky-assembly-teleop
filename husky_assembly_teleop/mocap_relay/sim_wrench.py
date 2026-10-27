"""
A fake force/torque reading for every simulated UR arm: sim_wrench, since the UR driver's fake hardware reads NaN.

It publishes a WrenchStamped on each arm's `ft_sensor_wrench` (what the real driver's reading is remapped to), which
the robot's rate limiter and the cartesian controllers read: the monitor's F/T plots, the retreat's contact check and
the servo's load guard then have a reading in a simulation. Arms are found by their `joint_states` topic, every
SIM_DISCOVERY_SEC.

- Near zero by default: a small noise, plus `bias` (six values, N and Nm) on every arm.
- ! The compliance controller reads the same topic and moves the arm by it: keep `bias` and `contact_n` small
  (a 10 N bias at 500 N/m stiffness shifts the arm 2 cm).
- `contact_n`: a force along the tool's z while any motor of the arm's simulated scaffolding tool reports STALLED
  (the screw bit), so a stalled insert reads a contact.
- ! A simulation only: refuses the robots' ROS domain.
"""

from __future__ import annotations

import re
from typing import Dict, List, Optional

import numpy as np
import rclpy
from crl_husky.simulated_parts import fleet_domain_problem
from crl_husky_msgs.msg import ScaffoldingToolStatus
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .node import SIM_DISCOVERY_SEC

#: A simulated arm's joint_states topic: the robot's namespace and the arm's ("ur5e", "left_ur5e", "right_ur5e").
ARM_TOPIC = re.compile(r"^/(a200_\d{4})/((?:left_|right_)?ur5e)/joint_states$")
#: The scaffolding tool simulator's state while a screw bit.
STALLED = "STALLED"


def gripper_namespace(robot: str, arm: str) -> str:
    """The tool namespace of an arm: "left_gripper" beside "left_ur5e", "gripper" beside "ur5e"."""
    side = arm[:-len("ur5e")]
    return f"/{robot}/{side}gripper"


class SimWrench(Node):
    """Publishes a wrench per simulated arm."""

    def __init__(self) -> None:
        """Read the bias and the contact force, then look for arms.

        Raises:
            RuntimeError: In the robots' ROS domain.
        """
        super().__init__("sim_wrench")
        problem = fleet_domain_problem()
        if problem:
            raise RuntimeError(f"sim_wrench refused: {problem}")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("noise_n", 0.05)
        self.declare_parameter("bias", [0.0] * 6)
        self.declare_parameter("contact_n", 0.0)
        self.declare_parameter("noise_seed", 0)
        self._rng = np.random.default_rng(int(self.get_parameter("noise_seed").value))
        self._noise = float(self.get_parameter("noise_n").value)
        self._bias = np.array([float(value) for value in self.get_parameter("bias").value], dtype=float)
        if self._bias.shape != (6,):
            raise ValueError(f"bias must be six values (N, N, N, Nm, Nm, Nm), not {self._bias.tolist()}")
        self._contact = float(self.get_parameter("contact_n").value)
        #: Per arm topic: its publisher and whether its tool reports a stalled screw.
        self._arms: Dict[str, dict] = {}
        self.create_timer(SIM_DISCOVERY_SEC, self._look_for_arms)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._publish)

    def _look_for_arms(self) -> None:
        """Take on every simulated arm whose joint_states appeared since the last look."""
        for topic, _types in self.get_topic_names_and_types():
            match = ARM_TOPIC.match(topic)
            if match is None or topic in self._arms:
                continue
            robot, arm = match[1], match[2]
            entry = {"publisher": self.create_publisher(WrenchStamped, f"/{robot}/{arm}/ft_sensor_wrench", 10),
                     "stalled": False, "frame": f"{arm}_tool0"}
            self._arms[topic] = entry
            self.create_subscription(ScaffoldingToolStatus, f"{gripper_namespace(robot, arm)}/tool_status",
                                     lambda message, e=entry: self._on_tool(e, message), 10)
            self.get_logger().info(f"{robot}/{arm}: faking its force/torque reading, bias {self._bias.tolist()}, "
                                   f"noise {self._noise:g} N, contact {self._contact:g} N while a screw is stalled")

    @staticmethod
    def _on_tool(entry: dict, message: ScaffoldingToolStatus) -> None:
        entry["stalled"] = STALLED in (message.state_m1, message.state_m2)

    def _publish(self) -> None:
        """One reading per arm: the bias, the noise, and the contact while its screw is stalled."""
        stamp = self.get_clock().now().to_msg()
        for entry in self._arms.values():
            values = self._bias + self._rng.normal(0.0, self._noise, 6) * np.array([1, 1, 1, 0.01, 0.01, 0.01])
            if entry["stalled"]:
                values[2] -= self._contact  # the bar pushes back along the tool's z
            message = WrenchStamped()
            message.header.stamp, message.header.frame_id = stamp, entry["frame"]
            (message.wrench.force.x, message.wrench.force.y, message.wrench.force.z,
             message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z) = (float(v) for v in values)
            entry["publisher"].publish(message)


def main(args: Optional[List[str]] = None) -> None:
    """Run until Ctrl-C; exit 1 in the robots' domain or on a bad configuration."""
    rclpy.init(args=args)
    try:
        node = SimWrench()
    except (RuntimeError, ValueError) as exc:
        rclpy.logging.get_logger("sim_wrench").error(str(exc))
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
