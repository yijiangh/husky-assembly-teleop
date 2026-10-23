"""
A fake Motive between the simulators and the relay: every simulated body's true pose, with Motive's imperfections.

It knows no robots or arms: it reads each true pose a simulator publishes on TRUTH_PREFIX (crl-husky's husky_sim: the
base_footprint, and tool0 of an arm carrying the calibration marker) and publishes it on SIM_MOTIVE_PREFIX, where the
relay takes it in place of the truth; launch/sim_mocap.launch.py starts it.

- Imperfections, all off by default: a delay (Motive's latency), noise, dropouts, reported marker errors, slow drifts,
  and offsets: where Motive's body frame sits from the true pose. A robot's base comes out relative to crl-husky's
  default calibration, which the relay takes off, so its offset is how far the true calibration is from that default.
- Per-body values are `body_values` text: values for every body, or `<id>:<values>` per body (space separated).
- Announces "mocap" on the simulated_parts of each robot whose base it fakes, so a monitor finds these parameters.
- ! A simulation only: refuses the robots' ROS domain.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from crl_husky.sim.husky_sim import TRUTH_PREFIX
from crl_husky.simulated_parts import announce, fleet_domain_problem
from crl_husky_msgs.msg import MocapRigidBodyPose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert

from ..robot_interface.simulated_parts import body_values
from ..world.calibration_sessions import robot_mocap_ids
from .node import SIM_DISCOVERY_SEC, SIM_MOTIVE_PREFIX, SIM_TOPIC


def offset_pose(values: Sequence[float]) -> Pose:
    """A pose from (x, y, z) metres and (roll, pitch, yaw) degrees."""
    return Pose.from_arrays(values[:3], Rotation.from_euler("xyz", values[3:6], degrees=True).as_quat())


def noisy(pose: Pose, rng: np.random.Generator, position_std: float, angle_std: float) -> Pose:
    """One noisy sample of `pose`: Gaussian position noise (metres) and a small random turn in the pose's frame.

    Args:
        pose: The true pose.
        rng: Where the noise comes from.
        position_std: Standard deviation per axis, metres.
        angle_std: Standard deviation of the turn's rotation vector per axis, degrees.
    """
    if position_std == 0.0 and angle_std == 0.0:
        return pose
    turn = Rotation.from_rotvec(rng.normal(0.0, angle_std, 3), degrees=True)
    return Pose.from_arrays(np.add(pose.position, rng.normal(0.0, position_std, 3)),
                            (Rotation.from_quat(pose.orientation) * turn).as_quat())


def drifted(pose: Pose, rate: Sequence[float], minutes: float) -> Pose:
    """`pose` after `minutes` of `rate` in its own frame: x, y, z metres and a rotation vector (degrees) per minute."""
    if not any(rate):
        return pose
    shift = Rotation.from_rotvec(np.multiply(rate[3:6], minutes), degrees=True)
    return compose(pose, Pose.from_arrays(np.multiply(rate[:3], minutes), shift.as_quat()))


def observed(truth: Pose, offset: Sequence[float], drift: Sequence[float], minutes: float, rng: np.random.Generator,
             noise: Sequence[float]) -> Pose:
    """What Motive reports for a body truly at `truth`: its frame off by `offset`, `minutes` of `drift`, then `noise`.

    Args:
        truth: The body's true pose.
        offset: The true pose in Motive's body frame, as a calibration gives it: x, y, z metres, roll, pitch, yaw
            degrees.
        drift: The body frame's creep, see `drifted`.
        minutes: How long it has crept.
        rng: Where the noise comes from.
        noise: Position std metres, angle std degrees.
    """
    body = compose(truth, invert(offset_pose(offset))) if any(offset) else truth
    return noisy(drifted(body, drift, minutes), rng, noise[0], noise[1])


def lost(rng: np.random.Generator, probability: float, burst_s: float, now: float,
         lost_until: float) -> Tuple[bool, float]:
    """Whether a body is lost this sample, as Motive loses one when its markers are hidden.

    Args:
        rng: Where the chance comes from.
        probability: Chance per sample that a loss starts.
        burst_s: How long a loss lasts, seconds; 0 loses that one sample.
        now: This sample's time, seconds.
        lost_until: When the current loss ends, seconds (from the previous call).

    Returns:
        Tuple[bool, float]: Whether the body is lost now, and when its loss ends (pass back next time).
    """
    if now < lost_until:
        return True, lost_until
    if probability > 0.0 and rng.random() < probability:
        return True, now + burst_s
    return False, lost_until


class SimMotive(Node):
    """Publishes every simulated body's true pose as Motive would, with its imperfections."""

    def __init__(self) -> None:
        """Read the imperfections, then look for true poses.

        Raises:
            RuntimeError: In the robots' ROS domain.
        """
        super().__init__("sim_motive")
        problem = fleet_domain_problem()
        if problem:
            raise RuntimeError(f"sim_motive refused: {problem}")
        self.declare_parameter("rate_hz", 120.0)
        # * Seconds from a pose being true to it reaching the relay: Motive's latency. Stamped with when it was true.
        self.declare_parameter("delay_sec", 0.0)
        # * Per body (`body_values`): noise per sample [position std m, angle std deg], the reported mean marker error
        # * (m), drift [x, y, z m/min, rotation vector deg/min] and offset [x, y, z m, roll, pitch, yaw deg].
        self.declare_parameter("noise", "")
        self.declare_parameter("marker_error", "0.0003")
        self.declare_parameter("drift", "")
        self.declare_parameter("offsets", "")
        self.declare_parameter("noise_seed", 0)
        # * Chance per sample that a body is lost (markers hidden), held for `dropout_burst_s` seconds.
        self.declare_parameter("dropout", 0.0)
        self.declare_parameter("dropout_burst_s", 0.0)
        self._delay = max(float(self.get_parameter("delay_sec").value), 0.0)
        self._texts = {name: str(self.get_parameter(name).value) for name in ("noise", "marker_error", "drift",
                                                                              "offsets")}
        self._rng = np.random.default_rng(int(self.get_parameter("noise_seed").value))
        self._dropout = float(self.get_parameter("dropout").value)
        self._dropout_burst_s = float(self.get_parameter("dropout_burst_s").value)
        self._start = self.get_clock().now()
        #: Robot namespace by base rigid body, to announce the robots whose base this fakes.
        self._robots = {mocap_id: f"a200_{serial[-4:]}" for mocap_id, serial in robot_mocap_ids().items()}
        #: Per rigid body: its imperfections, latest truth (pose, stamp), held pose, loss, queue and publisher.
        self._bodies: Dict[int, dict] = {}
        self.create_timer(SIM_DISCOVERY_SEC, self._look_for_truths)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._publish)

    def _look_for_truths(self) -> None:
        """Take on every simulated body whose truth appeared since the last look."""
        for topic, _types in self.get_topic_names_and_types():
            match = SIM_TOPIC.match(topic)
            if match is None or match[1] != TRUTH_PREFIX or int(match[2]) in self._bodies:
                continue
            body_id = int(match[2])
            self._bodies[body_id] = body = self._new_body(body_id)
            self.create_subscription(MocapRigidBodyPose, topic, lambda message, b=body: self._on_truth(b, message), 10)
            self.get_logger().info(f"rigid body {body_id}: faking it, {body['describe']}")
            if body_id in self._robots:
                announce(self, "mocap", "fake motive", topic=f"/{self._robots[body_id]}/simulated_parts",
                         simulator=self.get_fully_qualified_name())

    def _new_body(self, body_id: int) -> dict:
        """A body's state, with its own imperfections."""
        values = {"noise": body_values(self._texts["noise"], body_id, 2),
                  "marker_error": body_values(self._texts["marker_error"], body_id, 1)[0],
                  "drift": body_values(self._texts["drift"], body_id, 6),
                  "offset": body_values(self._texts["offsets"], body_id, 6)}
        return {**values, "truth": None, "stamp": None, "held": None, "lost_until": -np.inf, "queue": deque(),
                "describe": ", ".join(f"{name} {value}" for name, value in values.items() if np.any(value)) or "exact",
                "publisher": self.create_publisher(MocapRigidBodyPose, f"{SIM_MOTIVE_PREFIX}/id_{body_id}/pose", 10)}

    def _on_truth(self, body: dict, message: MocapRigidBodyPose) -> None:
        p, q = message.pose.position, message.pose.orientation
        body["truth"], body["stamp"] = Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), message.header.stamp

    def _publish(self) -> None:
        """Observe every body whose truth is known, then send what has waited out the delay."""
        now = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        for body in self._bodies.values():
            if body["truth"] is not None:
                pose = observed(body["truth"], body["offset"], body["drift"], now / 60.0, self._rng, body["noise"])
                self._send(body, pose, now)
        due = time.monotonic() - self._delay
        for body in self._bodies.values():
            while body["queue"] and body["queue"][0][0] <= due:
                body["publisher"].publish(body["queue"].popleft()[1])

    def _send(self, body: dict, pose: Pose, now: float) -> None:
        """Queue a body's pose, or its last one marked lost while a dropout lasts, stamped with the truth's time."""
        gone, body["lost_until"] = lost(self._rng, self._dropout, self._dropout_burst_s, now, body["lost_until"])
        if gone and body["held"] is not None:
            pose = body["held"]
        body["held"] = pose
        message = MocapRigidBodyPose(pose_valid=not gone, tracking_valid=not gone, marker_error=body["marker_error"])
        message.header.stamp = body["stamp"]
        message.header.frame_id = "mocap_world"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position
        (message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z,
         message.pose.orientation.w) = pose.orientation
        body["queue"].append((time.monotonic(), message))


def main(args: Optional[List[str]] = None) -> None:
    """Run until Ctrl-C; exit 1 in the robots' domain or on a bad configuration."""
    rclpy.init(args=args)
    try:
        node = SimMotive()
    except (RuntimeError, ValueError) as exc:
        rclpy.logging.get_logger("sim_motive").error(str(exc))
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
