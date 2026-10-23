"""
Motive for a simulation: simulated robots' base rigid bodies and arm markers, as raw rigid bodies for the relay.

The relay reads them with `source:=sim` (SIM_PREFIX topics, Z-up world); launch/sim_mocap.launch.py starts both.

- Base: the simulator's true base_footprint (husky_sim with `mocap_relay:=true` puts it on TRUTH_PREFIX) turned into
  the rigid body by a true calibration: crl-husky's default, moved by `calibration_error`, so that calibrating in the
  simulation has something to find.
- Markers: an arm's tool0 by forward kinematics from its joint states, at `marker_offset`: the calibration marker on
  the flange.
- Imperfections, all off by default: noise, dropouts, reported marker errors, arm joint offsets and slow drifts
  (`observed`, `lost`, `offset_joints`), so calibrating meets what the real mocap does.
- ! A simulation only: refuses the robots' ROS domain.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import rclpy
from crl_husky.simulated_parts import fleet_domain_problem
from crl_husky_msgs.msg import MocapRigidBodyPose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from ..config import robot_config_from_serial
from ..world.calibration_sessions import default_calibration
from .node import SIM_PREFIX

#: Where husky_sim puts a simulated base's true base_footprint when a relay runs: <prefix>/id_<n>/pose.
TRUTH_PREFIX = "/mocap_sim/truth"
#: The calibration marker's Motive rigid body, a marker's default.
DEFAULT_TOOL_BODY = 1866
#: The repository's data folder: robot models and calibration sweeps.
DATA = Path(__file__).resolve().parents[2] / "data"


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


def observed(pose: Pose, drift: Sequence[float], minutes: float, rng: np.random.Generator,
             noise: Sequence[float]) -> Pose:
    """What Motive reports for a body truly at `pose`: `minutes` of `drift`, then `noise` (metres, degrees)."""
    return noisy(drifted(pose, drift, minutes), rng, noise[0], noise[1])


def lost(rng: np.random.Generator, probability: float, burst_s: float, now: float,
         lost_until: float) -> Tuple[bool, float]:
    """Whether a body is lost in this sample: a loss starts with `probability` and lasts `burst_s` seconds.

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


def offset_joints(joints: Mapping[str, float], arm: str, offsets: Sequence[float]) -> Dict[str, float]:
    """The joints the arm truly has when its encoders are off by `offsets` (degrees, in UR_JOINT_NAMES order)."""
    extra = {f"{arm}_{name}": float(np.radians(offset)) for name, offset in zip(UR_JOINT_NAMES, offsets)}
    return {name: value + extra.get(name, 0.0) for name, value in joints.items()}


def rigid_body(footprint: Pose, true_calibration: Pose) -> Pose:
    """The base rigid body Motive would see: base_footprint with the true calibration taken off."""
    return compose(footprint, invert(true_calibration))


def marker_pose(urdf: Path, footprint: Pose, joints: Mapping[str, float], arm: str, offset: Pose) -> Pose:
    """The marker body on an arm's flange: tool0 by forward kinematics, then `offset` in tool0."""
    return compose(link_pose(urdf, footprint, joints, f"{arm}_tool0"), offset)


class SimMotive(Node):
    """Publishes simulated robots' base rigid bodies and arm markers as raw rigid bodies."""

    def __init__(self) -> None:
        """Read the robots and markers, and subscribe to their truths and joint states.

        Raises:
            RuntimeError: In the robots' ROS domain, or for a robot without a mocap calibration.
        """
        super().__init__("sim_motive")
        problem = fleet_domain_problem()
        if problem:
            raise RuntimeError(f"sim_motive refused: {problem}")
        # * Comma separated: robots' serials, and markers "<serial>:<arm>[:<rigid body>]" (the rigid body defaults to
        # * the arm's tool body in the calibration sweeps).
        self.declare_parameter("robots", "0805")
        self.declare_parameter("markers", "0805:ur_arm")
        self.declare_parameter("calibration_error", [0.0] * 6)
        self.declare_parameter("marker_offset", [0.01, -0.005, 0.025, 0.0, 0.0, 15.0])
        self.declare_parameter("marker_error", 0.0003)
        self.declare_parameter("rate_hz", 120.0)
        # * Imperfections, all zero by default. Noise per sample and body: [position std m, angle std deg]; the base
        # * tracker and the tool marker differ in reality.
        self.declare_parameter("base_noise", [0.0, 0.0])
        self.declare_parameter("marker_noise", [0.0, 0.0])
        self.declare_parameter("noise_seed", 0)
        # * The base body's reported mean marker error, metres: a badly defined base rigid body in Motive.
        self.declare_parameter("base_marker_error", 0.0003)
        # * Chance per sample that a body is lost (markers hidden), held for `dropout_burst_s` seconds.
        self.declare_parameter("dropout", 0.0)
        self.declare_parameter("dropout_burst_s", 0.0)
        # * The marker arm's encoder zero offsets, degrees: an arm whose kinematics differ from the URDF. Only the
        # * marker's forward kinematics see them; the arm's reported joints stay as they are.
        self.declare_parameter("joint_offsets", [0.0] * 6)
        # * Slow drifts in the body's own frame, [x, y, z m/min, rotation vector deg/min]: the marker sliding on the
        # * flange (a loose tool body), and the base tracker creeping on the robot.
        self.declare_parameter("marker_drift", [0.0] * 6)
        self.declare_parameter("base_drift", [0.0] * 6)
        error = offset_pose(self._floats("calibration_error", 6))
        self._offset = offset_pose(self._floats("marker_offset", 6))
        self._marker_error = float(self.get_parameter("marker_error").value)
        self._base_marker_error = float(self.get_parameter("base_marker_error").value)
        self._base_noise, self._marker_noise = self._floats("base_noise", 2), self._floats("marker_noise", 2)
        self._rng = np.random.default_rng(int(self.get_parameter("noise_seed").value))
        self._dropout = float(self.get_parameter("dropout").value)
        self._dropout_burst_s = float(self.get_parameter("dropout_burst_s").value)
        self._joint_offsets = self._floats("joint_offsets", 6)
        self._marker_drift, self._base_drift = self._floats("marker_drift", 6), self._floats("base_drift", 6)
        self._start = self.get_clock().now()
        #: Per base rigid body: its true calibration, the latest truth (pose, stamp), and its publisher.
        self._bases: Dict[int, dict] = {}
        #: Per marker rigid body: its robot's base id, arm, URDF, joint values and publisher.
        self._markers: Dict[int, dict] = {}
        configs = {}
        for serial in (s.strip() for s in str(self.get_parameter("robots").value).split(",") if s.strip()):
            config = configs[serial[-4:]] = robot_config_from_serial(serial, DATA)
            default = default_calibration(config.mocap_id) if config.mocap_id is not None else None
            if default is None:
                raise RuntimeError(f"robot {serial}: no default mocap calibration in crl-husky")
            truth = compose(Pose(default.position, default.orientation), error)
            self._bases[config.mocap_id] = {"truth_calibration": truth, "footprint": None, "stamp": None,
                                            "held": None, "lost_until": -np.inf, "publisher": self.create_publisher(
                                                MocapRigidBodyPose, f"{SIM_PREFIX}/id_{config.mocap_id}/pose", 10)}
            self.create_subscription(MocapRigidBodyPose, f"{TRUTH_PREFIX}/id_{config.mocap_id}/pose",
                                     lambda message, i=config.mocap_id: self._on_truth(i, message), 10)
            self.get_logger().info(f"robot {config.serial}: base rigid body {config.mocap_id}, true calibration "
                                   f"{np.round(truth.position, 4).tolist()}")
        for entry in (e.strip() for e in str(self.get_parameter("markers").value).split(",") if e.strip()):
            serial, arm, *rest = entry.split(":")
            config = configs.get(serial[-4:])
            if config is None or arm not in {a.name for a in config.arms}:
                raise RuntimeError(f"marker {entry!r}: no such simulated robot or arm (robots: {list(configs)})")
            mocap_id = int(rest[0]) if rest else DEFAULT_TOOL_BODY
            namespace = next(a.ros_namespace for a in config.arms if a.name == arm)
            self._markers[mocap_id] = {"base": config.mocap_id, "arm": arm, "urdf": config.model.urdf, "joints": {},
                                       "held": None, "lost_until": -np.inf, "publisher": self.create_publisher(
                                           MocapRigidBodyPose, f"{SIM_PREFIX}/id_{mocap_id}/pose", 10)}
            self.create_subscription(JointState, f"/{config.ros_namespace}/{namespace}/joint_states",
                                     lambda message, i=mocap_id: self._on_joints(i, message), 10)
            self.get_logger().info(f"marker {mocap_id} on {config.serial} {arm}")
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._publish)

    def _floats(self, name: str, count: int) -> List[float]:
        values = [float(v) for v in self.get_parameter(name).value]
        if len(values) != count:
            raise RuntimeError(f"parameter {name}: {count} values expected, got {values}")
        return values

    def _on_truth(self, mocap_id: int, message: MocapRigidBodyPose) -> None:
        p, q = message.pose.position, message.pose.orientation
        base = self._bases[mocap_id]
        base["footprint"] = Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        base["stamp"] = message.header.stamp

    def _on_joints(self, mocap_id: int, message: JointState) -> None:
        marker = self._markers[mocap_id]
        # * The driver names joints without the arm's prefix; the URDF with it.
        names = set(UR_JOINT_NAMES)
        marker["joints"] = {f"{marker['arm']}_{name}": value for name, value in zip(message.name, message.position)
                            if name in names}

    def _publish(self) -> None:
        """Publish every rigid body whose truth is known, stamped with the truth's time."""
        now = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        for base in self._bases.values():
            if base["footprint"] is not None:
                pose = observed(rigid_body(base["footprint"], base["truth_calibration"]), self._base_drift, now / 60.0,
                                self._rng, self._base_noise)
                self._send(base, pose, now, base["stamp"], self._base_marker_error)
        for marker in self._markers.values():
            base = self._bases[marker["base"]]
            if base["footprint"] is not None and marker["joints"]:
                joints = offset_joints(marker["joints"], marker["arm"], self._joint_offsets)
                pose = observed(marker_pose(marker["urdf"], base["footprint"], joints, marker["arm"], self._offset),
                                self._marker_drift, now / 60.0, self._rng, self._marker_noise)
                self._send(marker, pose, now, base["stamp"], self._marker_error)

    def _send(self, body: dict, pose: Pose, now: float, stamp, marker_error: float) -> None:
        """Publish a body's pose, or its last one marked lost while a dropout lasts."""
        gone, body["lost_until"] = lost(self._rng, self._dropout, self._dropout_burst_s, now, body["lost_until"])
        if gone and body["held"] is not None:
            pose = body["held"]
        body["held"] = pose
        message = MocapRigidBodyPose(pose_valid=not gone, tracking_valid=not gone, marker_error=marker_error)
        message.header.stamp = stamp
        message.header.frame_id = "mocap_world"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position
        (message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z,
         message.pose.orientation.w) = pose.orientation
        body["publisher"].publish(message)


def main(args: Optional[List[str]] = None) -> None:
    """Run until Ctrl-C; exit 1 in the robots' domain or on a bad configuration."""
    rclpy.init(args=args)
    try:
        node = SimMotive()
    except RuntimeError as exc:
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
