"""
Check the calibration's takes and validation poses (data/calibration/sweeps.yaml) against the robot; make new ones.

    python3 scripts/calibration_poses.py check 0805 ur_arm         each take and validation pose: clear, or what it hits
    python3 scripts/calibration_poses.py fix 0805 ur_arm           the smallest change that clears each colliding take
    python3 scripts/calibration_poses.py validation 0805 ur_arm    new validation poses, as YAML for the file

- Checks see the robot alone, its other arms at stow with no tool, and the calibration marker on the arm grown by
  `--margin` on every side: the real holder is a spider of markers that the model's plate does not cover.
- Fixes: a J1 take is first trimmed at its colliding end (at most TRIM), then any take may change one held joint, or
  two, by the fewest degrees (at most SHIFT); J0 of a J1 take stays 0. A fix must stay clear with each held joint
  nudged by NUDGE, and a J1 take with its range NUDGE longer.
- Validation poses: spread evenly over the arm's reach (farthest-point selection of tool positions), the tool's z
  pointing outwards from the robot and up (the markers face the cameras), clear when every joint is nudged by
  NUDGE, each held joint within `UNWOUND` of stow, and plannable by the arm planner from stow and from the pose
  before, in the order the plugin visits them.
"""

from __future__ import annotations

import argparse
import tempfile
import threading
import xml.etree.ElementTree as ElementTree
from dataclasses import replace
from itertools import combinations, product
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from husky_assembly_teleop import tool_urdfs
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.arm_planner.planner import path_hit, plan_arm
from husky_assembly_teleop.plugins.calibration.analysis.fit import arm_base_link
from husky_assembly_teleop.plugins.calibration.robot.sweeps import (SWEEPS_FILE, Sweep, joint_limits, load_sweeps,
                                                                    load_validation_poses, nearest_turns)
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.mirrors import MirrorPool
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

DATA = Path(__file__).resolve().parents[1] / "data"
#: Grown on every side of the marker's collision shapes, metres.
MARGIN = 0.025
#: Joint nudges a validation pose must stay clear under: how far, radians, and how many random ones.
NUDGE, NUDGES = np.radians(4.0), 8
#: Where the tool's z should point: outwards from the robot's centre, raised by ELEVATION; accepted within SPREAD.
ELEVATION, SPREAD = np.radians(45.0), np.radians(35.0)
#: The tool0 positions kept: at least LIFT above the arm base, REACH metres from its axis.
LIFT, REACH = 0.10, (0.30, 0.75)
#: Largest trim of a J1 take's range, and largest change of a held joint, radians; their search step.
TRIM, SHIFT, STEP = np.radians(25.0), np.radians(20.0), np.radians(1.0)
#: Random joint samples drawn.
SAMPLES = 20000


def grown_marker(margin: float, folder: Path) -> Path:
    """Write the calibration marker's URDF with every collision shape grown by `margin`; return the file."""
    tree = ElementTree.parse(DATA / tool_urdfs.TOOL_URDF_DIRECTORY / tool_urdfs.TOOL_URDFS["calibration_marker"])
    for collision in tree.getroot().iter("collision"):
        box, cylinder = collision.find("geometry/box"), collision.find("geometry/cylinder")
        if box is not None:
            box.set("size", " ".join(f"{float(v) + 2 * margin:.4f}" for v in box.get("size").split()))
        if cylinder is not None:
            cylinder.set("radius", f"{float(cylinder.get('radius')) + margin:.4f}")
            cylinder.set("length", f"{float(cylinder.get('length')) + 2 * margin:.4f}")
    path = folder / "calibration_marker_grown.urdf"
    tree.write(path)
    return path


class Robot:
    """One robot alone, the marker on the measured arm, its other arms bare at stow; checks and plans that arm.

    Checks see the marker grown by `margin`; plans the marker as it is, as the arm planner does live (stow itself is
    within the margin).
    """

    def __init__(self, digits: str, arm: str, margin: float, folder: Path):
        """Load the robot twice: with the marker as it is and grown by `margin` (written into `folder`)."""
        base = robot_config_from_serial(digits, DATA)
        tools = tuple("calibration_marker" if config.name == arm else None for config in base.arms)
        self.plain = robot_config_from_serial(digits, DATA, tools=tools)
        marker = tool_urdfs.TOOL_URDFS["calibration_marker"]
        # * The grown marker by absolute path: joined to the tool folder, an absolute path stays as it is.
        tool_urdfs.TOOL_URDFS["calibration_marker"] = str(grown_marker(margin, folder))
        try:
            self.config = robot_config_from_serial(digits, DATA, tools=tools)
        finally:
            tool_urdfs.TOOL_URDFS["calibration_marker"] = marker
        self.digits, self.arm, self.names = digits, arm, arm_joint_names(arm)
        self.stow = np.array(next(config.stow_joints for config in self.config.arms if config.name == arm))
        self.lower, self.upper = joint_limits(self.config.model.urdf, arm)
        self._rest = {f"{config.name}_{name}": value for config in self.config.arms
                      for name, value in zip(("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"),
                                             config.stow_joints)}
        self.world, self.plain_world = MirrorPool(), MirrorPool()

    def _mirror(self, world: MirrorPool, config, joints) -> tuple:
        joints = {**self._rest, **dict(zip(self.names, joints))}
        robot = RobotObject(robot_id(self.digits), config.model, Pose(), joints)
        scene = Scene(bodies={}, world_poses={}, robots={robot.id: robot})
        return world.sync(scene, self.digits), scene

    def hit(self, path) -> str:
        """What a joint path (its waypoints and the moves between) hits, with the grown marker; "" if it is clear."""
        path = np.atleast_2d(path)
        found = path_hit(*self._mirror(self.world, self.config, path[0]), self.arm, path)
        return found.message if found is not None else ""

    def plans(self, start, goal) -> str:
        """Why the arm planner finds no move from `start` to `goal`, with the marker as it is; "" if it does."""
        result = plan_arm(*self._mirror(self.plain_world, self.plain, start), self.arm, goal, threading.Event())
        return "" if result.path is not None else result.reason

    def close(self) -> None:
        """Disconnect both worlds."""
        self.world.close()
        self.plain_world.close()

    def tool0(self, joints) -> Pose:
        """tool0 in base_footprint."""
        return link_pose(self.config.model.urdf, Pose(), dict(zip(self.names, joints)), f"{self.arm}_tool0")

    def arm_base(self) -> np.ndarray:
        """The arm base's position in base_footprint."""
        return np.array(link_pose(self.config.model.urdf, Pose(), {}, arm_base_link(self.arm)).position)


def check(robot: Robot) -> None:
    """Print each take and validation pose of the arm: clear, or what it hits."""
    for index, sweep in enumerate(load_sweeps(DATA, robot.digits, robot.arm).sweeps):
        print(f"take {index + 1} J{sweep.joint}: {robot.hit(sweep.waypoints()) or 'clear'}")
    for index, pose in enumerate(load_validation_poses(DATA, robot.digits, robot.arm)):
        print(f"validation {index + 1}: {robot.hit(pose) or 'clear'}")


def clear(robot: Robot, sweep: Sweep) -> bool:
    """Whether a take is clear, also with each held joint nudged by NUDGE and a J1 take's range NUDGE longer."""
    if robot.hit(sweep.waypoints()):
        return False
    nudged = [replace(sweep, joints=tuple(value + sign * NUDGE if c == column else value
                                          for c, value in enumerate(sweep.joints)))
              for column in range(1, 6) if column != sweep.joint for sign in (-1.0, 1.0)]
    if sweep.joint == 1:
        direction = np.sign(sweep.end - sweep.start)
        nudged.append(replace(sweep, start=sweep.start - direction * NUDGE, end=sweep.end + direction * NUDGE))
    return not any(robot.hit(take.waypoints()) for take in nudged)


def fixed(robot: Robot, sweep: Sweep) -> tuple[Sweep, str] | None:
    """The take changed least so that it is clear, and what changed; None if no change within TRIM and SHIFT does."""
    if sweep.joint == 1:
        for total in np.arange(STEP, TRIM + STEP / 2, STEP):
            for at_start in np.arange(0.0, total + STEP / 2, STEP):
                direction = np.sign(sweep.end - sweep.start)
                start, end = sweep.start + direction * at_start, sweep.end - direction * (total - at_start)
                trimmed = replace(sweep, start=round(float(start), 3), end=round(float(end), 3))
                if clear(robot, trimmed):
                    ends = [f"{abs(np.degrees(trim)):.0f}° at the {end}" for trim, end in
                            ((at_start, "start"), (total - at_start, "end")) if abs(trim) > STEP / 2]
                    return trimmed, "range trimmed by " + " and ".join(ends)
    held = [column for column in range(1, 6) if column != sweep.joint]
    for degrees in np.arange(STEP, SHIFT + STEP / 2, STEP):
        for count in (1, 2):
            for columns in combinations(held, count):
                for signs in product((-1.0, 1.0), repeat=count):
                    joints = list(sweep.joints)
                    for column, sign in zip(columns, signs):
                        joints[column] = round(joints[column] + sign * degrees, 3)
                    changed = replace(sweep, joints=tuple(joints))
                    if clear(robot, changed):
                        what = ", ".join(f"J{column} {sign * np.degrees(degrees):+.0f}°"
                                         for column, sign in zip(columns, signs))
                        return changed, what
    return None


def fix(robot: Robot) -> None:
    """Print each colliding take changed least so that it is clear, as a line for the sweeps file."""
    for index, sweep in enumerate(load_sweeps(DATA, robot.digits, robot.arm).sweeps):
        hit = robot.hit(sweep.waypoints())
        if not hit:
            continue
        print(f"take {index + 1} J{sweep.joint}: {hit}")
        found = fixed(robot, sweep)
        if found is None:
            print("  no small change clears it")
            continue
        take, what = found
        print(f"      - {{joint: {take.joint}, joints: [{', '.join(f'{v:g}' for v in take.joints)}], "
              f"from: {take.start:g}, to: {take.end:g}, steps: {take.steps}}}  # {what} for clearance")


def facing(robot: Robot, joints) -> tuple[np.ndarray, float] | None:
    """tool0 relative to the arm base, and its z's angle from outwards-and-up (radians); None outside LIFT and REACH."""
    pose = robot.tool0(joints)
    position, base = np.array(pose.position), robot.arm_base()
    relative = position - base
    if relative[2] < LIFT or not REACH[0] <= np.linalg.norm(relative[:2]) <= REACH[1]:
        return None
    outwards = position[:2] / np.linalg.norm(position[:2])
    wanted = np.array([*(outwards * np.cos(ELEVATION)), np.sin(ELEVATION)])
    z = Rotation.from_quat(pose.orientation).as_matrix()[:, 2]
    return relative, float(np.arccos(np.clip(z @ wanted, -1.0, 1.0)))


def validation(robot: Robot, count: int, seed: int) -> list[np.ndarray]:
    """`count` validation poses spread evenly over the clear, outward-facing tool positions, in J0 order, plannable.

    Farthest-point selection: each next pose is the clear candidate farthest from those chosen, whatever the
    candidates' density, so the sparse space behind the arm counts as much as the dense space in front of it.
    """
    rng = np.random.default_rng(seed)
    # * Upper arm above horizontal (J1 in -π..0), elbow short of folding flat.
    low = np.array([-np.pi, -np.pi, -2.9, -np.pi, -np.pi, -np.pi])
    high = np.array([np.pi, 0.0, 2.9, np.pi, np.pi, np.pi])
    candidates, positions = [], []
    for joints in rng.uniform(low, high, (SAMPLES, 6)):
        found = facing(robot, joints)
        if found is None or found[1] > SPREAD:
            continue
        joints = nearest_turns(joints, robot.stow, robot.lower, robot.upper, robot.stow)
        if robot.hit(joints) or any(robot.hit(q) for q in joints + rng.uniform(-NUDGE, NUDGE, (NUDGES, 6))):
            continue
        candidates.append(joints)
        positions.append(found[0])
    print(f"{len(candidates)} clear candidates facing outwards and up, of {SAMPLES} samples")
    positions = np.array(positions)
    # * Start from the highest: the markers face most cameras there.
    left, chosen = list(range(len(candidates))), []
    nearest = np.full(len(candidates), np.inf)
    pick = int(np.argmax(positions[:, 2]))
    while len(chosen) < count and left:
        left.remove(pick)
        if robot.plans(robot.stow, candidates[pick]):
            nearest[pick] = -np.inf
        else:
            chosen.append(pick)
            nearest = np.minimum(nearest, np.linalg.norm(positions - positions[pick], axis=1))
        nearest[chosen] = -np.inf
        if left:
            pick = max(left, key=lambda index: nearest[index])
    chosen.sort(key=lambda index: candidates[index][0])
    poses = [candidates[index] for index in chosen]
    for before, after in zip([robot.stow, *poses], poses):
        if problem := robot.plans(before, after):
            print(f"no move from {np.round(before, 2).tolist()} to {np.round(after, 2).tolist()}: {problem}")
    return poses


def main() -> None:
    """Check, fix or generate for the robot arm given on the command line."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("what", choices=("check", "fix", "validation"))
    parser.add_argument("digits", help="the robot's four serial digits, e.g. 0805")
    parser.add_argument("arm", help="e.g. ur_arm")
    parser.add_argument("--margin", type=float, default=MARGIN, help="grown on the marker's every side, metres")
    parser.add_argument("--count", type=int, default=8, help="validation poses to generate")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as folder:
        robot = Robot(args.digits, args.arm, args.margin, Path(folder))
        if args.what in ("check", "fix"):
            (check if args.what == "check" else fix)(robot)
            robot.close()
            return
        poses = validation(robot, args.count, args.seed)
        print(f"    validation:  # generated by scripts/calibration_poses.py (margin {args.margin * 1e3:g} mm, "
              f"seed {args.seed})")
        for joints in poses:
            print(f"      - [{', '.join(f'{v:.3f}' for v in joints)}]")
        print(f"(sweeps file: {SWEEPS_FILE})")
        robot.close()


if __name__ == "__main__":
    main()
