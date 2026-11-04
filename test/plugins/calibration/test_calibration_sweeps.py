"""The calibration sweeps: each file entry loads, passes its checks and moves little between takes; bad ones named."""

from dataclasses import replace
from itertools import permutations

import numpy as np
import pytest
import yaml

from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.arm_planner.planner import path_hit
from husky_assembly_teleop.plugins.calibration.robot.sweeps import (SWEEPS_FILE, TURN, UNWOUND,
                                                                    ArmSweeps, Sweep, joint_limits, load_sweeps,
                                                                    load_validation_poses, nearest_take,
                                                                    nearest_turns, short_moves, shortest_order,
                                                                    sweep_problems)
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.mirrors import MirrorPool
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

pytestmark = pytest.mark.safety

ARMS = [(digits, arm) for digits, arms in yaml.safe_load((DATA / SWEEPS_FILE).read_text()).items() for arm in arms]
#: Farthest any joint may turn between two takes, or from the stow pose to the first, radians. The takes in their
#: recorded order and directions needed up to 12.3 (a 700° swing of J0); `short_moves` brings every arm under 3.6.
LONGEST_MOVE = 3.7


def moves_between(sweeps, begin) -> np.ndarray:
    """How far each joint turns to reach each take's first waypoint, (takes, 6) radians.

    Row 0 is the move from `begin`, each later row the move from the take before's last waypoint.
    """
    moves, here = [], np.asarray(begin, dtype=float)
    for sweep in sweeps:
        first, last = sweep.ends()
        moves.append(np.abs(first - here))
        here = last
    return np.array(moves).reshape(-1, 6)


def arm_limits_and_stow(digits: str, arm: str) -> tuple[np.ndarray, np.ndarray, tuple[float, ...]]:
    """Return the arm's lower and upper joint limits and its stow joints."""
    robot = robot_config_from_serial(digits, DATA)
    stow = next(config.stow_joints for config in robot.arms if config.name == arm)
    return *joint_limits(robot.model.urdf, arm), stow


def same_take(a: Sweep, b: Sweep) -> bool:
    """Whether two takes sweep one joint over one range from one held pose, up to whole turns and direction.

    The file rounds shifted angles to 0.001 rad.
    """
    def wrapped(angles):
        return (np.asarray(angles) + np.pi) % TURN - np.pi
    return (a.joint == b.joint and a.steps == b.steps and abs(abs(a.end - a.start) - abs(b.end - b.start)) < 2e-3
            and np.allclose(wrapped(np.subtract(a.joints, b.joints)), 0.0, atol=2e-3)
            and np.allclose(wrapped(np.subtract(sorted((a.start, a.end)), sorted((b.start, b.end)))), 0.0, atol=2e-3))


@pytest.mark.parametrize("digits, arm", ARMS)
def test_every_arm_in_the_file_has_usable_sweeps(digits, arm):
    """Each robot's sweeps load, name an arm its URDF has, and pass every check."""
    robot = robot_config_from_serial(digits, DATA)
    assert arm in {config.name for config in robot.arms}
    sweeps = load_sweeps(DATA, digits, arm)
    assert sweep_problems(sweeps, robot.model.urdf, arm) == []
    assert [sweep.joint for sweep in sweeps.sweeps] == sorted(sweep.joint for sweep in sweeps.sweeps)


@pytest.mark.parametrize("digits, arm", ARMS)
def test_the_file_keeps_moves_between_takes_short(digits, arm):
    """From the stow pose on, no joint turns far between takes, and the file is near the shortest J0-first order.

    Within half a radian: the plugin runs the shortest order anyway, the file's order only numbers the takes.
    """
    lower, upper, stow = arm_limits_and_stow(digits, arm)
    sweeps = load_sweeps(DATA, digits, arm)
    moves = moves_between(sweeps.sweeps, stow)
    assert moves.max() < LONGEST_MOVE
    best = moves_between(short_moves(sweeps, lower, upper, stow).sweeps, stow)
    assert moves.max(axis=1).sum() <= best.max(axis=1).sum() + 0.5


@pytest.mark.parametrize("digits, arm", ARMS)
def test_short_moves_keeps_every_take_and_shortens_the_moves(digits, arm):
    """Takes reversed and run backwards, held joints a whole turn off: `short_moves` keeps them all and moves little."""
    lower, upper, stow = arm_limits_and_stow(digits, arm)
    robot = robot_config_from_serial(digits, DATA)
    sweeps = load_sweeps(DATA, digits, arm)
    far = [replace(sweep, start=sweep.end, end=sweep.start, joints=tuple(
        value + TURN if column not in (0, sweep.joint) and value < stow[column] and value + TURN <= upper[column]
        else value
        for column, value in enumerate(sweep.joints))) for sweep in reversed(sweeps.sweeps)]
    scrambled = ArmSweeps(tuple(sorted(far, key=lambda sweep: sweep.joint)))
    better = short_moves(scrambled, lower, upper, stow)
    assert sweep_problems(better, robot.model.urdf, arm) == []
    assert len(better.sweeps) == len(sweeps.sweeps)
    assert all(sum(same_take(sweep, other) for other in better.sweeps) == 1 for sweep in sweeps.sweeps)
    assert moves_between(better.sweeps, stow).max() < LONGEST_MOVE
    assert moves_between(scrambled.sweeps, stow).max() > LONGEST_MOVE


@pytest.mark.parametrize("digits, arm", ARMS)
def test_every_arm_has_validation_poses_within_its_limits(digits, arm):
    """Validate cycles through these at every base spot: enough of them, six joints each, within the URDF's limits."""
    poses = load_validation_poses(DATA, digits, arm)
    lower, upper, _ = arm_limits_and_stow(digits, arm)
    assert len(poses) >= 5
    for pose in poses:
        assert pose.shape == (6,) and np.all(pose >= lower) and np.all(pose <= upper)


def test_a_validation_pose_is_reached_without_turning_a_joint_round():
    """Each joint goes to its whole-turn equivalent nearest where it is, within its limits; the elbow is not shifted."""
    lower, upper = np.radians([-360.0, -360.0, -180.0, -360.0, -360.0, -360.0]), -np.radians([-360.0] * 6)
    upper[2] = np.radians(180.0)
    goal = np.radians([170.0, -90.0, 170.0, 0.0, 300.0, 10.0])
    here = np.radians([-175.0, -90.0, -175.0, 0.0, -50.0, 10.0])
    turned = np.degrees(nearest_turns(goal, here, lower, upper, np.zeros(6)))
    assert turned == pytest.approx([-190.0, -90.0, 170.0, 0.0, -60.0, 10.0])
    # * A joint already wound past UNWOUND from stow goes back rather than further round.
    wound = np.degrees(nearest_turns(np.radians([-60.0] + [0.0] * 5), np.radians([300.0] + [0.0] * 5), lower, upper,
                                     np.zeros(6)))
    assert wound[0] == pytest.approx(-60.0)


@pytest.mark.parametrize("digits, arm", ARMS)
def test_validation_poses_keep_the_joints_unwound(digits, arm):
    """From the last take on, each validation pose keeps every joint that can wind within UNWOUND of the stow pose.

    A joint can wind when its range is over a whole turn: not the elbow.
    """
    lower, upper, stow = arm_limits_and_stow(digits, arm)
    winds = upper - lower > TURN + 1e-6
    here = load_sweeps(DATA, digits, arm).sweeps[-1].ends()[1]
    for pose in load_validation_poses(DATA, digits, arm):
        here = nearest_turns(pose, here, lower, upper, stow)
        assert np.abs(here - np.asarray(stow))[winds].max() <= UNWOUND + 1e-9


def _cost(move: np.ndarray) -> float:
    """A move's cost as `shortest_order` weighs it: the longest joint travel, ties broken by the total."""
    return float(np.abs(move).max() + 0.01 * np.abs(move).sum())


@pytest.mark.parametrize("digits, arm", ARMS)
def test_a_take_out_of_order_starts_nearest_the_arm(digits, arm):
    """From any take's end, every other take, turned and directed by `nearest_take`, costs no more than as filed."""
    lower, upper, stow = arm_limits_and_stow(digits, arm)
    sweeps = load_sweeps(DATA, digits, arm).sweeps
    for before in sweeps:
        here = before.ends()[1]
        for sweep in sweeps:
            nearest = nearest_take(sweep, here, stow, lower, upper)
            assert nearest.joint == sweep.joint and nearest.steps == sweep.steps
            assert _cost(nearest.ends()[0] - here) <= _cost(sweep.ends()[0] - here) + 1e-9


def test_alice_s_long_swing_is_gone():
    """Take 2's end to take 4's start swung J0 433° with the file's angles; turned, it is a short move."""
    lower, upper, stow = arm_limits_and_stow("0804", "ur_arm")
    sweeps = load_sweeps(DATA, "0804", "ur_arm").sweeps
    here = sweeps[1].ends()[1]
    assert np.degrees(abs(sweeps[3].ends()[0][0] - here[0])) > 400
    assert np.degrees(abs(nearest_take(sweeps[3], here, stow, lower, upper).ends()[0][0] - here[0])) < 90


def test_the_shortest_order_is_the_cheapest_of_all_orders():
    """For a few pending takes of both joints, from a take's end: no order costs less than the one found."""
    lower, upper, stow = arm_limits_and_stow("0805", "ur_arm")
    sweeps = load_sweeps(DATA, "0805", "ur_arm").sweeps
    pending, at = [1, 3, 4, 7, 9, 10], sweeps[5].ends()[1]

    def total(order):
        here, cost = at, 0.0
        for index in order:
            sweep = nearest_take(sweeps[index], here, stow, lower, upper)
            cost, here = cost + _cost(sweep.ends()[0] - here), sweep.ends()[1]
        return cost

    found = [index for index, _ in shortest_order(sweeps, pending, at, stow, lower, upper)]
    assert sorted(found) == pending
    groups = [[i for i in pending if sweeps[i].joint == joint] for joint in (0, 1)]
    every = [list(a) + list(b) for first, second in (groups, groups[::-1])
             for a in permutations(first) for b in permutations(second)]
    assert total(found) == pytest.approx(min(total(order) for order in every))


def test_waypoints_turn_only_the_swept_joint():
    """A take's waypoints hold every other joint and step the swept one evenly from start to end."""
    sweep = load_sweeps(DATA, "0805", "ur_arm").sweeps[0]
    waypoints = sweep.waypoints()
    assert waypoints.shape == (sweep.steps, 6)
    held = np.delete(waypoints, sweep.joint, axis=1)
    assert np.ptp(held, axis=0).max() == 0.0
    assert waypoints[[0, -1], sweep.joint] == pytest.approx([sweep.start, sweep.end])
    assert np.diff(waypoints[:, sweep.joint]) == pytest.approx(np.full(sweep.steps - 1, (sweep.end - sweep.start)
                                                                       / (sweep.steps - 1)))


def test_bad_sweeps_are_named():
    """Too few takes, short takes, J0 off zero in a J1 take, a repeated take and a limit overrun are all reported."""
    robot = robot_config_from_serial("0805", DATA)
    good = load_sweeps(DATA, "0805", "ur_arm")
    j0 = [sweep for sweep in good.sweeps if sweep.joint == 0]
    j1 = [sweep for sweep in good.sweeps if sweep.joint == 1]
    bad = ArmSweeps((replace(j0[0], steps=4), replace(j0[0], end=7.0), j0[1],
                     replace(j1[0], joints=(0.3,) + j1[0].joints[1:])))
    problems = "\n".join(sweep_problems(bad, robot.model.urdf, "ur_arm"))
    assert "J1: 1 take(s)" in problems
    assert "take 1 (J0): 4 waypoints" in problems
    assert "take 2 (J0): holds the same joints as an earlier take" in problems
    assert "shoulder_pan_joint goes" in problems and "outside its limits" in problems
    assert "J1 takes need J0 at 0" in problems


def test_an_arm_without_sweeps_is_refused():
    """Asking for an arm the file does not list says which."""
    with pytest.raises(KeyError, match="0805 left_ur_arm"):
        load_sweeps(DATA, "0805", "left_ur_arm")


@pytest.mark.slow
def test_validation_poses_are_clear_of_the_robot_itself():
    """Every arm's validation poses miss the robot, its tools and its other arm at stow; the planner would skip them."""
    world = MirrorPool()
    try:
        for digits, arm in ARMS:
            config = robot_config_from_serial(digits, DATA)
            joints = {name: value for config_arm in config.arms
                      for name, value in zip(arm_joint_names(config_arm.name), config_arm.stow_joints)}
            robot = RobotObject(robot_id(digits), config.model, Pose(), joints)
            scene = Scene(bodies={}, world_poses={}, robots={robot.id: robot})
            mirror = world.sync(scene, digits)
            for pose in load_validation_poses(DATA, digits, arm):
                assert path_hit(mirror, scene, arm, np.array([pose])) is None, (digits, arm, pose)
    finally:
        world.close()
