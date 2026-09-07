"""Loader and reference-path math for precomputed open-loop dual-arm trajectories.

* Input: one "assembly-open-loop-json-v2" file (planner output for the
* fixtureless assembly paper): uniformly sampled 12-joint q/qd samples for two
* arms plus gripper open/close annotations and a per-sample servo flag.
* Output: an OpenLoopTraj that the execution engine and the previewer share --
* columns already reordered to physical left/right, gripper events reduced to
* rising edges, and one cubic Hermite spline per arm for time-continuous
* (q_ref, qd_ref) lookups.

! This module deliberately imports only json/numpy/scipy so it can be smoke
! tested against the real trajectory file with plain venv python -- no ROS,
! no colcon build, no UI.
"""

import json
import os
from bisect import bisect_right
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicHermiteSpline

# The one schema this loader understands (written by the assembly planner).
SCHEMA = 'assembly-open-loop-json-v2'
ARM_SIDES = ('left', 'right')  # index 0 = left, 1 = right (repo convention)


@dataclass
class GripperEvent:
    """One gripper command extracted from the trajectory annotations.

    Args:
        time (float): Seconds from trajectory start when the command fires.
        arm_index (int): Physical arm, 0 = left, 1 = right (after any swap).
        kind (str): 'close' or 'open'.
        robot (str): The robot's name in the source file ('a1'/'a2'), kept
            only so logs can be compared against the planner output.
    """
    time: float
    arm_index: int
    kind: str
    robot: str


@dataclass
class OpenLoopTraj:
    """A loaded dual-arm open-loop trajectory, in physical left/right order.

    All 12-wide arrays are ordered left arm (columns 0-5) then right arm
    (columns 6-11), matching HUSKY_DUAL_UR5e_JOINT_NAMES[0]+[1], regardless of
    how the source file ordered its robots.

    Attributes:
        dt (float): Sample spacing in seconds (verified uniform at load).
        duration (float): Time of the last sample.
        n_samples (int): Number of samples.
        times (np.ndarray): (n,) sample times, starting at 0.
        q12 (np.ndarray): (n, 12) joint positions [rad].
        qd12 (np.ndarray): (n, 12) joint velocities [rad/s].
        servo_flags (np.ndarray): (n,) bool, the planner's "use the servo
            controller here" annotation. Parsed for display only -- the engine
            currently routes everything through the same speedJ tracker.
        grip_closed (np.ndarray): (n, 2) bool, cumulative closed/open state of
            each arm's gripper at every sample (for the preview indicator).
        events (list): GripperEvent list, rising edges only, sorted by time.
        splines (list): Per arm [left, right], CubicHermiteSpline over that
            arm's 6 columns -- reproduces the authored q AND qd exactly at
            every sample because both are interpolation constraints.
        vel_splines (list): Per arm, the derivative spline of splines[i].
        source_path (str): Absolute path of the loaded json.
        swap_arms (bool): Whether the a1/a2 -> left/right mapping was swapped.
        grasp_wait (float): The planner's gripper hold duration [s] (already
            baked into the samples as a still period; informational).
    """
    dt: float
    duration: float
    n_samples: int
    times: np.ndarray
    q12: np.ndarray
    qd12: np.ndarray
    servo_flags: np.ndarray
    grip_closed: np.ndarray
    events: list
    splines: list
    vel_splines: list
    source_path: str
    swap_arms: bool
    grasp_wait: float

    def sample(self, arm_index: int, t: float) -> tuple:
        """Reference position and velocity of one arm at an arbitrary time.

        Clamped at both ends like Valentin's TkSpline: before the start it
        returns the first pose, past the end the last pose, both with zero
        velocity. The engine relies on this for its start pre-roll (t < 0
        holds the start pose) and its end settle (t > duration holds the
        final pose).

        Args:
            arm_index (int): 0 = left, 1 = right.
            t (float): Seconds from trajectory start.

        Returns:
            tuple: (q6, qd6) numpy arrays, position [rad] and velocity [rad/s].
        """
        cols = slice(6 * arm_index, 6 * arm_index + 6)
        if t <= 0.0:
            return self.q12[0, cols].copy(), np.zeros(6)
        if t >= self.duration:
            return self.q12[-1, cols].copy(), np.zeros(6)
        return self.splines[arm_index](t), self.vel_splines[arm_index](t)

    def check_velocity_limit(self, vmax: float, oversample: int = 4) -> float:
        """Largest joint speed the reference splines ever ask for.

        Evaluates the velocity splines on a grid `oversample` times denser
        than the samples, so speed overshoot between knots is caught too
        (the full-file version of the 100-point check in Valentin's
        controller). The engine refuses to execute when the result exceeds
        its velocity clamp -- a clamped reference would distort the path.

        Args:
            vmax (float): Velocity limit to report against [rad/s] (only used
                in the log message the caller prints; the check itself just
                measures).
            oversample (int): Grid densification factor.

        Returns:
            float: max |qd_ref| over all joints, both arms [rad/s].
        """
        dense_t = np.linspace(0.0, self.duration, self.n_samples * oversample)
        return float(max(
            np.abs(vs(dense_t)).max() for vs in self.vel_splines))

    def state_at(self, t: float) -> int:
        """Index of the sample active at time t (for previews and readouts).

        Args:
            t (float): Seconds from trajectory start.

        Returns:
            int: Sample index in [0, n_samples - 1].
        """
        return min(max(bisect_right(self.times, t) - 1, 0), self.n_samples - 1)

    def summary(self) -> str:
        """One human-readable paragraph describing the loaded trajectory.

        Returns:
            str: Sample count, timing, per-arm peak speeds, servo-flag count,
            and the full gripper event list.
        """
        qd_max = [float(np.abs(self.qd12[:, 6 * i:6 * i + 6]).max())
                  for i in range(2)]
        lines = [
            f'{os.path.basename(self.source_path)}: '
            f'{self.n_samples} samples, dt={self.dt}s, '
            f'duration={self.duration:.2f}s, swap_arms={self.swap_arms}',
            f'max |qd| left={qd_max[0]:.3f} right={qd_max[1]:.3f} rad/s, '
            f'servo_controller flagged on {int(self.servo_flags.sum())}'
            f'/{self.n_samples} samples, grasp_wait={self.grasp_wait}s',
            f'{len(self.events)} gripper events:',
        ]
        lines += [f'  t={ev.time:7.2f}s  {ev.kind:5s} {ARM_SIDES[ev.arm_index]}'
                  f' ({ev.robot})' for ev in self.events]
        return '\n'.join(lines)


def load_open_loop_traj(path: str, swap_arms: bool = False) -> OpenLoopTraj:
    """Load and validate an assembly-open-loop-json-v2 trajectory file.

    The a1/a2 -> left/right column mapping happens exactly once, here:
    everything downstream (arm IPs, gains, gripper indices, plots) works in
    physical left/right space and never sees the planner's robot names again.

    Args:
        path (str): Path to the trajectory json.
        swap_arms (bool): False maps the file's first robot (a1) to the left
            arm; True reverses the mapping.

    Returns:
        OpenLoopTraj: The validated, reordered trajectory.

    Raises:
        ValueError: On a schema mismatch or malformed/non-uniform data.
    """
    with open(path) as f:
        data = json.load(f)

    if data.get('schema') != SCHEMA:
        raise ValueError(
            f"schema {data.get('schema')!r} != expected {SCHEMA!r} in {path}")

    robots = data['robots']
    slices = data['robot_slices']
    if len(robots) != 2:
        raise ValueError(f'expected exactly 2 robots, got {robots}')

    samples = data['samples']
    times = np.array([s['time'] for s in samples])
    q_raw = np.array([s['q'] for s in samples])
    qd_raw = np.array([s['qd'] for s in samples])
    servo_flags = np.array([s['servo_controller'] for s in samples], dtype=bool)
    dt = float(data['dt'])

    if q_raw.shape[1] != 12 or qd_raw.shape != q_raw.shape:
        raise ValueError(f'expected (n, 12) q/qd, got {q_raw.shape} / {qd_raw.shape}')
    if not np.allclose(np.diff(times), dt):
        raise ValueError('sample times are not uniformly spaced by dt')

    # * Physical mapping: which source robot drives which arm. The column
    # * permutation rebuilds q/qd as [left 6 | right 6] in one shot.
    arm_robots = list(reversed(robots)) if swap_arms else list(robots)
    perm = []
    for name in arm_robots:
        start, end = slices[name]
        if end - start != 6:
            raise ValueError(f'robot {name} slice {slices[name]} is not 6 wide')
        perm += list(range(start, end))
    q12, qd12 = q_raw[:, perm], qd_raw[:, perm]
    robot_to_arm = {name: i for i, name in enumerate(arm_robots)}

    # * Gripper events: the planner marks close commands on ~grasp_wait worth
    # * of consecutive samples (the arm holds still there) and opens on single
    # * samples. Only the rising edge is a command; the rest is the hold.
    # * grip_closed integrates those memberships into a per-sample state so
    # * the preview can show "closed since t=8.65s" while scrubbing.
    events = []
    grip_closed = np.zeros((len(samples), 2), dtype=bool)
    state = [False, False]  # both grippers assumed open at trajectory start
    prev_closing, prev_opening = set(), set()
    for i, s in enumerate(samples):
        closing, opening = set(s['closing_gripper']), set(s['opening_gripper'])
        for name in sorted(closing - prev_closing):
            events.append(GripperEvent(times[i], robot_to_arm[name], 'close', name))
            state[robot_to_arm[name]] = True
        for name in sorted(opening - prev_opening):
            events.append(GripperEvent(times[i], robot_to_arm[name], 'open', name))
            state[robot_to_arm[name]] = False
        grip_closed[i] = state
        prev_closing, prev_opening = closing, opening
    events.sort(key=lambda ev: ev.time)

    # * One Hermite spline per arm: q and qd are both interpolation
    # * constraints, so the authored trajectory is reproduced exactly at every
    # * sample -- no re-fitting drift like a plain cubic spline would have.
    splines = [CubicHermiteSpline(times, q12[:, 6 * i:6 * i + 6],
                                  qd12[:, 6 * i:6 * i + 6], axis=0)
               for i in range(2)]
    vel_splines = [sp.derivative() for sp in splines]

    return OpenLoopTraj(
        dt=dt, duration=float(times[-1]), n_samples=len(samples), times=times,
        q12=q12, qd12=qd12, servo_flags=servo_flags, grip_closed=grip_closed,
        events=events, splines=splines, vel_splines=vel_splines,
        source_path=os.path.abspath(path), swap_arms=swap_arms,
        grasp_wait=float(data.get('grasp_wait', 0.0)))
