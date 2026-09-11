"""Loader and reference-path math for precomputed open-loop dual-arm trajectories.

* Input: one "assembly-open-loop-json-v2" or "-v3" file (planner output for
* the fixtureless assembly paper): uniformly sampled 12-joint q/qd samples for two
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
import time
from bisect import bisect_right
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicHermiteSpline

# The schemas this loader understands (written by the assembly planner).
# ? v3 (planner master, 2026-09-09) keeps every key of v2. What changed is a
# ? HANDOVER's timing: the giver's open is no longer at the same instant as the
# ? receiver's close but at the end of the receiver's close dwell, and during
# ? that dwell "attached_objects" names the part on BOTH arms. Everything here
# ? reads the file the same way; PartTracker's handover rule already copes.
SCHEMAS = ('assembly-open-loop-json-v2', 'assembly-open-loop-json-v3')
SCHEMA = SCHEMAS[0]           # kept for callers that import the old name
ARM_SIDES = ('left', 'right')  # index 0 = left, 1 = right (repo convention)

# Beyond this much planned motion of the arm that holds the part being
# inserted INTO, say so: the engine keeps that arm still through an insertion,
# so the finished assembly ends up this far from where the plan put it.
HOLDER_TRAVEL_WARN_M = 0.010


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


class TrajClock:
    """The shared trajectory clock both arms read, at an adjustable rate.

    Turns wall time into trajectory time: `tau = tau0 + scale * (now - wall0)`.
    Every tracker thread computes its reference from `now()`, so ONE clock is
    what keeps the two arms in lockstep -- there is no message passing between
    them.

    The rate can also be walked smoothly from one value to another over a
    fixed stretch of wall time (`ramp_to`), which is how the operator's pause
    brings the arms to a standstill without asking for a step change in speed.
    While a ramp is running the rate is a linear function of wall time and the
    trajectory time is therefore quadratic in it; both are worked out from the
    wall clock on every read, so the ramp is just as smooth whether it is read
    at 125 Hz or once a second.

    ! The whole state is a single tuple, replaced in one assignment. Rebinding
    ! an attribute is atomic under the GIL, so a reader either sees the old
    ! tuple or the new one, never a torn mixture of the two. That is what
    ! makes changing the speed mid-run safe: the arms can disagree by at most
    ! one control period's worth of the rate change, not by a broken clock.
    ! It is also why a ramp is stored as its end points rather than stepped
    ! forward by whoever happens to call in -- nobody has to keep it moving.

    Playing at `scale` < 1 stretches trajectory seconds over more wall seconds,
    so everything measured in trajectory time -- the cutoff brake, the end
    settle, a resume blend -- stretches with it. That is intended: the whole
    motion slows down together.

    Attributes:
        scale (float): Trajectory seconds per wall second, right now.
        started (bool): Whether `start` has been called.
    """

    def __init__(self):
        # (wall0, tau0, scale_from, scale_to, ramp_s): the rate walks from
        # scale_from to scale_to over ramp_s wall seconds after wall0, then
        # stays there. ramp_s = 0 means "no ramp", i.e. a plain fixed rate.
        self._map = None

    def start(self, tau0: float = 0.0, delay_s: float = 0.0,
              scale: float = 1.0):
        """Begin (or restart) the clock, optionally after a pre-roll.

        With `delay_s` > 0 the clock reads BELOW `tau0` until the delay is up.
        The engine uses that as its start pre-roll: a negative trajectory time
        makes `OpenLoopTraj.sample` hold the start pose, so a thread that wins
        the race to launch simply waits there for the other one.

        Any ramp in progress is discarded: a fresh phase always starts at a
        steady rate.

        Args:
            tau0 (float): Trajectory time the clock starts from [s].
            delay_s (float): Wall seconds before it reaches `tau0`.
            scale (float): Trajectory seconds per wall second.
        """
        self._map = (time.monotonic() + float(delay_s), float(tau0),
                     float(scale), float(scale), 0.0)

    def now(self) -> float:
        """Current trajectory time [s], or 0.0 before the clock is started."""
        if self._map is None:
            return 0.0
        wall0, tau0, s_from, s_to, ramp = self._map
        elapsed = time.monotonic() - wall0
        if ramp <= 0.0:
            return tau0 + s_to * elapsed
        if elapsed <= 0.0:
            # Still in a pre-roll: the ramp has not begun.
            return tau0 + s_from * elapsed
        # Trajectory time is the area under the rate curve. While the rate
        # ramps linearly that area is a quadratic; past the ramp it is the
        # whole trapezoid plus the final rate times whatever came after.
        if elapsed >= ramp:
            return (tau0 + 0.5 * (s_from + s_to) * ramp
                    + s_to * (elapsed - ramp))
        return tau0 + s_from * elapsed + 0.5 * (s_to - s_from) * elapsed ** 2 / ramp

    def set_scale(self, scale: float):
        """Change the rate at once, without moving the current trajectory time.

        Rebases the map onto right now, so `now()` is continuous across the
        change and only its slope alters -- the reference never jumps.

        Args:
            scale (float): The new trajectory seconds per wall second.
        """
        self.ramp_to(scale, 0.0)

    def ramp_to(self, scale: float, ramp_s: float):
        """Walk the rate to `scale` over `ramp_s` wall seconds.

        Trajectory time stays continuous and so does the rate: the ramp starts
        from whatever the rate is at this instant, so reversing a ramp halfway
        through simply turns it around from there rather than jumping.

        Args:
            scale (float): Rate to end at [trajectory seconds per wall second].
            ramp_s (float): Wall seconds to take getting there; 0 changes it
                at once.
        """
        if self._map is None:
            self._map = (time.monotonic(), 0.0, float(scale), float(scale), 0.0)
            return
        # Read both through the current map before replacing it.
        tau_now, rate_now = self.now(), self.scale
        self._map = (time.monotonic(), tau_now, rate_now, float(scale),
                     max(float(ramp_s), 0.0))

    @property
    def scale(self) -> float:
        """Trajectory seconds per wall second right now (1.0 before started)."""
        if self._map is None:
            return 1.0
        wall0, _, s_from, s_to, ramp = self._map
        if ramp <= 0.0:
            return s_to
        elapsed = time.monotonic() - wall0
        if elapsed <= 0.0:
            return s_from
        if elapsed >= ramp:
            return s_to
        return s_from + (s_to - s_from) * elapsed / ramp

    @property
    def ramping(self) -> bool:
        """Whether a rate ramp is still in progress."""
        if self._map is None:
            return False
        wall0, _, _, _, ramp = self._map
        return ramp > 0.0 and (time.monotonic() - wall0) < ramp

    @property
    def started(self) -> bool:
        """Whether the clock has been started."""
        return self._map is not None


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
        attached (list): One dict per sample, robot name -> the part that robot
            holds there, or None. Straight from the file's "attached_objects";
            this is what tells the part visualization which part each gripper
            is carrying. Empty dicts for a generated trajectory.
        arm_of_robot (dict): Source robot name ('a1'/'a2') -> physical arm
            index, i.e. the a1/a2 -> left/right mapping this load resolved.
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
    attached: list
    arm_of_robot: dict
    events: list
    splines: list
    vel_splines: list
    source_path: str
    swap_arms: bool
    grasp_wait: float

    def sample(self, arm_index: int, t: float, t_end: float = None,
               brake_time: float = 0.0) -> tuple:
        """Reference position and velocity of one arm at an arbitrary time.

        Clamped at both ends like Valentin's TkSpline: before the start it
        returns the first pose, and after the cutoff it holds a resting pose.
        The engine relies on this for its start pre-roll (t < 0 holds the
        start pose) and its end settle.

        A cutoff earlier than the end of the file is generally reached
        MID-MOTION, where the reference still carries speed. Braking there by
        jumping straight to "hold this pose, zero velocity" would make the
        controller fight the arm's momentum and overshoot, so the reference
        itself decelerates: over `brake_time` the velocity decays linearly to
        zero while the position integrates that decay, and only then holds.

        Args:
            arm_index (int): 0 = left, 1 = right.
            t (float): Seconds from trajectory start.
            t_end (float): Cutoff time [s]; defaults to the end of the file.
            brake_time (float): Deceleration ramp after the cutoff [s].

        Returns:
            tuple: (q6, qd6) numpy arrays, position [rad] and velocity [rad/s].
        """
        cols = slice(6 * arm_index, 6 * arm_index + 6)
        if t <= 0.0:
            return self.q12[0, cols].copy(), np.zeros(6)
        end = self.duration if t_end is None else min(max(t_end, 0.0),
                                                      self.duration)
        if t < end:
            return self.splines[arm_index](t), self.vel_splines[arm_index](t)
        # At and past the cutoff: brake along the reference, then hold.
        q_end = self.splines[arm_index](end)
        qd_end = self.vel_splines[arm_index](end)
        past = t - end
        if brake_time > 0.0 and past < brake_time:
            u = past / brake_time
            return q_end + qd_end * brake_time * (u - 0.5 * u * u), \
                qd_end * (1.0 - u)
        return q_end + 0.5 * qd_end * brake_time, np.zeros(6)

    def speed_at(self, t: float) -> float:
        """Largest reference joint speed over both arms at one time.

        Used to warn when a chosen cutoff sample lands mid-motion rather than
        at one of the trajectory's natural still moments.

        Args:
            t (float): Seconds from trajectory start.

        Returns:
            float: max |qd_ref| across all 12 joints [rad/s].
        """
        return float(max(np.abs(self.sample(i, t)[1]).max() for i in range(2)))

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

    def first_pick_part(self, ev) -> str | None:
        """The part a close picks up off the table for the FIRST time, else None.

        The engine's "stop after each first pick" mode asks this for every
        gripper event: the run only stops where a part leaves the spot the
        operator laid it on, which is the one place worth marking on the sheet.

        A close counts when the part it grabs has not been attached to ANY
        robot at any earlier sample. Looking at both robots is what rejects a
        handover receive (the giver held it earlier); the same test rejects a
        re-close on a part already in the jaws and a re-pick of a part the
        robot set down itself.

        Args:
            ev (GripperEvent): The gripper command to classify.

        Returns:
            str | None: The part's name when this close is its first grasp in
            the file; None for an open, a handover receive, a re-close or a
            re-pick -- and always None for a generated trajectory, whose
            attachment records are empty.
        """
        if ev.kind != 'close':
            return None
        i = self.state_at(ev.time)
        # The attachment record can lag the close by one sample, so read the
        # close sample and the one after it (open_loop_parts reads the same
        # edge the same way).
        part = (self.attached[i].get(ev.robot)
                or self.attached[min(i + 1, self.n_samples - 1)].get(ev.robot))
        if part is None:
            return None
        first = next(k for k, held in enumerate(self.attached)
                     if part in held.values())
        # ...and it can lead the close by one sample too, hence the i - 1.
        return part if first >= i - 1 else None

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
        parts = sorted({p for a in self.attached for p in a.values() if p})
        if parts:
            lines.insert(2, f'parts handled: {", ".join(parts)} '
                            f'(robot->arm {self.arm_of_robot})')
        lines += [f'  t={ev.time:7.2f}s  {ev.kind:5s} {ARM_SIDES[ev.arm_index]}'
                  f' ({ev.robot})' for ev in self.events]
        return '\n'.join(lines)


@dataclass
class Insertion:
    """One mate of the plan, located in the trajectory as a straight funnel.

    The planner builds every `assemble` as a move to a funnel mouth followed
    by a pure translation along the mate axis (`pre_insertion`, 50 mm by
    default). That translation is what a compliant controller has to take
    over, and this is where it sits in the file.

    Args:
        arm_index (int): Arm carrying the part being inserted (0 L, 1 R).
        holder_index (int): The other arm, holding the part being inserted
            INTO.
        child (str): Part being inserted.
        parent (str): Part it goes into.
        robot (str): Source-file name of the inserting robot ('a1'/'a2').
        i_start (int): Sample where the straight funnel begins.
        i_open (int): Sample of the gripper-open that ends the mate.
        t_start (float): Time of `i_start` [s].
        t_open (float): Time of `i_open` [s].
        depth_m (float): Length of the funnel, i.e. how far the inserting tool
            travels relative to the holder from the mouth to seated [m].
        holder_travel_m (float): How far the HOLDING arm moves over the same
            window in the plan. The engine freezes that arm during the
            insertion, so a large value means the assembly ends up that far
            from its planned world pose (the mate itself is unaffected).
        q12_start (np.ndarray): 12-joint configuration at `i_start`.
        q12_open (np.ndarray): 12-joint configuration at `i_open`.
    """
    arm_index: int
    holder_index: int
    child: str
    parent: str
    robot: str
    i_start: int
    i_open: int
    t_start: float
    t_open: float
    depth_m: float
    holder_travel_m: float
    q12_start: np.ndarray
    q12_open: np.ndarray

    def describe(self) -> str:
        """One line naming this insertion, for logs and the UI.

        Returns:
            str: Who inserts what into what, when, and how deep.
        """
        return (f'{self.child} -> {self.parent}: {ARM_SIDES[self.arm_index]} '
                f'arm inserts {self.depth_m * 1000:.0f} mm over '
                f't={self.t_start:.2f}..{self.t_open:.2f}s, '
                f'{ARM_SIDES[self.holder_index]} arm holds '
                f'(planned to move {self.holder_travel_m * 1000:.0f} mm)')


def find_insertions(traj, plan_json: str, fk, *, straight_tol_m: float = 0.002,
                    straight_tol_deg: float = 3.0, min_depth_m: float = 0.010,
                    max_depth_m: float = 0.055, log=print) -> list:
    """Locate every mate of a plan inside a loaded trajectory.

    The per-sample `servo_controller` flag does NOT mark these -- it marks the
    simulator's settle and guard windows -- so the mates are found from the
    plan's `assemble` actions instead, each tied to the gripper-open that
    releases that part, and the funnel is measured backwards from there for as
    long as the motion stays a straight translation.

    ! Straightness is measured in the HOLDING arm's frame, not the robot base:
    ! the planner builds a mate as a pure translation of the part relative to
    ! the part it goes into, and that other part is usually held by the second
    ! arm, which is moving too. In the base frame the same motion is a curve.

    ! Matching is by the CHILD part, not by robot: a plan re-solved after its
    ! trajectory was exported can hand the same mate to the other arm, and the
    ! trajectory is the truth about which arm actually does it.

    Args:
        traj (OpenLoopTraj): A file-loaded trajectory (needs `attached`).
        plan_json (str): Path to the planner's plan.json.
        fk (callable): `fk(side, q6) -> 4x4` tool0 transform, e.g.
            `ssik_inprocess.fk`. Both arms go through the same function and
            only their RELATIVE pose is used, so any consistent frame works.
        straight_tol_m (float): How far the relative motion may stray from its
            chord before the funnel is considered to have started [m].
        straight_tol_deg (float): Relative rotation allowed over the funnel.
        min_depth_m (float): Funnels shorter than this are skipped [m]. The
            planner shortens a funnel when the full 50 mm will not plan, so a
            few of them legitimately come out very short.
        max_depth_m (float): Never look further back than this [m]; defaults
            to a little over the planner's 50 mm `pre_insertion`.
        log (callable): Where to report skips and mismatches.

    Returns:
        list: Insertion records, in trajectory order.

    Raises:
        ValueError: If the plan file carries no action list.
    """
    with open(plan_json) as f:
        plan = json.load(f)
    if not plan.get('plan'):
        raise ValueError(f'{plan_json}: no "plan" action list')
    mates = {}
    for action in plan['plan']:
        if len(action) >= 4 and action[0] == 'assemble':
            mates.setdefault(action[2], []).append((action[3], action[1]))

    # Every open, with the part it releases (the record lags an open by one
    # sample). The mate is the LAST one: an earlier open of the same part is a
    # handover give, after which the part is picked up again.
    opens = {}
    for ev in traj.events:
        if ev.kind == 'open':
            i = traj.state_at(ev.time)
            part = (traj.attached[max(i - 1, 0)].get(ev.robot)
                    or traj.attached[i].get(ev.robot))
            if part is not None:
                opens[part] = (i, ev)

    found = []
    for child, entries in mates.items():
        parent, plan_robot = entries[0]
        if child not in opens:
            log(f'[insertions] {child} is assembled in {os.path.basename(plan_json)} '
                f'but never released in the trajectory -- skipped')
            continue
        i_open, ev = opens[child]
        if ev.robot != plan_robot:
            log(f'[insertions] plan has {child} -> {parent} inserted by '
                f'{plan_robot}, the trajectory uses {ev.robot} -- following '
                f'the trajectory')
        arm = ev.arm_index
        holder = 1 - arm
        held = traj.attached[max(i_open - 1, 0)]
        if parent not in held.values():
            log(f'[insertions] nothing holds {parent} while {child} goes into '
                f'it (arms hold {held}) -- inserting anyway, the other arm '
                f'will just hold its pose')

        # The inserting tool seen from the holding tool: this is the frame the
        # planner's straight-line mate lives in. Cached, since the walk-back
        # re-reads the same samples many times.
        cache = {}

        def relative(i: int) -> np.ndarray:
            """Inserting tool0 expressed in the holding arm's tool0 frame."""
            if i not in cache:
                a = fk(ARM_SIDES[holder], traj.q12[i, 6 * holder:6 * holder + 6])
                b = fk(ARM_SIDES[arm], traj.q12[i, 6 * arm:6 * arm + 6])
                cache[i] = np.linalg.inv(a) @ b
            return cache[i]

        end = relative(i_open)
        i_start = i_open
        for j in range(i_open - 1, -1, -1):
            here = relative(j)
            chord = end[:3, 3] - here[:3, 3]
            length = float(np.linalg.norm(chord))
            if length > max_depth_m:
                break
            turn = np.degrees(np.arccos(np.clip(
                (np.trace(here[:3, :3].T @ end[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
            if turn > straight_tol_deg:
                break
            # ? Only judge straightness once the chord is long enough to point
            # ? anywhere: a mate ends with a dwell of a millimetre or two, and
            # ? over such a chord any jitter reads as a huge deviation.
            if length > max(0.005, 2.0 * straight_tol_m):
                axis = chord / length
                # Distance of every intermediate sample from the chord.
                stray = 0.0
                for k in range(j, i_open + 1):
                    d = relative(k)[:3, 3] - here[:3, 3]
                    stray = max(stray, float(np.linalg.norm(d - (d @ axis) * axis)))
                if stray > straight_tol_m:
                    break
            i_start = j
        depth = float(np.linalg.norm(end[:3, 3] - relative(i_start)[:3, 3]))
        if depth < min_depth_m:
            log(f'[insertions] {child} -> {parent}: only {depth * 1000:.1f} mm '
                f'of straight approach (< {min_depth_m * 1000:.0f} mm) -- '
                f'skipped, it stays on the tracker')
            continue
        hold_side = ARM_SIDES[holder]
        hold_cols = slice(6 * holder, 6 * holder + 6)
        travel = float(np.linalg.norm(
            fk(hold_side, traj.q12[i_open, hold_cols])[:3, 3]
            - fk(hold_side, traj.q12[i_start, hold_cols])[:3, 3]))
        if travel > HOLDER_TRAVEL_WARN_M:
            log(f'[insertions] {child} -> {parent}: the plan moves the holding '
                f'{hold_side} arm {travel * 1000:.0f} mm during this mate, but '
                f'the engine holds it still -- the mate is unaffected, the '
                f'assembly just ends that far from its planned world pose')
        found.append(Insertion(
            arm_index=arm, holder_index=holder, child=child, parent=parent,
            robot=ev.robot, i_start=i_start, i_open=i_open,
            t_start=float(traj.times[i_start]), t_open=float(traj.times[i_open]),
            depth_m=depth, holder_travel_m=travel,
            q12_start=traj.q12[i_start].copy(),
            q12_open=traj.q12[i_open].copy()))
    found.sort(key=lambda ins: ins.i_start)
    return found


def _build_splines(times: np.ndarray, q12: np.ndarray,
                   qd12: np.ndarray) -> tuple:
    """Per-arm Hermite splines (and their derivatives) over sampled arrays.

    Position AND velocity are interpolation constraints, so the splines
    reproduce the given samples exactly at every knot.

    Args:
        times (np.ndarray): (n,) strictly increasing sample times [s].
        q12 (np.ndarray): (n, 12) positions [rad].
        qd12 (np.ndarray): (n, 12) velocities [rad/s].

    Returns:
        tuple: (splines, vel_splines), each a list of two scipy splines.
    """
    splines = [CubicHermiteSpline(times, q12[:, 6 * i:6 * i + 6],
                                  qd12[:, 6 * i:6 * i + 6], axis=0)
               for i in range(2)]
    return splines, [sp.derivative() for sp in splines]


def rebase_to_branch(traj, q12_live, lower=None, upper=None) -> tuple:
    """Shift whole joints of a trajectory by full turns onto the arms' branch.

    ! A revolute joint at q and at q +/- 2*pi is the SAME arm pose, so a
    ! trajectory can be moved bodily onto either branch without changing a
    ! single tool position. What it does change is the number the tracker
    ! commands: if the arm stands on one branch and the reference starts on
    ! the other, the very first command asks for a full revolution.
    !
    ! This is not hypothetical -- `open_loop_approach.unwrap_goal` DELIBERATELY
    ! drives the arms to the branch nearest their live pose, because reaching
    ! the authored one would mean turning a joint the long way round. The
    ! trajectory has to follow that decision, or the start check sees a 2*pi
    ! delta on a joint that is physically already in place.

    The whole column is shifted, so the trajectory stays continuous and its
    velocities are untouched (a constant offset has zero derivative).

    Args:
        traj (OpenLoopTraj): The trajectory to rebase.
        q12_live: Where the arms actually are [rad], 12 values.
        lower: Per-joint lower limits [rad], or None to skip the check.
        upper: Per-joint upper limits [rad], or None to skip the check.

    Returns:
        tuple: ``(traj, turns, blocked)``. `turns` is the (12,) integer count
        of full turns applied per joint (all zero when nothing was needed, and
        also when the shift was refused). `blocked` lists the joint indices
        whose shift would have left the limits -- non-empty means NOTHING was
        rebased and the caller must not run.
    """
    import dataclasses

    q_live = np.asarray(q12_live, dtype=float)
    turns = np.round((q_live - traj.q12[0]) / (2.0 * np.pi)).astype(int)
    if not turns.any():
        return traj, np.zeros(12, dtype=int), []

    shifted = traj.q12 + turns * 2.0 * np.pi
    blocked = []
    if lower is not None and upper is not None:
        for j in range(12):
            if turns[j] and (shifted[:, j].min() < lower[j]
                             or shifted[:, j].max() > upper[j]):
                blocked.append(j)
    if blocked:
        return traj, np.zeros(12, dtype=int), blocked

    splines, vel_splines = _build_splines(traj.times, shifted, traj.qd12)
    return (dataclasses.replace(traj, q12=shifted, splines=splines,
                                vel_splines=vel_splines),
            turns, [])


def traj_from_arrays(times, q12, qd12, *, label: str = '<generated>',
                     swap_arms: bool = False) -> OpenLoopTraj:
    """Wrap already-sampled arrays as an OpenLoopTraj with no gripper events.

    Used for motions the engine generates itself (the planned approach), so
    they can run through exactly the same tracker, preview and logging as a
    trajectory loaded from a file.

    Args:
        times (np.ndarray): (n,) sample times starting at 0 [s].
        q12 (np.ndarray): (n, 12) positions in left+right order [rad].
        qd12 (np.ndarray): (n, 12) velocities [rad/s].
        label (str): Name reported in summaries and logs.
        swap_arms (bool): Recorded for provenance only; the arrays are
            expected to already be in physical left/right order.

    Returns:
        OpenLoopTraj: The generated trajectory.
    """
    times = np.asarray(times, dtype=float)
    q12 = np.asarray(q12, dtype=float)
    qd12 = np.asarray(qd12, dtype=float)
    splines, vel_splines = _build_splines(times, q12, qd12)
    return OpenLoopTraj(
        dt=float(times[1] - times[0]) if len(times) > 1 else 0.0,
        duration=float(times[-1]), n_samples=len(times), times=times,
        q12=q12, qd12=qd12,
        servo_flags=np.zeros(len(times), dtype=bool),
        grip_closed=np.zeros((len(times), 2), dtype=bool),
        attached=[{} for _ in times], arm_of_robot={},
        events=[], splines=splines, vel_splines=vel_splines,
        source_path=label, swap_arms=swap_arms, grasp_wait=0.0)


def load_open_loop_traj(path: str, swap_arms: bool = False) -> OpenLoopTraj:
    """Load and validate an assembly-open-loop-json-v2 or -v3 trajectory file.

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

    if data.get('schema') not in SCHEMAS:
        raise ValueError(
            f"schema {data.get('schema')!r} not in {SCHEMAS!r} in {path}")

    robots = data['robots']
    slices = data['robot_slices']
    if len(robots) != 2:
        raise ValueError(f'expected exactly 2 robots, got {robots}')

    samples = data['samples']
    times = np.array([s['time'] for s in samples])
    q_raw = np.array([s['q'] for s in samples])
    qd_raw = np.array([s['qd'] for s in samples])
    servo_flags = np.array([s['servo_controller'] for s in samples], dtype=bool)
    # * Which part each robot holds at each sample. The planner writes this per
    # * sample, so the part visualization can follow a part from the table into
    # * a gripper and on into the assembly without guessing.
    attached = [dict(s.get('attached_objects') or {}) for s in samples]
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
    splines, vel_splines = _build_splines(times, q12, qd12)

    return OpenLoopTraj(
        dt=dt, duration=float(times[-1]), n_samples=len(samples), times=times,
        q12=q12, qd12=qd12, servo_flags=servo_flags, grip_closed=grip_closed,
        attached=attached, arm_of_robot=robot_to_arm,
        events=events, splines=splines, vel_splines=vel_splines,
        source_path=os.path.abspath(path), swap_arms=swap_arms,
        grasp_wait=float(data.get('grasp_wait', 0.0)))
