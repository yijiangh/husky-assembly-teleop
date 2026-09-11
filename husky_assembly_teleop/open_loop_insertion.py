"""Force-aware Cartesian insertion for one arm, over ur_rtde speedL.

* The open-loop tracker replays joint angles. That is fine everywhere except
* the mates: a leg tenon is 30 x 22 mm and its mortise 32 x 32 mm, so ONE
* millimetre of accumulated pickup and grasp error on the tight axis jams the
* part on the rim. This module takes over those few centimetres and finds the
* hole instead of assuming it.
* Input: the two planned configurations that bracket the mate (the funnel
* mouth and the seated pose) and a live RTDE pair. Output: a commanded tool
* velocity every cycle, plus a full record of what the forces did.

The skill is the classic model-based one, and mirrors the planner's own
simulated version (`sim._SkillServo.seat`) so hardware and simulation fail the
same way:

    zero -> approach -> [search] -> insert -> [backoff -> insert] -> seated

`approach` runs down the planned line until the part touches something.
If it touched short of depth, `search` walks an Archimedean spiral across the
rim while pressing down, until the part drops into the mouth -- the pitch is
kept under the diametral clearance so the spiral cannot step over the hole.
`insert` then pushes to depth under a force cap, and a jam buys ONE bounded
back-off and retry before the skill gives up and hands the operator the
decision.

The control law is a velocity-resolved admittance, written in the robot's BASE
frame and split ACROSS the insertion axis and ALONG it, because those two
directions want opposite things:

    across:  v = kp * (lateral_ref - lateral) + A * deadband(f_lateral)
    along:   v = A * (f_axial - f_target)                    [force control]
    turning: w = kr * rotvec(R_ref * R^T) + Ar * deadband(tau)

Across the axis the part is positioned -- that is what the search steers.
Along it the part is pressed with a regulated force, which is the only way to
push something home without knowing exactly how far away home is: the same
command works whether the part is 5 mm out or already touching. The whole
thing goes out as `speedL`, and the UR's own controller does the inverse
kinematics, which is why this stays a few hundred lines with no Jacobian.

! The dead-band belongs on the axes whose target force is ZERO, where sensor
! noise would otherwise drive a slow drift. The axial push is regulated
! against a 10 N target that stands well clear of the noise, so dead-banding
! it there would only bias the force by the dead-band's own width.

! The wrench is the UR's built-in wrist sensor, whose accuracy is +-4 N and
! +-0.3 Nm; everything smaller is inside `deadband_force`/`deadband_torque`
! and is ignored, so sensor noise cannot drive the arm.

! `zeroFtSensor` is called at the start of every insertion, with the part
! already in the gripper and the arm holding still. Skipping that is what made
! the earlier ROS compliance controller sag: an uncompensated tool weight is
! read as an external force and the spring yields to it forever.

! Poses here are the UR's own 6-vectors [x, y, z, rx, ry, rz] (metres and a
! rotation vector, base frame) -- NOT pybullet_planning poses, and NOT the
! ssik frame, which is turned 180 degrees about z from the UR base.

! This module imports only numpy/scipy -- no ROS, no UI, no pybullet -- so it
! runs against the fake RTDE pair in `scripts/fake_rtde.py`.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

# Phase names in the order they can be entered. Logged as their index so a
# whole run's phases fit in one numeric array next to the forces.
PHASES = ('zero', 'approach', 'search', 'insert', 'backoff', 'seated', 'failed')

# Outcomes that end a skill. Everything but 'seated' hands over to the
# operator, and only 'seated' may be followed by the gripper opening.
OUTCOME_SEATED = 'seated'
OUTCOME_SEATED_CONTACT = 'seated_contact'
GOOD_OUTCOMES = (OUTCOME_SEATED, OUTCOME_SEATED_CONTACT)


@dataclass
class InsertionParams:
    """Tunables of the insertion skill, all SI units.

    The defaults come from the planner's own simulated skill where it has one
    (`sim.py` INSERT_* constants and `_SkillServo`), from the UR5e wrist
    sensor's quoted accuracy for the dead-bands, and from the stool joint's
    geometry for the search (a 30 x 22 mm tenon in a 32 x 32 mm mortise
    leaves 1 mm per side on the tight axis, so the spiral pitch must be
    smaller than that).

    Args:
        approach_speed (float): Tool speed down the funnel before contact.
        insert_speed (float): Tool speed while pressing in after contact.
        search_speed (float): Tool speed along the spiral.
        search_pitch (float): Radial growth per spiral turn [m]; must stay
            under the joint's diametral clearance or the spiral steps over
            the hole.
        search_radius (float): Give up searching past this radius [m].
        contact_force (float): Above this the part is touching something [N].
        push_force (float): Force held along the mate axis while searching and
            inserting [N] (the planner's own `insert_force`).
        max_force (float): The admittance yields to keep the axial force here.
        guard_force (float): Any measured force this large aborts at once [N].
        catch_drop (float): Axial advance that counts as having found the
            mouth during the search [m].
        depth_tol (float): How close to full depth counts as seated [m].
        stall_progress (float): Axial progress that resets the jam timer [m].
        stall_time (float): No progress for this long while pressing = jam [s].
        backoff (float): How far to retreat before a retry [m].
        constrained_lateral (float): While searching, this much lag between
            the spiral and the part means it cannot move sideways [m] -- it
            is inside the joint and blocked, so the skill treats it as a jam
            rather than carrying on sweeping.
        kp (float): Cartesian position gain [1/s].
        kr (float): Cartesian orientation gain [1/s].
        admittance (float): Yield rate per newton of excess force [m/s/N].
        rot_admittance (float): Yield rate per newton-metre [rad/s/Nm].
        deadband_force (float): Forces smaller than this are noise [N].
        deadband_torque (float): Torques smaller than this are noise [Nm].
        lowpass_tau (float): Time constant of the wrench filter [s].
        vmax (float): Commanded translation speed cap [m/s].
        wmax (float): Commanded rotation speed cap [rad/s].
        accel (float): Acceleration argument passed to speedL [m/s^2].
        settle_s (float): Hold still this long when zeroing and when seated.
        budget_s (float): Give up if the whole skill takes longer than this.
        tcp_offset (tuple): tool0 -> TCP pose for the UR, [x, y, z, rx, ry, rz].
    """
    approach_speed: float = 0.020
    insert_speed: float = 0.010
    search_speed: float = 0.010
    search_pitch: float = 0.0015
    search_radius: float = 0.008
    contact_force: float = 8.0
    push_force: float = 10.0
    max_force: float = 30.0
    guard_force: float = 40.0
    catch_drop: float = 0.002
    depth_tol: float = 0.002
    stall_progress: float = 0.0002
    stall_time: float = 0.3
    backoff: float = 0.003
    constrained_lateral: float = 0.003
    kp: float = 4.0
    kr: float = 2.0
    admittance: float = 0.002
    rot_admittance: float = 0.05
    deadband_force: float = 4.0
    deadband_torque: float = 0.3
    lowpass_tau: float = 0.05
    vmax: float = 0.05
    wmax: float = 0.3
    accel: float = 0.5
    settle_s: float = 0.3
    budget_s: float = 30.0
    # 0.152 gripper + utils.ROBOTIQ_COUPLING_M. Mirrored rather than imported so
    # this module stays free of the pybullet import utils pulls in; the engine
    # overwrites this field from utils.TOOL0_FROM_GRIPPER_TCP at every run.
    tcp_offset: tuple = (0.0, 0.0, 0.1632, 0.0, 0.0, 0.0)


def deadband(values, width: float) -> np.ndarray:
    """Shrink a vector towards zero by `width`, componentwise.

    Anything smaller than the sensor's own accuracy has to read as exactly
    zero, or noise alone would drive the arm; anything larger keeps only the
    part that rises above it, so the response is continuous at the threshold.

    Args:
        values (np.ndarray): The measured components.
        width (float): Half-width of the dead zone, in the same unit.

    Returns:
        np.ndarray: The dead-banded values.
    """
    values = np.asarray(values, dtype=float)
    return np.sign(values) * np.maximum(np.abs(values) - width, 0.0)


def pose_error(reference, actual) -> tuple:
    """Position and rotation error between two UR pose vectors.

    Args:
        reference (np.ndarray): Desired pose [x, y, z, rx, ry, rz].
        actual (np.ndarray): Measured pose, same convention.

    Returns:
        tuple: (position error [m], rotation error as a rotation vector
        [rad]), both in the base frame and both pointing from actual to
        reference.
    """
    reference = np.asarray(reference, dtype=float)
    actual = np.asarray(actual, dtype=float)
    turn = (Rotation.from_rotvec(reference[3:]) *
            Rotation.from_rotvec(actual[3:]).inv())
    return reference[:3] - actual[:3], turn.as_rotvec()


def spiral_offset(arc_length: float, pitch: float) -> tuple:
    """A point on an Archimedean spiral, parameterised by arc length.

    Walking the spiral by ARC LENGTH rather than by angle keeps the tool speed
    constant as the radius grows, which is what makes the search predictable:
    it sweeps at `search_speed` from the first turn to the last.

    The spiral is r = pitch * theta / (2 pi), whose arc length is very nearly
    (pitch / 4 pi) * theta^2 for theta beyond the first turn; inverting that
    gives theta, and the small-theta error only affects the innermost
    millimetre, which the approach has already probed.

    Args:
        arc_length (float): Distance travelled along the spiral [m].
        pitch (float): Radial growth per full turn [m].

    Returns:
        tuple: (offset (2,) [m], radius [m]).
    """
    theta = np.sqrt(max(4.0 * np.pi * arc_length / max(pitch, 1e-9), 0.0))
    radius = pitch * theta / (2.0 * np.pi)
    return np.array([radius * np.cos(theta), radius * np.sin(theta)]), radius


def frame_axes(axis) -> tuple:
    """Two unit vectors spanning the plane perpendicular to `axis`.

    Args:
        axis (np.ndarray): The insertion direction (need not be normalised).

    Returns:
        tuple: (u, v), unit vectors with u x v along `axis`.
    """
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    # Any vector not parallel to the axis seeds the perpendicular pair.
    seed = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, seed)
    u /= np.linalg.norm(u)
    return u, np.cross(axis, u)


@dataclass
class InsertionLog:
    """Per-cycle record of one insertion, for the plots and the npz.

    Every list holds one entry per control cycle, so they can be stacked into
    arrays straight from here.
    """
    t: list = field(default_factory=list)
    phase: list = field(default_factory=list)
    pose: list = field(default_factory=list)
    v_cmd: list = field(default_factory=list)
    wrench_raw: list = field(default_factory=list)
    wrench_lp: list = field(default_factory=list)
    axial: list = field(default_factory=list)
    search_r: list = field(default_factory=list)
    lateral_err: list = field(default_factory=list)

    def arrays(self, prefix: str = '') -> dict:
        """This log as numpy arrays, ready for `np.savez`.

        Args:
            prefix (str): Prepended to every key.

        Returns:
            dict: Key -> array.
        """
        return {f'{prefix}{name}': np.asarray(rows)
                for name, rows in self.__dict__.items()}


class InsertionController:
    """Runs one mate on one arm, one control cycle per `step` call.

    The caller owns the loop and the clock -- `step` never sleeps -- so the
    same controller drives the real robot at 125 Hz and the fake one as fast
    as the test can go.

    Args:
        rtde_c: An RTDEControlInterface (or the fake one).
        rtde_r: The matching RTDEReceiveInterface.
        params (InsertionParams): Tunables.
        dt (float): Control period [s].
        log_fn (callable): One-line logger for phase changes.

    Attributes:
        phase (str): Current phase, one of PHASES.
        outcome (str): Empty until the skill ends, then why it ended.
        record (dict): Summary of the whole skill, for insertions.json.
        log (InsertionLog): Per-cycle history.
    """

    def __init__(self, rtde_c, rtde_r, params: InsertionParams, dt: float,
                 log_fn=print):
        self.rtde_c = rtde_c
        self.rtde_r = rtde_r
        self.params = params
        self.dt = float(dt)
        self.log_fn = log_fn
        self.phase = 'zero'
        self.outcome = ''
        self.log = InsertionLog()
        self.record = {}
        self.wrench_lp = np.zeros(6)
        self._restore_tcp = None

    # --- --- SETUP --- ---

    def setup(self, q_start6, q_open6):
        """Measure the funnel this skill has to follow, from two configurations.

        The line is taken from the robot's OWN forward kinematics at the two
        planned configurations, so the reference lives in exactly the frame
        the wrench and the pose feedback do. Nothing here is derived from a
        model outside the UR.

        Args:
            q_start6 (np.ndarray): The 6 joints at the funnel mouth.
            q_open6 (np.ndarray): The 6 joints at the seated pose.

        Raises:
            RuntimeError: If the two configurations are the same pose, so
                there is no insertion direction to follow.
        """
        self._restore_tcp = list(self.rtde_c.getTCPOffset())
        self.rtde_c.setTcp(list(self.params.tcp_offset))
        x_start = np.asarray(self.rtde_c.getForwardKinematics(
            list(q_start6), list(self.params.tcp_offset)), dtype=float)
        x_open = np.asarray(self.rtde_c.getForwardKinematics(
            list(q_open6), list(self.params.tcp_offset)), dtype=float)
        travel = x_open[:3] - x_start[:3]
        self.depth = float(np.linalg.norm(travel))
        if self.depth < 1e-4:
            raise RuntimeError('the two configurations bracket no motion: '
                               'there is no insertion axis to follow')
        self.axis = travel / self.depth
        self.perp = frame_axes(self.axis)
        # ! The orientation is held at the funnel mouth's for the whole skill.
        # ! The planner's mate is a pure translation, and letting the tool
        # ! rotate while a tenon is inside a mortise is how you wedge it.
        self.x_ref = x_start.copy()
        self.p_mouth = x_start[:3].copy()

        self.t0 = None
        self.phase = 'zero'
        self.outcome = ''
        self.entered = 0.0
        self.search_s = 0.0
        self.search_r = 0.0
        self.search_centre = np.zeros(2)
        self.hold_lateral = np.zeros(2)  # lateral target once past the rim
        self.lateral_err = 0.0          # how far the part lags the search
        self.contact_axial = None      # axial travel where the part touched
        self.best_axial = None         # deepest axial travel of this attempt
        self.stalled_for = 0.0
        self.retried = False
        self.peak_force = 0.0
        self.phase_log = []
        self.wrench_lp = np.zeros(6)
        self.log = InsertionLog()
        self.log_fn(f'insertion set up: {self.depth * 1000:.1f} mm along '
                    f'{np.round(self.axis, 3)} (base frame)')

    # --- --- GEOMETRY HELPERS --- ---

    def axial_of(self, position) -> float:
        """How far along the insertion axis a point has travelled.

        Args:
            position (np.ndarray): A point in the base frame [m].

        Returns:
            float: Signed distance from the funnel mouth along the axis [m].
        """
        return float(np.dot(np.asarray(position, dtype=float) - self.p_mouth,
                            self.axis))

    def lateral_of(self, position) -> np.ndarray:
        """A point's offset from the funnel line, in the search plane.

        Args:
            position (np.ndarray): A point in the base frame [m].

        Returns:
            np.ndarray: (2,) offset along the two perpendicular axes [m].
        """
        delta = np.asarray(position, dtype=float) - self.p_mouth
        return np.array([float(np.dot(delta, self.perp[0])),
                         float(np.dot(delta, self.perp[1]))])

    def _enter(self, phase: str, t: float, why: str = ''):
        """Switch phase, recording when and why.

        Args:
            phase (str): The phase being entered.
            t (float): Skill time [s].
            why (str): Short reason, for the log and the record.
        """
        self.phase_log.append({'phase': phase, 't': round(t, 3), 'why': why})
        self.log_fn(f'insertion {self.phase} -> {phase} at t={t:.2f}s'
                    + (f' ({why})' if why else ''))
        self.phase = phase
        self.entered = t

    def _finish(self, outcome: str, t: float):
        """End the skill with an outcome and stop the arm.

        Args:
            outcome (str): Why it ended.
            t (float): Skill time [s].
        """
        self.outcome = outcome
        self._enter('seated' if outcome in GOOD_OUTCOMES else 'failed', t,
                    outcome)
        try:
            self.rtde_c.speedStop()
        except Exception:
            pass

    # --- --- THE CONTROL CYCLE --- ---

    def step(self, t: float) -> bool:
        """Run one control cycle.

        Args:
            t (float): Seconds since the skill started.

        Returns:
            bool: True while the skill is still running, False once it has
            finished (successfully or not -- read `outcome`).
        """
        par = self.params
        if self.outcome:
            return False
        if self.t0 is None:
            self.t0 = t

        pose = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
        raw = np.asarray(self.rtde_r.getActualTCPForce(), dtype=float)
        # First-order low pass: the sensor is noisy and every threshold in
        # here is compared against the filtered value.
        alpha = self.dt / max(par.lowpass_tau, self.dt)
        self.wrench_lp += alpha * (raw - self.wrench_lp)
        force = self.wrench_lp[:3]
        f_axial = float(np.dot(force, self.axis))
        axial = self.axial_of(pose[:3])
        self.peak_force = max(self.peak_force, float(np.linalg.norm(force)))

        # --- guards, checked in every phase ---
        if np.linalg.norm(force) > par.guard_force:
            self._finish('wrench_guard', t)
        elif t > par.budget_s:
            self._finish('budget', t)
        else:
            self._advance(t, pose, axial, f_axial)

        v, w = self._command(t, pose, axial, f_axial)
        self.log.t.append(t)
        self.log.phase.append(PHASES.index(self.phase))
        self.log.pose.append(pose.copy())
        self.log.v_cmd.append(np.concatenate([v, w]))
        self.log.wrench_raw.append(raw.copy())
        self.log.wrench_lp.append(self.wrench_lp.copy())
        self.log.axial.append(axial)
        self.log.search_r.append(self.search_r)
        self.log.lateral_err.append(self.lateral_err)

        if self.outcome:
            self._summarise(t, axial)
            return False
        self.rtde_c.speedL(list(np.concatenate([v, w])), par.accel, self.dt)
        return True

    def _advance(self, t: float, pose, axial: float, f_axial: float):
        """Decide whether this cycle ends the current phase.

        Args:
            t (float): Skill time [s].
            pose (np.ndarray): Measured TCP pose.
            axial (float): Travel along the insertion axis [m].
            f_axial (float): Filtered force along the insertion axis [N].
        """
        par = self.params
        # ! Pressing IN is -axis on the sensor: the part pushes back.
        pressing = -f_axial > par.contact_force
        remaining = self.depth - axial

        if self.phase == 'zero':
            if t - self.entered >= par.settle_s:
                self.rtde_c.zeroFtSensor()
                self.wrench_lp = np.zeros(6)
                self._enter('approach', t, 'sensor zeroed with the part held')

        elif self.phase == 'approach':
            if remaining <= par.depth_tol:
                self._finish(OUTCOME_SEATED, t)
            elif pressing:
                self.contact_axial = axial
                self.best_axial = axial
                if remaining <= par.catch_drop + par.depth_tol:
                    self._finish(OUTCOME_SEATED_CONTACT, t)
                else:
                    # Touched down early: this is the rim, not the bottom.
                    self.search_centre = self.lateral_of(pose[:3])
                    self.hold_lateral = self.search_centre.copy()
                    self.search_s = 0.0
                    self._enter('search', t,
                                f'contact {remaining * 1000:.1f} mm short')

        elif self.phase == 'search':
            # Dropping past the rim means the mouth has been found.
            if self.contact_axial is not None and \
                    axial - self.contact_axial > par.catch_drop:
                self.stalled_for = 0.0
                self.best_axial = axial
                self.hold_lateral = self.lateral_of(pose[:3])
                self._enter('insert', t,
                            f'caught at r={self.search_r * 1000:.1f} mm')
            elif self.lateral_err > par.constrained_lateral:
                # ! The spiral is running away from the part: it cannot move
                # ! sideways, so it is already INSIDE something and simply
                # ! blocked. That is a jam, not a hole still to be found.
                if self.retried:
                    self._finish('stalled', t)
                else:
                    self.retried = True
                    self.backoff_from = axial
                    self._enter('backoff', t, 'part is trapped, not on a rim')
            elif self.search_r > par.search_radius:
                self._finish('search_exhausted', t)

        elif self.phase == 'insert':
            if remaining <= par.depth_tol:
                self._finish(OUTCOME_SEATED, t)
                return
            if pressing:
                if self.best_axial is None or axial > self.best_axial + par.stall_progress:
                    self.best_axial, self.stalled_for = axial, 0.0
                else:
                    self.stalled_for += self.dt
            else:
                self.stalled_for = 0.0
            if self.stalled_for >= par.stall_time:
                # Stuck. Right at the bottom that is a seat; anywhere else it
                # is a jam, worth exactly one bounded retry.
                if remaining <= par.catch_drop + par.depth_tol:
                    self._finish(OUTCOME_SEATED_CONTACT, t)
                elif self.retried:
                    self._finish('stalled', t)
                else:
                    self.retried = True
                    self.backoff_from = axial
                    self._enter('backoff', t,
                                f'jammed {remaining * 1000:.1f} mm short')

        elif self.phase == 'backoff':
            retreated = self.backoff_from - axial
            if retreated >= par.backoff and not pressing:
                # Re-centre the spiral on where we are now and try again at
                # half speed, as the simulated skill does.
                self.search_centre = self.lateral_of(pose[:3])
                self.hold_lateral = self.search_centre.copy()
                self.search_s = 0.0
                self.contact_axial = axial
                self._enter('search', t, 'retreated, searching again')

    def _command(self, t: float, pose, axial: float, f_axial: float) -> tuple:
        """The velocity to send this cycle.

        Args:
            t (float): Skill time [s].
            pose (np.ndarray): Measured TCP pose.
            axial (float): Travel along the insertion axis [m].
            f_axial (float): Filtered force along the insertion axis [N].

        Returns:
            tuple: (linear velocity (3,) [m/s], angular velocity (3,) [rad/s]).
        """
        par = self.params
        if self.outcome or self.phase in ('zero', 'seated', 'failed'):
            return np.zeros(3), np.zeros(3)

        halved = 0.5 if self.retried else 1.0
        lateral_now = self.lateral_of(pose[:3])
        lateral_ref = self.hold_lateral.copy()

        # * HYBRID CONTROL. The insertion axis and the two axes across it want
        # * opposite things, so they are driven differently:
        # *   - across the axis, POSITION control puts the part where the
        # *     search says, yielding only to forces above the sensor noise;
        # *   - along the axis, FORCE control regulates the push, which is the
        # *     only way to press a part home without knowing exactly how far
        # *     away home is.
        if self.phase == 'approach':
            # ! Free space, so the axial target IS zero force and the
            # ! dead-band applies: a feed-forward descent that yields the
            # ! instant it meets anything. It stalls just past the contact
            # ! threshold, which is how contact gets noticed.
            v_axial = (par.approach_speed
                       + par.admittance * float(deadband(f_axial,
                                                         par.deadband_force)))
            lateral_ref = np.zeros(2)
        elif self.phase == 'search':
            # Sweep the spiral across the rim while leaning on the joint. The
            # push is regulated directly: at 10 N it stands well clear of the
            # sensor's own +-4 N, so a dead-band here would only bias it.
            self.search_s += par.search_speed * halved * self.dt
            offset, self.search_r = spiral_offset(self.search_s, par.search_pitch)
            lateral_ref = self.search_centre + offset
            v_axial = min(par.admittance * (f_axial + par.push_force),
                          par.approach_speed)
        elif self.phase == 'insert':
            v_axial = min(par.admittance * (f_axial + par.push_force),
                          par.insert_speed * halved)
        else:                                    # backoff
            v_axial = -par.insert_speed

        # Never drive past the seated depth, whatever the forces say.
        if axial >= self.depth and v_axial > 0.0:
            v_axial = 0.0

        # Assemble the commanded velocity from its three components.
        lateral_err = lateral_ref - lateral_now
        f_lateral = np.array([float(np.dot(self.wrench_lp[:3], self.perp[0])),
                              float(np.dot(self.wrench_lp[:3], self.perp[1]))])
        v_lateral = (par.kp * lateral_err
                     + par.admittance * deadband(f_lateral, par.deadband_force))
        self.lateral_err = float(np.linalg.norm(lateral_err))

        v = (v_axial * self.axis
             + v_lateral[0] * self.perp[0] + v_lateral[1] * self.perp[1])
        _p_err, r_err = pose_error(self.x_ref, pose)
        w = (par.kr * r_err
             + par.rot_admittance * deadband(self.wrench_lp[3:],
                                             par.deadband_torque))
        return (_clamp(v, par.vmax * halved), _clamp(w, par.wmax))

    def _summarise(self, t: float, axial: float):
        """Fill `record` once the skill has ended.

        Args:
            t (float): Skill time when it ended [s].
            axial (float): Final travel along the insertion axis [m].
        """
        self.record = {
            'outcome': self.outcome,
            'seated': self.outcome in GOOD_OUTCOMES,
            'duration_s': round(t, 3),
            'planned_depth_mm': round(self.depth * 1000, 2),
            'reached_depth_mm': round(axial * 1000, 2),
            'short_by_mm': round((self.depth - axial) * 1000, 2),
            'contact_depth_mm': (None if self.contact_axial is None
                                 else round(self.contact_axial * 1000, 2)),
            'search_radius_mm': round(self.search_r * 1000, 2),
            'retried': self.retried,
            'peak_force_N': round(self.peak_force, 2),
            'axis_base': [round(float(v), 5) for v in self.axis],
            'phases': self.phase_log,
            'params': dict(self.params.__dict__),
        }

    # --- --- OUTSIDE THE LOOP --- ---

    def retry(self, t: float):
        """Lift clear and search again, at the operator's request.

        Args:
            t (float): Current skill time [s], used only for the phase log.
        """
        self.outcome = ''
        self.retried = True
        self.stalled_for = 0.0
        self.backoff_from = self.axial_of(
            np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)[:3])
        self.params.budget_s += self.params.budget_s
        self._enter('backoff', t, 'operator retry')

    def hold(self):
        """Command zero velocity, e.g. while the operator decides."""
        self.rtde_c.speedL([0.0] * 6, self.params.accel, self.dt)

    def stop(self):
        """Stop the arm and put the TCP offset back the way it was."""
        try:
            self.rtde_c.speedStop()
        finally:
            if self._restore_tcp is not None:
                self.rtde_c.setTcp(self._restore_tcp)
                self._restore_tcp = None


def _clamp(vector, limit: float) -> np.ndarray:
    """Scale a vector down to a magnitude limit, keeping its direction.

    Args:
        vector (np.ndarray): The vector to limit.
        limit (float): Largest allowed magnitude.

    Returns:
        np.ndarray: The limited vector.
    """
    vector = np.asarray(vector, dtype=float)
    size = float(np.linalg.norm(vector))
    return vector * (limit / size) if size > limit else vector
