#!/usr/bin/env python3
"""A fake ur_rtde pair, so the engine and the insertion skill run with no robot.

* `FakeReceive`/`FakeControl` integrate whatever velocity is commanded --
* perfectly, with no dynamics -- and answer the same calls the real
* RTDEControlInterface and RTDEReceiveInterface do. That is enough to exercise
* the tracker threads, the state machine, the logging and the insertion skill.
* `VirtualMortise` adds the one thing a kinematic fake cannot give: contact.
* It models the stool joint as a rectangular tenon entering a rectangular
* blind pocket through a sharp rim, and returns the wrench that geometry would
* produce, so a search can actually find a hole and a jam can actually stall.

! A fake, not a simulator. There is no mass, no friction and no gripper
! compliance; a skill that works here still has to be proven on the robot.
! Its job is to catch logic errors -- a spiral that steps over the hole, a jam
! that is never detected, a guard that never fires -- before hardware time is
! spent on them.
"""

import time

import numpy as np


class VirtualMortise:
    """A rectangular peg-in-hole contact model, in the robot's base frame.

    The hole is a rectangular pocket of `hole` half-extents in the plane, its
    mouth at `mouth` and its floor `depth` below, entered along `axis`. The
    tenon is a rectangular prism of `peg` half-extents. Contact is a stiff
    spring:

    * a tenon whose footprint overhangs the opening rests ON the rim and goes
      no deeper -- the rim is square, so it gives no sideways guidance either;
    * a tenon that fits drops in, and from there the pocket WALLS are what
      stop it moving sideways;
    * the floor stops it at `depth`.

    Args:
        mouth (np.ndarray): Point on the rim plane the hole is centred on [m].
        axis (np.ndarray): Unit vector pointing INTO the hole.
        depth (float): Pocket depth from the rim [m].
        hole (tuple): Hole half-extents across the two lateral axes [m].
        peg (tuple): Tenon half-extents across the same axes [m].
        offset (tuple): Where the hole really is, relative to where the robot
            thinks it is -- the pickup/grasp error the search must find [m].
        stiffness (float): Contact stiffness [N/m]. Stiff enough that a 10 N
            push sinks a fraction of a millimetre, so pressing on the rim is
            never mistaken for dropping into the hole.
        rim_lip (float): Thickness of the rim [m]. Above this the part rests
            ON the rim; below it the part is inside the pocket and it is the
            walls that hold it, not the rim.
        floor_depth (float): Depth at which the pocket is blocked [m]; the
            default is the true depth, a smaller value fakes a fouled hole.
    """

    def __init__(self, mouth, axis, depth, hole=(0.016, 0.016),
                 peg=(0.015, 0.011), offset=(0.0, 0.0), stiffness=50000.0,
                 floor_depth=None, rim_lip=0.003):
        self.mouth = np.asarray(mouth, dtype=float)
        self.axis = np.asarray(axis, dtype=float)
        self.axis /= np.linalg.norm(self.axis)
        self.depth = float(depth)
        self.hole = np.asarray(hole, dtype=float)
        self.peg = np.asarray(peg, dtype=float)
        self.offset = np.asarray(offset, dtype=float)
        self.stiffness = float(stiffness)
        self.floor_depth = self.depth if floor_depth is None else float(floor_depth)
        self.rim_lip = float(rim_lip)
        seed = (np.array([0.0, 0.0, 1.0]) if abs(self.axis[2]) < 0.9
                else np.array([1.0, 0.0, 0.0]))
        u = np.cross(self.axis, seed)
        self.perp = (u / np.linalg.norm(u), np.cross(self.axis, u / np.linalg.norm(u)))

    def wrench(self, tcp_position) -> np.ndarray:
        """The wrench the joint puts on the tool at a given TCP position.

        Args:
            tcp_position (np.ndarray): The TCP in the base frame [m].

        Returns:
            np.ndarray: (6,) wrench [fx, fy, fz, tx, ty, tz] in the base
            frame. Torques are always zero -- the fake tool is a point.
        """
        delta = np.asarray(tcp_position, dtype=float) - self.mouth
        axial = float(np.dot(delta, self.axis))
        lateral = np.array([float(np.dot(delta, self.perp[0])),
                            float(np.dot(delta, self.perp[1]))]) - self.offset
        force = np.zeros(3)
        if axial <= 0.0:
            return np.zeros(6)          # still above the rim: free space

        # How far the tenon's footprint overhangs the opening, per axis.
        over = np.abs(lateral) + self.peg - self.hole
        overhanging = bool(over.max() > 0.0)

        if overhanging and axial <= self.rim_lip:
            # ! Resting on the rim, and the rim is FLAT and sharp: it holds the
            # ! part up and does nothing else. No lateral nudge towards the
            # ! opening, because a square edge does not provide one -- the
            # ! search has to find the hole geometrically, exactly as on the
            # ! real parts, whose chamfer is far too small to steer with.
            force += -self.stiffness * axial * self.axis
        else:
            # Below the lip: the part is IN the pocket, so it is the walls
            # that constrain it sideways, not the rim underneath it.
            for k in range(2):
                if over[k] > 0.0:
                    force += (-self.stiffness * over[k] * np.sign(lateral[k])
                              * self.perp[k])
            if axial > self.floor_depth:
                force += -self.stiffness * (axial - self.floor_depth) * self.axis
        return np.concatenate([force, np.zeros(3)])


class FakeReceive:
    """Stands in for RTDEReceiveInterface, reading the shared fake state.

    Args:
        state (FakeState): The state this pair shares.
    """

    def __init__(self, state):
        self.state = state

    def getActualQ(self):
        """Joint positions [rad]."""
        self.state.integrate()
        return list(self.state.q)

    def getActualQd(self):
        """Joint velocities [rad/s]."""
        return list(self.state.qd)

    def getActualTCPPose(self):
        """TCP pose [x, y, z, rx, ry, rz] in the base frame."""
        self.state.integrate()
        return list(self.state.pose)

    def getActualTCPSpeed(self):
        """TCP velocity [m/s, rad/s] in the base frame."""
        return list(self.state.tcp_speed)

    def getActualTCPForce(self):
        """Wrench at the TCP in the base frame, minus whatever was zeroed."""
        self.state.integrate()
        return list(self.state.wrench() - self.state.ft_bias)


class FakeControl:
    """Stands in for RTDEControlInterface, driving the shared fake state.

    Args:
        state (FakeState): The state this pair shares.
        frequency (float): Control rate the fake clock advances at [Hz].
    """

    def __init__(self, state, frequency=125.0):
        self.state = state
        self.dt = 1.0 / float(frequency)

    # --- timing ---
    def initPeriod(self):
        """Start of a control period."""
        return time.perf_counter() if self.state.realtime else self.state.now

    def waitPeriod(self, t0):
        """Finish a control period: really sleep, or advance the fake clock.

        Args:
            t0: Whatever initPeriod returned.
        """
        if self.state.realtime:
            remaining = self.dt - (time.perf_counter() - t0)
            if remaining > 0:
                time.sleep(remaining)
            self.state.integrate()
        else:
            self.state.advance(self.dt)

    # --- motion ---
    def speedJ(self, qd, acceleration=0.5, time_=0.0):
        """Command a joint velocity [rad/s]."""
        self.state.set_qd(np.asarray(qd, dtype=float))
        return True

    def speedL(self, xd, acceleration=0.25, time_=0.0):
        """Command a TCP velocity [m/s, rad/s] in the base frame."""
        self.state.set_tcp_speed(np.asarray(xd, dtype=float))
        return True

    def speedStop(self, a=10.0):
        """Stop all commanded motion."""
        self.state.set_qd(np.zeros(6))
        self.state.set_tcp_speed(np.zeros(6))
        return True

    def servoStop(self, a=10.0):
        """Stop servoing (same effect here as speedStop)."""
        return self.speedStop(a)

    def moveJ(self, q, speed=1.05, acceleration=1.4, asynchronous=False):
        """Jump to a joint configuration (instant in the fake)."""
        self.state.integrate()
        self.state.q = np.asarray(q, dtype=float).copy()
        self.state.qd = np.zeros(6)
        return True

    def stopScript(self):
        """No script to stop."""
        return True

    # --- tool and sensing ---
    def zeroFtSensor(self):
        """Take the current wrench as the new zero."""
        self.state.ft_bias = self.state.wrench().copy()
        return True

    def setTcp(self, offset):
        """Set the tool offset [x, y, z, rx, ry, rz]."""
        self.state.tcp_offset = list(offset)
        return True

    def getTCPOffset(self):
        """The tool offset in force."""
        return list(self.state.tcp_offset)

    def setPayload(self, mass, cog):
        """Accepted and ignored."""
        return True

    def getForwardKinematics(self, q=None, tcp_offset=None):
        """TCP pose of a configuration, through the fake's own kinematics.

        Args:
            q (list): Joint values, or None for the current ones.
            tcp_offset (list): Tool offset, or None for the active one.

        Returns:
            list: The pose [x, y, z, rx, ry, rz].
        """
        return list(self.state.fk(
            self.state.q if q is None else np.asarray(q, dtype=float)))

    def isProtectiveStopped(self):
        """The fake never protective-stops."""
        return False


class FakeState:
    """The world the fake pair shares: one arm, one clock, optional contact.

    Kinematics are deliberately trivial and invertible -- the TCP position is
    an affine map of the first three joints -- so a test can command a joint
    velocity or a tool velocity and reason about the result exactly.

    Args:
        q0 (np.ndarray): Starting joint configuration [rad].
        origin (np.ndarray): TCP position at q = 0 [m].
        scale (float): Metres of TCP travel per radian of the mapped joints.
        contact (VirtualMortise): The joint being assembled, or None for free
            space.
        realtime (bool): False runs on a virtual clock that only advances when
            `waitPeriod` is called, so a test finishes as fast as the CPU
            allows. True runs on the wall clock and `waitPeriod` really
            sleeps -- which is what the engine needs, because its tracker
            threads time themselves against `time.monotonic`.
    """

    def __init__(self, q0=None, origin=(0.5, 0.0, 0.5), scale=0.25,
                 contact=None, realtime=False):
        self.q = (np.zeros(6) if q0 is None
                  else np.asarray(q0, dtype=float).copy())
        self.qd = np.zeros(6)
        self.tcp_speed = np.zeros(6)
        self.origin = np.asarray(origin, dtype=float)
        self.scale = float(scale)
        self.contact = contact
        self.tcp_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ft_bias = np.zeros(6)
        self.realtime = bool(realtime)
        self.now = time.perf_counter() if self.realtime else 0.0
        self._last = self.now
        self.pose = self.fk(self.q)
        self._cart = False   # driven by speedL rather than speedJ

    def fk(self, q) -> np.ndarray:
        """TCP pose of a joint configuration.

        Args:
            q (np.ndarray): Joint values [rad].

        Returns:
            np.ndarray: (6,) pose [x, y, z, rx, ry, rz].
        """
        q = np.asarray(q, dtype=float)
        position = self.origin + self.scale * q[:3]
        return np.concatenate([position, q[3:6] * 0.1])

    def set_qd(self, qd):
        """Command joint velocity; switches the fake to joint mode."""
        self.integrate()
        self.qd = np.asarray(qd, dtype=float).copy()
        self._cart = False

    def set_tcp_speed(self, xd):
        """Command tool velocity; switches the fake to Cartesian mode."""
        self.integrate()
        self.tcp_speed = np.asarray(xd, dtype=float).copy()
        self._cart = True

    def advance(self, dt: float):
        """Move the fake clock forward.

        Args:
            dt (float): Seconds to advance (ignored on the wall clock).
        """
        if not self.realtime:
            self.now += float(dt)
        self.integrate()

    def integrate(self):
        """Apply whatever velocity was commanded up to the current time."""
        if self.realtime:
            self.now = time.perf_counter()
        dt = self.now - self._last
        if dt <= 0.0:
            return
        self._last = self.now
        if self._cart:
            self.pose = self.pose + self.tcp_speed * dt
            # Keep the joints consistent, so getActualQ still means something.
            self.q[:3] = (self.pose[:3] - self.origin) / self.scale
            self.q[3:6] = self.pose[3:] / 0.1
        else:
            self.q = self.q + self.qd * dt
            self.pose = self.fk(self.q)

    def wrench(self) -> np.ndarray:
        """The contact wrench at the current TCP, zero without a joint."""
        if self.contact is None:
            return np.zeros(6)
        return self.contact.wrench(self.pose[:3])


def make_pair(frequency=125.0, **state_kwargs) -> tuple:
    """One fake control/receive pair over a fresh state.

    Args:
        frequency (float): Control rate [Hz].
        **state_kwargs: Passed to FakeState.

    Returns:
        tuple: (FakeControl, FakeReceive, FakeState).
    """
    state = FakeState(**state_kwargs)
    return FakeControl(state, frequency), FakeReceive(state), state


def rotvec_pose(position, rotvec=(0.0, 0.0, 0.0)) -> list:
    """Build a UR pose vector from a position and a rotation vector.

    Args:
        position (tuple): [x, y, z] in metres.
        rotvec (tuple): Rotation vector [rad].

    Returns:
        list: The 6-vector pose.
    """
    return list(np.concatenate([np.asarray(position, dtype=float),
                                np.asarray(rotvec, dtype=float)]))
