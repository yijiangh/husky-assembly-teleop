"""
A timed base path, the RRT's turn-drive-turn "steer", arcs for its corners, and where a skid steer really drives.

Pure math, safe on any thread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np
from crl_husky.follower_path import SAME_SPOT
from crl_husky.pure_pursuit import PurePursuitParams, turning_centre_poses

from ..planning.path import TimedPath

#: Planning speeds; low, since the onboard follower sets its own speed.
MAX_LINEAR_SPEED = 0.2   # m/s
MAX_ANGULAR_SPEED = 0.3  # rad/s

#: Closer than this counts as already there; the move only turns.
POSITION_TOLERANCE = 0.005  # m
#: Spacing of the poses along an arc, metres.
ARC_STEP = 0.05
#: Drive after a turn on the spot over which the follower brings the base back onto the path, metres.
# ? A guess of a few lookaheads of the pure pursuit (0.2 m), not measured.
SETTLE_DISTANCE = 0.5


@dataclass(frozen=True)
class BasePath(TimedPath):
    """A timed path of the base on the floor.

    `points` are (x, y, yaw) waypoints, metres and radians, world frame, shape (N, 3).
    Yaw is unwrapped so blending never turns the long way.
    """

    @property
    def length(self) -> float:
        """float: Distance driven, metres, turns not counted."""
        return float(np.sum(np.linalg.norm(np.diff(self.points[:, :2], axis=0), axis=1)))


def steer(q1, q2) -> list[tuple[float, float, float]]:
    """The corner poses of the move from `q1` to `q2`: turn, drive forwards or backwards (less turning), turn.

    ! Yaws continue from `q1` and may lie outside (-pi, pi]: the last pose has `q2`'s heading, not its yaw number.

    Args:
        q1: (x, y, yaw) to start from, metres and radians.
        q2: (x, y, yaw) to end at.

    Returns:
        list[tuple[float, float, float]]: One to three poses after `q1`, ending at `q2`.
    """
    x0, y0, yaw0 = (float(value) for value in q1)
    x1, y1, yaw1 = (float(value) for value in q2)
    corners = []
    yaw = yaw0
    if math.hypot(x1 - x0, y1 - y0) > POSITION_TOLERANCE:
        heading = math.atan2(y1 - y0, x1 - x0)
        forward = abs(_angle(heading - yaw0)) + abs(_angle(yaw1 - heading))
        reverse = abs(_angle(heading + math.pi - yaw0)) + abs(_angle(yaw1 - heading - math.pi))
        yaw = yaw0 + _angle((heading if forward <= reverse else heading + math.pi) - yaw0)
        corners += [(x0, y0, yaw), (x1, y1, yaw)]
    corners.append((x1, y1, yaw + _angle(yaw1 - yaw)))
    return corners


def steer_cost(q1, q2, rotation_weight: float = 0.3) -> float:
    """What the move from `q1` to `q2` costs: metres driven plus weighted radians turned.

    Args:
        q1: (x, y, yaw) start.
        q2: (x, y, yaw) end.
        rotation_weight: Metres one radian of turning is worth.

    Returns:
        float: The cost.
    """
    cost, last = 0.0, tuple(float(value) for value in q1)
    for corner in steer(q1, q2):
        cost += math.hypot(corner[0] - last[0], corner[1] - last[1]) + rotation_weight * abs(corner[2] - last[2])
        last = corner
    return cost


def steer_points(q1, q2, position_step: float, yaw_step: float) -> list[tuple[float, float, float]]:
    """The move from `q1` to `q2` in small steps, for collision checking.

    Args:
        q1: (x, y, yaw) start; not included.
        q2: (x, y, yaw) end; included, exactly as given.
        position_step: Largest step while driving, metres.
        yaw_step: Largest step while turning, radians.

    Returns:
        list[tuple[float, float, float]]: Poses after `q1`, ending with `q2`.
    """
    points = densify([q1] + steer(q1, q2), position_step, yaw_step)
    # * End exactly on `q2` so the RRT's connected nodes compare equal.
    points[-1] = tuple(float(value) for value in q2)
    return points


def densify(poses, position_step: float, yaw_step: float) -> list[tuple[float, float, float]]:
    """The path through `poses` in small steps, each a straight blend between two poses.

    Args:
        poses: (x, y, yaw) waypoints, yaw unwrapped.
        position_step: Largest step while driving, metres.
        yaw_step: Largest step while turning, radians.

    Returns:
        list[tuple[float, float, float]]: Poses after the first, ending with the last.
    """
    points, last = [], np.array(poses[0], dtype=float)
    for pose in poses[1:]:
        pose = np.array(pose, dtype=float)
        count = max(int(math.ceil(max(np.linalg.norm(pose[:2] - last[:2]) / position_step,
                                      abs(pose[2] - last[2]) / yaw_step))), 1)
        points += [tuple(last + (pose - last) * (i / count)) for i in range(1, count + 1)]
        last = pose
    return points


def driven(poses, x_icr: float, x_model: float, position_step: float,
           yaw_step: float) -> list[tuple[float, float, float]]:
    """Where the base really goes when the follower, assuming xICR `x_model`, drives `poses` and the true one is `x_icr`.

    xICR as in crl_husky's base models: how far base_footprint lies ahead of the turning centre (negative: behind).

    - The follower steers the point it takes for the turning centre along the path, cut as crl_husky `make_path` does:
      a turn on the spot becomes a turn about that point, a short drive and a turn.
    - Where the true centre lies `d` ahead of that point, a turn on the spot swings the point around it, and the drive
      after it brings the point back onto the path over SETTLE_DISTANCE; on a curve the body points `atan(d / R)` off.

    Args:
        poses: (x, y, yaw) waypoints as sent to the follower, yaw unwrapped.
        x_icr: The robot's true xICR, metres.
        x_model: The xICR the follower assumes (its model), metres; 0 for "ideal".
        position_step: Largest step while driving, metres.
        yaw_step: Largest step while turning, radians.

    Returns:
        list[tuple[float, float, float]]: Where base_footprint really is, in small steps after the first pose.
    """
    tracked_ahead, d = -x_model, x_model - x_icr
    tracked = list(zip(*turning_centre_poses(*np.array(poses, dtype=float).T, tracked_ahead,
                                             PurePursuitParams().settle_tolerance))) if tracked_ahead else poses
    out = []
    offset = np.zeros(2)  # how far the tracked point is off its path
    turn = None           # yaw and offset where the current turn on the spot began
    settle = None         # offset where the current drive began, and the distance driven since
    last = tracked[0]
    for pose in densify(tracked, position_step, yaw_step):
        dx, dy = pose[0] - last[0], pose[1] - last[1]
        ds = math.hypot(dx, dy)
        if ds < SAME_SPOT:
            if turn is None:
                turn, settle = (last[2], offset), None
            offset = turn[1] + d * (_heading(turn[0]) - _heading(pose[2]))
            yaw = pose[2]
        else:
            if settle is None:
                turn, settle = None, [offset, 0.0]
            settle[1] += ds
            offset = settle[0] * max(0.0, 1.0 - settle[1] / SETTLE_DISTANCE)
            # Signed: backwards the base drives against its heading.
            along = ds if math.cos(math.atan2(dy, dx) - pose[2]) >= 0.0 else -ds
            yaw = pose[2] + math.atan(d * (pose[2] - last[2]) / along)
        x, y = np.array(pose[:2]) + offset - tracked_ahead * _heading(yaw)
        out.append((float(x), float(y), yaw))
        last = pose
    return out


def move_margins(start, points, icr: float) -> list[float]:
    """The extra collision margin of each pose of one move among others, to cover `driven` in the worst case.

    The moves joined at either end are unknown, so every turn on the spot counts as a full swing (`2 * icr`), and the
    base may be that far off at both ends, back on the path SETTLE_DISTANCE from them.

    Args:
        start: (x, y, yaw) where the move starts.
        points: The poses after it, in small steps (`steer_points`).
        icr: How far the turning centre may be from base_footprint, metres.

    Returns:
        list[float]: Per pose, the extra margin, metres.
    """
    xy = np.array([start[:2]] + [point[:2] for point in points], dtype=float)
    distance = np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))
    return [2.0 * icr * max(0.0, 1.0 - min(d, distance[-1] - d) / SETTLE_DISTANCE) for d in distance]


def round_corners(poses, radii, clear: Callable[[list], bool]) -> list[tuple[float, float, float]]:
    """Replace each turn on the spot between two drives in the same direction by the widest arc that fits and is clear.

    Args:
        poses: (x, y, yaw) waypoints, yaw unwrapped; between two in a row the base turns on the spot or drives straight.
        radii: Arc radii to try, widest first, metres.
        clear: Whether some waypoints are free to drive; gets those from the corner before the arc to the one after.

    Returns:
        list[tuple[float, float, float]]: The waypoints, arcs sampled every ARC_STEP, yaw unwrapped.
    """
    # Runs of waypoints on one spot; straight drives join them.
    spots: list[list[tuple[float, float, float]]] = []
    for pose in poses:
        pose = tuple(float(value) for value in pose)
        if spots and math.hypot(pose[0] - spots[-1][-1][0], pose[1] - spots[-1][-1][1]) < POSITION_TOLERANCE:
            spots[-1].append(pose)
        else:
            spots.append([pose])

    for k in range(1, len(spots) - 1):
        before, here, after = spots[k - 1][-1], spots[k][0], spots[k + 1][0]
        into, out = np.subtract(here[:2], before[:2]), np.subtract(after[:2], here[:2])
        heading_in, heading_out = math.atan2(into[1], into[0]), math.atan2(out[1], out[0])
        backwards = math.cos(heading_in - here[2]) < 0.0
        if backwards != (math.cos(heading_out - spots[k][-1][2]) < 0.0):
            continue  # switching between forwards and backwards: keep the turn on the spot
        angle = _angle(heading_out - heading_in)
        # * The drive in is final (its corner before is done); share the drive out with the corner after.
        room_in = float(np.linalg.norm(into))
        room_out = float(np.linalg.norm(out)) / (1.0 if k + 1 == len(spots) - 1 else 2.0)
        for radius in radii:
            if radius * math.tan(abs(angle) / 2.0) > min(room_in, room_out):
                continue
            arc = _arc(here, heading_in, angle, radius)
            if arc is None:
                break
            # The drive out keeps its heading: move the rest by whole turns so yaw stays unwrapped.
            shift = 2.0 * math.pi * round((arc[-1][2] - spots[k][-1][2]) / (2.0 * math.pi))
            if clear(spots[k - 1] + arc + [(after[0], after[1], after[2] + shift)]):
                spots[k] = arc
                spots[k + 1:] = [[(x, y, yaw + shift) for x, y, yaw in spot] for spot in spots[k + 1:]]
                break
    return [pose for spot in spots for pose in spot]


def _arc(corner, heading: float, angle: float, radius: float) -> list[tuple[float, float, float]] | None:
    """The arc that rounds `corner`, from the drive in (`heading`) to the drive out (turned by `angle`).

    Args:
        corner: (x, y, yaw) of the turn on the spot it replaces; yaw is the drive in's.
        heading: Direction of travel into the corner, radians.
        angle: Turn of the direction of travel, radians, in (-pi, pi].
        radius: Arc radius, metres.

    Returns:
        list[tuple[float, float, float]] | None: Poses every ARC_STEP from the arc's start to its end, or None for a
        U-turn, which no arc rounds.
    """
    if abs(angle) > math.pi - 1e-3:
        return None
    side = 1.0 if angle >= 0.0 else -1.0
    along = np.array([math.cos(heading), math.sin(heading)])
    start = np.array(corner[:2]) - radius * math.tan(abs(angle) / 2.0) * along
    centre = start + side * radius * np.array([-math.sin(heading), math.cos(heading)])
    count = max(int(math.ceil(radius * abs(angle) / ARC_STEP)), 1)
    poses = []
    for i in range(count + 1):
        direction = heading + angle * i / count
        x, y = centre - side * radius * np.array([-math.sin(direction), math.cos(direction)])
        poses.append((float(x), float(y), corner[2] + angle * i / count))
    return poses


def timed_path(poses, linear_speed: float = MAX_LINEAR_SPEED, angular_speed: float = MAX_ANGULAR_SPEED) -> BasePath:
    """Time a list of poses, each leg by its slower part (drive or turn); legs that go nowhere are dropped.

    Args:
        poses: (x, y, yaw) waypoints, the first being where the base starts.
        linear_speed: Driving speed, m/s.
        angular_speed: Turning speed, rad/s.

    Returns:
        BasePath: The timed path.
    """
    poses = np.array(poses, dtype=float)
    poses[:, 2] = np.unwrap(poses[:, 2])
    times, kept = [0.0], [poses[0]]
    for pose in poses[1:]:
        last = kept[-1]
        seconds = max(math.hypot(*(pose[:2] - last[:2])) / linear_speed, abs(pose[2] - last[2]) / angular_speed)
        if seconds > 1e-6:
            times.append(times[-1] + seconds)
            kept.append(pose)
    return BasePath(times=np.array(times), points=np.array(kept))


def _angle(angle: float) -> float:
    """An angle folded into (-pi, pi]: the shortest turn to the same heading."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _heading(yaw: float) -> np.ndarray:
    """The unit vector pointing along `yaw`."""
    return np.array([math.cos(yaw), math.sin(yaw)])
