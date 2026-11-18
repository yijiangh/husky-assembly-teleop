"""
Which joints on a bar are in the wrong place, from distances measured between joints.

A reading between a left and a right joint shows the right joint's error along the bar minus the left joint's. The
solver repeatedly takes the one joint whose move removes the most error against all the joints it was read with, until
the readings are explained, so several wrong joints are found one after the other. When two joints explain the readings
equally well it says so and asks for the reading that tells them apart. An error too small to pin on one joint is
reported as such and never as a nudge.

Pure, numpy only.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from itertools import combinations
from typing import Mapping, Sequence

import numpy as np

#: How far a reading may be from what explains it, mm, by how finely readings are taken (mm).
READING_LIMIT = {1.0: 1.0, 0.5: 0.75}
#: The smallest estimated error pinned on one joint, mm, by how finely readings are taken (mm). True errors from about
#: 2 mm (1 mm readings) or 1.5 mm (0.5 mm readings) are found reliably; their estimates scatter down to these values.
SMALLEST_NUDGE = {1.0: 1.5, 0.5: 1.25}
#: How precisely the jig places a joint, mm: a joint estimated within this is never called wrong.
JIG_PRECISION = 1.5
#: A joint this many readings agree on is named from just over the tolerance: so many readings pin its error down.
WELL_SUPPORTED = 3
#: An error from this size needs no further support once a closed triangle shows it, mm.
CLEAR_ERROR = 5.0
#: How far three readings around a closed loop may miss closing, mm, by how finely readings are taken (mm); longer loops
#: may miss by more, with the square root of their length. Joints in the wrong place can never make a loop miss.
LOOP_LIMIT = {1.0: 2.5, 0.5: 2.0}
#: Two explanations must predict a reading this far apart for that reading to tell them apart, mm.
TELLS_APART = 0.9
#: Another joint's move rivals the best one when it removes this share of the error, and the best move explains away
#: this share of what it would have removed: the two then explain the same readings.
EQUALLY_GOOD, EXPLAINED_AWAY = 0.6, 0.6

NO_READINGS, CONTRADICTS, FINE, WRONG, EITHER, UNDECIDED, TOO_SMALL, SHARED = (
    "no readings", "contradicts", "fine", "wrong", "either", "undecided", "too small", "shared")


@dataclass(frozen=True)
class Reading:
    """A distance measured between two joints.

    Attributes:
        first: One joint's id.
        second: The other's, in any order.
        error: Measured minus design, mm.
    """

    first: str
    second: str
    error: float


@dataclass(frozen=True)
class Candidate:
    """One explanation of the readings.

    Attributes:
        errors: Each joint it calls wrong and how far that joint is along the bar from its design place, mm.
    """

    errors: tuple[tuple[str, float], ...]

    @property
    def joints(self) -> tuple[str, ...]:
        """tuple[str, ...]: The joints it calls wrong."""
        return tuple(joint for joint, _error in self.errors)


@dataclass(frozen=True)
class Verdict:
    """What the readings of one bar say.

    Attributes:
        status: NO_READINGS, CONTRADICTS, FINE, WRONG, EITHER, UNDECIDED, TOO_SMALL or SHARED: a reading is out of
            tolerance while no joint on its own is. FINE means every reading is within tolerance.
        candidates: The explanations left, the best first.
        confirmed: WRONG only: every wrong joint is in a closed triangle with two joints that agree with each other.
        suspect: CONTRADICTS only: the reading to take again, or None when no single reading is to blame.
        next_pair: The two joints to measure next, or None.
        can_settle: Whether `next_pair` can tell the candidates apart at this reading resolution.
        unchecked: The joints without a reading.
        fixes: SHARED and TOO_SMALL: single moves, to the nearest 0.5 mm, that bring every reading within tolerance.
        loop: CONTRADICTS only: the joints around a loop of readings that does not close, in order, or empty.
        closure: How far that loop misses closing, mm.
        suspects: The readings of the loop that could be the wrong one, each as its two joints.
    """

    status: str
    candidates: tuple[Candidate, ...] = ()
    confirmed: bool = False
    suspect: tuple[str, str] | None = None
    next_pair: tuple[str, str] | None = None
    can_settle: bool = True
    unchecked: tuple[str, ...] = ()
    fixes: tuple[Candidate, ...] = ()
    loop: tuple[str, ...] = ()
    closure: float = 0.0
    suspects: tuple[tuple[str, str], ...] = ()


def _loop_through(reading: tuple[str, str, float],
                  readings: list[tuple[str, str, float]]) -> tuple[tuple[str, ...], float, list] | None:
    """The shortest loop through a reading: its joints in order, how far it misses closing and its readings."""
    left, right, error = reading
    steps: dict[str, list] = {}
    for other in readings:
        if other is not reading:
            steps.setdefault(other[0], []).append((other[1], other[2], other))
            steps.setdefault(other[1], []).append((other[0], -other[2], other))
    came_from: dict[str, tuple | None] = {left: None}
    queue = deque([left])
    while queue and right not in came_from:
        joint = queue.popleft()
        for following, change, used in steps.get(joint, []):
            if following not in came_from:
                came_from[following] = (joint, change, used)
                queue.append(following)
    if right not in came_from:
        return None
    path, total, used_readings, joint = [right], 0.0, [reading], right
    while came_from[joint] is not None:
        joint, change, used = came_from[joint]
        path.append(joint)
        total += change
        used_readings.append(used)
    return tuple(reversed(path)), error - total, used_readings


def _open_loops(readings: list[tuple[str, str, float]], resolution: float) -> list[tuple]:
    """The loops of readings that miss closing by more than reading errors can explain, the worst first."""
    loops = {}
    for reading in readings:
        found = _loop_through(reading, readings)
        if found is not None:
            allowed = LOOP_LIMIT[resolution] * np.sqrt(len(found[2]) / 3.0)
            if abs(found[1]) > allowed:
                loops[frozenset(item[:2] for item in found[2])] = (abs(found[1]) / allowed, found)
    return [item[1] for item in sorted(loops.values(), key=lambda item: -item[0])]


def _loop_verdict(loops: list[tuple], readings: list[tuple[str, str, float]], unread: Sequence[tuple[str, str]],
                  places: Mapping[str, tuple[float, float]], resolution: float, unchecked: tuple[str, ...]) -> Verdict:
    """The verdict on a loop that does not close: which readings could be wrong and the reading that tells which."""
    joints, closure, used = loops[0]
    used = sorted(used, key=lambda item: -abs(item[2]))
    alone = [item for item in used if not _open_loops([other for other in readings if other is not item], resolution)]
    suspects = alone or used
    pairs = [pair for pair in unread if set(pair) <= set(joints)]
    next_pair, can_tell = None, False
    if len(suspects) > 1 and pairs:
        measured = _measured(readings)
        options = []
        for suspect in suspects:
            others = [item for item in readings if item is not suspect]
            options.append(Candidate(tuple(zip(measured, map(float, _fit(others, measured)[0])))))
        next_pair, can_tell = _telling_pair(options, pairs, places)
    suspect = None if can_tell else suspects[0][:2]
    return Verdict(CONTRADICTS, suspect=suspect, next_pair=next_pair if can_tell else None, unchecked=unchecked,
                   loop=joints, closure=float(closure), suspects=tuple(item[:2] for item in suspects))


def _single_fixes(readings: list[tuple[str, str, float]], tolerance: float) -> tuple[Candidate, ...]:
    """The moves of one joint, to the nearest 0.5 mm, after which every reading is within tolerance; the best first."""
    fixes = []
    for joint in _measured(readings):
        error = round(float(_fit(readings, [joint])[0][0]) * 2.0) / 2.0
        left_over = [wanted - error * ((right == joint) - (left == joint)) for left, right, wanted in readings]
        worst = max(abs(value) for value in left_over)
        if error != 0.0 and round(worst, 6) <= tolerance:
            fixes.append((worst, joint, Candidate(((joint, error),))))
    return tuple(fix for _worst, _joint, fix in sorted(fixes, key=lambda item: item[:2]))


def _oriented(readings: Sequence[Reading], places: Mapping[str, tuple[float, float]]) -> list[tuple[str, str, float]]:
    """Each reading as (left joint, right joint, error), by the joints' places along the bar."""
    middle = {joint: low + high for joint, (low, high) in places.items()}
    return [(a, b, reading.error) if middle[a] <= middle[b] else (b, a, reading.error)
            for reading in readings for a, b in [(reading.first, reading.second)]]


def _fit(readings: list[tuple[str, str, float]], joints: Sequence[str]) -> tuple[np.ndarray, float]:
    """The errors of `joints` that best explain the readings, all other joints in place, and the largest misfit."""
    wanted = np.array([error for _left, _right, error in readings], dtype=float)
    if not joints:
        return np.zeros(0), float(np.abs(wanted).max(initial=0.0))
    column = {joint: index for index, joint in enumerate(joints)}
    matrix = np.zeros((len(readings), len(joints)))
    for row, (left, right, _error) in enumerate(readings):
        if left in column:
            matrix[row, column[left]] -= 1.0
        if right in column:
            matrix[row, column[right]] += 1.0
    errors = np.linalg.lstsq(matrix, wanted, rcond=None)[0]
    return errors, float(np.abs(matrix @ errors - wanted).max())


def _measured(readings: list[tuple[str, str, float]]) -> list[str]:
    """The joints with a reading, sorted."""
    return sorted({joint for left, right, _error in readings for joint in (left, right)})


@dataclass(frozen=True)
class _Move:
    """The best move of one joint against the readings still unexplained.

    Attributes:
        joint: The joint.
        error: How far it is from its place, by the joints agreeing on that, mm.
        agreeing: How many of its readings the move explains.
        gain: How much reading error the move removes, mm.
    """

    joint: str
    error: float
    agreeing: int
    gain: float


def _best_move(joint: str, readings: list[tuple[str, str, float]], limit: float) -> _Move | None:
    """The move of `joint` removing the most reading error, from the readings agreeing on it; None if none does."""
    signs = np.array([(right == joint) - (left == joint) for left, right, _error in readings], dtype=float)
    errors = np.array([error for _left, _right, error in readings], dtype=float)
    own = signs != 0.0
    wanted = signs[own] * errors[own]
    before = float(np.abs(wanted).sum())
    options = {float(wanted[np.abs(wanted - value) <= limit].mean()) for value in wanted}
    gain, error = max((before - float(np.abs(wanted - option).sum()), option) for option in options)
    # A large error may differ a little more between readings and still be the same error.
    agreeing = int((np.abs(wanted - error) <= max(limit, abs(error) * 0.35)).sum())
    return _Move(joint, error, agreeing, gain) if gain > 1e-6 else None


def _agreeing_moves(readings: list[tuple[str, str, float]], limit: float) -> tuple[list[_Move], list[_Move], list]:
    """Take the best joint's move again and again until the readings are explained or two joints are equally good.

    Returns:
        tuple: The moves that stood out, the equally good moves it stopped at, and the readings left after the moves.
    """
    left_over = list(readings)
    settled: list[_Move] = []
    joints = _measured(readings)
    while max(abs(error) for _left, _right, error in left_over) > limit and len(settled) < len(joints) - 1:
        moved = {move.joint for move in settled}
        options = (_best_move(joint, left_over, limit) for joint in joints if joint not in moved)
        moves = [move for move in options if move is not None]
        if not moves:
            break
        best = max(moves, key=lambda move: (move.gain, move.agreeing))
        after = _apply(best, left_over)
        rivals = [move for move in moves if move is not best and move.gain >= EQUALLY_GOOD * best.gain
                  and _explained_away(move, after, limit)]
        if rivals:
            return settled, sorted([best, *rivals], key=lambda move: -move.gain), left_over
        settled.append(best)
        left_over = after
    return settled, [], left_over


def _apply(move: _Move, readings: list[tuple[str, str, float]]) -> list[tuple[str, str, float]]:
    """The readings as they would be with the joint moved back."""
    return [(left, right, error - move.error * ((right == move.joint) - (left == move.joint)))
            for left, right, error in readings]


def _explained_away(move: _Move, after: list[tuple[str, str, float]], limit: float) -> bool:
    """Whether `move` has lost most of its point once another joint has moved: the two explain the same readings."""
    still = _best_move(move.joint, after, limit)
    return still is None or still.gain < EXPLAINED_AWAY * move.gain


def _distance(pair: tuple[str, str], places: Mapping[str, tuple[float, float]]) -> float:
    """The distance between two joints' outer edges, mm: shorter readings are easier."""
    first, second = places[pair[0]], places[pair[1]]
    return max(first[1], second[1]) - min(first[0], second[0])


def _linking_pair(readings: list[tuple[str, str, float]], unread: Sequence[tuple[str, str]], tolerance: float,
                  places: Mapping[str, tuple[float, float]]) -> tuple[str, str] | None:
    """The shortest unread pair comparing the joint most out of line with a joint that looks good, until it has support.

    Each joint's place comes from all readings together, against the middle joint: most joints are taken to be good.
    """
    measured = _measured(readings)
    fitted = _fit(readings, measured)[0]
    offsets = dict(zip(measured, map(float, fitted - np.median(fitted))))
    good = {joint for joint, offset in offsets.items() if abs(offset) <= tolerance / 2.0}
    counts = {joint: sum(joint in reading[:2] for reading in readings) for joint in measured}
    for joint in sorted(offsets, key=lambda name: -abs(offsets[name])):
        settled = _support(joint, offsets[joint], readings) >= WELL_SUPPORTED or counts[joint] > WELL_SUPPORTED
        if joint in good or settled:
            continue
        pairs = [pair for pair in unread if joint in pair]
        against_good = [pair for pair in pairs if set(pair) - {joint} <= good]
        unmeasured = [pair for pair in pairs if not set(pair) <= set(measured)]
        if against_good or unmeasured or pairs:
            return _shortest(against_good or unmeasured or pairs, places)
    return None


def _ordered_pairs(joints: Sequence[str], places: Mapping[str, tuple[float, float]]) -> list[tuple[str, str]]:
    """Every pair of `joints`, each left joint first."""
    ordered = sorted(joints, key=lambda joint: sum(places[joint]))
    return list(combinations(ordered, 2))


def _telling_pair(candidates: Sequence[Candidate], pairs: Sequence[tuple[str, str]],
                  places: Mapping[str, tuple[float, float]]) -> tuple[tuple[str, str] | None, bool]:
    """The pair telling the most candidates apart, and whether it tells any two of them apart."""
    best, best_score = None, None
    for left, right in pairs:
        predicted = [dict(candidate.errors).get(right, 0.0) - dict(candidate.errors).get(left, 0.0)
                     for candidate in candidates]
        gaps = [abs(a - b) for a, b in combinations(predicted, 2)]
        score = (sum(gap >= TELLS_APART for gap in gaps), round(min(gaps), 1), -_distance((left, right), places))
        if best_score is None or score > best_score:
            best, best_score = (left, right), score
    return best, best_score is not None and best_score[0] > 0


def _witnesses(joint: str, wrong: Sequence[str], read: set[frozenset[str]], joints: Sequence[str]) -> list[str]:
    """The joints not called wrong that have a reading with `joint`."""
    return [other for other in joints if other != joint and other not in wrong and frozenset((joint, other)) in read]


def _closed(joint: str, wrong: Sequence[str], read: set[frozenset[str]], joints: Sequence[str]) -> bool:
    """Whether `joint` has readings to two joints that also have a reading with each other."""
    return any(frozenset(pair) in read for pair in combinations(_witnesses(joint, wrong, read, joints), 2))


def _confirming_pair(candidate: Candidate, read: set[frozenset[str]], joints: Sequence[str],
                     places: Mapping[str, tuple[float, float]]) -> tuple[str, str] | None:
    """The shortest new reading that brings a wrong joint closer to a closed triangle."""
    wrong = candidate.joints
    for joint in wrong:
        if _closed(joint, wrong, read, joints):
            continue
        witnesses = _witnesses(joint, wrong, read, joints)
        if len(witnesses) >= 2:
            options = list(combinations(witnesses, 2))
        else:
            options = [(joint, other) for other in joints if other != joint and other not in wrong]
        options = [pair for pair in options if frozenset(pair) not in read]
        if options:
            return min(options, key=lambda pair: _distance(pair, places))
    return None


def _support(joint: str, error: float, readings: list[tuple[str, str, float]]) -> int:
    """How many of a joint's readings show about this error: within half of it."""
    shown = [wanted * ((right == joint) - (left == joint))
             for left, right, wanted in readings if joint in (left, right)]
    return sum(abs(value - error) <= abs(error) / 2.0 for value in shown)


def _named(joints: Sequence[str], readings: list[tuple[str, str, float]], read: set[frozenset[str]],
           all_joints: Sequence[str], places: Mapping[str, tuple[float, float]], unchecked: tuple[str, ...],
           unread: Sequence[tuple[str, str]], tolerance: float) -> Verdict:
    """The verdict naming `joints` as wrong, confirmed only when the readings leave no other explanation open.

    Two joints off by the same amount look like the joint between them being off, so a closed triangle alone is not
    enough: the named joint also needs three agreeing readings, a clear error, or to explain every other reading.
    """
    fitted, misfit = _fit(readings, joints)
    candidate = Candidate(tuple(zip(joints, map(float, fitted))))
    explained = misfit <= max(0.75, min(abs(error) for _joint, error in candidate.errors) / 2.0)
    closed = all(_closed(joint, candidate.joints, read, all_joints) for joint in candidate.joints)
    confirmed = closed and all(explained or abs(error) >= CLEAR_ERROR
                               or _support(joint, error, readings) >= WELL_SUPPORTED
                               for joint, error in candidate.errors)
    following = None
    if not confirmed:
        following = _confirming_pair(candidate, read, all_joints, places) or _linking_pair(readings, unread, tolerance,
                                                                                           places)
        # With every useful reading taken, a closed triangle is as sure as this bar can get.
        confirmed = closed and following is None
    return Verdict(WRONG, (candidate,), confirmed=confirmed, unchecked=unchecked, next_pair=following)


def solve(places: Mapping[str, tuple[float, float]], readings: Sequence[Reading], resolution: float = 1.0,
          tolerance: float = 1.0) -> Verdict:
    """Work out which joint of a bar is wrong and which reading to take next.

    Args:
        places: Every joint to consider -> its (low, high) edges along the bar as designed, mm.
        readings: The newest valid reading of each pair measured.
        resolution: How finely the tape is read, mm: 1.0 or 0.5.
        tolerance: An error up to this needs no nudge, mm.

    Returns:
        Verdict: What the readings say.
    """
    joints = sorted(places, key=lambda joint: sum(places[joint]))
    # Sorted: the loops found, and so the verdict, must not depend on the order the readings came in.
    oriented = sorted(_oriented(readings, places))
    if not oriented:
        return Verdict(NO_READINGS, unchecked=tuple(joints))
    limit = READING_LIMIT[resolution]
    measured = _measured(oriented)
    unchecked = tuple(joint for joint in joints if joint not in measured)
    read = {frozenset((left, right)) for left, right, _error in oriented}
    unread = [pair for pair in _ordered_pairs(joints, places) if frozenset(pair) not in read]

    loops = _open_loops(oriented, resolution)
    if loops:
        return _loop_verdict(loops, oriented, unread, places, resolution, unchecked)
    settled, equal, left_over = _agreeing_moves(oriented, limit)
    floor = max(SMALLEST_NUDGE[resolution], tolerance, JIG_PRECISION)
    # One reading alone names a joint only when it leaves nothing else on the bar unexplained.
    explained = not equal and max(abs(error) for _left, _right, error in left_over) <= limit
    needed = {move.joint: tolerance + 1e-6 if _support(move.joint, move.error, oriented) >= WELL_SUPPORTED else floor
              for move in settled}
    clear = [move.joint for move in settled if abs(move.error) >= needed[move.joint]
             and (move.agreeing >= 2 or explained)]
    while clear:
        # Fitted together a joint can turn out smaller than it looked on its own: then it is not named after all.
        kept = [joint for joint, error in zip(clear, _fit(oriented, clear)[0]) if abs(error) >= needed[joint]]
        if len(kept) == len(clear):
            return _named(clear, oriented, read, joints, places, unchecked, unread, tolerance)
        clear = kept

    joining = [pair for pair in unread if (pair[0] in unchecked) != (pair[1] in unchecked)]
    if max(abs(error) for _left, _right, error in oriented) <= tolerance + 1e-6:
        return Verdict(FINE, next_pair=_shortest(joining, places), unchecked=unchecked)
    moves = equal or settled
    candidates = tuple(Candidate(((move.joint, move.error),)) for move in moves)
    if len(joints) == 2:
        left, right, error = oriented[0]
        either = candidates or (Candidate(((left, -error),)), Candidate(((right, error),)))
        return Verdict(EITHER, either, unchecked=unchecked)
    # Each joint's error fitted to all readings on its own: a first estimate from a few readings can be too large.
    largest = max((abs(float(_fit(oriented, [move.joint])[0][0])) for move in moves), default=0.0)
    # Every loop closes, so what no single joint explains is spread over several joints.
    if largest <= tolerance + 1e-6 or (settled and equal) or not moves:
        doubted = {move.joint for move in moves}
        reaching = [pair for pair in joining if doubted & set(pair)] or joining
        fixes = _single_fixes(oriented, tolerance)
        linking = None if fixes else _linking_pair(oriented, unread, tolerance, places)
        return Verdict(SHARED, candidates, next_pair=linking or _shortest(reaching, places), unchecked=unchecked,
                       fixes=fixes)
    # Only rivals need a reading to tell them apart: joints found one after the other are all off.
    next_pair, can_settle = _telling_pair(candidates, unread, places) if len(equal) > 1 else (None, True)
    if largest < floor:
        if len(candidates) == 1:
            next_pair = _confirming_pair(candidates[0], read, joints, places)
        return Verdict(TOO_SMALL, candidates, next_pair=next_pair or _linking_pair(oriented, unread, tolerance, places),
                       can_settle=can_settle, unchecked=unchecked, fixes=_single_fixes(oriented, tolerance))
    return Verdict(UNDECIDED, candidates, next_pair=next_pair or _linking_pair(oriented, unread, tolerance, places),
                   can_settle=can_settle, unchecked=unchecked)


def _shortest(pairs: Sequence[tuple[str, str]], places: Mapping[str, tuple[float, float]]) -> tuple[str, str] | None:
    """The shortest of `pairs`, or None."""
    return min(pairs, key=lambda pair: _distance(pair, places), default=None)
