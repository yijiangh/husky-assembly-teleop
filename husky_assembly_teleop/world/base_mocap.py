"""
Judging a robot base's mocap: its fix (`mocap_check`), and whether what the relay applies to it may put it off.

Health and Robot control show the same chip, built here; `MocapWatch` says which amber chips should ask the operator.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Mapping

from ..robot_interface.mocap_definitions import OUT_OF_DATE
from ..robot_interface.mocap_relay import (FRAME_REFIT_PENDING, FRAME_UNCHECKED, FRAME_UNTRUSTED, TRUTH,
                                           AppliedCalibration, FloorCalibrationState, MocapRelayState)
from ..robot_interface.mocap_state import MocapBody
from .checks import BAD, GOOD, WARN, Check
from .mocap import mocap_check

#: Seconds a chip stays amber before it asks: a flicker does not.
ASK_AFTER = 3.0
#: Seconds a chip must stay green before what was acknowledged may ask again: a flicker of green does not count.
ASK_AGAIN_AFTER = 60.0
#: What `with_relay` adds to the worst reason when there are more.
_MORE = re.compile(r" \(\+\d+ more\)$")


def base_mocap_check(mocap_id: int | None, body: MocapBody, relay: MocapRelayState | None, now: float) -> Check:
    """A base's "mocap" chip: its fix, at least amber while its calibration or the floor frame is off (`with_relay`).

    Args:
        mocap_id: The base's rigid-body id, or None if it has none configured.
        body: The base's measured mocap state.
        relay: What the relay last announced, or None without a relay.
        now: Current ROS time, seconds.
    """
    return with_relay(mocap_check("mocap", mocap_id, body, now), applied_calibration(relay, mocap_id),
                      applied_frame(relay, mocap_id))


def applied_calibration(relay: MocapRelayState | None, mocap_id: int | None) -> AppliedCalibration | None:
    """What the relay applies to a base body (a fake Motive's from the simulated sessions); a truth or no id none."""
    if relay is None or mocap_id is None or relay.source(mocap_id) == TRUTH:
        return None
    return relay.calibrations.get(mocap_id)


def applied_frame(relay: MocapRelayState | None, mocap_id: int | None) -> FloorCalibrationState | None:
    """The floor calibration the relay applies to a base body; none to a simulator's truth (in the lab).

    None for a base without a mocap id too: never measured, it has no pose to put off, and its chip says why.
    """
    if relay is None or mocap_id is None or relay.source(mocap_id) == TRUTH:
        return None
    return relay.frame


def with_relay(mocap: Check, calibration: AppliedCalibration | None, frame: FloorCalibrationState | None) -> Check:
    """The base's mocap chip, at least amber while its calibration or the floor frame is off; the worst reason on hover.

    Worst first: a lost fix, then poses off the floor frame, then a calibration for an older body, then an
    imprecise fix; the others count as "+N more".

    Args:
        mocap: The fix's chip (`mocap_check`).
        calibration: The base calibration the relay applies, or None; built for an earlier body definition, it is off
            by a few mm.
        frame: The floor calibration the relay applies, or None; off while its poses may be (`poses_off`).
    """
    problems = [why for why in (frame_problem(frame), calibration_problem(calibration)) if why]
    if not problems:
        return mocap
    if mocap.level == BAD:
        problems.insert(0, mocap.detail)
    elif mocap.level == WARN:
        problems.append(mocap.detail)
    more = f" (+{len(problems) - 1} more)" if len(problems) > 1 else ""
    return Check(mocap.label, max(mocap.level, WARN), problems[0] + more)


def calibration_problem(calibration: AppliedCalibration | None) -> str:
    """Why the base calibration is off: made for an earlier rigid body; "" when it is not."""
    if calibration is None or calibration.definition_status != OUT_OF_DATE:
        return ""
    return "base calibration made for an older Motive rigid body: a few mm off. Calibrate the base again."


def frame_problem(frame: FloorCalibrationState | None) -> str:
    """Why poses may be off the floor calibration's frame (`poses_off`); "" when they are not."""
    if frame is None or not frame.poses_off:
        return ""
    if frame.error and not frame.floor_calibration:
        return "relay could not read its floor calibration: see Mocap probe"
    if frame.status == FRAME_UNCHECKED:
        return "floor frame not checked yet (no anchors or truss cameras seen)"
    if frame.status in (FRAME_REFIT_PENDING, FRAME_UNTRUSTED):
        return "Motive's frame moved: poses off until Use refit (Mocap probe → Floor calibration)"
    return "a newer floor calibration waits: Apply it in Mocap probe → Floor calibration"


@dataclass
class _Watched:
    """One robot's chip as `MocapWatch` follows it: since when amber, since when green, the reasons acknowledged."""

    amber_since: float | None = None
    green_since: float | None = None
    acknowledged: set[str] = field(default_factory=set)


class MocapWatch:
    """Which amber mocap chips should ask the operator: amber for ASK_AFTER, for a reason not acknowledged yet.

    A reason acknowledged stays quiet until the chip was green for ASK_AGAIN_AFTER; another reason asks at once.
    """

    def __init__(self):
        """Nothing watched yet."""
        self._robots: dict[str, _Watched] = {}
        self._asking: dict[str, str] = {}

    def update(self, chips: Mapping[str, Check], now: float) -> dict[str, str]:
        """Follow this tick's chips and return who asks now.

        Args:
            chips: Each robot's mocap chip, by serial; a robot without a rigid body is left out by the caller.
            now: Monotonic time, seconds.

        Returns:
            dict[str, str]: Why, without "(+N more)", by serial.
        """
        self._asking = {}
        for serial, chip in chips.items():
            watched = self._robots.setdefault(serial, _Watched())
            if chip.level != WARN:
                watched.amber_since = None
            elif watched.amber_since is None:
                watched.amber_since = now
            if chip.level != GOOD:
                watched.green_since = None
            elif watched.green_since is None:
                watched.green_since = now
            if watched.green_since is not None and now - watched.green_since >= ASK_AGAIN_AFTER:
                watched.acknowledged.clear()
            reason = _MORE.sub("", chip.detail)
            if (watched.amber_since is not None and now - watched.amber_since >= ASK_AFTER
                    and reason not in watched.acknowledged):
                self._asking[serial] = reason
        return dict(self._asking)

    def acknowledge(self) -> None:
        """Keep quiet about what asks now."""
        for serial, reason in self._asking.items():
            self._robots[serial].acknowledged.add(reason)
        self._asking = {}
