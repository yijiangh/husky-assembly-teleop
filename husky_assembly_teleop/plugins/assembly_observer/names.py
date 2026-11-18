"""
The names a recording holds that the Rerun viewer reads: tick signals and their channels, events, log lines.

The observer builds them from here (`columns.py`, `watch.py`, `plugin.py`), and the viewer (`rerun_view.py`) and the
copy (`export.py`) match them from here.

- ! A recording keeps its names: renaming one here changes its meaning for those made before, so bump FORMAT and add
  the old names to `rerun_view.UPGRADES`. A new name needs neither.
- The last part holds names written outside the observer (the process plugin, the tick loop, the mocap bars, the
  assembly run): only the viewer and the copy follow them from here.
"""

from __future__ import annotations

import re

from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from ...plugin_api.signals import QUATERNION, XYZ

#: The recording's format, in its session description ("format"; none means 1). Bumped only when the meaning of an
#: existing name changes; new signals or events never need a bump.
FORMAT = 1


def signal(*parts: str) -> str:
    """A tick signal's name from its parts: "0806", "left_ur_arm", JOINTS make "0806_left_ur_arm_joints"."""
    return "_".join(parts)


# --- --- --- --- --- TICK SIGNALS (`columns.py`) --- --- --- --- ---

#: A robot's signals, after its short serial: signal(short, BASE_POSE).
BASE_POSE, BASE_MOCAP, FOLLOWER, ODOMETRY, TWIST_SENT, BATTERY, STREAMS = (
    "base_pose", "base_mocap", "follower", "odometry", "twist_sent", "battery", "streams")
#: An arm's signals, after its robot's short serial and its name: signal(short, arm, TCP).
JOINTS, TCP, WRENCH, WRENCH_PEAK, TARGET, TOOL = "joints", "tcp", "wrench", "wrench_peak", "target", "tool"
#: A tracked object's signals: OBJECT + its name made safe + its pose or its mocap quality, "object_B20_pose".
OBJECT, OBJECT_POSE, OBJECT_MOCAP = "object", "pose", "mocap"
#: The lab's signals: the relay's frame drift, the monitor's share of late ticks.
RELAY_DRIFT, LATE_SHARE = "relay_drift", "monitor_late_share"

#: A pose's channels.
POSE = (*XYZ, *QUATERNION)
#: An arm's joint channels, UR order; the URDF's joint is "<arm>_<channel>_joint" (`urdf_joint`).
JOINT_CHANNELS = tuple(name.removesuffix("_joint") for name in UR_JOINT_NAMES)
#: How well mocap sees a body (BASE_MOCAP, OBJECT_MOCAP): Motive's mean marker error, the markers it labeled, how many
#: of those it filled in (occluded), and the largest marker residual.
NOISE = ("marker_error", "markers", "occluded", "largest_residual")
#: A stream's channels in STREAMS, each as "<stream>_<channel>"; the streams are BASE_STREAMS, then each arm's.
STREAM = ("rate", "max_gap", "delay")
BASE_STREAMS = ("base", "mocap")
#: The follower's channels by what they mean (the viewer plots each group on its own): its progress and errors, its
#: command, the pose it steered from, its reference and lookahead.
FOLLOWER_GROUPS = {"progress": ("progress",), "errors": ("cross_track", "along_track", "yaw_error"),
                   "command": ("v", "w"), "pose": ("x", "y", "yaw"),
                   "reference": ("reference_x", "reference_y", "reference_yaw"),
                   "lookahead": ("lookahead_x", "lookahead_y")}
FOLLOWER_CHANNELS = tuple(channel for channels in FOLLOWER_GROUPS.values() for channel in channels)


def object_key(name: str) -> str:
    """A tracked object's name as its signals' stem: OBJECT, then the name with anything not a word character as "_"."""
    return signal(OBJECT, re.sub(r"[^A-Za-z0-9_]", "_", name))


def urdf_joint(arm: str, channel: str) -> str:
    """The URDF joint of an arm's joint channel: "left_ur_arm_shoulder_pan_joint"."""
    return f"{arm}_{channel}_joint"


# --- --- --- --- --- EVENTS (`watch.py`, `plugin.py`) --- --- --- --- ---

SESSION_START, SESSION_END, OBSERVER_ERROR = "session start", "session end", "observer error"
#: Every state at a moment (`Watcher.snapshot`): a robot's, the relay's, the tracked objects'.
STATE, RELAY_STATE, OBJECTS_STATE = "state", "relay state", "objects state"
STATES = (STATE, RELAY_STATE, OBJECTS_STATE)
#: A log line of any plugin, and lines pushed out of the console before the observer read them.
LOG, LOG_LINES_LOST = "log", "log lines lost"

#: A robot's status keys told as "<key>" events with a value (a lease change is LEASE_TAKEN or LEASE_RELEASED).
LEASE, SOFT_STOP, ESTOP, FOLLOWER_STATE = "lease", "soft stop", "e-stop", "follower"
#: A robot's status key: "real", "sim", or "sim: <parts>" when only some of its base and arms are simulated.
ENVIRONMENT = "environment"
LEASE_TAKEN, LEASE_RELEASED = "lease taken", "lease released"
#: The follower's value while it follows a path.
FOLLOWING = "following"
#: Status keys after an arm's or a controller manager's name: "<arm> problem", "<arm> controller switch error".
PROBLEM, STOP_REASON, CONTROLLER, SWITCH_ERROR = "problem", "stop reason", "controller", "switch error"

#: A motion starting and ending: "<part> moving", "<part> still", the part "base" or "arm" (the arm's name in "arm").
MOVING, STILL = "moving", "still"
ARM_MOVING, ARM_STILL, BASE_MOVING, BASE_STILL = "arm moving", "arm still", "base moving", "base still"
#: A good-or-bad state told once it held: "<stream> stream" (BASE_STREAMS, then each arm), the mocap tracking of the
#: base or of an object; value LOST or BACK.
STREAM_EVENT, TRACKING, LOST, BACK = "stream", "tracking", "lost", "back"
MOCAP_TRACKING, OBJECT_TRACKING = "mocap tracking", "object mocap tracking"
OBJECT_TRACKED, OBJECT_UNTRACKED = "object tracked from now on", "object no longer tracked"
#: The relay's changes: "relay <its key>", e.g. "relay frame status", "relay drift".
RELAY, FRAME_STATUS, DRIFT = "relay", "frame status", "drift"

#: Commands sent: a path (a stop is an empty path), a trajectory or the braking one of a hold, a wrench, a tool's.
PATH_SENT, PATH_STOP_SENT, TRAJECTORY_SENT, HOLD_SENT = "path sent", "path stop sent", "trajectory sent", "hold sent"
WRENCH_SENT, TOOL_COMMAND_SENT, FORCE_SENSOR_ZEROED = "wrench sent", "tool command sent", "force sensor zeroed"
CLEARANCE_TRIP = "clearance trip"


def stream_event(stream: str) -> str:
    """A stream's running-or-lost event: "base stream", "left_ur_arm stream"."""
    return f"{stream} {STREAM_EVENT}"


def relay_event(key: str) -> str:
    """A relay state's change event: "relay frame status"."""
    return f"{RELAY} {key}"


# --- --- --- --- --- WRITTEN OUTSIDE THE OBSERVER --- --- --- --- ---

#: The process plugin's log line naming the process folder it loaded (its runs are under it), from source PROCESS.
PROCESS = "process"
LOADED = re.compile(r"loaded (/.+?) in \d")
#: The tick loop's warning of a late tick (`plugin_api/concurrency.py`).
LATE_TICK = re.compile(r"tick \d+ started [\d.]+ ms late")
#: The mocap_bars plugin's scene bodies, each a bar where its raw markers put it: "mocap_bars/B20" is "bars/B20"
#: (`plugins/mocap_bars.py`); recorded as objects named BAR_OBJECT + "B20". ! Older sessions' "fitted_bar_B20" objects
#: are Motive's bar assets, whose frames sit anywhere on the bar: never drawn as bars.
BAR_BODY, BAR_OBJECT, BAR_PREFIX = "mocap_bars/", "mocap_bars_", "bars/"
#: An assembly run's event (`shared/runs/runs.py`) shown next to the observer's as RUN + its event, "run exec".
RUN = "run "
RUN_EXEC, RUN_SERVO, RUN_PLAN, RUN_MARK_DONE, RUN_CANCEL, RUN_SOFT_STOP = (
    "exec", "servo", "plan", "mark_done", "cancel", "soft_stop")
RUN_PROMPT, RUN_CONFIRM, RUN_CHECK_GOAL, RUN_GUIDE = "prompt", "confirm", "check_goal", "guide"
RUN_FORGET, RUN_REOPEN, RUN_NEW, RUN_CONTINUE, RUN_AS_BUILT, RUN_SESSION = (
    "forget", "reopen", "new_run", "continue", "as_built", "session")
