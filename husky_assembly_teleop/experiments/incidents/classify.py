"""
From raw lab logs to incidents: a line's time, its kind and robot, and bursts of the same thing merged into one.

Pure text work, tested on the real line forms. What a line says is kept word for word: nothing here says why a thing
happened, and what a log's silence is taken to mean is marked `inferred`.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from datetime import datetime
from typing import Iterable, Mapping, Sequence

#: The kinds in the count matrix's order: key -> (the page's words, the chip's colour).
KINDS = {
    "estop": ("E-stop", "k-red"), "gripper": ("Gripper", "k-amber"), "crash": ("Monitor crash", "k-red"),
    "hand": ("Stopped by hand", "k-grey"), "no_answer": ("Robot not answering", "k-amber"),
    "process": ("Robot process died", "k-amber"), "step": ("Step failed", "k-blue"),
    "controller": ("Controller", "k-blue"), "plan": ("Plan failed", "k-blue"),
    "servo": ("Servo", "k-violet"), "mocap": ("MoCap", "k-violet"), "stop_all": ("Stop all", "k-grey"),
    "clearance": ("Clearance", "k-amber"), "hold": ("HOLD NOT SENT", "k-red"),
}
#: Lines of one kind and robot this close (seconds) are one burst, one incident.
BURST_GAP_S = 120.0
#: In these kinds a burst merges whatever each line's words; in the others only the same words (numbers aside) merge.
MERGE_ANY_WORDS = {"stop_all", "clearance", "no_answer", "process", "gripper", "hold", "estop", "mocap"}
#: A burst keeps this many raw lines from its start and from its end.
RAW_HEAD, RAW_TAIL = 8, 3
SERIALS = {"0804": "Alice", "0805": "Belle", "0806": "Cindy"}
NO_ROBOT = "–"
ALL_ROBOTS = "All"
MONITOR = "Monitor"

_ANSI = re.compile(r"\x1b\[[0-9;?]*[a-zA-Z]|\x1b\][^\x07]*\x07|\x1b[=>]")
_STAMPED = re.compile(r"^\[(INFO|WARN|ERROR|FATAL|DEBUG)\] \[(\d+\.\d+)\] \[([^\]]*)\]: ?(.*)$")
_PANE = re.compile(r"^\[([\w./-]+-\d+)\] \[(INFO|WARN|ERROR|FATAL|DEBUG)\] \[(\d+\.\d+)\] \[([^\]]*)\]: ?(.*)$")
_PANE_PLAIN = re.compile(r"^\[([\w./-]+-\d+)\] \[(\d+\.\d+)\] (INFO|WARN|ERROR|FATAL|DEBUG) (.*)$")
_LAUNCH = re.compile(r"^\[(INFO|WARN|ERROR)\] \[([\w./-]+-\d+)\]: (.*)$")
_SERIAL = re.compile(r"a200[-_](\d{4})")
_NAME = re.compile(r"\b(alice|belle|cindy)\b", re.IGNORECASE)
_EXEC_ON = re.compile(r"\bExec (\S+) on (\w+) \(a200-(\d{4})\)")
_NUMBER = re.compile(r"\d+(\.\d+)?")
#: A chip's change that is a stop of the arm itself, not one of its other states.
_ESTOP_CHIP = re.compile(r"\[health/chips\] .*\barm (?:ROBOT |SYSTEM |SAFEGUARD |PROTECTIVE )\w*(?: STOP)? \w+ -> red")

#: (kind, what the line must say), the first match counts; lines of `[health/chips]` and `[ticks]` never get here.
_MONITOR_RULES = (
    ("hold", re.compile(r"HOLD NOT SENT")),
    ("step", re.compile(r"\bExec \S+: (failed|refused|cancelled)\b")),
    ("gripper", re.compile(r"check FAIL grippers")),
    ("step", re.compile(r"check FAIL |the arms did NOT reach|the tools did not reach")),
    ("servo", re.compile(r"### SERVOING (stopped|ran all|cancelled)")),
    ("mocap", re.compile(r"fit FAILED|no marker sample of|mocap bar is not usable|marker snapshot", re.IGNORECASE)),
    ("stop_all", re.compile(r"SOFT STOP: (stopping every robot|Stop all|cancelled)|robot \S+: SOFT STOP: Stop all")),
    ("clearance", re.compile(r"came within \d+ cm of|cannot be checked")),
    ("no_answer", re.compile(r"not reachable|cannot (stop|resume) arms|switch_controller is not available|"
                             r"controller manager is not available")),
    ("controller", re.compile(r"not starting \w+|not sending a trajectory|not configured afresh")),
    ("plan", re.compile(r"no goal at the live base|timed out .* planner|\bno plan\b|plan failed", re.IGNORECASE)),
)
_PANE_RULES = (
    ("estop", re.compile(r"safety mode changed: (?!NORMAL)")),
    ("gripper", re.compile(r"Serial command .* failed|Unexpected response to command|"
                           r"RobotiqGripperHardwareInterface.*(Error|Failed)")),
    ("no_answer", re.compile(r"Failed to connect to robot|Lost connection to stream|Currently reconnecting to the RTDE|"
                             r"Failed to read from stream|failed to get package from RTDE|Failed to initialize RTDE|"
                             r"dashboard server|failed to get (safety|robot) mode")),
    ("process", re.compile(r"process has died")),
)
_SHELL_RULES = (
    ("no_answer", re.compile(r"ssh: connect to host .*(No route to host|Connection refused|Connection timed out)|"
                             r"Connection to \S+ closed by remote host|Broken pipe")),
)
_RELAY_RULES = (("mocap", re.compile(r"No NatNet frames")),)
#: Printed when a process ends badly, whichever monitor it was.
_DIED = re.compile(r"Segmentation fault|core dumped|X connection to \S+ broken|^Aborted|^Killed|terminate called")
#: A traceback's last line, its indented frames and source lines being cleaned of their indent like any other line.
_FINAL_ERROR = re.compile(r"^(?!File )[\w.]*(Error|Exception|Interrupt|Exit|Warning)\b")
_PROCESS_DIED = re.compile(r"process has died \[pid \d+, exit code (-?\d+), cmd '(\S+)")
#: A process stopped by a signal it was sent, the launch being stopped: rviz has no display on a robot either.
_DIED_ON_PURPOSE = re.compile(r"exit code -(2|9|15)\b|\[rviz2?-")


@dataclass
class LogLine:
    """One line of a log, its time the last stamp seen when it has none of its own."""

    t: float
    level: str
    text: str
    raw: str
    source: str
    stamped: bool = True


@dataclass
class Incident:
    """One thing that went wrong, or a burst of the same thing."""

    kind: str
    robot: str
    what: str
    start: float
    end: float
    count: int = 1
    raw: list[str] = field(default_factory=list)
    source: str = ""
    inferred: str = ""
    #: How many lines in the middle of a long burst were left out of `raw`.
    dropped: int = 0

    @property
    def day(self) -> str:
        """str: The local day it started, YYYYMMDD."""
        return datetime.fromtimestamp(self.start).strftime("%Y%m%d")


def clean(line: str) -> str:
    """A terminal line without its colours and control characters."""
    return _ANSI.sub("", line).replace("\r", "").strip()


def read_log(text: str, source: str, started: float = 0.0) -> list[LogLine]:
    """Every non-empty line of a log with its time; a line without a stamp takes the last stamp before it.

    Args:
        text: The whole log.
        source: The file's name, kept on each line.
        started: The time of lines before the first stamp, usually from the file's name.
    """
    lines, now = [], started
    for raw in text.split("\n"):
        line = clean(raw)
        if not line:
            continue
        match = _STAMPED.match(line)
        pane = _PANE.match(line) if match is None else None
        plain = _PANE_PLAIN.match(line) if match is None and pane is None else None
        if match is not None:
            now = float(match.group(2))
            lines.append(LogLine(now, match.group(1), match.group(4), line, source))
        elif pane is not None:
            now = float(pane.group(3))
            lines.append(LogLine(now, pane.group(2), f"{pane.group(4)}: {pane.group(5)}", line, source))
        elif plain is not None:
            now = float(plain.group(2))
            lines.append(LogLine(now, plain.group(3), plain.group(4), line, source))
        else:
            launch = _LAUNCH.match(line)
            lines.append(LogLine(now, launch.group(1) if launch else "", launch.group(3) if launch else line, line,
                                 source, stamped=False))
    return lines


def robot_in(text: str) -> str | None:
    """The robot a line names, by its serial or its name, or None."""
    serial = _SERIAL.search(text)
    if serial is not None and serial.group(1) in SERIALS:
        return SERIALS[serial.group(1)]
    name = _NAME.search(text)
    return name.group(1).capitalize() if name is not None else None


def classify_monitor(line: LogLine, acting: str | None = None) -> tuple[str, str] | None:
    """A stamped monitor line's (kind, robot), or None for a line that is no incident.

    Args:
        line: The line.
        acting: The robot the last Exec ran on: a step's own robot, when the line names another one in its reason.
    """
    text = line.text
    if text.startswith("[health/chips]"):
        return ("estop", robot_in(text) or NO_ROBOT) if _ESTOP_CHIP.match(text) else None
    if text.startswith("[ticks]") or "waiting for Confirm" in text:
        return None
    for kind, rule in _MONITOR_RULES:
        if rule.search(text):
            if kind == "stop_all":
                return kind, ALL_ROBOTS
            if kind in ("step", "servo", "mocap", "gripper", "hold") and acting:
                return kind, acting
            return kind, robot_in(text) or NO_ROBOT
    return None


def classify_pane(line: LogLine) -> str | None:
    """A robot pane's line's kind, or None: what the robot's own launch and the pane's ssh printed."""
    text = line.text
    if "process has died" in text and _DIED_ON_PURPOSE.search(line.raw):
        return None
    for kind, rule in (*_PANE_RULES, *_SHELL_RULES):
        if rule.search(text):
            return kind
    return None


def monitor_incidents(lines: Sequence[LogLine]) -> list[Incident]:
    """Every line of a monitor's log that is an incident, one incident per line, unmerged."""
    found, acting = [], None
    for line in lines:
        if not line.stamped:
            continue
        on = _EXEC_ON.search(line.text)
        if on is not None:
            acting = SERIALS.get(on.group(3))
        hit = classify_monitor(line, acting)
        if hit is not None:
            found.append(Incident(hit[0], hit[1], _what(line.text), line.t, line.t, raw=[line.raw], source=line.source))
    return found


def pane_incidents(lines: Sequence[LogLine], robot: str) -> list[Incident]:
    """Every line of a robot's pane that is an incident, one incident per line, unmerged.

    A connection the pane's own `shutdown` closed is left out. A line with no stamp of its own says so, as its time is
    only the last stamp before it.
    """
    found, shutdown_at = [], -10
    for number, line in enumerate(lines):
        if "shutdown now" in line.text or "sudo reboot" in line.text:
            shutdown_at = number
        kind = classify_pane(line)
        if kind is None or (kind == "no_answer" and "closed by remote host" in line.text
                            and number - shutdown_at < 5):
            continue
        found.append(Incident(kind, robot, _what(line.text), line.t, line.t, raw=[line.raw], source=line.source,
                              inferred="" if line.stamped else "time: the last stamp before this line"))
    return found


def relay_incidents(lines: Sequence[LogLine]) -> list[Incident]:
    """Every line of the MoCap relay's pane that is an incident, one incident per line, unmerged."""
    return [Incident(kind, NO_ROBOT, _what(line.text), line.t, line.t, raw=[line.raw], source=line.source)
            for line in lines for kind, rule in _RELAY_RULES if rule.search(line.text)]


def _what(text: str) -> str:
    """A line's message without the plugin's tag in front; a process's death without its long command."""
    died = _PROCESS_DIED.search(text)
    if died is not None:
        return f"{died.group(2).rsplit('/', 1)[-1]} died (exit code {died.group(1)})"
    return re.sub(r"^\[(movement_exec|robot_control|assembly_steps)\] ", "", text)


def log_ending(lines: Sequence[LogLine], still_written: bool = False) -> list[Incident]:
    """How a monitor's log says the monitor ended, and any crash on the way.

    - A traceback or a process death is a crash, unless the traceback ends in KeyboardInterrupt (a stop by hand whose
      shutdown broke).
    - A log ending on the monitor's own stopping lines is a stop by hand. These lines do not say who sent the signal.
    - A log ending on neither is a crash, inferred from its silence, unless it is still being written.

    Args:
        lines: The whole log.
        still_written: The file changed moments ago: the monitor is taken to be running.
    """
    crashes, by_hand = [], None
    for number, line in enumerate(lines):
        if line.text.startswith("Traceback (most recent call last)") and not line.stamped:
            block = _traceback(lines, number)
            if block[-1].raw.startswith("KeyboardInterrupt"):
                by_hand = block
            else:
                crashes.append(_ended("crash", block, block[-1].raw))
        elif not line.stamped and _DIED.search(line.raw):
            crashes.append(_ended("crash", [line], line.raw))
    if crashes:
        return crashes
    if by_hand is not None:
        return [_ended("hand", by_hand, "Stopped with Ctrl-C; the shutdown ended in a traceback",
                       "inferred: KeyboardInterrupt in the traceback")]
    tail = lines[-6:]
    if any("Server stopped" in line.raw or "stopping: cancelling plugin tasks" in line.text for line in tail):
        return [_ended("hand", tail, "The monitor stopped through its own shutdown lines",
                       "inferred: Ctrl-C or a terminal closing; the log does not say which")]
    if still_written or not lines:
        return []
    return [_ended("crash", tail, "The log ends with no \"Server stopped\" line",
                   "inferred from the log's end: the process may have been killed, or the PC turned off")]


def _traceback(lines: Sequence[LogLine], start: int) -> list[LogLine]:
    """A traceback's lines from its first up to its error: the first line that is not a frame or its source."""
    for number in range(start + 1, len(lines)):
        if _FINAL_ERROR.match(lines[number].raw):
            return list(lines[start:number + 1])
    return list(lines[start:])


def _ended(kind: str, lines: Sequence[LogLine], what: str, inferred: str = "") -> Incident:
    """An incident of how a log ended, at its last stamped time."""
    t = lines[-1].t
    return Incident(kind, MONITOR, what, t, t, raw=[line.raw for line in lines], source=lines[-1].source,
                    inferred=inferred)


def events_incidents(events: Iterable[Mapping], run: str) -> list[Incident]:
    """What a run's `events.jsonl` says went wrong: steps, servos, Stop all, goals with no solution.

    Args:
        events: The run's events, oldest first.
        run: The run's name, for the source.
    """
    found, acting = [], NO_ROBOT
    for event in events:
        data, name = event.get("data") or {}, event.get("event")
        when = datetime.fromisoformat(event["time"]).timestamp() if event.get("time") else 0.0
        movement = event.get("movement") or event.get("action") or ""
        if name == "check_goal" and data.get("acting"):
            acting = robot_in(str(data["acting"])) or acting
        hit = None
        if name == "exec" and data.get("result") in ("failed", "refused", "cancelled"):
            hit = ("step", f"Exec {movement}: {data['result']} ({data.get('status')})")
        elif name == "servo" and data.get("status") in ("failed", "refused", "cancelled"):
            hit = ("servo", f"Servo of {movement}: {data['status']} after {data.get('passes')} passes")
        elif name == "check_goal" and data.get("ok") is False:
            hit = ("plan", f"Check goal {movement}: {data.get('reason') or 'no goal'}")
        elif name == "soft_stop":
            hit = ("stop_all", f"Soft stop during {movement or 'a run'}")
        if hit is not None:
            robot = ALL_ROBOTS if hit[0] == "stop_all" else acting
            found.append(Incident(hit[0], robot, hit[1], when, when, raw=[json.dumps(event)[:400]],
                                  source=f"{run}/events.jsonl"))
    return found


def merge_bursts(incidents: Iterable[Incident], gap_s: float = BURST_GAP_S) -> list[Incident]:
    """Merge lines of one kind and robot close in time into one incident, with a count, a time range and their lines.

    In the kinds of `MERGE_ANY_WORDS` any lines merge; in the others only the same words, numbers aside.
    """
    merged: list[Incident] = []
    open_bursts: dict[tuple, Incident] = {}
    for item in sorted(incidents, key=lambda each: each.start):
        words = "" if item.kind in MERGE_ANY_WORDS else _NUMBER.sub("N", item.what)
        key = (item.kind, item.robot, words)
        burst = open_bursts.get(key)
        if burst is not None and item.start - burst.end <= gap_s:
            burst.end = max(burst.end, item.end)
            burst.count += item.count
            burst.raw = [*burst.raw, *item.raw]
            burst.inferred = burst.inferred or item.inferred
            _trim(burst)
        else:
            open_bursts[key] = item
            merged.append(item)
    return merged


def _trim(burst: Incident) -> None:
    """Keep a long burst's first and last raw lines, counting those left out."""
    extra = len(burst.raw) - RAW_HEAD - RAW_TAIL
    if extra > 0:
        burst.raw = [*burst.raw[:RAW_HEAD], *burst.raw[-RAW_TAIL:]]
        burst.dropped += extra


def add_events(incidents: list[Incident], from_events: Iterable[Incident], gap_s: float = 30.0) -> list[Incident]:
    """Add what the runs' events say: to the log's incident of that kind near that time, else as a new incident.

    A servo's end also joins a step's incident: the step's line already says how its servo ended.
    """
    out = list(incidents)
    for item in from_events:
        kinds = ("servo", "step") if item.kind == "servo" else (item.kind,)
        same = next((each for each in out if each.kind in kinds and each.start - gap_s <= item.start
                     <= each.end + gap_s), None)
        if same is not None:
            same.raw = [*same.raw, *item.raw]
        else:
            out.append(item)
    return sorted(out, key=lambda each: each.start)


def count_matrix(incidents: Iterable[Incident]) -> dict[tuple[str, str], dict[str, int]]:
    """How many incidents per (day, robot) and kind: {(day, robot): {kind: n}}; a burst counts once."""
    matrix: dict[tuple[str, str], dict[str, int]] = {}
    for item in incidents:
        row = matrix.setdefault((item.day, item.robot), {})
        row[item.kind] = row.get(item.kind, 0) + 1
    return matrix
