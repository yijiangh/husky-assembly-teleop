"""
The incident report's page: what the logs and the runs' events hold for a day or a date range, as one offline .html.

`read_incidents` gathers the sources through `classify.py`; `build_report` is what `scripts/incident_report.py` and the
monitor's "Build incident report" button run.
"""

from __future__ import annotations

import json
import re
from datetime import datetime, timedelta
from pathlib import Path
from typing import Mapping, Sequence

from ...shared.runs.as_built import AS_BUILT_FOLDER
from ..sheet import esc, method, page, section, table, titleblock, write
from .classify import (KINDS, Incident, add_events, count_matrix, events_incidents, log_ending, merge_bursts,
                       monitor_incidents, pane_incidents, read_log, relay_incidents)

#: The lab scripts' log folder.
DEFAULT_LOGS = Path.home() / "husky_real_lab" / "logs"
#: A monitor's log that changed this recently (seconds) is taken to be still written.
STILL_WRITTEN_S = 300.0
ROBOT_LOGS = {"alice": "Alice", "belle": "Belle", "cindy": "Cindy"}
_FILE_STAMP = re.compile(r"_(\d{8})_(\d{6})\.log$")


def days_between(first: str, last: str) -> list[str]:
    """Every day from `first` to `last`, YYYYMMDD."""
    day, end = datetime.strptime(first, "%Y%m%d"), datetime.strptime(last, "%Y%m%d")
    return [(day + timedelta(days=k)).strftime("%Y%m%d") for k in range((end - day).days + 1)]


def _started(path: Path) -> float:
    """When a log's name says it was started, or the start of its file's day when it does not say."""
    match = _FILE_STAMP.search(path.name)
    return datetime.strptime(match.group(1) + match.group(2), "%Y%m%d%H%M%S").timestamp() if match else 0.0


def logs_in_range(folder: Path, first: str, last: str) -> list[Path]:
    """The logs that may hold lines of these days: started by the last day, and last changed from the first day on."""
    start = datetime.strptime(first, "%Y%m%d").timestamp()
    end = (datetime.strptime(last, "%Y%m%d") + timedelta(days=1)).timestamp()
    return [path for path in sorted(Path(folder).glob("*.log")) if path.stat().st_mtime >= start
            and _started(path) < end]


def read_incidents(logs: Path, first: str, last: str, drive: Path | None = None, now: float | None = None) -> dict:
    """Every incident of the days from the logs and the runs' events, bursts merged, oldest first.

    Args:
        logs: The folder of `monitor*.log`, `<robot>_*.log` and `relay_*.log`.
        first: The first day, YYYYMMDD.
        last: The last day.
        drive: The Drive root, for `data_design_study/*/runs/<day>_*/events.jsonl`; None skips the runs.
        now: The time now, to tell a log still being written.

    Returns:
        dict: {incidents, monitor_logs, robot_logs, running: monitor logs still written, runs, days_without_log,
        health_lines: the chip lines left out}.
    """
    now = datetime.now().timestamp() if now is None else now
    days = days_between(first, last)
    raw: list[Incident] = []
    monitors, robots, running = [], [], []
    health = 0
    logs_days: set[str] = set()
    for path in logs_in_range(logs, first, last):
        text = path.read_text(errors="replace")
        lines = read_log(text, path.name, _started(path))
        prefix = path.name.split("_")[0].split(".")[0]
        if prefix == "monitor":
            monitors.append(path.name)
            still = now - path.stat().st_mtime < STILL_WRITTEN_S
            running += [path.name] if still else []
            raw += monitor_incidents(lines) + log_ending(lines, still)
            health += sum(line.text.startswith("[health/chips]") for line in lines
                          if _in_days(line.t, days))
            logs_days |= {_day(line.t) for line in lines if line.stamped}
        elif prefix in ROBOT_LOGS:
            robots.append(path.name)
            raw += pane_incidents(lines, ROBOT_LOGS[prefix])
        elif prefix == "relay":
            raw += relay_incidents(lines)
    merged = merge_bursts(raw)
    runs = runs_in_range(drive, days) if drive is not None else []
    for run in runs:
        merged = add_events(merged, events_incidents(_events(run), run.name))
    return {"incidents": [item for item in merged if item.day in days], "monitor_logs": monitors, "robot_logs": robots,
            "running": running, "runs": [run.name for run in runs],
            "days_without_log": [day for day in days if day not in logs_days], "health_lines": health}


def _day(t: float) -> str:
    """A time's local day, YYYYMMDD."""
    return datetime.fromtimestamp(t).strftime("%Y%m%d")


def _in_days(t: float, days: Sequence[str]) -> bool:
    """Whether a time falls on one of the days."""
    return _day(t) in days


def runs_in_range(drive: Path, days: Sequence[str]) -> list[Path]:
    """The run folders of the days, of any process, oldest first."""
    found = [run for day in days for run in Path(drive).glob(f"data_design_study/*/runs/{day}_*") if run.is_dir()]
    return sorted(found, key=lambda run: run.name)


def _events(run: Path) -> list[dict]:
    """A run's events; a line cut short is left out."""
    path = run / "events.jsonl"
    if not path.exists():
        return []
    events = []
    for line in path.read_text().splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            continue
    return events


# * ================================================================ the page

def _pretty(day: str) -> str:
    """A day for people: "4 Nov 2026"."""
    return datetime.strptime(day, "%Y%m%d").strftime("%-d %b %Y")


def matrix_table(incidents: Sequence[Incident]) -> str:
    """The count per day, robot and kind; only the kinds and rows that occur, plus a sum row."""
    matrix = count_matrix(incidents)
    kinds = [kind for kind in KINDS if any(kind in row for row in matrix.values())]
    if not matrix:
        return '<p class="muted">No incident on these days in these sources.</p>'
    header = '<th scope="col">Day</th><th scope="col">Robot</th>'
    header += "".join(f'<th scope="col" class="n">{esc(KINDS[kind][0])}</th>' for kind in kinds)
    header += '<th scope="col" class="n">Total</th>'
    rows = []
    for (day, robot), row in sorted(matrix.items()):
        cells = "".join(f'<td class="n">{row[kind]}</td>' if kind in row else '<td class="n z">·</td>'
                        for kind in kinds)
        rows.append(f'<tr><th scope="row">{esc(_pretty(day))}</th><td>{esc(robot)}</td>{cells}'
                    f'<td class="n tot">{sum(row.values())}</td></tr>')
    totals = {kind: sum(row.get(kind, 0) for row in matrix.values()) for kind in kinds}
    cells = "".join(f'<td class="n">{totals[kind]}</td>' for kind in kinds)
    rows.append(f'<tr class="sum"><th scope="row">All</th><td></td>{cells}<td class="n">{sum(totals.values())}</td>'
                "</tr>")
    return (f'<div class="table-wrap"><table class="matrix"><thead><tr>{header}</tr></thead>'
            f'<tbody>{"".join(rows)}</tbody></table></div>')


def _time(item: Incident) -> str:
    """An incident's time, or its range when a burst lasted."""
    start, end = (datetime.fromtimestamp(t).strftime("%H:%M:%S") for t in (item.start, item.end))
    return start if start == end else f"{start}–{end}"


def incident_row(item: Incident) -> list[tuple[str, str]]:
    """One row of a day's table: time, robot, kind, what happened with its raw log lines."""
    title, colour = KINDS[item.kind]
    times = f" <small>({item.count} lines)</small>" if item.count > 1 else ""
    inferred = f'<span class="inf">{esc(item.inferred)}</span>' if item.inferred else ""
    left = f"<code>... {item.dropped} more lines ...</code>" if item.dropped else ""
    head = item.raw[:-3] if item.dropped else item.raw
    tail = item.raw[-3:] if item.dropped else []
    codes = "".join(f"<code>{esc(line)}</code>" for line in head) + left
    codes += "".join(f"<code>{esc(line)}</code>" for line in tail)
    return [(esc(_time(item)), "time"), (esc(item.robot), ""), (f'<span class="chip {colour}">{esc(title)}</span>', ""),
            (f"{esc(item.what)}{times}{inferred}<details><summary>Log lines</summary><div class=\"ev\">{codes}</div>"
             f"<small>{esc(item.source)}</small></details>", "what")]


def day_tables(incidents: Sequence[Incident]) -> str:
    """One expandable table per day, newest first, the newest open."""
    days = sorted({item.day for item in incidents}, reverse=True)
    out = []
    for number, day in enumerate(days):
        rows = [item for item in incidents if item.day == day]
        robots: dict[str, int] = {}
        for item in rows:
            robots[item.robot] = robots.get(item.robot, 0) + 1
        words = " · ".join(f"{robot} {count}" for robot, count in sorted(robots.items()))
        inner = table([("Time", ""), ("Robot", ""), ("Kind", ""), ("What happened", "")],
                      [incident_row(item) for item in rows], "inc")
        out.append(f'<details class="day"{" open" if number == 0 else ""}><summary>{esc(_pretty(day))}'
                   f"<span>{len(rows)} incident{'' if len(rows) == 1 else 's'} · {esc(words)}</span></summary>{inner}"
                   "</details>")
    return "".join(out) or '<p class="muted">Nothing to list.</p>'


def _stat(title: str, value: int, words: str) -> str:
    """One count on a card."""
    return (f'<div class="note"><p class="eyebrow">{esc(title)}</p><p class="big num">{value}</p>'
            f"<p>{esc(words)}</p></div>")


def cannot_show(found: Mapping) -> str:
    """Section body: what these sources cannot show."""
    notes = [
        ("E-stops", "An E-stop shows here only through the health chip of a monitor running at that moment, a step "
                    "refused for it, or the safety sync in a robot pane being logged. A press with none of these "
                    "running leaves no line. The UR controller's own log is not read."),
        ("Why", "The logs say what happened, not why, and not who pressed what. Operators' reasons, a robot driven by "
                "hand, a bar touched: none of it is here unless a line says so."),
        ("Stops by hand", "The monitor's shutdown lines look the same after Ctrl-C and after its terminal closing. A "
                          "stop after a fault is listed like any other stop."),
        ("Robot panes", "Panes are logged only when started by the lab's tmux script. Their stamps are the robot PC's "
                        "clock, and lines without a stamp (ssh, the shell) only have the last stamp before them."),
    ]
    if found["days_without_log"]:
        days = ", ".join(_pretty(day) for day in found["days_without_log"])
        notes.append(("Days without a monitor log",
                      f"No monitor line in the log folder for: {days}. A day with no line may be a day nobody ran the "
                      "monitor, or logs kept elsewhere."))
    return ('<div class="cards">' + "".join(f'<div class="note"><h3>{esc(title)}</h3><p>{esc(words)}</p></div>'
                                            for title, words in notes) + "</div>")


METHOD = [
    ("Sources", "The monitor's terminal logs, the robots' pane logs and the MoCap relay's pane log in the log folder, "
                "and each run's <span class=\"mono\">events.jsonl</span>. An event near a log incident of the same "
                "kind is added to its lines; otherwise it is an incident of its own."),
    ("Time", "A line's own stamp, else the last stamp before it. A crash prints no time of its own."),
    ("Merging", "Lines of one kind and robot within 120 s of each other are one incident, with a count and a time "
                "range. Stop all, clearance, no answer, process, gripper, E-stop and MoCap lines merge whatever their "
                "words; the other kinds only when the words are the same, numbers aside. One failure can show under "
                "two kinds: a servo that ran all its passes, and the step that failed for it."),
    ("Kinds", "By the words of the line alone: Exec failed, refused or cancelled; ### SERVOING stopped, ran all or "
              "cancelled; fit FAILED and marker snapshot; SOFT STOP; came within or cannot be checked; HOLD NOT SENT; "
              "not reachable; not starting a controller; no goal; a health chip turning red on an E-stop or a "
              "protective stop."),
    ("Crash or stop by hand", "A traceback or a process death is a crash, unless the traceback ends in "
                              "KeyboardInterrupt. A log ending on the shutdown lines is a stop by hand. A log ending "
                              "with neither is a crash, inferred, unless the file changed in the last five minutes."),
    ("Left out", "Every other health chip change (their flicker is no incident), late-tick reports, refused clicks "
                 "while a step ran, 'not measured' warnings, and processes that ended on a stop signal in a robot "
                 "pane. Inserts that stopped short are listed whatever the distance."),
    ("Inferred", "Marked where the words of a line do not say it themselves."),
]


def render(found: Mapping, first: str, last: str) -> str:
    """The whole page."""
    incidents = found["incidents"]
    span = _pretty(first) if first == last else f"{_pretty(first)} to {_pretty(last)}"
    kinds = {kind: sum(item.kind == kind for item in incidents) for kind in KINDS}
    running = f" · still being written: {', '.join(found['running'])}" if found["running"] else ""
    fields = [("Days", esc(span)), ("Incidents", str(len(incidents))),
              ("Data", esc(f"Monitor logs: {len(found['monitor_logs'])}{running} · robot pane logs: "
                           f"{len(found['robot_logs'])} · runs' events: {len(found['runs'])}")),
              ("Not counted", esc(f"{found['health_lines']} health chip lines (see the method)"))]
    lede = ("Every crash and failure the logs and the runs' events hold for these days. This page is generated: it "
            "lists what the lines say and gives no causes; anything else is marked inferred.")
    cards = ('<div class="cards">' + _stat("Incidents", len(incidents), "after merging bursts")
             + _stat("E-stop", kinds["estop"], "safety stops the logs show")
             + _stat("Gripper", kinds["gripper"], "driver errors, retreat checks")
             + _stat("Monitor crash", kinds["crash"], "traceback, process death or a silent end") + "</div>")
    body = (titleblock("Three-robot assembly test · incidents", f"Crashes and failures, {span}", lede, fields) + cards
            + section("Counts", "By day, robot and kind", "A dot is zero. A burst counts once.",
                      matrix_table(incidents))
            + section("Days", "What happened", "Open a day for its list; each row keeps its raw log lines.",
                      day_tables(incidents))
            + section("Limits", "What the data cannot show", "", cannot_show(found))
            + section("Method", "How the list is made", "", method(METHOD))
            + f"<footer>Built {datetime.now().isoformat(timespec='seconds')}.</footer>")
    return page(f"Incidents, {span}", body)


def build_report(logs: Path, first: str | None = None, last: str | None = None, drive: Path | None = None,
                 out: Path | None = None) -> Path:
    """Build the incident report of a day or a date range and return its path.

    Args:
        logs: The log folder.
        first: The first day, YYYYMMDD; default today.
        last: The last day; default the first.
        drive: The Drive root, for the runs' events and the default output.
        out: The .html to write; default beside the day's records,
            `<drive>/.../<last>-asbuilt-result/incidents_<first>[-<last>].html`.

    Raises:
        SystemExit: Neither a Drive root nor `out` to write to.
    """
    first = first or datetime.now().strftime("%Y%m%d")
    last = last or first
    if out is None and drive is None:
        raise SystemExit("no Drive root and no --out: nowhere to write the report")
    found = read_incidents(logs, first, last, drive)
    print(f"[assembly] {len(found['incidents'])} incidents from {len(found['monitor_logs'])} monitor logs, "
          f"{len(found['robot_logs'])} robot pane logs and {len(found['runs'])} runs.")
    name = f"incidents_{first}.html" if first == last else f"incidents_{first}-{last}.html"
    target = Path(out) if out else Path(drive) / AS_BUILT_FOLDER / f"{last}-asbuilt-result" / name
    return write(render(found, first, last), target)
