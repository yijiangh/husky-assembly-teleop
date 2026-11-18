"""The doc plugin on a mock Drive: the report buttons, the list of runs, what can't be deleted, the warning window."""

from __future__ import annotations

import asyncio
import functools
import json
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from plugin_fakes import FakeContext

from husky_assembly_teleop.plugins.doc.plugin import (CANCEL, DELETE, DELETE_TICKED, DocPlugin, actions_text,
                                                      date_text)
from husky_assembly_teleop.plugins.doc.report import (ACCURACY_REPORT, AS_BUILT_PAGE, INCIDENT_REPORT, WRITTEN,
                                                      build)
from husky_assembly_teleop.shared.assembly_run_contract import RunView
from husky_assembly_teleop.shared.runs.as_built import AS_BUILT_FOLDER
from husky_assembly_teleop.shared.runs.runs import (ActionRecord, DONE, PENDING, Progress, Run, RunInfo, RunSettings,
                                                    read_run, save_info, save_progress)

PROBLEM, OLD, NEWER, LAST = "261103_SRD", "20261104_175016", "20261104_183353", "20261105_100441"
ACTIONS = ("B20_J", "B20_R", "B21_J", "B21_R")


def on_loop(test):
    """Run an async test on its own event loop."""

    @functools.wraps(test)
    def run(*args, **kwargs):
        asyncio.run(test(*args, **kwargs))

    return run


def write_run(process: Path, name: str, current: str | None, done: tuple[str, ...] = ()) -> Run:
    """Write a run in `process`, with its four actions, at action `current`."""
    folder = process / "runs" / name
    folder.mkdir(parents=True)
    actions = {action: ActionRecord(status=DONE if action in done else PENDING) for action in ACTIONS}
    run = Run(folder, RunInfo(name, f"{name[:4]}-{name[4:6]}-{name[6:8]}T{name[9:11]}:{name[11:13]}:00+01:00", PROBLEM,
                              "hash", {}, {}, {}, RunSettings()),
              Progress(actions, (current, 0) if current else None, {}, "2026-11-05T10:00:00.000000+01:00"))
    save_info(run)
    save_progress(run)
    return read_run(folder)


class FakeAssemblyRun:
    """assembly_run as this plugin sees it: only its view."""

    def __init__(self, process: Path | None):
        """No run in use, nothing offered."""
        self.process, self.folder, self.current, self.offer, self.match, self.revision = process, None, None, "", "", 0

    @property
    def view(self) -> RunView:
        """The run now."""
        loaded = SimpleNamespace(source=self.process) if self.process is not None else None
        return RunView(self.folder, loaded, {}, frozenset(), self.current, RunSettings(), self.offer, self.match,
                       self.revision)


@pytest.fixture
def lab(tmp_path):
    """Three runs on a mock Drive, the plugin set up and looking at them: `tick` reads what changed."""
    process = tmp_path / "data_design_study" / PROBLEM
    write_run(process, OLD, "B20_J")
    write_run(process, NEWER, "B20_R", ("B20_J",))
    write_run(process, LAST, None, ACTIONS)
    assembly_run = FakeAssemblyRun(process)
    ctx = FakeContext(plugins={"assembly_run": assembly_run, "process": SimpleNamespace(rhino_file=None)},
                      drive_root=tmp_path)
    plugin = DocPlugin()
    plugin.setup(ctx)

    async def tick():
        plugin.update(ctx)
        await settle(ctx)

    return SimpleNamespace(ctx=ctx, plugin=plugin, process=process, run=assembly_run, tick=tick, drive=tmp_path)


async def settle(ctx: FakeContext) -> None:
    """Let every task started so far finish; they run on threads, so this waits a real moment."""
    while pending := [task for task in ctx._tasks if not task.done()]:
        await asyncio.wait(pending)
    for task in ctx._tasks:
        task.result()


def press(lab, label: str) -> None:
    """Press a button by its label: its callback runs at once in the fake context."""
    next(button for button in lab.ctx.gui.of_kind("button") if button.label == label).callback(None)


def tick(lab, *names: str) -> None:
    """Tick these runs in the list."""
    for name in names:
        lab.plugin._list._boxes[name].value = True


def labels(lab) -> list[str]:
    """The list's lines, top to bottom."""
    return [box.label for box in lab.plugin._list._boxes.values()]


def test_progress_is_the_current_action_of_all_of_them():
    """The current action's number; with none current, how many are done."""
    actions = {action: ActionRecord() for action in ACTIONS}
    run = SimpleNamespace(progress=Progress(actions, None, {}, ""))
    assert actions_text(run, "B21_J") == "ACTIONS: 3/4"
    assert actions_text(run, None) == "ACTIONS: 0/4"
    assert actions_text(run, "dropped") == "ACTIONS: 0/4"


def test_the_date_reads_as_a_date_or_as_the_file_says():
    """A time that can't be read is shown as it is, not a failure."""
    started = SimpleNamespace(info=SimpleNamespace(created_at="2026-11-05T10:04:41+01:00"))
    assert date_text(started) == "05 Nov 2026 10:04"
    assert date_text(SimpleNamespace(info=SimpleNamespace(created_at="soon"))) == "soon"


@on_loop
async def test_every_run_can_be_ticked_newest_first_and_the_one_in_use_is_marked(lab):
    """Name, date and progress on each run's own line; the run in use takes its live action, not its file's."""
    await lab.tick()
    assert list(lab.plugin._list._boxes) == [LAST, NEWER, OLD]
    assert "ACTIONS: 4/4" in labels(lab)[0] and "ACTIONS: 2/4" in labels(lab)[1] and "ACTIONS: 1/4" in labels(lab)[2]
    assert not any("in use" in label for label in labels(lab))
    lab.run.folder, lab.run.current, lab.run.revision = lab.process / "runs" / OLD, ("B21_J", 0), 1
    await lab.tick()
    assert "ACTIONS: 3/4" in labels(lab)[2] and "in use" in labels(lab)[2]


@on_loop
async def test_a_new_run_keeps_what_is_ticked_and_comes_on_top(lab):
    """Reading the runs again must not untick anything."""
    await lab.tick()
    tick(lab, OLD)
    write_run(lab.process, "20261106_090000", None)
    lab.run.revision = 1
    await lab.tick()
    assert list(lab.plugin._list._boxes)[0] == "20261106_090000" and lab.plugin._list.selected() == [OLD]


@on_loop
async def test_without_a_process_or_runs_nothing_can_be_deleted(tmp_path):
    """The button stays where it is, greyed."""
    assembly_run = FakeAssemblyRun(None)
    ctx = FakeContext(plugins={"assembly_run": assembly_run, "process": SimpleNamespace(rhino_file=None)},
                      drive_root=tmp_path)
    plugin = DocPlugin()
    plugin.setup(ctx)
    plugin.update(ctx)
    await settle(ctx)
    assert plugin._delete.disabled
    assembly_run.process = tmp_path / "empty"
    plugin.update(ctx)
    await settle(ctx)
    assert plugin._delete.disabled and not plugin._list._boxes


@on_loop
async def test_nothing_ticked_says_so(lab):
    """No window opens."""
    await lab.tick()
    press(lab, DELETE_TICKED)
    await settle(lab.ctx)
    assert not lab.plugin._popup.is_open and "tick the runs" in lab.plugin._line.content


@on_loop
@pytest.mark.parametrize("who", ["folder", "offer", "match"])
async def test_a_run_in_use_or_offered_is_refused_before_it_asks(lab, who):
    """assembly_run would write to it, or read it on Continue: no window opens, and the folder stays."""
    await lab.tick()
    if who == "folder":
        lab.run.folder = lab.process / "runs" / OLD
    else:
        setattr(lab.run, who, OLD)
    tick(lab, OLD)
    press(lab, DELETE_TICKED)
    await settle(lab.ctx)
    assert not lab.plugin._popup.is_open and (lab.process / "runs" / OLD).is_dir()
    assert OLD in lab.plugin._line.content


@on_loop
async def test_a_run_in_use_stays_when_others_are_ticked_with_it(lab):
    """Select all ticks the run in use too: the window says it stays, and the others go."""
    await lab.tick()
    lab.run.folder = lab.process / "runs" / LAST
    tick(lab, LAST, OLD)
    press(lab, DELETE_TICKED)
    await settle(lab.ctx)
    assert f"Stays: {LAST}" in lab.plugin._popup._text.content
    press(lab, DELETE)
    await settle(lab.ctx)
    assert (lab.process / "runs" / LAST).is_dir() and not (lab.process / "runs" / OLD).exists()


@on_loop
async def test_cancel_deletes_nothing(lab):
    """The window says what would go; Cancel closes it."""
    await lab.tick()
    tick(lab, OLD)
    press(lab, DELETE_TICKED)
    await settle(lab.ctx)
    assert lab.plugin._popup.is_open and "shared Drive" in lab.plugin._popup._text.content
    assert OLD in lab.plugin._popup._text.content
    press(lab, CANCEL)
    await settle(lab.ctx)
    assert not lab.plugin._popup.is_open and (lab.process / "runs" / OLD).is_dir()


@on_loop
async def test_delete_takes_each_run_and_its_rows_and_rebuilds_the_page_once(lab, monkeypatch):
    """Two older runs ticked: folders and rows gone, the shared page rebuilt once, the list read again, the log says."""
    day = lab.drive / AS_BUILT_FOLDER / "20261104-asbuilt"
    day.mkdir(parents=True)
    records = day / f"as_built_{PROBLEM}.jsonl"
    records.write_text("".join(json.dumps({"run_id": run}) + "\n" for run in (OLD, NEWER, LAST)))
    page = day.with_name("20261104-asbuilt-result") / "accuracy_20261104.html"
    page.parent.mkdir()
    page.write_text("<html>")
    rebuilt = []
    monkeypatch.setattr("husky_assembly_teleop.plugins.doc.plugin.rebuild_page",
                        lambda listed, drive, process: rebuilt.append((listed.path, drive, process)) or "")
    await lab.tick()
    tick(lab, OLD, NEWER)
    press(lab, DELETE_TICKED)
    await settle(lab.ctx)
    assert "As-built rows: 1" in lab.plugin._popup._text.content and page.name in lab.plugin._popup._text.content
    press(lab, DELETE)
    await settle(lab.ctx)
    await lab.tick()
    assert not lab.plugin._popup.is_open
    assert not (lab.process / "runs" / OLD).exists() and not (lab.process / "runs" / NEWER).exists()
    assert [json.loads(line)["run_id"] for line in records.read_text().splitlines()] == [LAST]
    assert rebuilt == [(page, lab.drive, lab.process)]
    assert list(lab.plugin._list._boxes) == [LAST] and not lab.plugin._delete.disabled
    assert sum(OLD in line or NEWER in line for line in lab.ctx.lines("warn")) == 2


@on_loop
async def test_a_failure_says_how_far_it_got_and_the_button_comes_back(lab, monkeypatch):
    """Deleting stops at the failure; the runs not deleted are listed again."""
    await lab.tick()
    tick(lab, OLD)
    press(lab, DELETE_TICKED)
    await settle(lab.ctx)

    def fail(_deletion):
        raise OSError("busy")

    monkeypatch.setattr("husky_assembly_teleop.plugins.doc.plugin.delete_run", fail)
    press(lab, DELETE)
    await settle(lab.ctx)
    await lab.tick()
    assert "failed after 0 of 1" in lab.plugin._line.content and not lab.plugin._delete.disabled
    assert OLD in lab.plugin._list._boxes


@on_loop
async def test_a_run_coming_into_use_while_the_window_is_open_is_not_deleted(lab):
    """Delete checks again: the operator may have pressed New run or Continue meanwhile."""
    await lab.tick()
    tick(lab, OLD)
    press(lab, DELETE_TICKED)
    await settle(lab.ctx)
    lab.run.folder = lab.process / "runs" / OLD
    press(lab, DELETE)
    await settle(lab.ctx)
    assert (lab.process / "runs" / OLD).is_dir() and "in use" in lab.plugin._line.content


@on_loop
async def test_loading_another_process_closes_the_window_and_empties_the_list(lab, tmp_path):
    """The question was about the process before."""
    await lab.tick()
    tick(lab, OLD)
    press(lab, DELETE_TICKED)
    await settle(lab.ctx)
    lab.run.process = tmp_path / "other"
    await lab.tick()
    assert not lab.plugin._popup.is_open and not lab.plugin._list._boxes


def test_the_report_buttons_stand_above_the_runs_and_their_link_below_them(lab):
    """The report buttons first, then Delete, whose list grows below it, so nothing moves under the pointer."""
    assert [button.label for button in lab.ctx.gui.of_kind("button") if button.label not in (DELETE, CANCEL)] == [
        "Build as-built page", "Build accuracy report", "Build incident report", DELETE_TICKED]


@pytest.mark.parametrize("label, page", [
    ("Build as-built page", AS_BUILT_PAGE),
    ("Build accuracy report", ACCURACY_REPORT),
    ("Build incident report", INCIDENT_REPORT),
])
@on_loop
async def test_each_report_button_builds_its_page_on_a_thread_and_shows_its_link(lab, monkeypatch, label, page):
    """Busy while it builds, one build at a time; the link once built, the same server for every page."""
    lab.ctx.plugins["process"].rhino_file = lab.drive / "room.3dm"
    release, calls, rooms = threading.Event(), [], []

    def built(asked, drive_root, problem, process_folder, day=None, room=None):
        calls.append((asked, drive_root, problem, process_folder, day))
        rooms.append(room)
        release.wait(5.0)
        return lab.drive / "2026-result" / f"{asked.script}.html", "wrote"
    monkeypatch.setattr("husky_assembly_teleop.plugins.doc.plugin.build_page", built)
    monkeypatch.setattr(lab.plugin._page_server, "serve", lambda folder: f"http://localhost:8082/{folder.name}/")

    press(lab, label)
    await asyncio.sleep(0.01)
    assert lab.plugin.page_line == (f"building the {page.name}...", "busy")
    press(lab, label)
    assert lab.plugin.page_line == ("a page is still being built", "busy")
    release.set()
    await lab.plugin._page_task
    assert len(calls) == 1 and calls[0][0] is page and calls[0][1] == lab.drive
    assert calls[0][2] == PROBLEM and calls[0][3] == lab.process
    assert rooms == [lab.drive / "room.3dm"]
    url = f"http://localhost:8082/2026-result/{page.script}.html"
    assert lab.plugin.page_line == (url, "ok")
    lab.plugin.draw(lab.ctx)
    assert f'href="{url}"' in lab.plugin._page.content


@on_loop
async def test_a_report_that_fails_says_why_on_its_line_and_nothing_loaded_or_no_drive_refuses(lab, monkeypatch):
    """The script's last line on the page line; no process or no Drive root: nothing starts."""
    drive, lab.ctx.config.drive_root = lab.ctx.config.drive_root, None
    press(lab, "Build incident report")
    assert lab.plugin.page_line == ("no Drive root (drive_root): nothing to build", "failed")
    assert lab.plugin._page_task is None
    lab.ctx.config.drive_root = drive
    monkeypatch.setattr("husky_assembly_teleop.plugins.doc.plugin.build_page",
                        lambda *args, **kwargs: (None, "Traceback\nSystemExit: no records"))
    press(lab, "Build accuracy report")
    await lab.plugin._page_task
    assert lab.plugin.page_line == ("not built: SystemExit: no records", "failed")
    assert "[as-built] the accuracy report was not built:\nTraceback\nSystemExit: no records" in lab.ctx.lines("warn")
    lab.run.process = None
    press(lab, "Build accuracy report")
    assert lab.plugin.page_line == ("no process loaded: nothing to build", "failed")


def test_the_scripts_are_told_the_process_or_the_logs_and_the_written_path_is_read_back(monkeypatch, tmp_path):
    """The records' pages get the process, the incident page the log folder; a failing script gives no path."""
    commands = []

    def run(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0, stdout=f"note\n{WRITTEN}{tmp_path}/a b/page.html (0.1 MB)\n", stderr="")
    monkeypatch.setattr("husky_assembly_teleop.plugins.doc.report.subprocess.run", run)
    monkeypatch.setenv("REAL_LAB_HOME", str(tmp_path / "lab"))
    path, tail = build(ACCURACY_REPORT, tmp_path, "prob", tmp_path / "prob")
    assert path == tmp_path / "a b" / "page.html" and tail.startswith("note")
    assert commands[0][1].endswith("scripts/as_built_report.py") and Path(commands[0][1]).exists()
    assert "--problem" in commands[0] and "--logs" not in commands[0]
    build(INCIDENT_REPORT, tmp_path, "prob", tmp_path / "prob")
    assert commands[1][1].endswith("scripts/incident_report.py") and Path(commands[1][1]).exists()
    assert "--problem" not in commands[1] and commands[1][-2:] == ["--logs", str(tmp_path / "lab" / "logs")]
    assert Path(AS_BUILT_PAGE.script).name == "as_built_page.py"
    build(AS_BUILT_PAGE, tmp_path, "prob", tmp_path / "prob", room=tmp_path / "my room.3dm")
    assert commands[2][-2:] == ["--env-3dm", str(tmp_path / "my room.3dm")]
    build(AS_BUILT_PAGE, tmp_path, "prob", tmp_path / "prob")
    assert commands[3][-1] == "--no-env" and "--env-3dm" not in commands[3]
    assert "--env-3dm" not in commands[0] and "--no-env" not in commands[0]
    monkeypatch.setattr("husky_assembly_teleop.plugins.doc.report.subprocess.run",
                        lambda command, **kwargs: SimpleNamespace(returncode=1, stdout="", stderr="SystemExit: no"))
    assert build(AS_BUILT_PAGE, tmp_path, "prob", tmp_path / "prob") == (None, "SystemExit: no")
