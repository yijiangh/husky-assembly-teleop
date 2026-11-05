"""
The Doc tab: the day's report pages to build, and the loaded process's assembly runs to tick and delete.

Each page is built on a thread by its script (`report.py`) and served; the link shows below the buttons. Deleting
asks first in a warning window, and also takes the runs' as-built rows and rebuilds those days' report pages
(`shared/runs/delete_run.py`). The run in use, or one `assembly_run` offers to continue, is kept.
"""

from __future__ import annotations

import asyncio
from datetime import datetime
from functools import partial
from html import escape
from pathlib import Path

import viser

from ...colors import rgb
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.assembly_run_contract import AssemblyRunContract, RunView
from ...shared.guide.panel import set_changed
from ...shared.runs.delete_run import Page, RunDeleted, RunDeletion, delete_run, plan_deletion, rebuild_page
from ...shared.runs.runs import DONE, Run, find_runs
from ...ui.checklist import CheckList
from ...ui.popup import Popup
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, block, note, note_line, section, warning
from .report import ACCURACY_REPORT, AS_BUILT_PAGE, INCIDENT_REPORT, Page as ReportPage, PageServer
from .report import build as build_page

#: The report buttons, top to bottom: the page, the label, the hover text.
REPORT_BUTTONS = (
    (AS_BUILT_PAGE, "Build as-built page",
     "Build today's 3D assembly page from the records so far and serve it: the link appears below "
     "(scripts/as_built_page.py)"),
    (ACCURACY_REPORT, "Build accuracy report",
     "Build today's numbers per step, per bar and per servo run, without conclusions, and serve it: the link appears "
     "below (scripts/as_built_report.py)"),
    (INCIDENT_REPORT, "Build incident report",
     "Build today's crashes and failures from the logs and the runs, and serve it: the link appears below "
     "(scripts/incident_report.py)"),
)
#: The delete button, and the warning window's.
DELETE_TICKED, DELETE, CANCEL = "Delete ticked runs", "Delete", "Cancel"


def actions_text(run: Run, current: str | None) -> str:
    """How far a run got, e.g. "ACTIONS: 3/76": the current action's number; with none current, how many are done.

    Args:
        run: The run.
        current: The action it is at; for the run in use the live one, ahead of what its file says.
    """
    actions = list(run.progress.actions)
    number = (actions.index(current) + 1 if current in actions
              else sum(record.status == DONE for record in run.progress.actions.values()))
    return f"ACTIONS: {number}/{len(actions)}"


def date_text(run: Run) -> str:
    """When a run was started, e.g. "05 Nov 2026 10:04"; its file's text if that can't be read as a time."""
    try:
        return datetime.fromisoformat(run.info.created_at).strftime("%d %b %Y %H:%M")
    except ValueError:
        return run.info.created_at


def in_use_refusal(name: str, view: RunView) -> str:
    """Why a run can't be deleted now: it is written to, or offered to continue; "" when it can."""
    if view.folder is not None and view.folder.name == name:
        return f"{name} is the run in use: start a New run first"
    if name in (view.offer, view.match):
        return f"{name} is offered to continue: start a New run or continue another first"
    return ""


def run_label(run: Run, view: RunView) -> str:
    """A run's line in the list: name, date, how far it got, and whether it is in use."""
    used = view.folder is not None and view.folder == run.folder
    current = view.current[0] if used and view.current is not None else (
        run.progress.current[0] if run.progress.current is not None else None)
    return " · ".join((run.info.name, date_text(run), actions_text(run, current), *(("in use",) if used else ())))


def deletion_text(deletions: list[RunDeletion], runs: list[Run], refused: list[str]) -> str:
    """What the warning window says deleting these runs would take, and which ticked runs stay."""
    lines = []
    for deletion, run in zip(deletions, runs):
        lines.append(f"<b>{escape(run.info.name)}</b>, started {date_text(run)}, "
                     f"{actions_text(run, run.progress.current[0] if run.progress.current else None)}")
        lines.append(f"Run folder: {deletion.file_count} files, {deletion.size / 1e6:.1f} MB")
        lines += [f"As-built rows: {count}, in {escape(path.parent.name)}/{escape(path.name)}"
                  for path, count in deletion.rows.items()]
    pages = sorted({page.path.name for deletion in deletions for page in deletion.pages})
    lines += [f"Report page rebuilt without them, or deleted if nothing is left: {escape(name)}" for name in pages]
    lines += [f"Stays: {escape(why)}" for why in refused]
    return warning("This deletes from the shared Drive, for everyone, and may not be undoable.") + note(
        "<br>".join(lines), "inherit")


def deleted_text(deletion: RunDeletion, deleted: RunDeleted) -> str:
    """What deleting a run did, for the log and the panel."""
    return (f"deleted run {deletion.folder.name}: {deletion.file_count} files, {deleted.rows} as-built rows"
            + (f"; pages deleted: {', '.join(path.name for path in deleted.pages_removed)}"
               if deleted.pages_removed else "")
            + (f"; pages kept, they may show another process: {', '.join(path.name for path in deleted.kept)}"
               if deleted.kept else ""))


def page_html(text: str, state: str) -> str:
    """The last page build, on one line that is always there: the link once built, else the message.

    Args:
        text: A URL to open, or a message.
        state: "ok" for a URL; "busy", "failed" or "" for a message.
    """
    shown = (f'<a href="{escape(text, quote=True)}" target="_blank">{escape(text)}</a>' if state == "ok"
             else escape(text) or "&nbsp;")
    color = {"ok": OK, "busy": BUSY, "failed": FAIL}.get(state, NONE)
    return block(f'<div style="color:{color};font-size:12px;white-space:nowrap;overflow:hidden;'
                 f'text-overflow:ellipsis">{shown}</div>')


@register
class DocPlugin(HuskyPlugin):
    """A tab with the report buttons and the loaded process's runs to tick, with Delete ticked runs."""

    name = "doc"
    requires = ("assembly_run", "process")
    experimental = True
    icon = viser.Icon.FILE_TEXT

    def __init__(self):
        """Nothing built or listed yet; setup builds the widgets."""
        #: The report pages: their server, the build running, and the last build's outcome for the tab.
        self._page_server = PageServer()
        self._page_task: asyncio.Task | None = None
        self._page_line: tuple[str, str] = ("", "")
        self._page: viser.GuiHtmlHandle | None = None
        self._list: CheckList | None = None
        self._delete: viser.GuiButtonHandle | None = None
        self._line: viser.GuiHtmlHandle | None = None
        self._unreadable: viser.GuiHtmlHandle | None = None
        self._popup: Popup | None = None
        #: The runs and unreadable folders as last read, of the process in `_source`; the runs' names as listed.
        self._runs: list[Run] = []
        self._unread: list[tuple[str, str]] = []
        self._source: Path | None = None
        self._listed: list[str] = []
        #: The (process folder, run revision) last read or being read; None reads again.
        self._read_for: tuple[Path, int] | None = None
        self._reading = False
        #: The deletions the warning window asks about, and whether some are being carried out.
        self._asking: list[RunDeletion] = []
        self._deleting = False

    def setup(self, ctx: PluginContext) -> None:
        """Build the tab and the warning window.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            gui.add_html(section("reports", SECTION_CTRL))
            for page, label, hint in REPORT_BUTTONS:
                button = gui.add_button(label, hint=hint)
                button.on_click(ctx.defer(f"build the {page.name}", partial(self._request_page, ctx, page)))
            # ! Below the buttons: the link comes and goes.
            self._page = gui.add_html(page_html("", ""))
            gui.add_html(section("runs", SECTION_CTRL))
            self._delete = gui.add_button(DELETE_TICKED, icon=viser.Icon.TRASH, color=rgb(FAIL), disabled=True,
                                          hint="Asks first in a warning window. Takes each run's folder and its "
                                               "as-built rows, and rebuilds those days' report pages.")
            self._line = gui.add_html(note_line(""))
            # * Below the button: the list grows without moving it.
            self._list = CheckList(ctx, gui, "Runs", empty_text="no runs of the loaded process")
            self._unreadable = gui.add_html("")
        self._popup = Popup(ctx.view, "Delete these runs?", viser.Icon.ALERT_TRIANGLE,
                            {DELETE: rgb(FAIL), CANCEL: None})
        self._delete.on_click(ctx.defer("ask to delete runs", lambda: self._ask_delete(ctx)))
        self._popup.buttons[DELETE].on_click(ctx.defer("delete runs", lambda: self._confirm_delete(ctx)))
        self._popup.buttons[CANCEL].on_click(ctx.defer("keep the runs", self._cancel))

    def update(self, ctx: PluginContext) -> None:
        """Read the runs again when another process is loaded or the run changed.

        Args:
            ctx: This plugin's context.
        """
        run: AssemblyRunContract = ctx.require("assembly_run")
        view = run.view
        source = view.process.source if view.process is not None else None
        if source != self._source:
            self._source, self._runs, self._unread, self._read_for = source, [], [], None
            self._cancel()
            self._fill(view)
        if source is not None and (source, view.revision) != self._read_for and not self._reading:
            self._read_for, self._reading = (source, view.revision), True
            # * Survives a soft stop: it only reads, and the list must not go stale.
            ctx.spawn("read the runs", self._read(ctx, run, source), survives_soft_stop=True)

    def draw(self, ctx: PluginContext) -> None:
        """Keep the list's Select all and its empty note right, and show the last page build.

        Args:
            ctx: This plugin's context.
        """
        self._list.sync()
        set_changed(self._page, content=page_html(*self._page_line))

    @property
    def page_line(self) -> tuple[str, str]:
        """tuple[str, str]: The last page build's outcome: (a URL or a message, ok/busy/failed)."""
        return self._page_line

    def _request_page(self, ctx: PluginContext, page: ReportPage) -> None:
        """Start one page's build, one at a time, or say on the page line why not."""
        if self._page_task is not None and not self._page_task.done():
            self._page_line = ("a page is still being built", "busy")
            return
        view = ctx.require("assembly_run").view
        if view.process is None:
            self._page_line = ("no process loaded: nothing to build", "failed")
            return
        if ctx.config.drive_root is None:
            self._page_line = ("no Drive root (drive_root): nothing to build", "failed")
            return
        self._page_task = ctx.spawn(f"build the {page.name}", self._build_page(ctx, page, view.process),
                                    survives_soft_stop=True)

    async def _build_page(self, ctx: PluginContext, page: ReportPage, loaded) -> None:
        """Run a page's script for the loaded process, then serve its folder."""
        self._page_line = (f"building the {page.name}...", "busy")
        ctx.log_info(f"[as-built] building the {page.name}: scripts/{page.script}")
        path, tail = await ctx.run_in_thread(build_page, page, ctx.config.drive_root, loaded.source.name, loaded.source,
                                             room=ctx.require("process").rhino_file)
        if path is None:
            ctx.log_warn(f"[as-built] the {page.name} was not built:\n{tail}")
            self._page_line = (f"not built: {tail.splitlines()[-1] if tail else 'see the log'}", "failed")
            return
        # * A chosen Rhino file without a room still builds: only the script's output says so.
        for line in tail.splitlines():
            if "[env]" in line and ("no layer named" in line or "could not read" in line or "not installed" in line):
                ctx.log_warn(f"[as-built] {line.strip()}")
        url = self._page_server.serve(path.parent) + path.name
        ctx.log_info(f"[as-built] {page.name} written: {path}; open {url}")
        self._page_line = (url, "ok")

    async def _read(self, ctx: PluginContext, run: AssemblyRunContract, source: Path) -> None:
        """Read the process's runs on a thread and list them, unless another process was loaded meanwhile."""
        try:
            runs, unread = await ctx.run_in_thread(find_runs, source)
        finally:
            self._reading = False
        if source == self._source:
            self._runs, self._unread = sorted(runs, key=lambda run: run.info.name, reverse=True), unread
            self._fill(run.view)

    def _fill(self, view: RunView) -> None:
        """Show what was read, newest first, keeping what is ticked; a folder that can't be read says why."""
        names = [run.info.name for run in self._runs]
        if names != self._listed:
            ticked = set(self._list.selected())
            for name in self._listed:
                self._list.remove(name)
            for run in self._runs:
                self._list.add(run.info.name, run_label(run, view), selected=run.info.name in ticked)
            self._listed = names
        for run in self._runs:
            self._list.set_label(run.info.name, run_label(run, view))
        set_changed(self._delete, disabled=not names or self._deleting)
        set_changed(self._unreadable, content="".join(note(f"{escape(name)} can't be read: {escape(why)}", FAIL)
                                                      for name, why in self._unread))

    def _say(self, text: str, color: str = NONE) -> None:
        """Show one line under the button."""
        set_changed(self._line, content=note_line(text, color))

    def _ask_delete(self, ctx: PluginContext) -> None:
        """Delete ticked runs pressed: keep those in use, find what the others would take, show the warning window."""
        if self._deleting:
            return
        ticked = set(self._list.selected())
        view = ctx.require("assembly_run").view
        runs = [run for run in self._runs if run.info.name in ticked]
        refused = [why for run in runs if (why := in_use_refusal(run.info.name, view))]
        runs = [run for run in runs if not in_use_refusal(run.info.name, view)]
        if not runs:
            self._say("; ".join(refused) or "tick the runs to delete first", FAIL if refused else BUSY)
            return
        self._say("looking up what deleting them would take...", BUSY)
        ctx.spawn("look up deletions", self._plan(ctx, runs, refused), survives_soft_stop=True)

    async def _plan(self, ctx: PluginContext, runs: list[Run], refused: list[str]) -> None:
        """Find what deleting `runs` would take on a thread, and ask."""
        try:
            deletions = [await ctx.run_in_thread(plan_deletion, run.folder, ctx.config.drive_root) for run in runs]
        except (OSError, ValueError) as error:
            ctx.log_warn(f"can't delete runs: {error}")
            self._say(f"can't delete: {error}", FAIL)
            return
        self._say("")
        self._asking = deletions
        self._popup.show(deletion_text(deletions, runs, refused))

    def _cancel(self) -> None:
        """Close the warning window; nothing is deleted."""
        self._asking = []
        if self._popup.is_open:
            self._popup.close()

    def _confirm_delete(self, ctx: PluginContext) -> None:
        """Delete in the warning window pressed: delete, keeping a run that came into use meanwhile."""
        deletions = self._asking
        self._cancel()
        if self._deleting:
            return
        view = ctx.require("assembly_run").view
        refused = [why for deletion in deletions if (why := in_use_refusal(deletion.folder.name, view))]
        deletions = [deletion for deletion in deletions if not in_use_refusal(deletion.folder.name, view)]
        if refused:
            self._say("; ".join(refused), FAIL)
        if not deletions:
            return
        self._deleting = True
        set_changed(self._delete, disabled=True)
        # * Survives a soft stop: it never commands a robot, and must not stop half way.
        ctx.spawn("delete runs", self._carry_out(ctx, deletions), survives_soft_stop=True)

    async def _carry_out(self, ctx: PluginContext, deletions: list[RunDeletion]) -> None:
        """Delete the runs on a thread, one at a time; then rebuild each page once; then list the runs again."""
        done, rows, problems = 0, 0, []
        pages: dict[Path, Page] = {}
        try:
            for deletion in deletions:
                self._say(f"deleting {deletion.folder.name}...", BUSY)
                deleted = await ctx.run_in_thread(delete_run, deletion)
                ctx.log_warn(deleted_text(deletion, deleted))
                done, rows = done + 1, rows + deleted.rows
                pages.update((page.path, page) for page in deleted.to_rebuild)
            # * A later run may have taken a page's last records with it.
            standing = [page for page in pages.values() if page.path.is_file()]
            for number, page in enumerate(standing, 1):
                self._say(f"rebuilding {page.path.name} ({number}/{len(standing)})...", BUSY)
                problem = await ctx.run_in_thread(rebuild_page, page, ctx.config.drive_root,
                                                  deletions[0].folder.parent.parent)
                if problem:
                    ctx.log_warn(problem)
                    problems.append(problem)
        except OSError as error:
            ctx.log_warn(f"deleting runs failed: {error}")
            problems.append(f"failed after {done} of {len(deletions)} runs: {error}")
        finally:
            self._deleting, self._read_for = False, None
            set_changed(self._delete, disabled=False)
        self._say("; ".join(problems) if problems else f"runs deleted: {done}; as-built rows taken out: {rows}",
                  FAIL if problems else OK)
