"""
The day's report pages built from the monitor: a page's script run as a subprocess, its folder served over HTTP.

A page can be built any time, mid-experiment too, from the data so far. It is built by its script in the venv's Python
(the code lives in `experiments/`, which a plugin never imports), on a thread, and the day's result folder is then
served by a small HTTP server of its own, so one click in the browser opens the page.
"""

from __future__ import annotations

import functools
import os
import subprocess
import sys
import threading
from dataclasses import dataclass
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

#: The port the result folder is served on; the monitor's own page is on 8080, the phone's on 8081.
PAGE_PORT = 8082
#: The scripts, beside the package.
SCRIPTS = Path(__file__).resolve().parents[3] / "scripts"
#: What every page script prints for its output, followed by the path and its size in brackets.
WRITTEN = "[assembly] wrote "


@dataclass(frozen=True)
class Page:
    """One page a button builds: its name in the panel and the log, its script, and what the script is told."""

    name: str
    script: str
    #: Tell it the loaded process (the records' pages); the incident page reads the logs of the day instead.
    takes_process: bool = True
    takes_logs: bool = False
    #: Tell it the Rhino file drawn as the room.
    takes_room: bool = False


AS_BUILT_PAGE = Page("as-built page", "as_built_page.py", takes_room=True)
ACCURACY_REPORT = Page("accuracy report", "as_built_report.py")
INCIDENT_REPORT = Page("incident report", "incident_report.py", takes_process=False, takes_logs=True)


def log_folder() -> Path:
    """Where the lab scripts save the logs of the monitor and the robots' panes."""
    home = os.environ.get("REAL_LAB_HOME") or os.environ.get("FAKE_LAB_HOME") or Path.home() / "husky_real_lab"
    return Path(home) / "logs"


class PageServer:
    """One HTTP server for the day's result folder, started at the first build; `serve` moves it to another folder."""

    def __init__(self, port: int = PAGE_PORT):
        """Nothing served yet."""
        self.port = port
        self._server: ThreadingHTTPServer | None = None
        self.folder: Path | None = None

    def serve(self, folder: Path) -> str:
        """Serve `folder` (restarting on another one) and return the URL of its root."""
        if self._server is not None and self.folder != folder:
            self.stop()
        if self._server is None:
            handler = functools.partial(_QuietHandler, directory=str(folder))
            self._server = ThreadingHTTPServer(("0.0.0.0", self.port), handler)
            threading.Thread(target=self._server.serve_forever, name="as-built page", daemon=True).start()
            self.folder = folder
        return f"http://localhost:{self.port}/"

    def stop(self) -> None:
        """Stop serving."""
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server, self.folder = None, None


class _QuietHandler(SimpleHTTPRequestHandler):
    """The standard file handler without a line per request on the monitor's console."""

    def log_message(self, format: str, *args) -> None:  # noqa: A002 (the base class's name)
        """Say nothing."""


def build(page: Page, drive_root: Path, problem: str, process_folder: Path, day: str | None = None,
          room: Path | None = None) -> tuple:
    """Run a page's script for a day (its own default day when None): (the page written or None, the last lines).

    Blocking: call it on a thread. The script's own output goes back as text, for the log.
    """
    command = [sys.executable, str(SCRIPTS / page.script), *([day] if day else []), "--drive", str(drive_root)]
    if page.takes_process:
        command += ["--problem", problem, "--process", str(process_folder)]
    if page.takes_logs:
        command += ["--logs", str(log_folder())]
    if page.takes_room:
        command += ["--env-3dm", str(room)] if room is not None else ["--no-env"]
    result = subprocess.run(command, capture_output=True, text=True)
    lines = (result.stdout + result.stderr).strip().splitlines()
    tail = "\n".join(lines[-6:])
    if result.returncode != 0:
        return None, tail
    written = next((line for line in reversed(lines) if line.startswith(WRITTEN)), "")
    path = Path(written[len(WRITTEN):].rsplit(" (", 1)[0]) if written else None
    return path, tail
