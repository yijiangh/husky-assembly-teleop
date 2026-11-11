"""
The as-built page built from the monitor: `scripts/as_built_page.py` run as a subprocess, its folder served over HTTP.

The page can be built any time, mid-experiment too, from the records so far. It is built by the script in the venv's
Python (the collector lives in `experiments/`, which a plugin never imports), on a thread, and the day's result folder
is then served by a small HTTP server of its own, so one click in the browser opens the page.
"""

from __future__ import annotations

import functools
import subprocess
import sys
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

#: The port the result folder is served on; the monitor's own page is on 8080, the phone's on 8081.
PAGE_PORT = 8082
#: The script, beside the package.
SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "as_built_page.py"


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


def build(drive_root: Path, problem: str, process_folder: Path, day: str | None = None) -> tuple[Path | None, str]:
    """Run the page script for a day (today's records by default): (the page written, the script's last lines).

    Blocking: call it on a thread. The script's own output goes back as text, for the log.
    """
    command = [sys.executable, str(SCRIPT), *([day] if day else []), "--problem", problem, "--process",
               str(process_folder), "--drive", str(drive_root)]
    result = subprocess.run(command, capture_output=True, text=True)
    lines = (result.stdout + result.stderr).strip().splitlines()
    tail = "\n".join(lines[-6:])
    if result.returncode != 0:
        return None, tail
    written = next((line for line in reversed(lines) if line.startswith("[assembly] wrote ")), "")
    path = Path(written[len("[assembly] wrote "):].rsplit(" (", 1)[0]) if written else None
    return path, tail
