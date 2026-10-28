"""
Capture what the planners print while they run, line by line, into a callback: tamp's `[ssik]` lines, PyBullet's.

At the file descriptor (fd 1), since PyBullet redirects stdout itself, so `contextlib.redirect_stdout` misses it.
Only fd 1 is captured: the ROS log and `ctx.log_info` write to stderr, so nothing of the monitor's is swallowed and
the callback may log freely. compas_fab's own fd redirect nests inside this one.
"""

from __future__ import annotations

import os
import sys
import threading
from contextlib import contextmanager
from typing import Callable, Iterator

#: Seconds to wait for the reader after the block, for a writer that did not close its copy of the pipe.
JOIN_TIMEOUT = 2.0


@contextmanager
def captured_lines(line: Callable[[str], None]) -> Iterator[None]:
    """Send every line written to fd 1 inside the block to `line`, from a reader thread; fd 1 is as before after it.

    Args:
        line: Gets each line without its newline, on the reader thread.
    """
    # * Lines must leave Python's buffer as they are printed, or they all arrive when the block ends: compas_fab's
    #   redirect replaces sys.stdout with a block-buffered file, and a pipe is never a terminal.
    sys.stdout.flush()
    sys.stdout.reconfigure(line_buffering=True)
    read_end, write_end = os.pipe()
    saved = os.dup(1)
    os.dup2(write_end, 1)
    reader = threading.Thread(target=_read_lines, args=(read_end, line), name="stdout-lines", daemon=True)
    reader.start()
    try:
        yield
    finally:
        sys.stdout.flush()
        os.dup2(saved, 1)
        os.close(saved)
        os.close(write_end)
        # ! Bounded: a child process started inside the block inherits fd 1 and would hold the pipe open.
        reader.join(JOIN_TIMEOUT)


def _read_lines(read_end: int, line: Callable[[str], None]) -> None:
    """Hand each line of the pipe to `line` until every writer has closed it; a failing `line` drops its line only."""
    with os.fdopen(read_end, "r", errors="replace") as pipe:
        for text in pipe:
            try:
                line(text.rstrip("\n"))
            except Exception:  # noqa: BLE001  * a dead reader would make the planner's next print raise
                pass
