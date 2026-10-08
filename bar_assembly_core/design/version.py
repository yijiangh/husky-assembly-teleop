"""
The schema this library reads and writes, and the commit it runs from.

! A file with another `schema` is refused; to read it, check out the commit named in its `writer`.
"""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Optional

from .types import Writer

#: Format version. Increment on every incompatible change to doc/design_format.md.
SCHEMA = 2
#: Library name written into every file.
LIBRARY = "design_io"

_PACKAGE_DIR = Path(__file__).resolve().parent


def writer_info() -> Writer:
    """The `writer` block for files written now: schema, this library's commit, uncommitted changes.

    ? Without git (Rhino on Windows) the commit is "unknown" and `dirty` False.
    """
    commit = _git("rev-parse", "--short=12", "HEAD")
    if commit is None:
        return Writer(SCHEMA, LIBRARY, "unknown", False)
    status = _git("status", "--porcelain", "--", str(_PACKAGE_DIR))
    return Writer(SCHEMA, LIBRARY, commit, bool(status))


def _git(*args: str) -> Optional[str]:
    """Run git in this package's folder; None if git is missing or fails."""
    try:
        result = subprocess.run(["git", *args], cwd=_PACKAGE_DIR, capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None
