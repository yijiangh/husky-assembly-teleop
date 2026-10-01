"""
Which schema this library reads and writes, and which commit of it is running.

! One schema only: a file with another `schema` is refused (format §7). To read an old design,
  check out the commit named in its `writer`.
"""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Optional

from .types import Writer

#: Format version. Increment on every incompatible change to doc/design_format.md.
SCHEMA = 1
#: Library name written into every file.
LIBRARY = "design_io"

_PACKAGE_DIR = Path(__file__).resolve().parent


def writer_info() -> Writer:
    """The `writer` block for files written now: schema, this library's commit, uncommitted changes.

    ? git may be missing (Rhino on Windows). Then the commit is read from the `.git` folder
      directly, and `dirty` is False because it cannot be checked.

    Returns:
        Writer: For this library as it runs.
    """
    commit = _git("rev-parse", "--short=12", "HEAD")
    if commit is None:
        return Writer(SCHEMA, LIBRARY, _commit_from_git_dir() or "unknown", False)
    status = _git("status", "--porcelain", "--", str(_PACKAGE_DIR))
    return Writer(SCHEMA, LIBRARY, commit, bool(status))


def _git(*args: str) -> Optional[str]:
    """Run git in this package's folder; None if git is missing or fails."""
    try:
        result = subprocess.run(["git", *args], cwd=_PACKAGE_DIR, capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def _commit_from_git_dir() -> Optional[str]:
    """Read HEAD's commit from the enclosing `.git` without git. Handles submodules (`.git` file)."""
    for folder in (_PACKAGE_DIR, *_PACKAGE_DIR.parents):
        dot_git = folder / ".git"
        if dot_git.is_file():  # submodule or worktree: "gitdir: <path>"
            dot_git = (folder / dot_git.read_text().split(":", 1)[1].strip()).resolve()
        if not dot_git.is_dir():
            continue
        head = (dot_git / "HEAD").read_text().strip()
        if not head.startswith("ref:"):
            return head[:12]
        ref = head.split(" ", 1)[1]
        ref_file = dot_git / ref
        if ref_file.is_file():
            return ref_file.read_text().strip()[:12]
        packed = dot_git / "packed-refs"
        if packed.is_file():
            for line in packed.read_text().splitlines():
                if line.endswith(" " + ref):
                    return line.split(" ", 1)[0][:12]
        return None
    return None
