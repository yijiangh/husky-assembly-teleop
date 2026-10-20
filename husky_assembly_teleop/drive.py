"""
The project's Google Drive folder, where experiment data lives, synced to this machine (e.g. by Insync).

- The root is the project folder itself ("2025-03 Husky Assembly"), given by the HUSKY_DRIVE_ROOT environment variable
  or the monitor's `drive_root` parameter. Where it sits locally differs per user (account folder, "Shared with me",
  a shortcut); below it, paths are the same for everyone, so code uses paths relative to it. Absolute paths pass
  through unchanged, for data elsewhere.
- ! Folders under the root are never created here: a folder made locally that exists unsynced in Drive is uploaded as
  a second folder of the same name. Create it in Drive, or sync it, instead.
- Saved runs and takes are named the same everywhere: `folder_name` for a name typed in the panel, `stamped_name` for
  a run, `started_at` for the time written into one.
"""

from __future__ import annotations

import os
import re
import time
from pathlib import Path

#: The environment variable holding the local path of the project's Drive folder.
DRIVE_ROOT_VARIABLE = "HUSKY_DRIVE_ROOT"


def drive_root(given: str | Path | None = None) -> Path | None:
    """The Drive root: `given` if set, else HUSKY_DRIVE_ROOT, else None."""
    text = str(given or "").strip() or os.environ.get(DRIVE_ROOT_VARIABLE, "").strip()
    return Path(text).expanduser() if text else None


def drive_folder(root: Path | None, path: Path) -> Path:
    """The existing folder `path`: under the Drive root if relative, as given if absolute (no root needed).

    Raises:
        FileNotFoundError: If the folder is missing, or a relative path has no root or the root is not a folder. Under
            the root, a missing folder was not created in Drive yet, or not synced to this machine.
    """
    path = Path(path).expanduser()
    if path.is_absolute():
        if not path.is_dir():
            raise FileNotFoundError(f"{path} is not a folder")
        return path
    if root is None:
        raise FileNotFoundError(f"no Drive root: set {DRIVE_ROOT_VARIABLE} to the local '2025-03 Husky Assembly' "
                                f"folder (or the monitor's drive_root parameter)")
    if not root.is_dir():
        raise FileNotFoundError(f"Drive root {root} is not a folder: check {DRIVE_ROOT_VARIABLE}")
    folder = root / path
    if not folder.is_dir():
        raise FileNotFoundError(f"{path} is missing under the Drive root {root}: create it in Google Drive, "
                                f"or sync it to this machine")
    return folder


def folder_name(text: str, default: str = "default") -> str:
    """A typed name made safe for a file or folder: letters, digits, "-" and "_", the rest "_"; `default` if empty."""
    return re.sub(r"[^A-Za-z0-9_-]", "_", str(text).strip()) or default


def stamped_name(*parts: str) -> str:
    """A name for a run or take that sorts by time: "<date>_<time>_<part>_<part>", e.g. "20261019_204500_alice_sim"."""
    return "_".join((time.strftime("%Y%m%d_%H%M%S"),) + parts)


def started_at() -> str:
    """The local time now with its UTC offset, as written into a run's description, e.g. "2026-10-19T20:45:00+0200"."""
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")
