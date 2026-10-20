"""Paths tests read from the repository, the same from every test folder."""

from pathlib import Path

#: The repository root.
REPO = Path(__file__).resolve().parents[1]
#: Robot, tool and lab data in git.
DATA = REPO / "data"
