"""Shared test setup: this folder's helper modules are importable from every test folder.

Tests under experiments/ check offline analysis that no robot session runs, so they are marked slow.
"""

from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


def pytest_collection_modifyitems(items):
    """Mark every test under experiments/ slow."""
    for item in items:
        if Path(str(item.fspath)).resolve().relative_to(HERE).parts[0] == "experiments":
            item.add_marker(pytest.mark.slow)
