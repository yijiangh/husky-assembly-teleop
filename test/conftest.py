"""Shared test setup: this folder's helper modules are importable from every test folder.

Tests under experiments/ check offline analysis that no robot session runs, so they are marked slow.
Every test holds TEST_LEASE, which the fake robots in `robot_fakes.py` are leased to.
"""

from pathlib import Path

import pytest
from robot_fakes import TEST_LEASE

HERE = Path(__file__).resolve().parent


def pytest_collection_modifyitems(items):
    """Mark every test under experiments/ slow."""
    for item in items:
        if Path(str(item.fspath)).resolve().relative_to(HERE).parts[0] == "experiments":
            item.add_marker(pytest.mark.slow)


@pytest.fixture(autouse=True)
def hold_the_test_lease():
    """Run every test holding TEST_LEASE, so fakes leased to it take commands."""
    with TEST_LEASE.use():
        yield
