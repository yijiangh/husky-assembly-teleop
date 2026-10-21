"""Shared test setup: this folder's helper modules are importable from every test folder.

Tests under experiments/ check offline analysis that no robot session runs, so they are marked slow.
Tests marked old_scripts run only with `--old-scripts`, even under `-m ""`.
Every test holds TEST_LEASE, which the fake robots in `robot_fakes.py` are leased to.
"""

import os

# ! Before numpy loads: one BLAS thread per process, or the parallel workers' thread pools fight over the cores.
for _variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_variable, "1")

from pathlib import Path  # noqa: E402

import pytest  # noqa: E402
from robot_fakes import TEST_LEASE  # noqa: E402

HERE = Path(__file__).resolve().parent


def pytest_addoption(parser):
    """Add `--old-scripts`."""
    parser.addoption("--old-scripts", action="store_true",
                     help="also run the tests marked old_scripts (the old calibration scripts, ~45 s)")


def pytest_collection_modifyitems(config, items):
    """Mark every test under experiments/ slow; skip old_scripts tests unless `--old-scripts` is given."""
    skip_old = pytest.mark.skip(reason="runs the old calibration scripts: pytest --old-scripts")
    for item in items:
        if Path(str(item.fspath)).resolve().relative_to(HERE).parts[0] == "experiments":
            item.add_marker(pytest.mark.slow)
        if "old_scripts" in item.keywords and not config.getoption("--old-scripts"):
            item.add_marker(skip_old)


@pytest.fixture(autouse=True)
def hold_the_test_lease():
    """Run every test holding TEST_LEASE, so fakes leased to it take commands."""
    with TEST_LEASE.use():
        yield
