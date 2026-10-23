"""Shared test setup: this folder's helper modules are importable from every test folder.

Tests under experiments/ check offline analysis that no robot session runs, so they are marked slow.
Tests under plugins/<name>/ are marked <name>: `pytest -m <name>` runs that plugin's tests, slow ones too.
Tests marked old_scripts run only with `--old-scripts`, even under `-m ""`.
Every test holds TEST_LEASE, which the fake robots in `robot_fakes.py` are leased to.
? Without pytest-xdist or pytest-timeout (requirements.txt), their settings in setup.cfg are accepted and ignored: the
  tests run in one process, with no time limit, and the header says so. Nothing is installed for it.
"""

import os
from importlib.util import find_spec

# ! Before numpy loads: one BLAS thread per process, or the parallel workers' thread pools fight over the cores.
for _variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_variable, "1")

from pathlib import Path  # noqa: E402

import pytest  # noqa: E402
from robot_fakes import TEST_LEASE  # noqa: E402

HERE = Path(__file__).resolve().parent
#: The plugins with tests of their own, each a marker.
PLUGINS = sorted(path.name for path in (HERE / "plugins").iterdir() if path.is_dir() and path.name != "__pycache__")


#: The plugins setup.cfg asks for that this Python lacks.
MISSING = tuple(name for name, module in (("pytest-xdist", "xdist"), ("pytest-timeout", "pytest_timeout"))
                if find_spec(module) is None)


def pytest_addoption(parser):
    """Add `--old-scripts`; accept `-n` and the `timeout` setting where their plugins are missing."""
    parser.addoption("--old-scripts", action="store_true",
                     help="also run the tests marked old_scripts (the old calibration scripts, ~45 s)")
    if "pytest-xdist" in MISSING:
        # ? `_addoption`: only a plugin's own group may take a short lowercase option such as xdist's -n.
        parser.getgroup("xdist")._addoption("-n", "--numprocesses", dest="numprocesses", default=None,
                                            help="pytest-xdist is not installed: ignored, the tests run in one process")
    if "pytest-timeout" in MISSING:
        parser.addini("timeout", "pytest-timeout is not installed: ignored, no time limit")


def pytest_configure(config):
    """Register a marker per plugin folder."""
    for name in PLUGINS:
        config.addinivalue_line("markers", f"{name}: the tests under plugins/{name}/")


def pytest_report_header(config):
    """Say which of setup.cfg's plugins are missing, and what that changes."""
    if MISSING:
        return f"not installed: {', '.join(MISSING)}; the tests run in one process, with no per-test time limit"
    return None


def pytest_collection_modifyitems(config, items):
    """Mark tests under experiments/ slow and under plugins/<name>/ <name>; skip old_scripts without `--old-scripts`."""
    skip_old = pytest.mark.skip(reason="runs the old calibration scripts: pytest --old-scripts")
    for item in items:
        parts = Path(str(item.fspath)).resolve().relative_to(HERE).parts
        if parts[0] == "experiments":
            item.add_marker(pytest.mark.slow)
        if parts[0] == "plugins" and len(parts) > 2:
            item.add_marker(getattr(pytest.mark, parts[1]))
        if "old_scripts" in item.keywords and not config.getoption("--old-scripts"):
            item.add_marker(skip_old)


@pytest.fixture(autouse=True)
def hold_the_test_lease():
    """Run every test holding TEST_LEASE, so fakes leased to it take commands."""
    with TEST_LEASE.use():
        yield
