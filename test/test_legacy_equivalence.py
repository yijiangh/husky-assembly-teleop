"""The old app, the export loader and the design library give the same planning objects, up to explained differences.

Runs scripts/legacy_equivalence.py on the export named by DESIGN_IO_EQUIVALENCE_EXPORT; skipped when unset or
missing. ! Slow (~5 minutes, ~4 GB of memory): set DESIGN_IO_EQUIVALENCE_COLLISIONS=0 to skip PyBullet.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

import pytest

EXPORT = Path(os.environ.get("DESIGN_IO_EQUIVALENCE_EXPORT", "/nonexistent"))
pytestmark = pytest.mark.skipif(not (EXPORT / "ActionSchedule.json").is_file(),
                                reason="DESIGN_IO_EQUIVALENCE_EXPORT does not name an export on this machine")

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "legacy_equivalence.py"


def _script():
    """The equivalence script as a module."""
    spec = importlib.util.spec_from_file_location("legacy_equivalence", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # ! dataclasses look their module up here
    spec.loader.exec_module(module)
    return module


def test_three_loaders_agree():
    """Every difference between any two loaders is one of the explained kinds (KNOWN in the script)."""
    script = _script()
    collisions = os.environ.get("DESIGN_IO_EQUIVALENCE_COLLISIONS", "1") != "0"
    report = script.run(EXPORT, collisions=collisions, log=lambda _line: None)
    assert not report.unexpected(), report.unexpected()
    # * The old app and the export loader read the same files: they must agree on everything but the floor.
    for name, category in report.pairs["old vs new"].items():
        assert all(kind.startswith("floor") for kind in category.counts), (name, dict(category.counts))
    assert report.pairs["new vs design"]["state base+joints"].compared > 0
