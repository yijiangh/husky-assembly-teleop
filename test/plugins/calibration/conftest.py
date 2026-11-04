"""The calibration tests replay Belle's 20261013 session: Solve judges it against her default then, not today's."""

from __future__ import annotations

import pytest
from calibration_fixtures import BELLE_ID, RECORDED_DEFAULT

from husky_assembly_teleop.plugins.calibration import run as run_module


@pytest.fixture(autouse=True)
def recorded_default(monkeypatch):
    """Belle's default as it was when the takes were recorded; other robots keep crl-husky's."""
    default = run_module.default_calibration
    monkeypatch.setattr(run_module, "default_calibration",
                        lambda mocap_id: RECORDED_DEFAULT if mocap_id == BELLE_ID else default(mocap_id))
