"""Belle's 20261013 calibration session as take samples, for tests of the calibration plugin's runs and steps."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from repo_paths import REPO
from husky_assembly_teleop.plugins.calibration.storage.session_files import write_take
from husky_assembly_teleop.world.calibration_sessions import Calibration

#: Belle's rigid body, and crl-husky's default for it when the session was recorded (its 20261013 calibration): the
#: takes are judged against it, whatever the default is today.
BELLE_ID = 1850
RECORDED_DEFAULT = Calibration(BELLE_ID, "a200-0805", (0.056041568517684937, 0.010792136192321777, -0.4787405729293823),
                               (-0.004496685229241848, -0.0024215581361204386, 0.7098574042320251, 0.704326868057251),
                               "20261013", "", ())
#: Six takes per joint, as Belle's six sweeps per joint (see test/plugins/calibration/test_calibration_fit.py).
SESSION = np.load(REPO / "test" / "plugins" / "calibration" / "data" / "20261013.npz")


def take_samples(joint: int, take: int) -> list[dict]:
    """Take `take` of `joint` as the plugin writes its samples (without the applied calibration)."""
    rows = SESSION[f"j{joint}_take"] == take
    return [{"joint_conf": joints.tolist(), "base_mocap_pose": [base[:3].tolist(), base[3:].tolist()],
             "flange_mocap_pose": [tool[:3].tolist(), tool[3:].tolist()]}
            for joints, base, tool in zip(SESSION[f"j{joint}_joints"][rows], SESSION[f"j{joint}_base"][rows],
                                          SESSION[f"j{joint}_tool"][rows])]


def record_take(run, joint: int, take: int) -> int:
    """Write Belle's take as the run's take of a sweep of that joint, and keep it; return the sweep index."""
    index = [i for i, sweep in enumerate(run.sweeps.sweeps) if sweep.joint == joint][take]
    name = f"j{joint}_take{take}_{len(list(Path(run.session_folder()).rglob('*.json')))}"
    run.keep_take(index, write_take(run.session_folder() / f"j{joint}", name, take_samples(joint, take)))
    return index
