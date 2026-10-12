"""Which base calibration file each husky loads (husky_world._base_calibration_file_for).

husky_world imports the ROS stack, so these tests skip when it is not sourced.
"""

from types import SimpleNamespace

import pytest

from husky_assembly_teleop.robot_registry import robot_by_name

husky_world = pytest.importorskip('husky_assembly_teleop.husky_world')


class _Logger:
    """Keeps every logged line, per level."""

    def __init__(self):
        self.lines = []

    def info(self, msg: str) -> None:
        self.lines.append(('info', msg))

    def warn(self, msg: str) -> None:
        self.lines.append(('warn', msg))


def _monitor() -> SimpleNamespace:
    """A stand-in monitor: the axis convention and a logger, nothing else."""
    logger = _Logger()
    return SimpleNamespace(MOCAP_AXIS_CONVENTION='rhino', get_logger=lambda: logger)


def _write_calibration(root, date: str, filename: str) -> str:
    """Create an (empty) calibration file at <root>/<date>/<filename>; return its path."""
    folder = root / date
    folder.mkdir(exist_ok=True)
    path = folder / filename
    path.write_text('{}')
    return str(path)


def test_each_robot_reads_its_own_date_folder(tmp_path, monkeypatch):
    """Alice reads her listed folder, Cindy (not listed) reads CALIBRATION_DATE."""
    monkeypatch.setattr(husky_world, 'CALIB_DATA_DIR', str(tmp_path))
    monkeypatch.setattr(husky_world, 'CALIBRATION_DATE', '20260916')
    monkeypatch.setattr(husky_world, 'BASE_CALIBRATION_DATE_BY_ROBOT', {'Alice': '20260623'})
    alice_file = _write_calibration(tmp_path, '20260623', 'calibrated_transformation_0804_rhino.json')
    cindy_file = _write_calibration(tmp_path, '20260916', 'calibrated_transformation_0806_rhino.json')
    # A stale Alice file in the shared folder must not be picked.
    _write_calibration(tmp_path, '20260916', 'calibrated_transformation_0804_rhino.json')

    monitor = _monitor()
    assert husky_world._base_calibration_file_for(monitor, robot_by_name('Alice')) == alice_file
    assert husky_world._base_calibration_file_for(monitor, robot_by_name('Cindy')) == cindy_file
    infos = [m for level, m in monitor.get_logger().lines if level == 'info']
    assert any('Alice (0804)' in m and '20260623/' in m for m in infos)


def test_missing_file_in_the_robot_folder_warns_and_returns_none(tmp_path, monkeypatch):
    """No file in the robot's own folder: None and one warning naming that folder."""
    monkeypatch.setattr(husky_world, 'CALIB_DATA_DIR', str(tmp_path))
    monkeypatch.setattr(husky_world, 'BASE_CALIBRATION_DATE_BY_ROBOT', {'Alice': '20260623'})

    monitor = _monitor()
    assert husky_world._base_calibration_file_for(monitor, robot_by_name('Alice')) is None
    warnings = [m for level, m in monitor.get_logger().lines if level == 'warn']
    assert len(warnings) == 1 and '20260623' in warnings[0]
