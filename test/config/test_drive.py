"""The Drive root: from the parameter or HUSKY_DRIVE_ROOT, and folders under it that must exist."""

from pathlib import Path

import pytest

from husky_assembly_teleop.drive import DRIVE_ROOT_VARIABLE, drive_folder, drive_root


def test_drive_root_prefers_the_given_path(monkeypatch, tmp_path):
    """The parameter wins over HUSKY_DRIVE_ROOT; with neither there is no root."""
    monkeypatch.setenv(DRIVE_ROOT_VARIABLE, "/from/env")
    assert drive_root(f" {tmp_path} ") == tmp_path
    assert drive_root("") == Path("/from/env")
    monkeypatch.delenv(DRIVE_ROOT_VARIABLE)
    assert drive_root() is None


def test_drive_folder_needs_root_and_folder(tmp_path):
    """A missing root or folder raises, and nothing is created under the root."""
    with pytest.raises(FileNotFoundError, match=DRIVE_ROOT_VARIABLE):
        drive_folder(None, Path("data_experiment/base_exp"))
    with pytest.raises(FileNotFoundError, match="not a folder"):
        drive_folder(tmp_path / "missing", Path("data_experiment/base_exp"))
    with pytest.raises(FileNotFoundError, match="create it in Google Drive"):
        drive_folder(tmp_path, Path("data_experiment/base_exp"))
    assert not (tmp_path / "data_experiment").exists()
    (tmp_path / "data_experiment/base_exp").mkdir(parents=True)
    assert drive_folder(tmp_path, Path("data_experiment/base_exp")) == tmp_path / "data_experiment/base_exp"


def test_drive_folder_passes_absolute_paths(tmp_path):
    """An absolute folder needs no root; a missing one still raises."""
    assert drive_folder(None, tmp_path) == tmp_path
    with pytest.raises(FileNotFoundError, match="not a folder"):
        drive_folder(None, tmp_path / "missing")
