"""Tests for design_io schema checks (T6): a file of another schema is refused, naming its commit."""

import json
from pathlib import Path

import pytest
from design_io_fixtures import build_design

from bar_assembly_core.design_io import DesignError, SchemaMismatch, read, write


def _set_writer(path: Path, **changes) -> None:
    """Change the writer block of one design file."""
    data = json.loads(path.read_text())
    data["writer"].update(changes)
    path.write_text(json.dumps(data))


@pytest.mark.parametrize("name", ["design.json", "actions/B2_H_hold.json"])
def test_other_schema_names_commit(tmp_path: Path, name: str):
    """Schema 999 in any file raises SchemaMismatch naming that file's commit."""
    write(build_design(tmp_path), tmp_path / "out")
    _set_writer(tmp_path / "out" / name, schema=999, commit="deadbeef1234")
    with pytest.raises(SchemaMismatch, match="deadbeef1234") as error:
        read(tmp_path / "out")
    assert error.value.schema == 999 and error.value.commit == "deadbeef1234"
    assert name in str(error.value)


def test_different_commits_are_fine(tmp_path: Path):
    """Files written by different commits of the same schema read without complaint."""
    write(build_design(tmp_path), tmp_path / "out")
    _set_writer(tmp_path / "out" / "actions" / "B1_J_joint.json", commit="0123456789ab", dirty=True)
    read(tmp_path / "out")


def test_wrong_format_refused(tmp_path: Path):
    """A file of another kind is refused."""
    write(build_design(tmp_path), tmp_path / "out")
    path = tmp_path / "out" / "actions" / "B1_J_joint.json"
    data = json.loads(path.read_text())
    data["format"] = "something_else"
    path.write_text(json.dumps(data))
    with pytest.raises(DesignError, match="something_else"):
        read(tmp_path / "out")
