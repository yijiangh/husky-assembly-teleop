"""The relay's frame log: a file per day, kept so many days, off when empty, a simulation's apart."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import pytest

from husky_assembly_teleop.mocap_relay.frame_log import FrameLog, frame_log_folder


def at(day: str, hour: int = 12) -> float:
    """Seconds since the epoch at `hour` on `day` (YYYYMMDD), local time."""
    return datetime.strptime(day, "%Y%m%d").replace(hour=hour).timestamp()


def test_lines_are_appended_to_the_file_of_their_day_and_a_new_day_starts_a_new_file(tmp_path):
    """Two looks on the 27th, one on the 28th; a relay started again appends."""
    log = FrameLog(tmp_path / "log", 30)
    log.write('{"time": 1}', at("20261027", 23))
    log.write('{"time": 2}', at("20261027", 23))
    log.write('{"time": 3}', at("20261028", 0))
    log.close()
    FrameLog(tmp_path / "log", 30).write('{"time": 4}', at("20261028", 1))
    assert (tmp_path / "log" / "20261027.jsonl").read_text() == '{"time": 1}\n{"time": 2}\n'
    assert (tmp_path / "log" / "20261028.jsonl").read_text() == '{"time": 3}\n{"time": 4}\n'


def test_day_files_older_than_the_days_kept_are_deleted_at_start_and_at_the_day_change(tmp_path):
    """Three days kept: on the 28th the 26th stays and the 25th goes; on the 29th the 26th goes. Other files stay."""
    folder = tmp_path / "log"
    folder.mkdir()
    for name in ("20261025.jsonl", "20261026.jsonl", "notes.jsonl", "20261025.txt"):
        (folder / name).write_text("{}\n")
    log = FrameLog(folder, 3)
    log.write("{}", at("20261028"))
    assert sorted(path.name for path in folder.iterdir()) == ["20261025.txt", "20261026.jsonl", "20261028.jsonl",
                                                              "notes.jsonl"]
    log.write("{}", at("20261029"))
    assert sorted(path.name for path in folder.iterdir()) == ["20261025.txt", "20261028.jsonl", "20261029.jsonl",
                                                              "notes.jsonl"]


def test_an_empty_folder_is_no_log_and_a_simulation_logs_apart():
    """An empty folder is off; `~` expanded; sim:=true below sim/domain_<id>/, never beside the real days."""
    assert frame_log_folder("", False) is None and frame_log_folder("  ", True) is None
    assert frame_log_folder("~/.ros/mocap_frame", False) == Path.home() / ".ros" / "mocap_frame"
    assert frame_log_folder("/data/frame", True, 70) == Path("/data/frame/sim/domain_70")


def test_a_write_that_fails_raises_and_the_next_one_tries_again(tmp_path):
    """The folder is a file at first; once it is gone the log writes."""
    (tmp_path / "log").write_text("")
    log = FrameLog(tmp_path / "log", 30)
    with pytest.raises(OSError):
        log.write("{}", at("20261028"))
    (tmp_path / "log").unlink()
    log.write("{}", at("20261028"))
    assert (tmp_path / "log" / "20261028.jsonl").read_text() == "{}\n"
