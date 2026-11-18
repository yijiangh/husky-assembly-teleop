"""The bar check's log: where it lives in a process folder and what each row records."""

import csv
import sys
from datetime import datetime
from pathlib import Path

import pytest

pytest.importorskip("tkinter")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

from bar_check import OUTER, Bar, Joint, MeasureLog, process_log  # noqa: E402

from husky_assembly_teleop.shared.runs.runs import find_runs  # noqa: E402

LEFT, RIGHT = "joints/left", "joints/right"


@pytest.fixture
def bar():
    """A bar with two joints."""
    joints = Joint(LEFT, "T20/Male", 100.0, 140.0, "B2"), Joint(RIGHT, "T20/Male", 500.0, 540.0, "B3")
    return Bar("B1", 1000.0, joints, ())


def rows(path):
    """The log's rows."""
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def test_log_is_a_file_in_runs_that_the_run_listing_ignores(tmp_path):
    """Log is a file in runs that the run listing ignores."""
    process = tmp_path / "process"
    path = process_log(process, tmp_path / "old")
    MeasureLog(path, "process", "hash")._append(("row",))
    assert path == process / "runs" / "bar_check.csv"
    assert find_runs(process) == ([], [])


def test_first_open_copies_the_old_log_and_leaves_it(tmp_path):
    """First open copies the old log and leaves it."""
    process, old = tmp_path / "process", tmp_path / "old" / "process.csv"
    old.parent.mkdir()
    old.write_text("time\n1\n", encoding="utf-8")
    path = process_log(process, old.parent)
    assert path.read_text(encoding="utf-8") == "time\n1\n" and old.is_file()


def test_later_opens_keep_the_process_log(tmp_path):
    """Later opens keep the process log."""
    process, old = tmp_path / "process", tmp_path / "old" / "process.csv"
    old.parent.mkdir()
    old.write_text("old\n", encoding="utf-8")
    path = process_log(process, old.parent)
    path.write_text("newer\n", encoding="utf-8")
    assert process_log(process, old.parent).read_text(encoding="utf-8") == "newer\n"


def test_every_row_has_its_date_and_time(tmp_path, bar):
    """Every row has its date and time."""
    path = tmp_path / "runs" / "bar_check.csv"
    log = MeasureLog(path, "process", "hash")
    log.add(bar, (LEFT, RIGHT), 3, 440.0, 1.0)
    log.mark_moved(bar, LEFT, 1.0, 1.0)
    written = rows(path)
    assert len(written) == 2
    assert all(datetime.fromisoformat(row["time"]).tzinfo is not None for row in written)


@pytest.mark.parametrize("cut", ["2026-11-18T12:00:00+01:00,process,hash,B1,joints/le",
                                 "x,process,hash,B1,a,b,TRI,outer,1,1,0,1,yes"])
def test_a_reading_after_a_line_cut_by_a_crash_survives_a_restart(tmp_path, bar, cut):
    """A reading after a line cut by a crash survives a restart."""
    path = tmp_path / "bar_check.csv"
    MeasureLog(path, "process", "hash").add(bar, (LEFT, RIGHT), 3, 440.0, 1.0)
    with path.open("a", encoding="utf-8", newline="") as file:
        file.write(cut)
    MeasureLog(path, "process", "hash").add(bar, (LEFT, RIGHT), 3, 441.0, 1.0)
    assert [item.measured for item in MeasureLog(path, "process", "hash").on_bar(bar)] == [441.0]


@pytest.mark.parametrize("damage", [b"\x00\x00\x00\r\n", b"\xff\xfe,broken\r\n", b'x,"broken\r\n',
                                    b"y" * 200_000 + b"\r\n",
                                    b"t,p,h,B1,a,b,TRI,outer,1,44\r\n",
                                    b"t,p,h,B1,a,b,TRI,outer,1,nan,0,1,yes\r\n",
                                    b"t,p,h,B1,a,b,TRI,outer,1,1,0,1,yes,extra\r\n"])
def test_damaged_rows_never_stop_the_log_opening_or_count(tmp_path, bar, damage):
    """Damaged rows never stop the log opening or count."""
    path = tmp_path / "bar_check.csv"
    MeasureLog(path, "process", "hash").add(bar, (LEFT, RIGHT), 3, 440.0, 1.0)
    with path.open("ab") as file:
        file.write(damage)
    MeasureLog(path, "process", "hash").add(bar, (LEFT, RIGHT), 3, 441.0, 1.0)
    log = MeasureLog(path, "process", "hash")
    assert [item.measured for item in log.on_bar(bar)] == [441.0] and ("B1", ("a", "b")) not in log.latest
    assert log.unreadable == (0 if damage.startswith(b"\x00") else 1)


def test_readings_of_a_moved_joint_stop_counting_after_a_restart(tmp_path, bar):
    """Readings of a moved joint stop counting after a restart."""
    path = tmp_path / "bar_check.csv"
    log = MeasureLog(path, "process", "hash")
    log.add(bar, (LEFT, RIGHT), 3, 440.0, 1.0)
    log.mark_moved(bar, LEFT, 1.0, 1.0)
    assert rows(path)[0]["kind"] == OUTER and MeasureLog(path, "process", "hash").on_bar(bar) == []
