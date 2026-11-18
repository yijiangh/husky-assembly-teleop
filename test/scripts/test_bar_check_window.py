"""The bar check window's decisions, on a stand-in for its widgets."""

import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

pytest.importorskip("tkinter")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import bar_check  # noqa: E402
from bar_check import BarCheck, process_folder  # noqa: E402


@pytest.fixture
def window():
    """A window waiting for nothing, with one joint to mark as moved."""
    stand_in = MagicMock(move_waiting=None, index=0, bars=[MagicMock()], tolerance=1.0)
    stand_in._move_targets.return_value = ("joints/a",)
    stand_in._name.return_value = "B1"
    stand_in.verdict.candidates = ()
    return stand_in


def press_at(window, monkeypatch, *times):
    """Press "I moved" at these times, seconds."""
    for now in times:
        monkeypatch.setattr(bar_check.time, "monotonic", lambda: now)
        BarCheck.press_moved(window)


def test_a_double_click_does_not_confirm_a_move(window, monkeypatch):
    """A second press right after the first is ignored, and the first still waits."""
    press_at(window, monkeypatch, 10.0, 10.1)
    window.log.mark_moved.assert_not_called()
    assert window.move_waiting == (("joints/a",), 10.0)


def test_a_second_press_confirms_a_move(window, monkeypatch):
    """A deliberate second press logs the move."""
    press_at(window, monkeypatch, 10.0, 11.0)
    window.log.mark_moved.assert_called_once()


def test_a_second_press_too_late_asks_again(window, monkeypatch):
    """A press long after the first starts over."""
    press_at(window, monkeypatch, 10.0, 16.0)
    window.log.mark_moved.assert_not_called()
    assert window.move_waiting == (("joints/a",), 16.0)


def test_a_new_tolerance_redraws_the_verdict(window):
    """The verdict, its arrow and its buttons follow the new tolerance."""
    window.tolerance_text.get.return_value = "2,5"
    BarCheck._tolerance_changed(window)
    assert window.tolerance == 2.5
    window.draw.assert_called_once()


@pytest.mark.parametrize("given", [".", "process.json"])
def test_a_relative_path_still_names_the_process(tmp_path, monkeypatch, given):
    """Started inside the process folder, the process keeps its name."""
    folder = tmp_path / "261117_process"
    folder.mkdir()
    (folder / "process.json").write_text("{}", encoding="utf-8")
    monkeypatch.chdir(folder)
    assert process_folder(given) == folder
