"""The Wi-Fi watch: what it logs of a ping, and when a host is slow or off."""

import csv
import sys
from pathlib import Path

import pytest

pytest.importorskip("tkinter")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

from wifi_watch import OFF_AFTER, SPELL_HOLD, Watch  # noqa: E402

T = 1000.0


@pytest.fixture
def watch(tmp_path):
    """A watch that pings nobody, with a limit of 200 ms."""
    watch = Watch(tmp_path, limit=200.0, keep=60.0)
    yield watch
    watch.stop()


def logged(folder):
    """The round trips in ping.csv."""
    with (folder / "ping.csv").open(newline="") as file:
        return [row["rtt_ms"] for row in csv.DictReader(file)]


def test_a_slow_ping_starts_a_spell_that_ends_after_the_hold(watch, tmp_path):
    """One slow ping is a spell; it is over once none was slow for SPELL_HOLD."""
    watch._log_ping("alice", "1", "12.0", T)
    assert not watch.spells
    watch._log_ping("alice", "2", "350", T + 1)
    assert watch.spells["alice"].worst == 350.0
    watch._log_ping("alice", "3", "12.0", T + 1 + SPELL_HOLD + 0.1)
    watch.check_spells(T + 1 + SPELL_HOLD + 0.2)
    assert not watch.spells
    assert "worst 350 ms, 1 pings" in (tmp_path / "events.log").read_text()


def test_an_unanswered_ping_is_slow_until_its_host_counts_as_off(watch):
    """A host that stopped answering is slow while it waits, not once it is off."""
    watch._log_ping("belle", "1", "5", T)
    watch.check_spells(T + 1)
    assert "belle" in watch.spells
    watch.check_spells(T + OFF_AFTER + SPELL_HOLD + 1)
    assert not watch.spells


def test_pings_to_a_host_that_is_off_are_not_logged(watch, tmp_path):
    """Robots left off overnight do not fill the log; the first answer is logged again."""
    watch._log_ping("cindy", "1", None, T)  # never answered yet
    watch._log_ping("cindy", "2", "5", T + 1)
    watch._log_ping("cindy", "3", None, T + 2)
    watch._log_ping("cindy", "4", None, T + 1 + OFF_AFTER + 1)
    watch._log_ping("cindy", "5", "6", T + 1 + OFF_AFTER + 2)
    watch.check_spells(T + 100)
    assert logged(tmp_path) == ["5", "late", "6"]
