"""The Wi-Fi watch: what it reads out of ping and iw, what it logs, and when a host is slow or off."""

import csv
import sys
from pathlib import Path

import pytest

pytest.importorskip("tkinter")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

from wifi_watch import OFF_AFTER, SPELL_HOLD, Watch  # noqa: E402

T = 1000.0

PING = """PING 192.168.0.113 (192.168.0.113) 56(84) bytes of data.
[1795602470.412567] no answer yet for icmp_seq=1
[1795602470.500000] 64 bytes from 192.168.0.113: icmp_seq=1 ttl=64 time=284 ms
[1795602470.621578] 64 bytes from 192.168.0.113: icmp_seq=2 ttl=64 time=0.384 ms
"""
#: `iw dev <card> info | grep channel` and `iw dev <card> station dump`, as a terminal sends them.
LINK = """\tchannel 36 (5180 MHz), width: 80 MHz, center1: 5210 MHz\r
Station c8:0e:14:03:6d:09 (on wlp2s0)\r
\trx bytes:\t57634539\r
\trx packets:\t324782\r
\ttx bytes:\t334531117\r
\ttx packets:\t474685\r
\ttx retries:\t29689\r
\ttx failed:\t0\r
\tbeacon loss:\t0\r
\tsignal:  \t-46 [-48, -46] dBm\r
\ttx bitrate:\t866.7 MBit/s VHT-MCS 9 80MHz short GI VHT-NSS 2\r
\trx bitrate:\t400.0 MBit/s VHT-MCS 9 40MHz short GI VHT-NSS 2\r
==\r
"""


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


def test_reads_the_pings(watch, tmp_path, monkeypatch):
    """A ping not answered in time is late, and its answer still counts with its time."""
    monkeypatch.setattr("wifi_watch.time.time", lambda: T)
    watch.last_answer["alice"] = T - 1
    monkeypatch.setattr(watch, "_lines", lambda key, command: iter(PING.splitlines(keepends=True)))
    watch._watch_pings("alice")
    assert [ms for _, ms in watch.pings["alice"]] == [284.0, 0.384]
    assert watch.spells["alice"].count == 1
    watch.check_spells(T)
    assert logged(tmp_path) == ["late", "284", "0.384"]


def test_reads_the_radio(watch, tmp_path, monkeypatch):
    """Every value is found, and a new channel is an event, once."""
    monkeypatch.setattr(watch, "_lines", lambda key, command: iter((LINK + LINK).splitlines(keepends=True)))
    watch._watch_link("alice")
    watch.check_spells(T)
    with (tmp_path / "link.csv").open(newline="") as file:
        first, second = csv.DictReader(file)
    assert (first["tx_packets"], first["tx_retries"]) == ("474685", "29689")
    assert first["signal"] == "-46 [-48, -46]"
    assert first["rx_rate"].startswith("400.0 MBit/s")
    assert all(second.values())
    assert watch.channels["alice"] == "36 (5180 MHz), width: 80 MHz, center1: 5210 MHz"
    assert (tmp_path / "events.log").read_text().count("channel 36") == 1
