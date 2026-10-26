"""The console log keeps the newest lines with their source, folds repeats, and the share of recent late ticks."""

import asyncio

from rclpy.logging import LoggingSeverity

import husky_assembly_teleop.plugin_api.console_log as console_log
from husky_assembly_teleop.plugin_api.concurrency import TickTimer
from husky_assembly_teleop.plugin_api.console_log import ROBOTS, ConsoleLog, ConsoleLogger

INFO, WARN, ERROR = LoggingSeverity.INFO, LoggingSeverity.WARN, LoggingSeverity.ERROR


def test_the_same_line_in_a_row_raises_the_count_of_the_last():
    """A repeat right after itself folds into one line; after another line it is a new one."""
    log = ConsoleLog()
    for text in ("a", "a", "a", "b", "a"):
        log.add(INFO, "robot_control", text)
    assert [(entry.text, entry.count) for entry in log.entries()] == [("a", 3), ("b", 1), ("a", 1)]


def test_only_the_newest_lines_are_kept(monkeypatch):
    """Past KEPT lines, the oldest go."""
    monkeypatch.setattr(console_log, "KEPT", 3)
    log = ConsoleLog()
    for index in range(5):
        log.add(INFO, "health", str(index))
    assert [entry.text for entry in log.entries()] == ["2", "3", "4"]


def test_the_late_share_counts_only_the_ticks_of_the_last_window(monkeypatch):
    """Late periods over all periods, without the ticks older than LATE_WINDOW; 0 before any tick."""
    clock = [100.0]
    monkeypatch.setattr(console_log.time, "monotonic", lambda: clock[0])
    log = ConsoleLog()
    assert log.late_share() == 0.0
    log.count_tick(0.02, period=0.05)
    clock[0] += 30
    for late in (0.02, 0.0, 0.0):
        log.count_tick(late, period=0.05)
    assert log.late_share() == 0.5
    clock[0] += 31
    assert log.late_share() == 1 / 3


def test_a_freeze_counts_for_every_period_it_lasted():
    """A tick 1 s late at 20 Hz stood for 21 periods, all late: next to 79 ticks on time that is 21 %, not 1 %."""
    log = ConsoleLog()
    for _ in range(79):
        log.count_tick(0.0, period=0.05)
    log.count_tick(1.0, period=0.05)
    assert log.late_share() == 21 / 100


def test_old_ticks_are_dropped_even_when_nobody_reads_the_share(monkeypatch):
    """Counting a tick forgets those older than LATE_WINDOW, so a paused or missing console never lets them pile up."""
    clock = [100.0]
    monkeypatch.setattr(console_log.time, "monotonic", lambda: clock[0])
    log = ConsoleLog()
    for _ in range(3 * 1200):
        log.count_tick(0.0, period=0.05)
        clock[0] += 0.05
    assert len(log._ticks) <= 1201


def _log_from_one_place(logger: ConsoleLogger) -> None:
    """Log at three levels, the warning throttled, as a robot interface would."""
    logger.info("connected")
    logger.error("switch failed")
    logger.warning("slow", throttle_duration_sec=60.0)


def test_the_node_logger_keeps_what_it_logs_under_its_source():
    """Lines logged through it land in the console log as "robots"; a throttled repeat neither logs nor lands.

    It also checks that rclpy still sees the real caller: logging several levels through the wrapper would raise
    otherwise.
    """
    log = ConsoleLog()
    logger = ConsoleLogger("test_console_log", ROBOTS, log)
    _log_from_one_place(logger)
    _log_from_one_place(logger)
    assert [(entry.level, entry.source, entry.text) for entry in log.entries()] == [
        (INFO, ROBOTS, "connected"), (ERROR, ROBOTS, "switch failed"), (WARN, ROBOTS, "slow"),
        (INFO, ROBOTS, "connected"), (ERROR, ROBOTS, "switch failed")]


def test_the_tick_timer_counts_every_tick_but_the_first_with_how_late_it_was():
    """The first tick has no previous one to be late against; then each tick says how late it was, 0 if on time."""
    ticks, loop = [], asyncio.new_event_loop()
    timer = TickTimer(loop, late_after=0.01, repeat_after=60.0, log_warn=lambda _report: None, on_tick=ticks.append)
    try:
        for index, late in enumerate((0.0, 0.0, 0.05)):
            timer.start_tick(index, late)
    finally:
        timer.stop()
        loop.close()
    assert ticks == [0.0, 0.05]
