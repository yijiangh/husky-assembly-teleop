"""The mocap_streams plugin: each body's state and how long it has held it, the summary, and the table on the fakes."""

from __future__ import annotations

from types import SimpleNamespace

from crl_husky_msgs.msg import MocapRigidBodyPose
from geometry_msgs.msg import Point
from plugin_fakes import FakeScene
from robot_lab import RobotLab

from husky_assembly_teleop.plugins.mocap_streams import (ASK_PERIOD, INVALID, MISSING, REFRESH_PERIOD, STALE, STREAMING,
                                                         UNEXPECTED, MocapStreamsPlugin, duration, relay_line,
                                                         state_since, stream_row, stream_state, summary)
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.mocap import store_sample
from husky_assembly_teleop.robot_interface.mocap_definitions import BodyDefinition, ScaledPose
from husky_assembly_teleop.robot_interface.mocap_relay import FloorCalibrationState, MocapRelayState
from husky_assembly_teleop.ui.style import BUSY, FAIL, INFO, OK
from husky_assembly_teleop.world.checks import BAD, PENDING, WARN
from husky_assembly_teleop.world.measured import TrackedObject
from scaffolding_assembly_core.geometry import Pose

NOW = 5000.0
#: 33 minutes: how long B20's asset stopped sending on 2026-11-04 with only a small chip to show it.
DROPOUT = 33 * 60.0


def sample(body, arrived: float, captured: float | None = None, pose_valid: bool = True, tracking_valid: bool = True,
           marker_error: float = 2e-4) -> None:
    """A relay message of `body`, three markers measured of four, stored as the monitor stores it."""
    captured = arrived if captured is None else captured
    message = MocapRigidBodyPose(pose_valid=pose_valid, tracking_valid=tracking_valid, marker_error=marker_error)
    message.header.stamp.sec, message.header.stamp.nanosec = int(captured), int((captured % 1.0) * 1e9)
    message.marker_ids = [1, 2, 3, 4]
    message.marker_positions = [Point() for _ in range(4)]
    message.marker_measured = [True, True, True, False]
    store_sample(body, message, arrived)


def streaming_for(body, seconds: float, until: float) -> None:
    """The relay's 50 Hz of a live body, for the last `seconds`."""
    for tick in range(int(seconds * 50), 0, -1):
        sample(body, until - tick / 50.0)
    sample(body, until)


def test_a_live_body_streams_and_a_silent_one_never_seen_is_missing():
    """A valid sample just captured is streaming; a body no message reached is missing, whatever the relay says."""
    body = TrackedObject("bar", 1020)
    assert stream_state(body, False, NOW) == MISSING and stream_state(body, True, NOW) == MISSING
    sample(body, NOW)
    assert stream_state(body, True, NOW) == STREAMING


def test_a_stopped_body_the_relay_keeps_republishing_is_stale():
    """Messages keep arriving, marked invalid, with the last capture's stamp: stale, for arrivals prove nothing."""
    body = TrackedObject("bar", 1020)
    sample(body, NOW, captured=NOW - DROPOUT, pose_valid=False)
    assert body.last_update_time == NOW and stream_state(body, False, NOW) == STALE


def test_a_body_whose_messages_stopped_is_stale_though_its_last_was_valid():
    """The relay itself gone: the last sample stays valid, and only its age tells."""
    body = TrackedObject("bar", 1020)
    sample(body, NOW - 10.0)
    assert body.tracked and stream_state(body, False, NOW) == STALE


def test_samples_still_captured_but_not_valid_are_invalid():
    """Fresh frames with the body lost (markers hidden) are invalid, not stale; so is one the relay marks invalid."""
    body = TrackedObject("bar", 1020)
    sample(body, NOW, pose_valid=False, tracking_valid=False)
    assert stream_state(body, True, NOW) == INVALID
    sample(body, NOW, pose_valid=False)
    assert stream_state(body, True, NOW) == INVALID


def test_a_body_nothing_expects_is_unexpected_while_listed_then_stale():
    """Only the relay's word is known of it: no numbers, blue while listed, grey once it is no longer."""
    clocks = {}
    row = stream_row(1866, "unknown", None, True, clocks, NOW)
    assert (row.state, row.expected, row.rate, row.error) == (UNEXPECTED, False, None, None)
    assert stream_row(1866, "unknown", None, False, clocks, NOW).state == STALE
    assert stream_row(1866, "unknown", None, False, clocks, NOW).level == PENDING


def test_the_clock_keeps_a_state_and_restarts_on_a_change():
    """Since when: unchanged while the state holds, now once it changes."""
    clocks = {}
    assert state_since(clocks, 1020, STREAMING, NOW, NOW) == NOW
    assert state_since(clocks, 1020, STREAMING, NOW + 7.0, NOW + 7.0) == NOW
    assert state_since(clocks, 1020, INVALID, NOW + 9.0, NOW + 9.0) == NOW + 9.0
    assert clocks[1020] == (INVALID, NOW + 9.0)


def test_a_dropout_counts_from_its_last_capture_even_when_first_seen_late():
    """A monitor started during the dropout, or a state noticed late, still reads how long it has really been."""
    clocks = {}
    assert state_since(clocks, 1020, STALE, NOW - DROPOUT, NOW) == NOW - DROPOUT
    assert state_since(clocks, 1020, STALE, NOW - DROPOUT, NOW + 60.0) == NOW - DROPOUT
    # * A stamp ahead of this clock counts from now, never the future.
    assert state_since({}, 1020, STALE, NOW + 3.0, NOW) == NOW


def test_a_row_reads_stale_for_33_min():
    """A body stopped 33 min ago: red, its clock 33 min, no rate and no marker error."""
    body = TrackedObject("probe", 1020)
    sample(body, NOW, captured=NOW - DROPOUT, pose_valid=False)
    row = stream_row(1020, "probe", body, False, {}, NOW)
    assert (row.state, row.level) == (STALE, BAD)
    assert duration(NOW - row.since) == "33 min"
    assert (row.rate, row.error) == (0.0, None)


def test_a_row_uses_the_mocap_chips_judgement_of_marker_error():
    """Over MARKER_ERROR_WARN a streaming body is amber, as on the health panel; its rate and error are measured."""
    good, noisy = BaseState(), BaseState()
    streaming_for(good, 2.0, NOW)
    streaming_for(noisy, 2.0, NOW)
    noisy.marker_errors.clear()
    sample(noisy, NOW, marker_error=1.5e-3)
    row = stream_row(1840, "Alice", good, True, {}, NOW)
    assert row.state == STREAMING and row.level == 0 and abs(row.rate - 50.0) <= 1.0
    assert abs(row.error - 2e-4) < 1e-9
    row = stream_row(1850, "Belle", noisy, True, {}, NOW)
    assert row.state == STREAMING and row.level == WARN


def test_the_summary_counts_the_expected_and_names_what_is_not_streaming():
    """A count of the expected, then each kind of trouble by name, in the worst expected row's colour."""
    live, stopped, absent = TrackedObject("a", 1), TrackedObject("b", 2), TrackedObject("c", 3)
    sample(live, NOW)
    sample(stopped, NOW, captured=NOW - DROPOUT, pose_valid=False)
    clocks = {}
    rows = [stream_row(1840, "Alice", live, True, clocks, NOW), stream_row(1025, "B25", absent, False, clocks, NOW)]
    assert summary(rows) == (WARN, "1 of 2 expected streaming", "missing: B25")
    rows.insert(1, stream_row(1020, "B20", stopped, False, clocks, NOW))
    rows.append(stream_row(1866, "mystery", None, True, clocks, NOW))
    assert summary(rows) == (BAD, "1 of 3 expected streaming", "stale: B20; missing: B25; extra: mystery")
    assert summary([]) == (PENDING, "0 of 0 expected streaming", "")


def test_the_relay_line_says_the_frame_and_how_far_it_moved_when_it_knows():
    """Each part only when known: the answers' freshness, the frame's status, the last look's trusted fit."""
    silent = MocapRelayState()
    assert relay_line(silent, NOW) == "relay does not answer; no frame announced; no recent look"
    relay = MocapRelayState(frame=FloorCalibrationState("held"), streaming_ids_time=NOW, metrics_time=NOW)
    assert relay_line(relay, NOW) == "frame held; no trusted fit"
    relay.fit_now = ScaledPose(Pose((0.001, 0.0, 0.0)))
    assert relay_line(relay, NOW) == "frame held; poses off 1.0 mm; moved 1.0 mm since the floor calibration"
    relay.refit_now = relay.fit_now
    assert "poses off 0.0 mm; moved 1.0 mm" in relay_line(relay, NOW)
    assert "no recent look" in relay_line(relay, NOW + 60.0)
    relay.fit_now, relay.refit_now = ScaledPose(scale=1.001), None
    assert relay_line(relay, NOW).endswith("poses off 5.2 mm; moved 5.2 mm since the floor calibration")


def test_durations_read_at_a_scale_that_suits_them():
    """Milliseconds to hours; a clock that went backwards reads nothing."""
    assert [duration(value) for value in (-1.0, 0.02, 12.0, 1980.0, 7560.0)] == ["0 ms", "20 ms", "12 s", "33 min",
                                                                                 "2.1 h"]


class Relay:
    """The relay as the plugin reads it: its state, the ids it says stream, and every time it was asked."""

    def __init__(self, ctx, answering: bool = True):
        """No answer yet."""
        self.ctx, self.answering, self.asked = ctx, answering, 0
        self.state, self.ids = MocapRelayState(), set()

    def ask_streaming_ids(self) -> bool:
        """Count the question; answered at once while `answering`, never blocking."""
        self.asked += 1
        if self.answering:
            self.state.streaming_ids, self.state.streaming_ids_time = frozenset(self.ids), self.ctx.now()
        return self.answering


def bench() -> SimpleNamespace:
    """mocap_streams on the three fake robots."""
    lab = RobotLab()
    ctx = lab.ctx
    for config in lab.configs:
        lab.robots[config.serial].config.mocap_id = config.mocap_id
    ctx.view.scene = FakeScene()
    relay = ctx.world.mocap_relay = Relay(ctx)
    streams = MocapStreamsPlugin()
    streams.setup(ctx)
    return SimpleNamespace(lab=lab, ctx=ctx, relay=relay, streams=streams)


def tick(bench_: SimpleNamespace, seconds: float = REFRESH_PERIOD) -> None:
    """Let `seconds` pass, then one tick of the plugin."""
    bench_.ctx.t += seconds
    bench_.lab.tick(bench_.ctx.t)
    bench_.streams.update(bench_.ctx)
    bench_.streams.draw(bench_.ctx)


def test_robots_show_and_a_tracked_object_appears_once_tracked():
    """The three robots from their config, by name; then a tracked object by its name."""
    run = bench()
    tick(run)
    table = run.streams._table.content
    for text in ("1840", "Alice", "1850", "Belle", "1860", "Cindy"):
        assert text in table
    assert "1020" not in table
    assert "0 of 3 expected streaming" in run.streams._summary_html
    assert "missing: Alice, Belle, Cindy" in run.streams._summary_html
    rows_before = table.count("<tr>")

    run.ctx.track_object("probe", 1020)
    tick(run)
    table = run.streams._table.content
    assert table.count("<tr>") == rows_before + 1
    assert "probe" in table and "1020" in table
    assert "0 of 4 expected streaming" in run.streams._summary.content


def test_a_body_dropping_out_turns_the_line_red_and_keeps_its_row_and_its_clock():
    """A tracked probe streams, then stops while the relay republishes: red summary, stale since its last capture."""
    run = bench()
    obj = run.ctx.track_object("probe", 1020)
    tick(run)
    captured = run.ctx.t + REFRESH_PERIOD
    run.relay.ids = {1020}
    streaming_for(obj, 2.0, captured)
    tick(run)
    assert ">live</span>" in run.streams._table.content
    rows = run.streams._table.content.count("<tr>")
    run.relay.ids = set()
    sample(obj, run.ctx.t + DROPOUT, captured=captured, pose_valid=False)
    tick(run, DROPOUT)
    table, line = run.streams._table.content, run.streams._summary.content
    assert table.count("<tr>") == rows
    assert "stale: probe" in line and f"background:{FAIL}" in line
    assert ">stale</span>" in table and "33 min" in table


def test_an_unexpected_body_gets_its_name_in_motive_below_the_others_and_stays():
    """An id the relay lists that nothing expects: a blue row under the expected, its Motive name, kept once gone."""
    run = bench()
    run.relay.state.body_definitions[1866] = BodyDefinition(1866, "spare_probe", ())
    run.relay.ids = {1866, 1999}
    tick(run)
    table = run.streams._table.content
    assert table.index("Cindy") < table.index("1866") < table.index("spare_probe") < table.index("1999")
    assert "unknown" in table and f"background:{INFO}" in table
    assert "extra: spare_probe, unknown" in run.streams._summary_html
    rows = table.count("<tr>")
    run.relay.ids = set()
    tick(run, ASK_PERIOD)
    assert run.streams._table.content.count("<tr>") == rows and "extra" not in run.streams._summary_html


def test_a_listed_body_that_turns_out_expected_moves_up_to_its_own_row():
    """An unexpected id then tracked is one row, the tracked object's."""
    run = bench()
    run.relay.ids = {1020}
    tick(run)
    assert "extra: unknown" in run.streams._summary_html
    run.ctx.track_object("probe", 1020)
    tick(run)
    assert "extra" not in run.streams._summary_html and run.streams._table.content.count(">1020<") == 1


def test_it_reads_only_tracks_nothing_of_its_own_and_takes_no_lease():
    """No tracked object of its own, no lease on any robot, nothing sent, the relay asked at the low rate."""
    run = bench()
    tick(run)
    assert not run.ctx.world.tracked_objects
    run.relay.asked = 0
    for _ in range(45):
        tick(run, 0.1)
    assert run.relay.asked == 2  # * 4.5 s at a question per ASK_PERIOD
    assert all(robot.lease_slot.lease is None for robot in run.lab.robots.values()) and run.lab.published() == 0


def test_a_relay_that_does_not_answer_is_said_and_stops_counting_what_it_listed():
    """Its last answer ages out: unexpected rows go grey, the line says so, the rest reads on."""
    run = bench()
    run.relay.ids = {1866}
    tick(run)
    assert "relay does not answer" not in run.streams._relay.content
    run.relay.answering = False
    tick(run, 30.0)
    assert "relay does not answer" in run.streams._relay.content
    assert "extra:" not in run.streams._summary_html


def test_the_summary_is_amber_when_only_a_tracked_object_is_missing_and_green_when_all_stream():
    """Missing only needs a look; every expected body live is green; the phone shows the same."""
    run = bench()
    for robot in run.lab.robots.values():
        streaming_for(robot.base.state, 1.0, run.ctx.t + REFRESH_PERIOD)
    tick(run)
    assert "3 of 3 expected streaming" in run.streams._summary.content
    assert f"background:{OK}" in run.streams._summary.content
    assert run.streams._summary.content in run.streams._phone.content
    run.ctx.track_object("probe", 1020)
    for robot in run.lab.robots.values():
        streaming_for(robot.base.state, 1.0, run.ctx.t + REFRESH_PERIOD)
    tick(run)
    assert f"background:{BUSY}" in run.streams._summary.content


def test_the_table_has_no_sideways_scroller_and_both_plugins_share_the_mocap_tab():
    """Its numbers change every second, viser then rebuilds the HTML and a scroller would jump back to the left."""
    run = bench()
    tick(run)
    assert "overflow-x" not in run.streams._table.content and "<th>age</th>" not in run.streams._table.content
    assert MocapStreamsPlugin.tab_path == ("Mocap", "Streams")
