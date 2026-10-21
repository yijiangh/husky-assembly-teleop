"""Traces: sampling signals per tick and per message, dropping the oldest when full, saving, and their lifetime."""

from __future__ import annotations

import json
from contextlib import nullcontext
from types import SimpleNamespace

import numpy as np
import pytest

from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.plugin_api import signals
from husky_assembly_teleop.plugin_api.trace import Signal, Trace
from husky_assembly_teleop.robot_interface.event import Event


def _counter() -> tuple[Signal, list[float]]:
    """A one-channel signal reading a list's last value, and the list."""
    source = [0.0]
    return Signal("count", lambda: source[-1]), source


def test_sample_reads_signals_and_keeps_order():
    """Each sample reads the signal now; times and values come back oldest first."""
    signal, source = _counter()
    trace = Trace([signal])
    for i in range(5):
        source.append(float(i))
        trace.sample(10.0 + i)
    assert trace.times().tolist() == [10.0, 11.0, 12.0, 13.0, 14.0]
    assert trace.values("count")[:, 0].tolist() == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert trace.latest("count").tolist() == [4.0]
    assert trace.start == 10.0


def test_full_trace_drops_the_oldest():
    """Past the cap, each new sample replaces the oldest, also after growing its buffer."""
    trace = Trace([Signal("v", lambda: 0.0)], max_samples=1000)
    for i in range(2500):
        trace.add(float(i), float(i))
    assert len(trace) == 1000
    assert trace.dropped == 1500
    assert trace.times().tolist() == list(np.arange(1500.0, 2500.0))
    assert trace.values("v")[:, 0].tolist() == list(np.arange(1500.0, 2500.0))
    assert trace.start == 0.0


def test_unknown_value_is_nan_and_wrong_width_raises():
    """None gives NaN in every channel; a value of the wrong width is refused."""
    trace = Trace([Signal("p", lambda: None, ("x", "y", "z"), "m")])
    trace.sample(0.0)
    assert np.isnan(trace.values("p")).all() and trace.values("p").shape == (1, 3)
    with pytest.raises(ValueError):
        trace.add(1.0, (1.0, 2.0))


def test_duplicate_or_reserved_names_are_refused():
    """Keys name the saved file's arrays: unique, no "t" or "stamp", no "." in a name, no two events of one name."""
    with pytest.raises(ValueError):
        Trace([Signal("a", lambda: 0.0), Signal("a", lambda: 0.0)])
    for name in ("t", "stamp", "a.b"):
        with pytest.raises(ValueError):
            Trace([Signal(name, lambda: 0.0)])
    with pytest.raises(ValueError):
        Trace([Signal("a", lambda r: 0.0, on=Event("e")), Signal("b", lambda r: 0.0, on=Event("e"))])
    Trace([Signal("a", lambda: 0.0), Signal("a", lambda r: 0.0, on=Event("e"))])  # one name, two triggers


def _record(t: float, value: float) -> SimpleNamespace:
    """A message's record: handled at `t`, stamped 0.1 s before."""
    return SimpleNamespace(update_time=t, stamp=t - 0.1, value=value)


def test_message_signals_keep_every_message_on_their_own_time_axis():
    """Each trigger keeps its own times; the trace's start is the earliest sample of any."""
    event = Event("e")
    tick, message = Signal("tick", lambda: 1.0), Signal("value", lambda r: r.value, on=event)
    trace = Trace([tick, message])
    for t in (0.98, 0.99, 1.0):
        trace.take(event, _record(t, 10 * t))
    trace.sample(1.05)
    assert trace.times("e.value").tolist() == [0.98, 0.99, 1.0] and trace.times().tolist() == [1.05]
    assert trace.times("tick").tolist() == [1.05]
    assert trace.stamps("e.value") == pytest.approx([0.88, 0.89, 0.9])
    assert trace.values("e.value")[:, 0] == pytest.approx([9.8, 9.9, 10.0])
    assert trace.latest("e.value") == pytest.approx([10.0])
    assert len(trace) == 4 and trace.start == 0.98
    with pytest.raises(KeyError):
        trace.stamps("tick")


def test_ready_made_signal_per_tick_or_per_message():
    """Per tick it reads the newest message (unknown before one); with `every_message`, every message's record."""
    odometry = Event("x_odometry")
    robot = SimpleNamespace(base=SimpleNamespace(state=SimpleNamespace(odometry=None), odometry_messages=odometry),
                            config=SimpleNamespace(serial="x"))
    trace = Trace([signals.wheel_odometry(robot), signals.wheel_odometry(robot, every_message=True)])
    trace.sample(0.0)
    message = SimpleNamespace(velocity=np.array([0.2, 0.1]), update_time=0.9, stamp=0.8)
    robot.base.state.odometry = message
    trace.take(odometry, message)
    trace.sample(1.0)
    per_tick = trace.values("x_wheel_odometry")
    assert np.isnan(per_tick[0]).all() and per_tick[1].tolist() == [0.2, 0.1]
    assert trace.values("x_odometry.velocity").tolist() == [[0.2, 0.1]]
    assert trace.signal("x_odometry.velocity").units == ("m/s", "rad/s")


def test_message_only_trace_ignores_the_tick():
    """Without tick signals, sampling keeps nothing and adding a tick sample is refused."""
    trace = Trace([Signal("value", lambda r: r.value, on=Event("e"))])
    trace.sample(1.0)
    assert len(trace) == 0 and trace.times().size == 0
    with pytest.raises(ValueError):
        trace.add(1.0, 2.0)


def test_save_round_trips(tmp_path):
    """The .npz holds the times, each signal's samples, its labels and a unit per channel, repeats included."""
    trace = Trace([Signal("p", lambda: (1.0, 2.0, 3.0), ("x", "y", "z"), "m"), Signal("s", lambda: 7.0)])
    trace.sample(1.0)
    trace.sample(2.0)
    path = trace.save(tmp_path / "sub" / "rec.npz")
    data = np.load(path)
    assert data["t"].tolist() == [1.0, 2.0]
    assert data["p"].tolist() == [[1.0, 2.0, 3.0]] * 2
    assert data["p.labels"].tolist() == ["x", "y", "z"] and data["p.units"].tolist() == ["m", "m", "m"]
    assert data["s.labels"].tolist() == ["s"] and data["s.units"].tolist() == [""]


def test_units_per_channel_must_match_the_channels():
    """A unit per channel is kept as given; another count than channels is refused."""
    signal = Signal("pose", lambda: (0.0, 0.0, 0.0), ("x", "y", "yaw"), ("m", "m", "rad"))
    assert signal.units == ("m", "m", "rad") and Signal("v", lambda: 0.0, unit="m/s").units == ("m/s",)
    with pytest.raises(ValueError):
        Signal("pose", lambda: (0.0, 0.0, 0.0), ("x", "y", "yaw"), ("m", "rad"))


def test_save_writes_each_event_as_a_group(tmp_path):
    """Per event its times and stamps, and per message signal its samples under "<event>.<name>"."""
    event = Event("0806_follower")
    trace = Trace([Signal("p", lambda: 1.0),
                   Signal("command", lambda r: (r.value, 0.0), ("v", "w"), ("m/s", "rad/s"), event)])
    trace.sample(1.0)
    trace.take(event, _record(0.5, 0.2))
    data = np.load(trace.save(tmp_path / "rec.npz"))
    assert data["t"].tolist() == [1.0] and data["0806_follower.t"].tolist() == [0.5]
    assert data["0806_follower.stamp"] == pytest.approx([0.4])
    assert data["0806_follower.command"].tolist() == [[0.2, 0.0]]
    assert data["0806_follower.command.labels"].tolist() == ["v", "w"]
    assert data["0806_follower.command.units"].tolist() == ["m/s", "rad/s"]
    with pytest.raises(ValueError):
        trace.save(tmp_path / "bad.npz", {"0806_follower.t": np.zeros(1)})


def test_record_samples_only_inside_its_block():
    """`ctx.record` is sampled by the tick while its block runs, and dropped after, even on error."""
    ctx = _context()
    signal, _source = _counter()
    with ctx.record(signal) as rec:
        ctx._sample_traces(1.0)
        ctx._sample_traces(2.0)
    ctx._sample_traces(3.0)
    assert rec.times().tolist() == [1.0, 2.0]
    with pytest.raises(RuntimeError), ctx.record(signal) as failed:
        raise RuntimeError("task failed")
    ctx._sample_traces(4.0)
    assert len(failed) == 0


def test_json_holds_the_same_as_npz(tmp_path):
    """A .json file reads back to the .npz keys and values: groups, labels, units, NaN as null, text extras."""
    from husky_assembly_teleop.config import RECORDING_FORMATS
    from husky_assembly_teleop.plugin_api.trace import FORMATS, load_arrays
    event = Event("x_follower")
    trace = Trace([Signal("x_pose", lambda: (1.0, None), ("x", "y"), "m"),
                   Signal("command", lambda r: (r.value, 0.0), ("v", "w"), "m/s", event)])
    trace.sample(1.0)
    trace.sample(1.05)
    trace.take(event, _record(1.02, 0.2))
    extra = {"path_poses": np.zeros((2, 3)), "robot": np.array("x"), "names": np.array(["a", "b"])}
    npz, text = load_arrays(trace.save(tmp_path / "r.npz", extra)), trace.save(tmp_path / "r.json", extra)
    document = json.loads(text.read_text())
    assert document["tick"]["signals"]["x_pose"]["values"][0] == [1.0, None]
    assert document["events"]["x_follower"]["signals"]["command"]["labels"] == ["v", "w"]
    assert document["tick"]["signals"]["x_pose"]["units"] == ["m", "m"]
    back = load_arrays(text)
    assert set(back) == set(npz)
    for key, value in npz.items():
        same = np.array_equal(back[key], value, equal_nan=value.dtype.kind == "f")
        assert back[key].shape == value.shape and same, key
    assert RECORDING_FORMATS == FORMATS
    with pytest.raises(ValueError):
        trace.save(tmp_path / "r.csv")


def _context() -> PluginContext:
    """A plugin context with a monitor that only times."""
    monitor = SimpleNamespace(timer=SimpleNamespace(part=lambda name: nullcontext()))
    return PluginContext("p", monitor, view=None, scene=None, kinematics=None)


def test_record_takes_messages_only_inside_its_block():
    """Message signals are attached to their event while the block runs, detached after."""
    ctx, event = _context(), Event("e")
    with ctx.record(Signal("value", lambda r: r.value, on=event)) as rec:
        event.fire(_record(1.0, 2.0))
        event.fire(_record(1.1, 3.0))
    event.fire(_record(1.2, 4.0))
    assert rec.values("e.value")[:, 0].tolist() == [2.0, 3.0]
    assert not event._listeners


def test_failing_message_signal_skips_the_message_and_is_reported():
    """A message signal that raises drops that message, not the callback; the monitor hears of it once."""
    ctx, event = _context(), Event("e")
    rec = ctx.trace(Signal("value", lambda r: 1.0 / r.value, on=event))
    event.fire(_record(1.0, 0.0))
    event.fire(_record(1.1, 2.0))
    assert rec.values("e.value")[:, 0].tolist() == [0.5]
    report = ctx._take_signal_failures()
    assert report.startswith("1 messages skipped") and "ZeroDivisionError" in report
    assert ctx._take_signal_failures() is None


def test_closing_the_plugin_detaches_its_traces():
    """A stopped plugin's traces take no more messages."""
    ctx, event = _context(), Event("e")
    rec = ctx.trace(Signal("value", lambda r: r.value, on=event))
    ctx._close()
    event.fire(_record(1.0, 2.0))
    assert len(rec) == 0 and not event._listeners


def test_plot_range_ignores_unknown_values():
    """The fitted y axis skips NaN, keeps a minimum span, and ends on whole steps."""
    from husky_assembly_teleop.ui.trace_plot import _nice_range
    assert _nice_range(np.array([[np.nan, 0.0], [np.nan, 0.0]]), 0.02) == (-0.01, 0.01)
    assert _nice_range(np.array([[np.nan], [np.nan]]), 2.0) == (-2.0, 2.0)
    assert _nice_range(np.array([[-0.0021], [0.0043]]), 0.002) == (-0.003, 0.005)
