"""Traces: sampling signals per tick, dropping the oldest when full, saving, and `ctx.record`'s lifetime."""

from __future__ import annotations

from contextlib import nullcontext
from types import SimpleNamespace

import numpy as np
import pytest

from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.plugin_api.trace import Signal, Trace


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
    """Names key the saved file, so they must be unique and not "t"."""
    with pytest.raises(ValueError):
        Trace([Signal("a", lambda: 0.0), Signal("a", lambda: 0.0)])
    with pytest.raises(ValueError):
        Trace([Signal("t", lambda: 0.0)])


def test_save_round_trips(tmp_path):
    """The .npz holds the times, each signal's samples, its labels and unit."""
    trace = Trace([Signal("p", lambda: (1.0, 2.0, 3.0), ("x", "y", "z"), "m"), Signal("s", lambda: 7.0)])
    trace.sample(1.0)
    trace.sample(2.0)
    path = trace.save(tmp_path / "sub" / "rec.npz")
    data = np.load(path)
    assert data["t"].tolist() == [1.0, 2.0]
    assert data["p"].tolist() == [[1.0, 2.0, 3.0]] * 2
    assert data["p.labels"].tolist() == ["x", "y", "z"] and str(data["p.unit"]) == "m"
    assert data["s.labels"].tolist() == ["s"]


def test_record_samples_only_inside_its_block():
    """`ctx.record` is sampled by the tick while its block runs, and dropped after, even on error."""
    monitor = SimpleNamespace(timer=SimpleNamespace(part=lambda name: nullcontext()))
    ctx = PluginContext("p", monitor, view=None, scene=None, kinematics=None)
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


def test_plot_range_ignores_unknown_values():
    """The fitted y axis skips NaN, keeps a minimum span, and ends on whole steps."""
    from husky_assembly_teleop.ui.trace_plot import _nice_range
    assert _nice_range(np.array([[np.nan, 0.0], [np.nan, 0.0]]), 0.02) == (-0.01, 0.01)
    assert _nice_range(np.array([[np.nan], [np.nan]]), 2.0) == (-2.0, 2.0)
    assert _nice_range(np.array([[-0.0021], [0.0043]]), 0.002) == (-0.003, 0.005)
