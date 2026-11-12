"""The live plot's thinning: evenly picked rows, or each line's highs and lows kept."""

from __future__ import annotations

import numpy as np

from husky_assembly_teleop.ui.trace_plot import MAX_POINTS, peak_thinned, thinned


def test_peak_thinning_keeps_a_short_spike_that_even_thinning_drops():
    """Half an hour at 20 Hz, a quiet line and one 0.1 s push on the second: only the peak thinning keeps the push."""
    t = np.arange(36000) / 20.0
    values = np.zeros((len(t), 2))
    values[17771:17773, 1] = 50.0
    even_t, even = thinned(t, values)
    peak_t, peak = peak_thinned(t, values)
    assert len(even_t) == MAX_POINTS and even[:, 1].max() == 0.0
    assert len(peak_t) <= MAX_POINTS and peak[:, 1].max() == 50.0
    assert peak_t[0] == t[0] and peak_t[-1] == t[-1] and np.all(np.diff(peak_t) > 0)


def test_peak_thinning_leaves_a_short_trace_and_unknown_values_alone():
    """Under MAX_POINTS rows nothing is cut; a stretch with only NaN still keeps rows, in order."""
    t = np.arange(10.0)
    values = np.arange(10.0)
    assert peak_thinned(t, values)[0] is t
    long_t = np.arange(5000.0)
    long = np.full((5000, 3), np.nan)
    long[:, 0] = np.sin(long_t / 100.0)
    kept_t, kept = peak_thinned(long_t, long)
    assert len(kept_t) <= MAX_POINTS and np.all(np.isnan(kept[:, 1:]))
    assert kept[:, 0].max() > 0.999 and kept[:, 0].min() < -0.999
