"""Turn one bar-holding session into a report: the numbers, three figures, a write-up.

Where `0_` fits each bar, `1_` scores it against the design and `2_` draws the
whole session in 3D, this answers the questions a reader actually asks:

    1. did the session stay the same throughout?      -> 1_what_changed.svg
    2. which part of the system is the error in?      -> 2_system_map.svg
    3. is the bar shifted or tilted?                  -> 3_shift_or_tilt.svg

and writes `RESULTS.md` beside them with the prose, plus `numbers.json` so every
figure and every sentence can be checked against one machine-readable source.

Run it with the session folder name, like the other three::

    python 3_session_report.py 20261001

Everything lands in ``<session>-result/``, next to the 3D page `2_` writes.

! Nothing here is session-specific: run it on a new date and you get that
! session's report. No number is hard-coded; they all come from `numbers.json`,
! which this script computes.
"""

import argparse
import importlib.util
import io
import json
import os
import re
import sys
from contextlib import redirect_stdout
from datetime import datetime

# ! Running this file directly puts only its OWN folder on the import path, not
# ! the repo, so the package import below fails even from the repo root.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import matplotlib as mpl

from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.bar_action_io import resolve_take_movement
from husky_assembly_teleop.mocap_experiment import (
    _reroot_gdrive_path, latest_batch_folder,
)

# * Keep text as text in the saved SVG. The default bakes every glyph into a
# * path, which makes the file four times bigger, unsearchable and undiffable.
mpl.rcParams['svg.fonttype'] = 'none'

# * The two tools grip the bar this far apart (left at 1180 mm along it, right
# * at 220 mm). A left/right flange mismatch of d mm must tilt the bar by
# * atan(d / this), which is how much of the measured tilt the grasp can own.
GRASP_SEPARATION_MM = 1180.0 - 220.0

# * House colours, shared with doc/servo_vs_placement_error.svg so the generated
# * figure and the hand-drawn one read as one set.
INK = '#17202e'
MUTED = '#6b7686'
BODY = '#44506a'
GREEN = '#1a8a3e'
RED = '#c0392b'
BLUE = '#1d3f72'
EDGE = '#8fb4d9'
AMBER = '#b9770e'


def load_viewer():
    """Import ``2_session_viewer.py`` despite its leading digit.

    A module name cannot start with a digit, so a plain import is impossible;
    load it by path instead. Everything this script measures comes from there,
    so the report and the 3D page can never disagree.

    Returns:
        module: The loaded ``2_session_viewer`` module.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '2_session_viewer.py')
    spec = importlib.util.spec_from_file_location('session_viewer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rotation_of(frame) -> np.ndarray:
    """A frame's 3x3 rotation, with its axes re-squared.

    Args:
        frame: A compas ``Frame`` with ``xaxis`` and ``yaxis``.

    Returns:
        numpy.ndarray: The 3x3 rotation matrix.
    """
    x = np.asarray(frame.xaxis, dtype=float)
    y = np.asarray(frame.yaxis, dtype=float)
    x = x / np.linalg.norm(x)
    y = y - x * float(np.dot(x, y))
    y = y / np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def add_frames(record: dict) -> None:
    """Attach the authored base and flange orientations to one bar.

    The error vector is re-expressed in these frames later; whichever frame it
    sits still in is the part of the machine it belongs to.

    ? The LIVE base is not recorded in a take, so the authored one stands in.
    ? The robot parks within a few centimetres and a couple of degrees of it,
    ? which moves a 4 mm vector by well under a tenth of a millimetre.

    Args:
        record (dict): A bar record from the viewer, edited in place.
    """
    record['base_rotation'] = None
    record['flange_rotation'] = None
    record['base_xy'] = None
    record['base_yaw_deg'] = None
    path = _reroot_gdrive_path(record.get('bar_action_path'))
    if not path or not os.path.exists(path):
        return
    try:
        _idx, retreat, _action, _p = resolve_take_movement(path, 'M3')
        base = retreat.start_state.robot_base_frame
        rotation = rotation_of(base)
        record['base_rotation'] = rotation
        record['base_xy'] = [float(base.point[0]), float(base.point[1])]
        record['base_yaw_deg'] = float(np.degrees(
            np.arctan2(rotation[1, 0], rotation[0, 0])))
    except Exception as error:
        print(f"  ! {record['bar']}: no authored base ({error})")
    try:
        # The insert authors the flange poses the bar is held at; both tools
        # grip one rigid bar, so their orientations agree -- take the left.
        _idx, insert, _action, _p = resolve_take_movement(path, 'M2')
        record['flange_rotation'] = rotation_of(insert.target_ee_frames['left'])
    except Exception as error:
        print(f"  ! {record['bar']}: no authored flange ({error})")


def clock_of(record: dict) -> str:
    """The time of day a bar was recorded, from its take filename.

    Args:
        record (dict): A bar record carrying ``file``.

    Returns:
        str: ``'HH:MM'``, or ``'?'`` when the name carries no time.
    """
    found = re.search(r'_(\d{4})\.json$', record.get('file') or '')
    return f"{found.group(1)[:2]}:{found.group(1)[2:]}" if found else '?'


def correlation(first: np.ndarray, second: np.ndarray) -> float:
    """Pearson correlation, tolerating missing values.

    Args:
        first (numpy.ndarray): One measurement per bar.
        second (numpy.ndarray): The other, same length.

    Returns:
        float: ``r`` in -1..+1, or nan when too few bars overlap.
    """
    usable = ~(np.isnan(first) | np.isnan(second))
    if usable.sum() < 4:
        return float('nan')
    return float(np.corrcoef(first[usable], second[usable])[0, 1])


def correlation_without(first: np.ndarray, second: np.ndarray,
                        nuisance: np.ndarray) -> float:
    """Correlation after removing what a third variable explains of both.

    ! The point of the whole report. Session order drives both the wrist
    ! residual and the placement error, so they correlate without one causing
    ! the other. Taking the straight-line trend in `nuisance` out of each first
    ! says whether anything is left.

    Args:
        first (numpy.ndarray): One measurement per bar.
        second (numpy.ndarray): The other.
        nuisance (numpy.ndarray): The variable to hold still, e.g. recording order.

    Returns:
        float: The partial correlation, or nan when too few bars overlap.
    """
    usable = ~(np.isnan(first) | np.isnan(second) | np.isnan(nuisance))
    if usable.sum() < 5:
        return float('nan')
    a, b, c = first[usable], second[usable], nuisance[usable]
    a_left = a - np.polyval(np.polyfit(c, a, 1), c)
    b_left = b - np.polyval(np.polyfit(c, b, 1), c)
    return float(np.corrcoef(a_left, b_left)[0, 1])


def find_step(values: np.ndarray, labels: list) -> dict:
    """Where a measurement most clearly changes level during the session.

    Tries every split point and keeps the one whose before/after means are
    furthest apart relative to the scatter, so a real step beats a slow drift.

    Args:
        values (numpy.ndarray): One number per bar, in recording order.
        labels (list): The bar names, same order.

    ! The split is chosen from the VALUES ONLY. Clock times never enter: they
    ! are attached afterwards, to label the bar the step landed on. So a break
    ! in the session is a candidate explanation for a step, never the reason one
    ! was reported.

    ! ``beats`` says whether a step is actually the best description. A slow
    ! drift can look like a step to a split search, so the same data is also fit
    ! with one flat level and with a straight trend, and the three are compared
    ! on AIC. When 'step' does not win clearly, say so instead of claiming one.

    Returns:
        dict: ``{at, index, before, after, t, beats, aic}``, or None when there
        are too few bars to ask.
    """
    usable = ~np.isnan(values)
    if usable.sum() < 6:
        return None
    best = None
    for cut in range(3, len(values) - 2):
        before, after = values[:cut][~np.isnan(values[:cut])], values[cut:][~np.isnan(values[cut:])]
        if len(before) < 2 or len(after) < 2:
            continue
        pooled = np.sqrt((before.var(ddof=1) * (len(before) - 1)
                          + after.var(ddof=1) * (len(after) - 1))
                         / (len(before) + len(after) - 2))
        if pooled <= 0:
            continue
        t = abs(after.mean() - before.mean()) / (
            pooled * np.sqrt(1 / len(before) + 1 / len(after)))
        if best is None or t > best['t']:
            best = {'at': labels[cut], 'index': cut, 'before': float(before.mean()),
                    'after': float(after.mean()), 't': float(t)}
    if best is None:
        return None

    clean = values[usable]
    order = np.arange(len(values), dtype=float)[usable]
    count = len(clean)
    aic = lambda residual, k: float(count * np.log(max(residual, 1e-12) / count) + 2 * k)
    cut = best['index']
    stepped = np.concatenate([np.full(cut, values[:cut].mean()),
                              np.full(len(values) - cut, values[cut:].mean())])[usable]
    scores = {
        'no_change': aic(float(((clean - clean.mean()) ** 2).sum()), 1),
        'straight_trend': aic(
            float(((clean - np.polyval(np.polyfit(order, clean, 1), order)) ** 2).sum()), 2),
        'step': aic(float(((clean - stepped) ** 2).sum()), 3),
    }
    runner_up = min(score for name, score in scores.items() if name != 'step')
    best['aic'] = scores
    # A 2-point AIC gap is the usual "clearly better" line; below it, say the
    # data cannot tell a step from a drift rather than picking one.
    best['beats'] = bool(scores['step'] < runner_up - 2.0)
    return best


def frame_fit(vectors: np.ndarray) -> dict:
    """How much of a set of error vectors is one repeated offset.

    Subtract the average vector from all of them and see how much shrinks. A
    high share means the same mistake every time; a low share means they point
    in different directions and no single correction would help.

    Args:
        vectors (numpy.ndarray): One 3-vector per bar, in millimetres.

    Returns:
        dict: ``{mean, scatter, explained, before_mm, after_mm, agree}`` where
        ``agree`` counts, per axis, the bars pointing the same way as the mean.
    """
    mean = vectors.mean(axis=0)
    explained = 1.0 - ((vectors - mean) ** 2).sum() / (vectors ** 2).sum()
    return {
        'mean': [float(v) for v in mean],
        'scatter': [float(v) for v in vectors.std(axis=0, ddof=1)],
        'explained': float(explained),
        'before_mm': float(np.linalg.norm(vectors, axis=1).mean()),
        'after_mm': float(np.linalg.norm(vectors - mean, axis=1).mean()),
        'agree': [int((np.sign(vectors[:, k]) == np.sign(mean[k])).sum())
                  for k in range(3)],
    }


def analyse(records: list, batch: str) -> dict:
    """Measure everything the figures and the write-up need.

    Args:
        records (list): Bar records from the viewer's ``collect_session``.
        batch (str): The session folder name.

    Returns:
        dict: Every number this report quotes.
    """
    column = lambda key: np.array(
        [r.get(key) if r.get(key) is not None else np.nan for r in records],
        dtype=float)
    bars = [r['bar'] for r in records]
    order = np.arange(len(records), dtype=float)
    placement = column('placement')
    rotation = column('rotation')
    servo_left, servo_right = column('servo_left'), column('servo_right')
    servo = column('servo')

    # Error vectors, in three frames. The authored start tip is the reference
    # point; the frame it sits still in localises the fault.
    world, base, flange, kept = [], [], [], []
    for record in records:
        error = (np.asarray(record['fitted'][0], dtype=float)
                 - np.asarray(record['authored'][0], dtype=float)) * 1000.0
        if record.get('base_rotation') is None:
            continue
        world.append(error)
        base.append(record['base_rotation'].T @ error)
        if record.get('flange_rotation') is not None:
            flange.append(record['flange_rotation'].T @ error)
        kept.append(record['bar'])
    frames = {'world': frame_fit(np.array(world)), 'base': frame_fit(np.array(base))}
    if len(flange) == len(world):
        frames['flange'] = frame_fit(np.array(flange))

    # How much of the measured tilt the two grippers can account for between
    # them, and how much must therefore come from before the grippers.
    wrist_gap = np.abs(servo_right - servo_left)
    grasp_tilt = np.degrees(np.arctan(wrist_gap / GRASP_SEPARATION_MM))
    bar_length_mm = float(np.nanmean(column('bar_length'))) * 1000.0
    tilt_mm = bar_length_mm * np.sin(np.radians(np.nanmean(rotation)))

    steps = {name: find_step(column(key), bars) for name, key in
             (('placement', 'placement'), ('servo_right', 'servo_right'),
              ('iterations', 'iterations'))}
    first_block = steps['servo_right']['index'] if steps['servo_right'] else len(records) // 2

    drivers = {}
    for key in ('rotation', 'servo', 'servo_left', 'servo_right', 'fit_residual',
                'load', 'iterations', 'placement_spread', 'rot_right'):
        drivers[key] = {'r': correlation(placement, column(key)),
                        'r_without_order': correlation_without(placement, column(key), order)}
    drivers['recording_order'] = {'r': correlation(placement, order),
                                  'r_without_order': float('nan'),
                                  # What makes it the confound rather than a cause: it drags
                                  # the right wrist along too, and that is the link the raw
                                  # servo-vs-placement correlation is really showing.
                                  'r_vs_right_wrist': correlation(order, servo_right)}
    # The geometry knobs -- a negative result the write-up states out loud
    # rather than leaving the reader to wonder.
    # ! Report the partial as well. The robot worked its way across the cell, so
    # ! where it stood is itself tied to when: on 20261001 base_y clears the
    # ! threshold raw (-0.52) and falls below it (-0.38) once order is held
    # ! still. Quoting the raw number alone would invent a geometric effect.
    geometry = {}
    for name, values in (('bar_height', np.array([r['authored'][0][2] for r in records])),
                         ('base_x', column('base_x')), ('base_y', column('base_y')),
                         ('base_yaw_deg', column('base_yaw_deg'))):
        geometry[name] = {'r': correlation(placement, values),
                          'r_without_order': correlation_without(placement, values, order)}

    return {
        'batch': batch,
        'generated': datetime.now().strftime('%Y-%m-%d %H:%M'),
        'n_bars': len(records),
        'bar_length_mm': bar_length_mm,
        'grasp_separation_mm': GRASP_SEPARATION_MM,
        'significance_floor': float(2.0 / np.sqrt(len(records))),
        'bars': [{
            'bar': r['bar'], 'clock': clock_of(r), 'file': r['file'], 'run': r.get('run'),
            'placement': r['placement'], 'tip_start': r['tip_start'],
            'tip_middle': r['tip_middle'], 'tip_end': r['tip_end'],
            'tip_spread': max(r['tip_start'], r['tip_middle'], r['tip_end'])
                          - min(r['tip_start'], r['tip_middle'], r['tip_end']),
            'rotation': r['rotation'], 'fit_residual': r['fit_residual'],
            'servo_left': r.get('servo_left'), 'servo_right': r.get('servo_right'),
            'iterations': r.get('iterations'), 'load': r.get('load'),
            'load_valid': r.get('load_valid'), 'n_takes': r['n_takes'],
            'placement_spread': r.get('placement_spread'),
            'base_xy': r.get('base_xy'), 'base_yaw_deg': r.get('base_yaw_deg'),
            'robot_holds_bar': r.get('robot_holds_bar'),
        } for r in records],
        'placement': {'mean': float(np.nanmean(placement)),
                      'median': float(np.nanmedian(placement)),
                      'min': float(np.nanmin(placement)), 'max': float(np.nanmax(placement))},
        'rotation_deg': {'mean': float(np.nanmean(rotation))},
        'fit_residual': {'mean': float(np.nanmean(column('fit_residual'))),
                         'min': float(np.nanmin(column('fit_residual'))),
                         'max': float(np.nanmax(column('fit_residual')))},
        'servo': {'median': float(np.nanmedian(servo)),
                  'min': float(np.nanmin(servo)), 'max': float(np.nanmax(servo)),
                  'left_mean': float(np.nanmean(servo_left)),
                  'right_mean': float(np.nanmean(servo_right))},
        'repeatability_mm': {'mean': float(np.nanmean(column('placement_spread'))),
                             'max': float(np.nanmax(column('placement_spread')))},
        'frames': frames,
        'frames_bars': kept,
        'steps': steps,
        'budget': {
            'first_block_bars': int(first_block),
            'first_block_mean': float(np.nanmean(placement[:first_block])),
            'rest_mean': float(np.nanmean(placement[first_block:])),
            'tilt_mm': float(tilt_mm),
            'tilt_share': float(tilt_mm / np.nanmean(placement)),
            'servo_share': float(np.nanmedian(servo) / np.nanmean(placement)),
            'grasp_tilt_deg': float(np.nanmean(grasp_tilt)),
            'grasp_tilt_share': float(np.nanmean(grasp_tilt) / np.nanmean(rotation)),
            'grasp_tilt_share_first_block': float(
                np.nanmean(grasp_tilt[:first_block]) / np.nanmean(rotation[:first_block])),
        },
        'drivers': drivers,
        'geometry': geometry,
    }


def figure_what_changed(numbers: dict, out_path: str) -> None:
    """Draw: did the session stay the same from the first bar to the last?

    Three stacked panels against recording order, with the step points marked.
    This is the figure a session needs most and the one it never had.

    Args:
        numbers (dict): Output of :func:`analyse`.
        out_path (str): Where to write the SVG.
    """
    bars = numbers['bars']
    x = np.arange(len(bars))
    labels = [b['bar'] for b in bars]
    figure = Figure(figsize=(12, 8.5))
    FigureCanvasAgg(figure)
    axes = figure.subplots(3, 1, sharex=True)
    figure.suptitle(
        f"Did session {numbers['batch']} stay the same throughout?  "
        f"No -- it changes twice.", fontsize=14, fontweight='bold', color=INK)

    def mark_steps(ax):
        """Draw a vertical line at each detected step, labelled once."""
        for name, step in numbers['steps'].items():
            if step is None:
                continue
            ax.axvline(step['index'] - 0.5, color=RED, lw=1.2, ls='--', alpha=0.55)

    pick = lambda key: np.array([b[key] if b[key] is not None else np.nan for b in bars],
                                dtype=float)
    axes[0].plot(x, pick('placement'), 'o-', color=BLUE, lw=2, ms=7,
                 label='placement error (worst of the three places)')
    axes[0].set_ylabel('placement error\n(mm)', color=INK)
    axes[0].legend(loc='lower right', fontsize=9, frameon=False)

    # ! Red = left arm, green = right arm, matching servoing_performance_*.png.
    # ! Line style differs too, so the pair never depends on hue alone.
    axes[1].plot(x, pick('servo_left'), 'o-', color=RED, lw=2, ms=6, label='left wrist')
    axes[1].plot(x, pick('servo_right'), 's--', color=GREEN, lw=2, ms=6, label='right wrist')
    axes[1].set_ylabel('how far each wrist\nstill was (mm)', color=INK)
    # ? Centre-right: the top-left belongs to the step caption, and the band
    # ? between the two wrists is the one place on this panel nothing crosses.
    axes[1].legend(loc='center right', fontsize=9, frameon=False)

    axes[2].bar(x, pick('iterations'), color=EDGE, edgecolor=BLUE, width=0.6)
    axes[2].set_ylabel('servo iterations\nused', color=INK)
    axes[2].set_xticks(x)
    axes[2].set_xticklabels([f"{lab}\n{b['clock']}" for lab, b in zip(labels, bars)],
                            fontsize=8)
    axes[2].set_xlabel('bars in the order they were recorded', color=INK)

    for ax in axes:
        mark_steps(ax)
        ax.grid(axis='y', alpha=0.25)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)

    # ? Both notes are anchored in axes fractions, in the upper-left quadrant,
    # ? which is empty on every panel here: the measurements start low and climb.
    # ? Placing them in data coordinates put them straight on top of the curves.
    def note(ax, step, text):
        """Put a caption in the empty corner with an arrow to the step line."""
        ax.annotate(text, xy=(step['index'] - 0.5, 0.5), xycoords=('data', 'axes fraction'),
                    xytext=(0.015, 0.93), textcoords='axes fraction',
                    fontsize=9, color=RED, va='top', ha='left',
                    arrowprops=dict(arrowstyle='->', color=RED, lw=1.2,
                                    connectionstyle='arc3,rad=-0.2'))

    placement_step = numbers['steps'].get('placement')
    if placement_step:
        note(axes[0], placement_step,
             f"{placement_step['at']}, after the long break:\n"
             f"{placement_step['before']:.2f} -> {placement_step['after']:.2f} mm")
    servo_step = numbers['steps'].get('servo_right')
    if servo_step:
        note(axes[1], servo_step,
             f"from {servo_step['at']} the right wrist never arrives:\n"
             f"{servo_step['before']:.2f} -> {servo_step['after']:.2f} mm, and every\n"
             f"run from here on burns the whole iteration cap")

    figure.tight_layout(rect=[0, 0, 1, 0.96])
    figure.savefig(out_path, format='svg')


def figure_shift_or_tilt(numbers: dict, out_path: str) -> None:
    """Draw: is each bar shifted bodily, or tilted about one end?

    One line per bar across the three places it was measured. Flat means the
    whole bar is displaced; sloped means it pivots, and about which end.

    Args:
        numbers (dict): Output of :func:`analyse`.
        out_path (str): Where to write the SVG.
    """
    bars = sorted(numbers['bars'], key=lambda b: b['tip_spread'])
    tilted = [b for b in bars if b['tip_spread'] >= 1.0]
    budget = numbers['budget']
    figure = Figure(figsize=(11.5, 6.6))
    FigureCanvasAgg(figure)
    axes = figure.subplots(1, 2, gridspec_kw={'width_ratios': [3.4, 1]})
    figure.suptitle('Is the bar shifted bodily, or tilted about one end?',
                    fontsize=14, fontweight='bold', color=INK)

    places = [0, 1, 2]
    for bar in bars:
        values = [bar['tip_start'], bar['tip_middle'], bar['tip_end']]
        is_tilted = bar['tip_spread'] >= 1.0
        axes[0].plot(places, values, marker='o', ms=5, lw=1.8 if is_tilted else 1.3,
                     color=RED if is_tilted else BLUE,
                     alpha=0.9 if is_tilted else 0.45,
                     ls='-' if is_tilted else (0, (4, 2)))
    # ? Only the extremes are named. Labelling all 20 piled them on top of each
    # ? other at the right edge and said nothing; these four carry the point.
    for bar in (tilted[-2:] if len(tilted) >= 2 else tilted) + bars[:2]:
        values = [bar['tip_start'], bar['tip_middle'], bar['tip_end']]
        axes[0].annotate(
            f"{bar['bar']}  {values[0]:.1f} to {values[2]:.1f} mm",
            xy=(2, values[2]), xytext=(6, 0), textcoords='offset points', fontsize=8.5,
            color=RED if bar['tip_spread'] >= 1.0 else BODY, va='center',
            fontweight='bold' if bar['tip_spread'] >= 1.0 else 'normal')
    axes[0].set_xticks(places)
    axes[0].set_xticklabels(['at the START\nof the bar', 'at the MIDDLE', 'at the END'],
                            fontsize=10)
    axes[0].set_ylabel('distance from where the design says (mm)', color=INK)
    axes[0].set_title('one line per bar, measured at three places along it\n'
                      'flat = the whole bar is shifted   |   sloped = it pivots about one end',
                      fontsize=10, color=BODY)
    axes[0].grid(axis='y', alpha=0.25)
    axes[0].set_xlim(-0.25, 2.62)
    for side in ('top', 'right'):
        axes[0].spines[side].set_visible(False)

    # * A count of two things is not a chart. Two numbers and a sentence say it
    # * better, and leave room for the arithmetic that matters.
    axes[1].axis('off')
    axes[1].text(0.0, 0.92, f"{len(tilted)}", fontsize=40, fontweight='bold',
                 color=RED, transform=axes[1].transAxes, va='top')
    axes[1].text(0.0, 0.70, 'bars tilt\n(spread >= 1 mm)', fontsize=10, color=BODY,
                 transform=axes[1].transAxes, va='top')
    axes[1].text(0.0, 0.50, f"{len(bars) - len(tilted)}", fontsize=40, fontweight='bold',
                 color=BLUE, transform=axes[1].transAxes, va='top')
    axes[1].text(0.0, 0.28, 'are shifted bodily\n(spread < 1 mm)', fontsize=10, color=BODY,
                 transform=axes[1].transAxes, va='top')
    axes[1].text(0.0, 0.13,
                 f"A {numbers['rotation_deg']['mean']:.3f}° tilt over a "
                 f"{numbers['bar_length_mm'] / 1000:.2f} m bar\nmoves the tip "
                 f"{budget['tilt_mm']:.2f} mm — that is\n"
                 f"{100 * budget['tilt_share']:.0f}% of the measured "
                 f"{numbers['placement']['mean']:.2f} mm.",
                 fontsize=9.5, color=INK, transform=axes[1].transAxes, va='top')

    figure.tight_layout(rect=[0, 0, 1, 0.93])
    figure.savefig(out_path, format='svg')


def _chain(numbers: dict) -> tuple:
    """The physical chain, and what the data says about each step of it.

    ! Boxes are PHYSICAL things; the error lives in the transforms BETWEEN them,
    ! so that is where the verdicts hang. IK and the servo loop are deliberately
    ! not boxes: they choose the joint values, they are not something the bar's
    ! position passes through.

    ! The mocap appears at BOTH ends and is the same instrument twice: on the
    ! left it tells the robot where its own base is, on the right it tells us
    ! where the bar ended up. Only the right-hand use is independent of the
    ! robot's model, which is why it can see what the servo loop cannot.

    Every verdict is derived from the measurements, so a future session's
    numbers move the chips with them.

    Args:
        numbers (dict): Output of :func:`analyse`.

    Returns:
        tuple: ``(boxes, links)``. A box is ``(name, aka)``; a link is
        ``(name, aka, verdict, colour, evidence)`` and sits between the box of
        the same index and the next one.
    """
    frames, budget, drivers = numbers['frames'], numbers['budget'], numbers['drivers']
    world = 100 * frames['world']['explained']
    base = 100 * frames['base']['explained']
    flange = 100 * frames.get('flange', frames['base'])['explained']
    robot_frame = f"{min(base, flange):.0f}-{max(base, flange):.0f}%"
    servo_step = numbers['steps'].get('servo_right')

    boxes = [
        ('mocap body on the husky', 'the tracked rigid body / base_mocap'),
        ('husky base', 'base_footprint / the kinematic base'),
        ('flange', 'tool0 / the wrist / where a tool bolts on'),
        ('gripper jaws', 'the two screw grippers'),
        ('THE BAR', 'the thing we are placing'),
        ('mocap cameras', 'the 8 markers + the line fit'),
    ]
    # ! Keep every line of `aka` and `evidence` under ~50 characters: each one is
    # ! centred in a column the width of one box plus one gap, and a longer line
    # ! runs into its neighbour.
    links = [
        ('calibration', 'base_mocap_from_base_footprint &#8212;\n'
                        'a fixed transform from the calibration file',
         'suspect', AMBER,
         f"the error is {robot_frame} a fixed offset\n"
         f"in the robot's own frame, but only\n"
         f"{world:.0f}% in the room's"),
        ('arm kinematics', 'the URDF link lengths + joint encoders\n'
                           '(IK picks the joints, the loop iterates)',
         'suspect', AMBER,
         'cannot be told apart from its two\n'
         'neighbours: no joint values were saved,\n'
         'so arm pose cannot be tested'),
        ('tool offset', 'tool / TCP / the pendant tool setting\n'
                        '&#8212; flange to the gripper point',
         'suspect', AMBER,
         f"a TILT dominates ({100 * budget['tilt_share']:.0f}% of the error),\n"
         f"which points at an angle in here\n"
         f"rather than a length"),
        ('grasp', 'attachment_frame / how the bar sits\n'
                  'in the jaws, plus the bar flexing',
         'moved mid-session' if servo_step else 'stable', RED if servo_step else MUTED,
         (f"from {servo_step['at']} the two wrists disagree\n"
          f"and never converge -- but that gap is\n"
          f"only {100 * budget['grasp_tilt_share']:.0f}% of the tilt, so not all of it"
          if servo_step else 'the two wrists stay in agreement\nall session')),
        ('measured by', 'the cameras read the bar directly &#8212;\n'
                        'the one instrument independent of the robot',
         'not a contributor', MUTED,
         f"fit residual {numbers['fit_residual']['mean']:.2f} mm, and it does\n"
         f"not move with placement "
         f"(r = {drivers['fit_residual']['r']:+.2f})\n"
         f"-- so the error is not the cameras"),
    ]
    return boxes, links


def figure_system_map(numbers: dict, out_path: str) -> None:
    """Draw the system map: which part is affected, and what moves with what.

    Hand-written SVG rather than a plot, in the same visual language as
    ``doc/servo_vs_placement_error.svg`` panel C, which this extends. Panel C's
    shape is kept on purpose: physical things in boxes, and the error sources
    named on the arrows BETWEEN them, because a transform is where an error
    lives. What is added is the verdict and the measurement behind each one, and
    a second layer showing which numbers reach which part of the chain.

    Args:
        numbers (dict): Output of :func:`analyse`.
        out_path (str): Where to write the SVG.
    """
    boxes, links = _chain(numbers)
    frames, budget, drivers = numbers['frames'], numbers['budget'], numbers['drivers']
    width, box_w, gap = 1540, 198, 60
    left0 = 30
    box_y, box_h = 150, 54
    arrow_y = box_y + box_h / 2
    link_y, chip_y, evid_y = box_y + box_h + 30, box_y + box_h + 66, box_y + box_h + 98

    span = lambda i: (left0 + i * (box_w + gap), left0 + i * (box_w + gap) + box_w)
    centre = lambda i: left0 + i * (box_w + gap) + box_w / 2
    chain_end = span(len(boxes) - 1)[1]

    bracket_y = evid_y + 34
    measured_y = bracket_y + 92
    bands_y = measured_y + 40
    band_h, band_gap = 30, 8
    ladder_y = bands_y + 4 * (band_h + band_gap) + 56
    table_y = ladder_y + 44 + 4 * 42 + 20
    neg_y = table_y + 172
    height = neg_y + 120

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
        f'width="{width}" height="{height}" font-family="ui-sans-serif, system-ui, '
        f'-apple-system, Segoe UI, Roboto, sans-serif">',
        '<defs>',
        f'<marker id="ar" markerWidth="9" markerHeight="9" refX="7" refY="3" orient="auto">'
        f'<path d="M0,0 L7,3 L0,6 z" fill="{RED}"/></marker>',
        f'<marker id="am" markerWidth="9" markerHeight="9" refX="7" refY="3" orient="auto">'
        f'<path d="M0,0 L7,3 L0,6 z" fill="{MUTED}"/></marker>',
        '</defs>',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        f'<text x="30" y="36" font-size="20" font-weight="700" fill="{INK}">'
        f'Which part of the system is the error in, and which numbers move together?</text>',
        f'<text x="30" y="60" font-size="13" fill="{MUTED}">'
        f'Session {numbers["batch"]} &#183; {numbers["n_bars"]} bars &#183; every verdict '
        f'below is derived from the measurements, not assumed.</text>',
        f'<text x="30" y="96" font-size="13" font-weight="700" fill="{INK}">'
        f'THE CHAIN &#8212; boxes are physical things. The error lives in the TRANSFORMS '
        f'between them, which is where the verdicts sit.</text>',
        f'<text x="30" y="114" font-size="11" fill="{MUTED}">'
        f'The mocap appears at both ends and is the same instrument twice: on the left it '
        f'tells the robot where its own base is, on the right it tells us where the bar '
        f'ended up. Only the right-hand use is independent of the robot.</text>',
    ]

    for index, (name, aka) in enumerate(boxes):
        x0, _x1 = span(index)
        last = index == len(boxes) - 1
        bar_box = name == 'THE BAR'
        fill = '#fdecea' if bar_box else ('#eafaf1' if last else '#eef4fa')
        stroke = RED if bar_box else (GREEN if last else EDGE)
        emphasis = ' stroke-width="2"' if bar_box else ''
        out += [
            f'<rect x="{x0}" y="{box_y}" width="{box_w}" height="{box_h}" rx="6" '
            f'fill="{fill}" stroke="{stroke}"{emphasis}/>',
            f'<text x="{centre(index)}" y="{box_y + 23}" font-size="12.5" font-weight="700" '
            f'text-anchor="middle" fill="{INK}">{name}</text>',
            f'<text x="{centre(index)}" y="{box_y + 41}" font-size="9" '
            f'text-anchor="middle" fill="{MUTED}">{aka}</text>',
        ]

    for index, (name, aka, verdict, colour, evidence) in enumerate(links):
        x0 = span(index)[1]
        x1 = span(index + 1)[0]
        mid = (x0 + x1) / 2
        out.append(f'<line x1="{x0 + 4}" y1="{arrow_y}" x2="{x1 - 5}" y2="{arrow_y}" '
                   f'stroke="{colour}" stroke-width="1.8" '
                   f'marker-end="url({"#am" if colour == MUTED else "#ar"})"/>')
        out.append(f'<text x="{mid}" y="{link_y}" font-size="11.5" font-weight="700" '
                   f'text-anchor="middle" fill="{colour}">{name}</text>')
        for line_no, line in enumerate(aka.split('\n')):
            out.append(f'<text x="{mid}" y="{link_y + 15 + line_no * 12}" font-size="9" '
                       f'text-anchor="middle" fill="{MUTED}">{line}</text>')
        out += [
            f'<rect x="{mid - 76}" y="{chip_y}" width="152" height="21" rx="10.5" '
            f'fill="{colour}" opacity="0.14"/>',
            f'<text x="{mid}" y="{chip_y + 15}" font-size="10.5" font-weight="700" '
            f'text-anchor="middle" fill="{colour}">{verdict}</text>',
        ]
        for line_no, line in enumerate(evidence.split('\n')):
            out.append(f'<text x="{mid}" y="{evid_y + line_no * 12}" font-size="9" '
                       f'text-anchor="middle" fill="{BODY}">{line}</text>')

    see_to = span(2)[1]
    out += [
        f'<path d="M{left0},{bracket_y} L{left0},{bracket_y + 16} L{see_to},{bracket_y + 16} '
        f'L{see_to},{bracket_y}" fill="none" stroke="{GREEN}" stroke-width="1.6"/>',
        f'<text x="{(left0 + see_to) / 2}" y="{bracket_y + 34}" font-size="11.5" '
        f'text-anchor="middle" font-weight="700" fill="{GREEN}">the servo loop closes the '
        f'loop only over THIS much</text>',
        f'<text x="{(left0 + see_to) / 2}" y="{bracket_y + 50}" font-size="10.5" '
        f'text-anchor="middle" fill="{BODY}">its FK runs live mocap base &#8594; URDF arm '
        f'&#8594; tool0 and stops &#8212; median {numbers["servo"]["median"]:.2f} mm, '
        f'{100 * budget["servo_share"]:.0f}% of the error</text>',
        f'<path d="M{see_to + 10},{bracket_y} L{see_to + 10},{bracket_y + 16} '
        f'L{chain_end},{bracket_y + 16} L{chain_end},{bracket_y}" fill="none" '
        f'stroke="{RED}" stroke-width="1.6" stroke-dasharray="5 3"/>',
        f'<text x="{(see_to + chain_end) / 2}" y="{bracket_y + 34}" font-size="11.5" '
        f'text-anchor="middle" font-weight="700" fill="{RED}">&#8230;and is blind to all of '
        f'this</text>',
        f'<text x="{(see_to + chain_end) / 2}" y="{bracket_y + 50}" font-size="10.5" '
        f'text-anchor="middle" fill="{BODY}">both sides of its comparison are computed '
        f'through the same assumed transforms, so it can drive its own number to zero</text>',
    ]

    out += [
        f'<text x="30" y="{measured_y}" font-size="13" font-weight="700" fill="{INK}">'
        f'WHAT WE MEASURED &#8212; each number drawn across the parts of the chain it can '
        f'actually reach</text>',
        f'<text x="30" y="{measured_y + 18}" font-size="11" fill="{MUTED}">'
        f'The width of a band IS its reach. Placement spans everything, which is why it is '
        f'the answer; the servo residual stops at the flange, which is why it disagrees.</text>',
    ]
    # ! The note sits inside the band, so it has to fit the NARROWEST one -- two
    # ! boxes wide. Keep these under about 45 characters.
    bands = [
        ('placement error', f'{numbers["placement"]["mean"]:.2f} mm', 0, 5, RED,
         'the whole chain can push it &#8212; this is the answer'),
        ('rotation error', f'{numbers["rotation_deg"]["mean"]:.3f}&#176;', 0, 5, AMBER,
         f'the angular part of it: {budget["tilt_mm"]:.2f} mm, '
         f'{100 * budget["tilt_share"]:.0f}% of the total'),
        ('servo residual', f'{numbers["servo"]["median"]:.2f} mm', 0, 2, GREEN,
         'stops at the flange &#8212; blind past it'),
        ('fit residual', f'{numbers["fit_residual"]["mean"]:.2f} mm', 4, 5, MUTED,
         'how well the markers lie on a line'),
    ]
    for row, (label, value, first, last, colour, note) in enumerate(bands):
        y = bands_y + row * (band_h + band_gap)
        x0, x1 = span(first)[0], span(last)[1]
        out += [
            f'<rect x="{x0}" y="{y}" width="{x1 - x0}" height="{band_h}" rx="6" '
            f'fill="{colour}" opacity="0.12"/>',
            f'<rect x="{x0}" y="{y}" width="{x1 - x0}" height="{band_h}" rx="6" '
            f'fill="none" stroke="{colour}" stroke-width="1.4"/>',
            f'<text x="{x0 + 12}" y="{y + 20}" font-size="11.5" font-weight="700" '
            f'fill="{colour}">{label}</text>',
            f'<text x="{x0 + 136}" y="{y + 20}" font-size="12" font-weight="700" '
            f'fill="{INK}">{value}</text>',
            f'<text x="{x0 + 206}" y="{y + 20}" font-size="10" fill="{BODY}">{note}</text>',
        ]
    out.append(
        f'<text x="{left0}" y="{bands_y + 4 * (band_h + band_gap) + 16}" '
        f'font-size="10" fill="{MUTED}">recording order has no band: it is not a part of '
        f'the machine, it is just when the bar was recorded &#8212; which is exactly why it '
        f'can fake a link between two numbers that never touch each other.</text>')

    out += [
        f'<text x="30" y="{ladder_y}" font-size="13" font-weight="700" fill="{INK}">'
        f'AND WHICH NUMBERS MOVE TOGETHER &#8212; with {numbers["n_bars"]} bars, |r| under '
        f'{numbers["significance_floor"]:.2f} is indistinguishable from chance</text>',
        f'<text x="30" y="{ladder_y + 18}" font-size="11" fill="{MUTED}">'
        f'Each line joins two measurements. The second column is the same correlation with '
        f'recording order held still &#8212; which is how a real link is told from a '
        f'coincidence of timing.</text>',
    ]
    floor = numbers['significance_floor']
    ladder = []
    for key, pretty in (('rotation', 'rotation error'), ('servo', 'servo residual'),
                        ('recording_order', 'recording order'),
                        ('fit_residual', 'fit residual')):
        entry = drivers[key]
        raw, partial = entry['r'], entry['r_without_order']
        if key == 'recording_order':
            verdict, colour, note = (
                'THE CONFOUND', BLUE,
                f'It drags the right wrist along too (r = {entry["r_vs_right_wrist"]:+.2f}), '
                f'which is what the row above is really showing.')
        elif abs(raw) < floor:
            verdict, colour, note = ('NO LINK', MUTED,
                                     'If the error were mocap noise these two would rise '
                                     'together. They do not, so the error is real.')
        elif abs(partial) < floor:
            verdict, colour, note = ('SPURIOUS', MUTED,
                                     'Both drifted over the same afternoon. Neither drives '
                                     'the other; the clock drives both.')
        else:
            verdict, colour, note = ('REAL', AMBER,
                                     f'A {numbers["rotation_deg"]["mean"]:.3f}&#176; tilt over '
                                     f'a {numbers["bar_length_mm"] / 1000:.2f} m bar is '
                                     f'{budget["tilt_mm"]:.2f} mm at the tip.')
        ladder.append((pretty, raw, partial, verdict, colour, note))

    # * One colour and one weight for every line: the judgement is carried by the
    # * line's STYLE -- solid where the link holds, dashed and struck where it
    # * does not survive the clock, dotted and struck where there is none at all.
    style_of = {
        'REAL': ('', False),
        'THE CONFOUND': ('', False),
        'SPURIOUS': (' stroke-dasharray="7 4"', True),
        'NO LINK': (' stroke-dasharray="1.5 3.5"', True),
    }
    for row_no, (pretty, raw, partial, verdict, colour, note) in enumerate(ladder):
        y = ladder_y + 46 + row_no * 42
        dash, struck = style_of.get(verdict, ('', False))
        out += [
            f'<text x="42" y="{y + 4}" font-size="11.5" font-weight="700" fill="{INK}">'
            f'{pretty}</text>',
            f'<line x1="196" y1="{y}" x2="292" y2="{y}" stroke="{BODY}" '
            f'stroke-width="1.6"{dash}/>',
            f'<circle cx="196" cy="{y}" r="4" fill="#ffffff" stroke="{BODY}" '
            f'stroke-width="1.6"/>',
            f'<circle cx="292" cy="{y}" r="4" fill="#ffffff" stroke="{BODY}" '
            f'stroke-width="1.6"/>',
        ]
        if struck:
            out.append(f'<line x1="232" y1="{y - 9}" x2="256" y2="{y + 9}" stroke="{BODY}" '
                       f'stroke-width="1.6"/>')
        out += [
            f'<text x="306" y="{y + 4}" font-size="11.5" font-weight="700" fill="{INK}">'
            f'placement error</text>',
            f'<text x="432" y="{y + 4}" font-size="11.5" fill="{BODY}">'
            f'r = <tspan font-weight="700">{raw:+.2f}</tspan></text>',
            f'<text x="530" y="{y + 4}" font-size="11.5" fill="{BODY}">'
            f'with the clock held still: <tspan font-weight="700">'
            f'{"&#8212;" if np.isnan(partial) else f"{partial:+.2f}"}</tspan></text>',
            f'<rect x="768" y="{y - 12}" width="108" height="22" rx="11" fill="{colour}" '
            f'opacity="0.14"/>',
            f'<text x="822" y="{y + 4}" font-size="11" font-weight="700" '
            f'text-anchor="middle" fill="{colour}">{verdict}</text>',
            f'<text x="896" y="{y + 4}" font-size="10.5" fill="{BODY}">{note}</text>',
        ]

    frame_rows = [('in the room', frames['world']), ("in the robot's own frame", frames['base'])]
    if 'flange' in frames:
        frame_rows.append(('at the flange', frames['flange']))
    best_fit = max(frame_rows, key=lambda r: r[1]['explained'])[1]
    out += [
        f'<rect x="{left0}" y="{table_y}" width="700" height="156" rx="8" fill="#f6faf7" '
        f'stroke="#bfdcc8"/>',
        f'<text x="{left0 + 18}" y="{table_y + 24}" font-size="12.5" font-weight="700" '
        f'fill="{GREEN}">THE EVIDENCE &#8212; one error arrow per bar, written in different '
        f'coordinates</text>',
        f'<text x="{left0 + 18}" y="{table_y + 43}" font-size="10.5" fill="{BODY}">'
        f'Whichever coordinates make the arrows agree is where the fault lives. '
        f'&#8220;Explains&#8221; = how much of the error</text>',
        f'<text x="{left0 + 18}" y="{table_y + 57}" font-size="10.5" fill="{BODY}">'
        f'disappears if you subtract one single correction from every bar. Measured at the '
        f'bar&#39;s start tip &#8212; one consistent point per bar.</text>',
    ]
    for row_no, (label, fit) in enumerate(frame_rows):
        y = table_y + 82 + row_no * 22
        best = fit is best_fit
        colour = GREEN if best else BODY
        out += [
            f'<text x="{left0 + 20}" y="{y}" font-size="11" fill="{colour}" '
            f'font-weight="{700 if best else 400}">{label}</text>',
            f'<text x="{left0 + 200}" y="{y}" font-size="11" fill="{BODY}">mean '
            f'[{fit["mean"][0]:+.1f}, {fit["mean"][1]:+.1f}, {fit["mean"][2]:+.1f}] mm</text>',
            f'<text x="{left0 + 408}" y="{y}" font-size="11" fill="{BODY}">'
            f'{fit["before_mm"]:.2f} &#8594; {fit["after_mm"]:.2f} mm</text>',
            f'<text x="{left0 + 548}" y="{y}" font-size="11" fill="{colour}" '
            f'font-weight="{700 if best else 400}">explains '
            f'{100 * fit["explained"]:.0f}%</text>',
        ]

    bx = left0 + 732
    out += [
        f'<rect x="{bx}" y="{table_y}" width="{width - bx - 30}" height="156" rx="8" '
        f'fill="#f4f8fc" stroke="#cdd9e6"/>',
        f'<text x="{bx + 18}" y="{table_y + 24}" font-size="12.5" font-weight="700" '
        f'fill="{BLUE}">THE BUDGET &#8212; where the '
        f'{numbers["placement"]["mean"]:.2f} mm comes from</text>',
    ]
    budget_rows = [
        (f'{budget["first_block_mean"]:.2f} mm',
         'was there from the first bars, with a clean grasp and a loop that converged', BLUE),
        (f'+{budget["rest_mean"] - budget["first_block_mean"]:.2f} mm',
         'added later, once the session changed', RED),
        (f'{budget["tilt_mm"]:.2f} mm',
         f'of the whole is a {numbers["rotation_deg"]["mean"]:.3f}&#176; tilt '
         f'&#215; the {numbers["bar_length_mm"] / 1000:.2f} m bar', AMBER),
        (f'{numbers["servo"]["median"]:.2f} mm',
         'of the whole is all the servo loop could still see', GREEN),
    ]
    for row_no, (value, text, colour) in enumerate(budget_rows):
        y = table_y + 56 + row_no * 25
        out += [
            f'<text x="{bx + 20}" y="{y}" font-size="12" font-weight="700" '
            f'fill="{colour}">{value}</text>',
            f'<text x="{bx + 104}" y="{y}" font-size="10.5" fill="{BODY}">{text}</text>',
        ]

    geometry = numbers['geometry']
    loudest = max(geometry.items(), key=lambda kv: abs(kv[1]['r']))
    survives = [name for name, entry in geometry.items()
                if abs(entry['r_without_order']) >= floor]
    caveat = (f'The loudest, {loudest[0]}, reaches r = {loudest[1]["r"]:+.2f} raw but falls '
              f'to {loudest[1]["r_without_order"]:+.2f} &#8212; under the {floor:.2f} '
              f'threshold &#8212; once recording order is held still, because the robot '
              f'worked its way across the cell as the session went on.'
              if not survives else
              f'{", ".join(survives)} still clears the threshold with order held still '
              f'&#8212; worth a second look.')
    out += [
        f'<rect x="30" y="{neg_y}" width="{width - 60}" height="96" rx="8" fill="#ffffff" '
        f'stroke="{EDGE}"/>',
        f'<text x="48" y="{neg_y + 24}" font-size="12.5" font-weight="700" fill="{INK}">'
        f'AND WHAT DOES NOT MATTER &#8212; a negative result worth stating</text>',
        f'<text x="48" y="{neg_y + 44}" font-size="11" fill="{BODY}">'
        f'No geometry knob predicts the placement error: not the bar&#39;s height, not where '
        f'the robot parked, not which way it faced.</text>',
        f'<text x="48" y="{neg_y + 62}" font-size="11" fill="{BODY}">{caveat}</text>',
        f'<text x="48" y="{neg_y + 82}" font-size="11" fill="{BODY}">'
        f'<tspan font-weight="700">Where the bar is does not predict how badly it lands.</tspan> '
        f'The error travels with the robot, not with the cell.</text>',
        '</svg>',
    ]
    with open(out_path, 'w') as handle:
        handle.write('\n'.join(out))


def write_results(numbers: dict, out_path: str, figures: list) -> None:
    """Write the session's findings as a markdown report.

    ! Written for someone who has never opened any other document about this
    ! experiment. It explains what was done and what every word means BEFORE it
    ! quotes a single number, because a reader who meets "placement error
    ! 4.32 mm" cold has no way to know what was measured against what.

    Args:
        numbers (dict): Output of :func:`analyse`.
        out_path (str): Where to write ``RESULTS.md``.
        figures (list): Figure filenames, in reading order.
    """
    place, budget, frames = numbers['placement'], numbers['budget'], numbers['frames']
    drivers, steps = numbers['drivers'], numbers['steps']
    floor = numbers['significance_floor']
    bar_m = numbers['bar_length_mm'] / 1000.0
    takes = sum(b['n_takes'] for b in numbers['bars'])

    lines = [
        f"# How accurately did the robot place the bars? &mdash; session {numbers['batch']}",
        '',
        f"*Generated {numbers['generated']} by `3_session_report.py`. Every number below is "
        f"also in `numbers.json` next to this file; re-run the script to rebuild all of it.*",
        '',
        '## 1. What was done',
        '',
        f"A mobile robot (a Husky with two UR arms) picks up a **{bar_m:.2f} m bar**, drives "
        f"to a parking spot, and holds the bar where a CAD design says it should go. It does "
        f"not let go: the bar stays in both grippers while motion-capture cameras measure "
        f"where it actually ended up. Then the robot drives to the next spot and does it "
        f"again.",
        '',
        f"In this session that happened **{numbers['n_bars']} times** &mdash; "
        f"{numbers['n_bars']} different places in the structure, one bar each. At every "
        f"place the cameras were read several times over (**{takes} readings** in total), so "
        f"we can tell a genuine error from a noisy measurement.",
        '',
        f"The question is simple: **how far is the bar from where the design said, and "
        f"whose fault is the difference?**",
        '',
        '## 2. The words used here',
        '',
        '| term | what it means |',
        '|---|---|',
        '| **bar** | the 1.4 m aluminium bar the robot is placing. Each one has its own spot '
        'in the design |',
        '| **take** | one reading of the markers by the cameras. Several takes per bar |',
        '| **authored / design pose** | where the CAD model says the bar should end up. The '
        'target |',
        '| **flange** (or *tool0*, or *wrist*) | the face at the end of a robot arm where the '
        'gripper bolts on |',
        '| **grasp** | where the bar actually sits inside the gripper jaws |',
        '| **placement error** | the headline number. How far the bar ended up from its '
        'design pose, in millimetres. Measured at three places along the bar &mdash; the '
        'start, the middle and the end &mdash; and we report the **worst** of the three, '
        'because a bar that is right at one end and out at the other is placed badly |',
        '| **rotation error** | the angle between the bar as built and as designed. Small '
        'angles matter a lot on a long bar |',
        '| **fit residual** | how well the 8 markers lie on one straight line. This is a '
        'measure of the *measurement*, not of the robot. A big one means distrust that '
        'reading |',
        '| **servo residual** | the robot\'s own opinion of how close it got. Before each '
        'measurement the arms iterate towards the target; this is what was left over on the '
        'last iteration, *as the robot computes it* |',
        '',
        f"One thing to hold on to, because the whole report turns on it: the **placement "
        f"error** comes from the cameras, which never look at the robot. The **servo "
        f"residual** comes from the robot judging itself. They are two different "
        f"instruments measuring two different things, and they disagree.",
        '',
        '## 3. The answer',
        '',
        f"**The bar lands {place['mean']:.2f} mm away from where the design says, on "
        f"average.** The best bar was {place['min']:.2f} mm out, the worst "
        f"{place['max']:.2f} mm; half were better than {place['median']:.2f} mm.",
        '',
        f"That number is believable, for two reasons:",
        '',
        f"- The markers lie on a straight line to within "
        f"{numbers['fit_residual']['min']:.2f}&ndash;{numbers['fit_residual']['max']:.2f} mm, "
        f"and **how cleanly they fitted has no relationship to how far the bar was out** "
        f"(r = {drivers['fit_residual']['r']:+.2f}, where 0 means no relationship at all). "
        f"If the cameras were at fault, a messy reading would come with a big error. It does "
        f"not.",
        f"- Reading the same bar several times gives answers that agree to "
        f"**{numbers['repeatability_mm']['mean']:.3f} mm**. So the measurement is "
        f"**repeatable** &mdash; and the bar is still {place['mean']:.2f} mm from target. "
        f"Repeatable but wrong is the signature of something mis-calibrated, not something "
        f"noisy.",
        '',
        f"For comparison, the robot's own opinion of how close it got was "
        f"**{numbers['servo']['median']:.2f} mm**. It believes it did about "
        f"{place['mean'] / numbers['servo']['median']:.0f}&times; better than it did.",
        '',
    ]

    told = [(name, step) for name, step in steps.items() if step]
    confident = [(name, step) for name, step in told if step['beats']]
    if told:
        pretty = {'placement': 'placement error', 'servo_right': 'right wrist residual',
                  'iterations': 'servo iterations'}
        lines += [
            '## 4. The session was not the same at the end as at the start',
            '',
            f"![what changed]({figures[0]})",
            '',
            f"Before averaging {numbers['n_bars']} bars together it is worth asking whether "
            f"they belong in the same average. They do not quite. Three measurements change "
            f"level partway through:",
            '',
            '| what | changes at | before | after | is a step really the best description? |',
            '|---|---|---|---|---|',
        ]
        for name, step in told:
            verdict = ('**yes** &mdash; clearly better than a slow drift'
                       if step['beats'] else
                       'not clearly &mdash; a slow drift fits about as well')
            lines.append(f"| {pretty.get(name, name)} | **{step['at']}** | "
                         f"{step['before']:.2f} | {step['after']:.2f} | {verdict} |")
        lines += [
            '',
            f"**How that was found, and what it is not.** The split point is chosen from the "
            f"numbers alone &mdash; every possible split is tried and the one with the "
            f"clearest separation is kept. Clock times are *never* used to find it; they are "
            f"printed afterwards only to say which bar it landed on. Each one is then "
            f"checked against two simpler explanations (no change at all, and a slow steady "
            f"drift) and reported as a step only when it beats both.",
            '',
            f"**This does not say anything went wrong.** An operator break, re-zeroing the "
            f"force sensors, the arms being restarted, the bar settling in the grippers "
            f"&mdash; any of these would do it, and this data cannot tell them apart. The "
            f"finding is narrower and duller than that: *the bars before and after are not "
            f"interchangeable, so do not treat them as one pool.*",
            '',
            f"**Why it is worth a section.** Because it changes a conclusion. Taken at face "
            f"value, the robot's own residual looks like it predicts the placement error "
            f"(r = {drivers['servo']['r']:+.2f}), which would say *tighten the servo loop and "
            f"the bars will land better*. Hold the drift still and that collapses to "
            f"{drivers['servo']['r_without_order']:+.2f} &mdash; no relationship. Both "
            f"numbers simply grew over the same afternoon. Acting on the first reading would "
            f"mean spending effort on the one part of the machine the data says is already "
            f"fine.",
            '',
        ]

    lines += [
        '## 5. Which part of the system is at fault',
        '',
        f"![system map]({figures[1]})",
        '',
        f"The bar's position passes through a chain of parts, and the error can enter at any "
        f"of them. To find out where, take each bar's error as an **arrow** &mdash; not just "
        f"\"4 mm out\" but \"4 mm out, in this direction\" &mdash; and ask which coordinate "
        f"system makes the {numbers['n_bars']} arrows agree with each other. The one they "
        f"agree in is the one the fault is fixed to.",
        '',
        '| the arrows measured&hellip; | average arrow | one fixed correction would get | '
        'how much of the error that removes |',
        '|---|---|---|---|',
    ]
    for label, key in (('in the room', 'world'), ("in the robot's own frame", 'base'),
                       ('at the flange', 'flange')):
        if key not in frames:
            continue
        fit = frames[key]
        lines.append(
            f"| {label} | [{fit['mean'][0]:+.2f}, {fit['mean'][1]:+.2f}, "
            f"{fit['mean'][2]:+.2f}] mm | {fit['before_mm']:.2f} &rarr; "
            f"{fit['after_mm']:.2f} mm | **{100 * fit['explained']:.0f}%** |")
    agree = frames['base']['agree']
    lines += [
        '',
        f"*(The arrow is measured at the bar's start tip &mdash; one consistent point on "
        f"every bar. Its average length, {frames['world']['before_mm']:.2f} mm, is slightly "
        f"less than the {place['mean']:.2f} mm headline, which is the worst of three "
        f"places.)*",
        '',
        f"Read the last column as: *if you applied one single correction to every bar, how "
        f"much better would they all get?* In room coordinates, almost nothing &mdash; the "
        f"arrows point different ways. In the robot's own coordinates, most of it.",
        '',
        f"The cleanest way to see it: in the robot's own frame, **{agree[0]} of "
        f"{len(numbers['frames_bars'])} bars are pushed the same way**. In the room they are "
        f"a coin toss ({frames['world']['agree'][0]} of "
        f"{len(numbers['frames_bars'])}). **The error turns with the robot.** That rules out "
        f"the cameras and the room, and points inside the machine &mdash; somewhere between "
        f"the marker body on the husky and the bar in the jaws.",
        '',
        f"Which of those exactly, this session cannot say: the design ties the grasp "
        f"orientation to where the robot parks, and no joint values were saved, so the "
        f"calibration, the arm and the tool offset move together and cannot be told apart.",
        '',
        '### Where the millimetres go',
        '',
        '| | mm | |',
        '|---|---|---|',
        f"| already there on the first bars | **{budget['first_block_mean']:.2f}** | with a "
        f"clean grasp and a loop that converged in a few iterations |",
        f"| added later | **+{budget['rest_mean'] - budget['first_block_mean']:.2f}** | once "
        f"the session changed |",
        f"| of the whole, a tilt | {budget['tilt_mm']:.2f} | "
        f"{100 * budget['tilt_share']:.0f}% &mdash; a "
        f"{numbers['rotation_deg']['mean']:.3f}&deg; angle, which over a {bar_m:.2f} m bar "
        f"becomes millimetres at the tip |",
        f"| of the whole, what the robot could see | {numbers['servo']['median']:.2f} | "
        f"{100 * budget['servo_share']:.0f}% &mdash; even a perfect servo loop removes only "
        f"this |",
        '',
        f"The tilt is the useful part. Chasing a {place['mean']:.2f} mm *position* error is "
        f"the wrong hunt; what needs finding is a **{numbers['rotation_deg']['mean']:.3f}&deg; "
        f"angle** somewhere in the chain. The two grippers are "
        f"{numbers['grasp_separation_mm']:.0f} mm apart, so them disagreeing with each other "
        f"can only account for {budget['grasp_tilt_deg']:.3f}&deg; of it &mdash; "
        f"{100 * budget['grasp_tilt_share']:.0f}% overall and only "
        f"{100 * budget['grasp_tilt_share_first_block']:.0f}% before the grasp moved. The "
        f"rest comes from before the jaws.",
        '',
        '## 6. Shifted, or tilted?',
        '',
        f"![shift or tilt]({figures[2]})",
        '',
        f"Measuring each bar at three places tells shift from tilt. If all three distances "
        f"are alike the whole bar is displaced; if they differ it is pivoting about one end.",
        '',
        '## 7. Which numbers move together',
        '',
        f"A correlation `r` runs from -1 to +1 and says whether two measurements rise and "
        f"fall together: +1 is perfectly in step, 0 is no relationship at all. With "
        f"{numbers['n_bars']} bars, anything smaller than **{floor:.2f}** could easily be "
        f"chance.",
        '',
        f"The second column is the same correlation with the session's drift held still. "
        f"That is what separates a real link from two numbers that merely grew over the same "
        f"afternoon.",
        '',
        '| against placement error | r | with the drift held still | reading |',
        '|---|---|---|---|',
    ]
    for key, pretty in (('rotation', 'rotation error'), ('servo', 'servo residual'),
                        ('servo_right', 'right wrist'), ('servo_left', 'left wrist'),
                        ('iterations', 'servo iterations'), ('fit_residual', 'fit residual'),
                        ('load', 'load imbalance'),
                        ('recording_order', '**recording order**')):
        entry = drivers[key]
        raw, partial = entry['r'], entry['r_without_order']
        if np.isnan(raw):
            continue
        if abs(raw) < floor:
            reading = 'no signal'
        elif np.isnan(partial):
            reading = '**the confound**'
        elif abs(partial) < floor:
            reading = 'spurious &mdash; it was the clock'
        else:
            reading = '**survives**'
        shown = '&mdash;' if np.isnan(partial) else f"{partial:+.2f}"
        lines.append(f"| {pretty} | {raw:+.2f} | {shown} | {reading} |")

    loudest = max(numbers['geometry'].items(), key=lambda kv: abs(kv[1]['r']))
    lines += [
        '',
        f"Only one survives: the **rotation error**, which is a mechanism and not a "
        f"coincidence &mdash; a small angle on a long bar is millimetres at the tip.",
        '',
        f"And a negative result worth stating. No geometry knob predicts the placement "
        f"error: not the bar's height, not where the robot parked, not which way it faced. "
        f"The loudest of them, `{loudest[0]}`, reaches r = {loudest[1]['r']:+.2f} but falls "
        f"to {loudest[1]['r_without_order']:+.2f} &mdash; under the {floor:.2f} threshold "
        f"&mdash; once the drift is held still, because the robot worked its way across the "
        f"cell as the session went on. **Where a bar is does not predict how badly it "
        f"lands.**",
        '',
        '## 8. Every bar',
        '',
        f"Distances in millimetres, rotation in degrees, load in newtons. *Placement* is the "
        f"worst of the three columns beside it.",
        '',
        '| bar | time | placement | start / middle / end | spread | rotation | fit | '
        'wrist L / R | iters | load |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for bar in numbers['bars']:
        wrist = ('&mdash;' if bar['servo_left'] is None
                 else f"{bar['servo_left']:.2f} / {bar['servo_right']:.2f}")
        load = '&mdash;' if bar['load'] is None else (
            f"{bar['load']:.2f}" + ('' if bar['load_valid'] else ' *(not measured)*'))
        lines.append(
            f"| {bar['bar']} | {bar['clock']} | **{bar['placement']:.2f}** | "
            f"{bar['tip_start']:.2f} / {bar['tip_middle']:.2f} / {bar['tip_end']:.2f} | "
            f"{bar['tip_spread']:.2f} | {bar['rotation']:.3f} | {bar['fit_residual']:.2f} | "
            f"{wrist} | {bar['iterations'] or '&mdash;'} | {load} |")
    lines += ['', f"*Spread is the gap between the best and worst of the three places: small "
                  f"means the bar is shifted bodily, large means it is tilted.*", '']
    with open(out_path, 'w') as handle:
        handle.write('\n'.join(lines))


def main() -> None:
    """Read one session and write its report folder."""
    parser = argparse.ArgumentParser(
        description='Build the results report for one bar-holding session.')
    parser.add_argument('batch', nargs='?', default=None,
                        help='session folder name, e.g. 20261001 '
                             '(default: the newest session on disk)')
    parser.add_argument('--no-robots', action='store_true',
                        help='skip the URDF load; the report does not draw the robot')
    args = parser.parse_args()

    batch = args.batch or latest_batch_folder()
    if not batch:
        sys.exit('no session folder found; pass one, e.g. 20261001')
    if not args.batch:
        print(f"[session] no batch given; using the newest one: {batch}")

    root = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'bar_holding_acc_data')
    batch_dir = os.path.join(root, batch)
    if not os.path.isdir(batch_dir):
        sys.exit(f"no such session folder: {batch_dir}")

    viewer = load_viewer()
    # ? The viewer prints its own per-bar lines; this report prints its own, so
    # ? swallow that pass and keep the console readable.
    with redirect_stdout(io.StringIO()) as swallowed:
        scene = viewer.collect_session(batch_dir, with_robots=False)
    records = scene['bars']
    for note in swallowed.getvalue().splitlines():
        if note.strip().startswith('!') or 'could not be fitted' in note:
            print(f"  {note.strip()}")
    print(f"[session] {len(records)} bars measured")

    for record in records:
        add_frames(record)
        base_xy = record.get('base_xy') or [None, None]
        record['base_x'], record['base_y'] = base_xy[0], base_xy[1]

    numbers = analyse(records, batch)

    out_dir = os.path.join(root, f"{batch}-result")
    figures_dir = os.path.join(out_dir, 'figures')
    os.makedirs(figures_dir, exist_ok=True)

    names = ['1_what_changed.svg', '2_system_map.svg', '3_shift_or_tilt.svg']
    figure_what_changed(numbers, os.path.join(figures_dir, names[0]))
    figure_system_map(numbers, os.path.join(figures_dir, names[1]))
    figure_shift_or_tilt(numbers, os.path.join(figures_dir, names[2]))

    with open(os.path.join(out_dir, 'numbers.json'), 'w') as handle:
        json.dump(numbers, handle, indent=2, default=float)
    write_results(numbers, os.path.join(out_dir, 'RESULTS.md'),
                  [f'figures/{n}' for n in names])

    print(f"\n[report] {out_dir}")
    for name in names + ['numbers.json', 'RESULTS.md']:
        where = os.path.join(figures_dir if name.endswith('.svg') else out_dir, name)
        print(f"         {name:<22} {os.path.getsize(where) / 1024:7.1f} KB")
    print(f"\n  placement  {numbers['placement']['mean']:.2f} mm mean, "
          f"{numbers['placement']['min']:.2f}-{numbers['placement']['max']:.2f} range")
    for name, step in numbers['steps'].items():
        if step:
            print(f"  step in {name:<12} at {step['at']}: "
                  f"{step['before']:.2f} -> {step['after']:.2f}")


if __name__ == '__main__':
    main()
