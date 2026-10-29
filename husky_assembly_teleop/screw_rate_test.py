"""The release test: how fast the grasp screws back out, measured from one button press.

With a bar held by both of Cindy's tools and the arms still, 'Release test (BOTH)'
(``HuskyMonitor._release_test_task``) records the bar's MoCap fit (an as-built record),
loosens both grippers in one call, samples the wrist forces and the tools' status for a
fixed window, stops both tools, and records the bar again. This module turns that into
numbers, per tool:

- **delay**: LOOSEN -> the axial force (tool0 z) leaves its value from before the LOOSEN
  (the screw starts pushing the bar);
- **push time**: from then until |F| is back at the empty-tool level (the thread is out).
  The motor current dropping to its free-running level cross-checks it; the longer of
  the two is used, so an error makes the rate lower (the safe side);
- **travel**: how far the bar moved at that tool, along the tool axis (its MoCap fit
  before vs after);
- **rate** = travel / push time;

and, over the runs, the speed to set for the stiff screw-out: the slowest rate x 0.9,
as ``husky_world.RETREAT_SCREW_RPM``. Guide: doc/screw_rate_push_test.md.

Re-print the results of saved runs::

    python3 -m husky_assembly_teleop.screw_rate_test <release_test_*.json> ...
"""

import argparse
import json
from typing import Optional

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

# * ------------------------------------------------------------- the test's timing
# ! All timing is done by the code, never counted by the operator (user, 2026-10-28).
BASELINE_S = 0.5            # samples before the LOOSEN: the force baseline
WINDOW_S = 8.0              # samples after it; the threads were out after ~3.5 s on 10-27
SETTLE_S = 1.0              # after the Stop, before the second MoCap record

# * ------------------------------------------------------------- the analysis
PUSH_START_N = 2.0          # axial force change from the baseline that marks the push start
FREE_ABOVE_EMPTY_N = 2.0    # |F| within this of the empty-tool level = the thread is out ...
FREE_HOLD_S = 0.5           # ... and stays there this long
MIN_PUSH_S = 0.5            # a shorter "push" is the force dropping straight to empty: no push seen
EMPTY_LEVEL_S = 1.0         # the empty level: median over the last this-many seconds before the Stop
CURRENT_DROP_FRACTION = 0.3  # current back within this fraction of (loaded - free) = running free
MIN_CURRENT_STEP_MA = 20.0  # a smaller loaded-vs-free step is no drop at all
MIN_SAMPLE_HZ = 10.0        # warn below this (the monitor ticks at ~14-20 Hz)
ARMS_STILL_DEG = 0.1        # the arms must not move during the test
SPEED_MARGIN = 0.9          # the suggested speed: the slowest rate x this
DELAY_WORTH_S = 0.3         # a start delay worth adding to the retreat
SIDES = ('left', 'right')


# * ------------------------------------------------------------- per tool
def tool_axis_from_quat(quat_xyzw) -> np.ndarray:
    """The tool axis (tool0 z) in world coordinates, from a PyBullet quaternion.

    Args:
        quat_xyzw (Sequence[float]): ``(x, y, z, w)``.

    Returns:
        np.ndarray: Unit 3-vector, pointing from the tool into what it holds.
    """
    x, y, z, w = (float(v) for v in quat_xyzw)
    axis = np.array([2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)])
    return axis / np.linalg.norm(axis)


def travel_at_tool_mm(T_before, T_after, tool0_fk) -> Optional[float]:
    """How far the bar moved at a tool, along the tool axis, between two MoCap fits.

    The tool0 point is fixed in the bar's frame with the fit before, then carried by
    the fit after; the difference is projected on the tool axis.

    Args:
        T_before (array-like | None): 4x4 world-from-bar before the LOOSEN.
        T_after (array-like | None): 4x4 world-from-bar after the Stop.
        tool0_fk (Sequence): ``[[x, y, z], [qx, qy, qz, qw]]``, the tool0 pose (world).

    Returns:
        float | None: Millimetres, + = pushed away from the tool; None without both fits.
    """
    if T_before is None or T_after is None or tool0_fk is None:
        return None
    point = np.asarray(tool0_fk[0], dtype=float)
    in_bar = np.linalg.solve(np.asarray(T_before, dtype=float), np.append(point, 1.0))
    moved = (np.asarray(T_after, dtype=float) @ in_bar)[:3] - point
    return float(np.dot(moved, tool_axis_from_quat(tool0_fk[1]))) * 1000.0


def _first_lasting(t: np.ndarray, ok: np.ndarray, start: int, end_t: float) -> Optional[float]:
    """The first time from index ``start`` at which ``ok`` holds for ``FREE_HOLD_S`` (before ``end_t``).

    Args:
        t (np.ndarray): Sample times.
        ok (np.ndarray): Bool per sample.
        start (int): First index to look at.
        end_t (float): The hold must end by this time.

    Returns:
        float | None: The time, or None when it never holds that long.
    """
    for i in range(start, len(t)):
        if t[i] + FREE_HOLD_S > end_t:
            return None
        span = (t >= t[i]) & (t <= t[i] + FREE_HOLD_S)
        if ok[span].all():
            return float(t[i])
    return None


def analyse_tool(t, wrench, current, state, t_loosen: float, t_stop: float) -> dict:
    """When one tool's screw pushed the bar: from its wrist force and motor current around the LOOSEN.

    Args:
        t (Sequence[float]): Sample times (epoch seconds), rising.
        wrench (Sequence): Raw wrench per sample, ``[fx, fy, fz, tx, ty, tz]`` in tool0.
        current (Sequence[float]): The tool's motor current per sample, mA (None if unknown).
        state (Sequence[str]): The gripper motor's state per sample.
        t_loosen (float): When the LOOSEN went out.
        t_stop (float): When the Stop went out.

    Returns:
        dict: ``motor_start_s`` (LOOSEN -> state LOOSENING), ``delay_s`` (LOOSEN -> push
        start), ``push_force_s`` (push start -> |F| at the empty level), ``push_current_s``
        (push start -> the current at its free level), ``push_s`` (the longer of the two),
        ``empty_N``, ``note``; None where it could not be told.
    """
    t = np.asarray(t, dtype=float)
    w = np.asarray(wrench, dtype=float)
    out = {'motor_start_s': None, 'delay_s': None, 'push_force_s': None, 'push_current_s': None,
           'push_s': None, 'empty_N': None, 'note': ''}
    before = t < t_loosen
    running = (t >= t_loosen) & (t <= t_stop)
    if before.sum() < 2 or running.sum() < 5:
        out['note'] = 'too few samples around the LOOSEN'
        return out
    fz = w[:, 2]
    fmag = np.linalg.norm(w[:, :3], axis=1)
    loosening = [i for i in np.flatnonzero(running) if state[i] == 'LOOSENING']
    if loosening:
        out['motor_start_s'] = float(t[loosening[0]] - t_loosen)

    # * The push starts when the axial force leaves its value from before the LOOSEN.
    fz0 = float(np.mean(fz[before]))
    pushing = running & (np.abs(fz - fz0) > PUSH_START_N)
    if not pushing.any():
        out['note'] = f'the axial force never left its baseline by {PUSH_START_N:.0f} N: no push seen'
        return out
    i0 = int(np.argmax(pushing))
    t_push = float(t[i0])
    out['delay_s'] = t_push - t_loosen

    # * It ends when |F| is back at the empty-tool level (the last second before the Stop).
    tail = running & (t >= t_stop - EMPTY_LEVEL_S)
    empty = float(np.median(fmag[tail]))
    out['empty_N'] = empty
    t_free = _first_lasting(t, running & (fmag <= empty + FREE_ABOVE_EMPTY_N), i0, t_stop)
    if t_free is not None:
        out['push_force_s'] = t_free - t_push

    # * Cross-check: the motor current falls to its free-running level when the thread is out.
    c = np.asarray([np.nan if v is None else float(v) for v in current], dtype=float)
    if np.isfinite(c[tail]).any() and t_free is not None:
        free_c = float(np.nanmedian(c[tail]))
        loaded = running & (t >= t_push) & (t <= t_push + 0.5 * (t_free - t_push))
        loaded_c = float(np.nanmedian(c[loaded])) if np.isfinite(c[loaded]).any() else free_c
        if loaded_c - free_c >= MIN_CURRENT_STEP_MA:
            ok = running & (c <= free_c + CURRENT_DROP_FRACTION * (loaded_c - free_c))
            t_drop = _first_lasting(t, ok, i0, t_stop)
            if t_drop is not None:
                out['push_current_s'] = t_drop - t_push
    pushes = [v for v in (out['push_force_s'], out['push_current_s']) if v is not None]
    if not pushes:
        out['note'] = '|F| never came back to the empty level: the thread did not come out?'
    elif max(pushes) < MIN_PUSH_S:
        # * The force went straight from the hold to the empty tool: what changed was the
        # * preload letting go, not a screw pushing the bar off.
        out['delay_s'] = None
        out['note'] = (f'no clear push: the force went straight to the empty level (a push under '
                       f'{PUSH_START_N:.0f} N, or the bar was already loose)')
    else:
        out['push_s'] = max(pushes)   # ! the longer one: an error then makes the rate lower
    return out


# * ------------------------------------------------------------- per run
def analyse_run(run: dict, design_travel_mm: float = 8.0) -> dict:
    """Everything one release test tells, per tool and as a whole.

    Args:
        run (dict): What ``HuskyMonitor._release_test_task`` saves: ``t_loosen``,
            ``t_stop``, ``samples`` (``t`` and per side ``wrench``, ``current``,
            ``state``, ``joints``), ``before`` (``T_world_from_bar``, ``tool0_fk`` per
            side) and ``after`` (``T_world_from_bar``).
        design_travel_mm (float): The travel used when a MoCap fit is missing (said so).

    Returns:
        dict: ``tools`` (per side: ``analyse_tool``'s numbers + ``travel_mm``,
        ``travel_source``, ``rate_mm_s``), ``sample_hz``, ``arms_moved_deg``, ``notes``.
    """
    s = run['samples']
    t = np.asarray(s['t'], dtype=float)
    notes = []
    sample_hz = float(1.0 / np.median(np.diff(t))) if len(t) > 1 else 0.0
    if sample_hz < MIN_SAMPLE_HZ:
        notes.append(f'only {sample_hz:.0f} Hz of samples (< {MIN_SAMPLE_HZ:.0f}): the times are coarse')
    moved = [float(np.degrees(np.abs(np.asarray(s[side]['joints'], dtype=float)
                                     - np.asarray(s[side]['joints'][0], dtype=float)).max()))
             for side in SIDES if s[side]['joints']]
    arms_moved = max(moved) if moved else 0.0
    if arms_moved > ARMS_STILL_DEG:
        notes.append(f'the arms moved {arms_moved:.2f} deg during the test: not a clean push')
    tools = {}
    for side in SIDES:
        r = analyse_tool(t, s[side]['wrench'], s[side]['current'], s[side]['state'],
                         run['t_loosen'], run['t_stop'])
        travel = travel_at_tool_mm((run.get('before') or {}).get('T_world_from_bar'),
                                   (run.get('after') or {}).get('T_world_from_bar'),
                                   ((run.get('before') or {}).get('tool0_fk') or {}).get(side))
        r['travel_mm'], r['travel_source'] = ((travel, 'MoCap') if travel is not None
                                              else (design_travel_mm, 'design, no MoCap fit'))
        r['rate_mm_s'] = (r['travel_mm'] / r['push_s']
                          if r['push_s'] and r['travel_mm'] and r['travel_mm'] > 0 else None)
        tools[side] = r
    return {'tools': tools, 'sample_hz': sample_hz, 'arms_moved_deg': arms_moved, 'notes': notes}


def summarise(results: list, pitch_mm: float) -> dict:
    """The speed to set for the stiff screw-out, over every run so far.

    Args:
        results (list[dict]): ``analyse_run`` results.
        pitch_mm (float): The grasp screws' pitch (``husky_world.RETREAT_SCREW_PITCH_MM``).

    Returns:
        dict: ``n_runs``, ``n_rates``, ``slowest_mm_s``, ``mean_mm_s``, ``speed_mm_s``,
        ``rpm``, ``max_delay_s`` (None where nothing was measured).
    """
    rates = [r['rate_mm_s'] for res in results for r in res['tools'].values() if r.get('rate_mm_s')]
    delays = [r['delay_s'] for res in results for r in res['tools'].values() if r.get('delay_s') is not None]
    out = {'n_runs': len(results), 'n_rates': len(rates), 'slowest_mm_s': None, 'mean_mm_s': None,
           'speed_mm_s': None, 'rpm': None, 'max_delay_s': max(delays) if delays else None}
    if rates:
        out['slowest_mm_s'] = min(rates)
        out['mean_mm_s'] = float(np.mean(rates))
        out['speed_mm_s'] = SPEED_MARGIN * min(rates)
        out['rpm'] = out['speed_mm_s'] / pitch_mm * 60.0
    return out


# * ------------------------------------------------------------- text
def _s(v: Optional[float], unit: str = 's') -> str:
    """A number for a result line, or '?' when it could not be told."""
    return '?' if v is None else f'{v:.2f} {unit}'


def tool_lines(result: dict) -> list:
    """One line per tool, plus the run's notes.

    Args:
        result (dict): An ``analyse_run`` result.

    Returns:
        list[str]: The lines, ready to log.
    """
    lines = []
    for side, r in result['tools'].items():
        lines.append(
            f"{side}: motor start {_s(r['motor_start_s'])}, push start {_s(r['delay_s'])} after the "
            f"LOOSEN, push {_s(r['push_s'])} (force {_s(r['push_force_s'])}, current "
            f"{_s(r['push_current_s'])}), bar moved {_s(r['travel_mm'], 'mm')} at the tool "
            f"({r['travel_source']}) -> {_s(r['rate_mm_s'], 'mm/s')}"
            + (f"  [{r['note']}]" if r['note'] else ''))
    lines += [f'note: {n}' for n in result['notes']]
    return lines


def summary_line(summary: dict) -> str:
    """The suggestion, as one line.

    Args:
        summary (dict): A ``summarise`` result.

    Returns:
        str: The line.
    """
    if not summary['n_rates']:
        return f"{summary['n_runs']} run(s), no rate measured yet: see the per-tool lines."
    text = (f"{summary['n_runs']} run(s), {summary['n_rates']} rate(s): slowest "
            f"{summary['slowest_mm_s']:.2f} mm/s (mean {summary['mean_mm_s']:.2f}) -> set the stiff "
            f"screw-out to {summary['speed_mm_s']:.2f} mm/s: husky_world.RETREAT_SCREW_RPM = "
            f"{summary['rpm']:.0f}")
    if summary['max_delay_s'] is not None and summary['max_delay_s'] > DELAY_WORTH_S:
        text += f"; the push started up to {summary['max_delay_s']:.2f} s after the LOOSEN: worth a start delay"
    return text


# * ------------------------------------------------------------- files
def save_run(path: str, run: dict, result: dict) -> None:
    """Save one release test (its samples and its result) as JSON.

    Args:
        path (str): The ``.json`` file to write.
        run (dict): The recorded run (see ``analyse_run``).
        result (dict): Its ``analyse_run`` result.
    """
    with open(path, 'w') as f:
        json.dump({'schema_version': 1, 'kind': 'release_test', 'run': run, 'result': result}, f, indent=1)


def plot_run(path: str, run: dict, result: dict) -> None:
    """Save the axial force, |F| and motor current of both tools, with the push marked, as a PNG.

    ! Pure Agg (Figure + FigureCanvasAgg), never pyplot: a GUI backend inside the
    ! live monitor's DearPyGui + PyBullet loop crashes it (see husky_world's servo plot).

    Args:
        path (str): The ``.png`` file to write.
        run (dict): The recorded run.
        result (dict): Its ``analyse_run`` result.
    """
    s = run['samples']
    t = np.asarray(s['t'], dtype=float) - run['t_loosen']
    fig = Figure(figsize=(11, 8))
    FigureCanvasAgg(fig)
    axes = fig.subplots(3, 1, sharex=True)
    for side, color in zip(SIDES, ('tab:red', 'tab:green')):
        w = np.asarray(s[side]['wrench'], dtype=float)
        axes[0].plot(t, w[:, 2], color=color, label=f'{side} Fz (tool0)')
        axes[1].plot(t, np.linalg.norm(w[:, :3], axis=1), color=color, label=f'{side} |F|')
        axes[2].plot(t, [np.nan if v is None else v for v in s[side]['current']], color=color,
                     label=f'{side} motor current')
        r = result['tools'][side]
        if r['delay_s'] is not None:
            for ax in axes:
                ax.axvline(r['delay_s'], color=color, linestyle='--', linewidth=0.8)
                if r['push_s'] is not None:
                    ax.axvline(r['delay_s'] + r['push_s'], color=color, linestyle=':', linewidth=0.8)
    for ax in axes:
        ax.axvline(0.0, color='k', linewidth=0.8)
        ax.axvline(run['t_stop'] - run['t_loosen'], color='grey', linewidth=0.8)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
    axes[0].set_ylabel('N')
    axes[1].set_ylabel('N')
    axes[2].set_ylabel('mA')
    axes[2].set_xlabel('s after the LOOSEN (black); grey = Stop; dashed = push start, dotted = push end')
    fig.suptitle(f"Release test {run.get('stamp', '')}: {run.get('bar') or 'bar ?'}")
    fig.savefig(path, dpi=110)


def main(argv: Optional[list] = None) -> None:
    """Re-print the per-tool lines and the summary of saved release tests.

    Args:
        argv (list[str] | None): Command-line arguments; None = ``sys.argv``.
    """
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('files', nargs='+', help='release_test_*.json files')
    parser.add_argument('--pitch-mm', type=float, default=2.0, help='grasp screw pitch (default 2)')
    parser.add_argument('--travel-mm', type=float, default=8.0,
                        help='travel to use when a MoCap fit is missing (default 8, the design)')
    args = parser.parse_args(argv)
    results = []
    for path in args.files:
        with open(path) as f:
            run = json.load(f)['run']
        result = analyse_run(run, design_travel_mm=args.travel_mm)
        results.append(result)
        print(f'{path}:')
        for line in tool_lines(result):
            print(f'  {line}')
    print(summary_line(summarise(results, args.pitch_mm)))


if __name__ == '__main__':
    main()
