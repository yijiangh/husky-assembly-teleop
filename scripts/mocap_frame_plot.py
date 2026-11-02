"""
Plot how Motive's frame drifts since the floor calibration, how well its fits know it, and which anchors are bad.

The mocap relay publishes one sample per look on `/mocap_relay/frame_metrics` (`world/frame_follower.py` metrics) and
keeps them, a file per day, in its frame log (`~/.ros/mocap_frame/<YYYYMMDD>.jsonl` by default). Every fit is against
the floor calibration's anchors and cameras, so using a refit changes none of them: it shows as the transform in use
(dashed, top panel) stepping to the deciding fit, a thin vertical line, and the second panel, how far poses are off now
(deciding fit against the transform in use), dropping to zero.

- Left, five panels: each fit's move of the lab's volume with its uncertainty, poses off now against the limit a refit
  is pending beyond, the largest residual, how many points it used, and how far the two fits place the lab apart; a
  cross marks a sample whose fit is not trusted.
- Right, one line per anchor: its error against a fit of the other anchors (the one that says an anchor is bad), its
  noise within the look, and the share of the look's frames it was seen in; gaps where it was not found, a cross
  where the fit left it out. Samples from before these were published leave them empty.
- `--file` takes files, folders (their `*.jsonl`, so not a simulation's `sim/`) and globs, and prints each anchor's
  median and 95th percentile. Minutes on the x axis, hours over 6 h.

    python3 scripts/mocap_frame_plot.py                         live, in the relay's ROS domain (ROS sourced)
    python3 scripts/mocap_frame_plot.py --record ~/frame.jsonl  live, keeping every sample
    python3 scripts/mocap_frame_plot.py --file ~/.ros/mocap_frame                       every day kept
    python3 scripts/mocap_frame_plot.py --file ~/.ros/mocap_frame/2026102[89].jsonl     two days

Run from the workspace with the venv (and ROS sourced for live).
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
import threading
from pathlib import Path

import numpy as np

#: Where the relay publishes its samples.
TOPIC = "/mocap_relay/frame_metrics"
#: The two fits: key in a sample, label, colour (the dataviz reference palette's categorical slots 1 and 2).
FITS = (("anchors", "anchors", "#2a78d6"), ("truss_cameras", "truss cameras", "#eb6834"))
#: The anchors' colours, in name order: the same palette's categorical slots 1 to 8, never cycled.
ANCHOR_COLOURS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948")
#: Ink and chrome (the same palette's light mode).
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#898781", "#e1e0d9", "#fcfcfb"
#: Over this many hours of samples, the x axis is in hours rather than minutes.
HOURS_OVER = 6.0


# --- --- --- --- --- READ --- --- --- --- ---

def sample_files(names: list[str]) -> list[Path]:
    """The files `names` stand for: a file, a folder's `*.jsonl` (not its subfolders), or a glob; sorted, each once."""
    files: list[Path] = []
    for name in names:
        path = Path(name).expanduser()
        if path.is_dir():
            files += sorted(path.glob("*.jsonl"))
        elif any(sign in name for sign in "*?["):
            files += sorted(Path(match) for match in glob.glob(str(path)))
        else:
            files.append(path)
    return list(dict.fromkeys(files))


def read_samples(files: list[Path]) -> tuple[list[dict], int]:
    """Every sample in `files`, by time; and how many lines did not read (a line cut short by a crash)."""
    samples, bad = [], 0
    for path in files:
        with path.open(encoding="utf-8") as lines:
            for line in lines:
                if not line.strip():
                    continue
                try:
                    sample = json.loads(line)
                except json.JSONDecodeError:
                    bad += 1
                    continue
                if isinstance(sample, dict) and isinstance(sample.get("time"), (int, float)):
                    samples.append(sample)
                else:
                    bad += 1
    return sorted(samples, key=lambda sample: sample["time"]), bad


# --- --- --- --- --- SERIES --- --- --- --- ---

def time_axis(samples: list[dict]) -> tuple[np.ndarray, str]:
    """Each sample's time since the first, in minutes, or in hours over HOURS_OVER; and the unit's name."""
    seconds = np.array([s["time"] - samples[0]["time"] for s in samples], dtype=float)
    if len(seconds) and seconds[-1] > HOURS_OVER * 3600.0:
        return seconds / 3600.0, "hours"
    return seconds / 60.0, "minutes"


def series(samples: list[dict], key: str, value) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Time (`time_axis`), the value of one fit per sample (NaN without the fit), and whether it was trusted."""
    values, trusted = [], []
    for sample in samples:
        fit = sample.get(key)
        values.append(value(fit) if fit else np.nan)
        trusted.append(bool(fit and fit["trusted"]))
    return time_axis(samples)[0], np.array(values, dtype=float), np.array(trusted)


def anchor_series(samples: list[dict], field: str) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """One of each anchor's `anchor_quality` fields per sample, by anchor name: (values, left out of the fit).

    A value is NaN where the sample has no such anchor or field (older samples, the anchor not found); the share of
    the look's frames for `field` "seen".
    """
    names = sorted({name for s in samples for name in (s.get("anchor_quality") or {})})
    found = {}
    for name in names:
        values, bumped = [], []
        for sample in samples:
            entry = (sample.get("anchor_quality") or {}).get(name) or {}
            if field == "seen":
                value = entry["frames"] / sample["frames"] if entry.get("frames") and sample.get("frames") else None
            else:
                value = entry.get(field)
            values.append(np.nan if value is None else value)
            bumped.append(entry.get("status") == "bumped")
        found[name] = (np.array(values, dtype=float), np.array(bumped))
    return found


def anchor_summary(samples: list[dict]) -> list[str]:
    """Lines for people: per anchor, how often used, left out and missing, and its errors' median and 95th percentile.

    The errors: against a fit of the others, its residual and its noise, mm. Empty without any per-anchor quality.
    """
    errors, residuals, noise = (anchor_series(samples, field) for field in ("left_out_mm", "residual_mm", "noise_mm"))
    if not errors:
        return []
    lines = [f"{'anchor':<7}{'used':>7}{'left out':>9}{'missing':>8}  {'error vs others':>16}  {'residual':>14}  "
             f"{'noise':>14}", f"{'':<7}{'':>7}{'':>9}{'':>8}  {'median / p95 mm':>16}  {'median / p95':>14}  "
             f"{'median / p95':>14}"]
    for name in errors:
        statuses = [((s.get("anchor_quality") or {}).get(name) or {}).get("status") for s in samples
                    if s.get("anchor_quality")]
        columns = []
        for values in (errors[name][0], residuals[name][0], noise[name][0]):
            seen = values[~np.isnan(values)]
            columns.append(f"{np.median(seen):6.2f} / {np.percentile(seen, 95):5.2f}" if len(seen) else "-")
        lines.append(f"{name:<7}{statuses.count('used'):>7}{statuses.count('bumped'):>9}"
                     f"{statuses.count('missing'):>8}  {columns[0]:>16}  {columns[1]:>14}  {columns[2]:>14}")
    return lines


# --- --- --- --- --- PLOT --- --- --- --- ---

def _style(axis) -> None:
    """Recessive chrome: surface, grid, muted ticks, no top or right spine."""
    axis.set_facecolor(SURFACE)
    axis.grid(True, color=GRID, linewidth=0.6)
    axis.tick_params(colors=MUTED, labelsize=8)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        axis.spines[side].set_color(GRID)


def plot_anchors(samples: list[dict], axes) -> None:
    """Draw each anchor's error against the others, its noise and the share of frames it was seen in on `axes`."""
    from matplotlib.ticker import PercentFormatter
    marks = time_axis(samples)[0]
    panels = (("anchor's error against a fit of the others (mm)", "left_out_mm"),
              ("anchor's noise within a look, RMS (mm)", "noise_mm"),
              ("frames the anchor was seen in, share of the look", "seen"))
    for axis, (title, field) in zip(axes, panels):
        for index, (name, (values, bumped)) in enumerate(anchor_series(samples, field).items()):
            colour = ANCHOR_COLOURS[index] if index < len(ANCHOR_COLOURS) else MUTED
            axis.plot(marks, values, color=colour, linewidth=1.5, label=name)
            axis.plot(marks[bumped], values[bumped], linestyle="none", marker="x", markersize=6, color=colour)
        axis.set_title(title, loc="left", fontsize=10, color=INK)
    axes[2].yaxis.set_major_formatter(PercentFormatter(1.0))
    if axes[0].lines:
        axes[0].legend(loc="upper left", fontsize=8, ncol=8, title="x: left out of the anchors' fit", title_fontsize=8,
                       facecolor=SURFACE, edgecolor=GRID, framealpha=0.9)
    else:
        axes[0].text(0.5, 0.5, "no per-anchor quality in these samples", transform=axes[0].transAxes, ha="center",
                     va="center", color=MUTED, fontsize=9)


def plot(samples: list[dict], figure=None):
    """Draw the frame's five panels and the anchors' three for `samples` on `figure` (a new one if None); returns it."""
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    if figure is None:
        figure = plt.figure(figsize=(16, 11), facecolor=SURFACE)
    figure.clear()
    grid = figure.add_gridspec(10, 2, width_ratios=(3, 2))
    axes = [figure.add_subplot(grid[0:2, 0])]
    axes += [figure.add_subplot(grid[2 * row:2 * row + 2, 0], sharex=axes[0]) for row in range(1, 5)]
    anchor_axes = [figure.add_subplot(grid[0:4, 1], sharex=axes[0]), figure.add_subplot(grid[4:7, 1], sharex=axes[0]),
                   figure.add_subplot(grid[7:10, 1], sharex=axes[0])]
    minutes, unit = time_axis(samples)
    mm = lambda key: np.array([s[key] * 1e3 if s.get(key) is not None else np.nan for s in samples])  # noqa: E731
    # * A refit used: the transform in use changed between two samples.
    in_use = [json.dumps((s.get("refit") or {}).get("raw_cal_from_raw_now")) for s in samples]
    refits = [minutes[k] for k in range(1, len(samples)) if in_use[k] != in_use[k - 1]]
    fits = (("lab moved since the floor calibration, worst corner (mm)", axes[0], lambda f: f["moves_volume"] * 1e3),
            ("largest residual (mm)", axes[2], lambda f: f["residual"] * 1e3),
            ("points used", axes[3], lambda f: len(f["used"])))
    for title, axis, value in fits:
        for key, label, colour in FITS:
            marks, values, trusted = series(samples, key, value)
            axis.plot(marks, values, color=colour, linewidth=2, label=label)
            if axis is axes[0]:
                _, uncertainty, _ = series(samples, key, lambda f: f["uncertainty"] * 1e3)
                axis.fill_between(marks, values - uncertainty, values + uncertainty, color=colour, alpha=0.15,
                                  linewidth=0)
            axis.plot(marks[~trusted], values[~trusted], linestyle="none", marker="x", markersize=8, color=colour)
        axis.set_title(title, loc="left", fontsize=10, color=INK)
    in_use_mm = np.array([(s.get("refit") or {}).get("moves_volume", 0.0) * 1e3 for s in samples])
    axes[0].plot(minutes, in_use_mm, color=INK, linewidth=1.5, linestyle="--", drawstyle="steps-post",
                 label="transform in use")
    axes[1].plot(minutes, mm("off"), color=INK, linewidth=2, label="deciding fit against the transform in use")
    axes[1].plot(minutes, mm("off_limit"), color=MUTED, linewidth=1, linestyle="--",
                 label="refit pending beyond: 1 mm, or twice the fit's uncertainty")
    axes[1].set_title("poses off now (mm)", loc="left", fontsize=10, color=INK)
    axes[1].legend(frameon=False, loc="upper left", fontsize=8)
    axes[3].yaxis.set_major_locator(MaxNLocator(integer=True))
    axes[4].plot(minutes, mm("apart"), color=INK, linewidth=2)
    axes[4].set_title("anchors and truss cameras place the lab apart, both trusted (mm)", loc="left", fontsize=10,
                      color=INK)
    axes[0].legend(frameon=False, loc="upper left", fontsize=9, title="band: ± uncertainty · x: not trusted",
                   title_fontsize=8)
    plot_anchors(samples, anchor_axes)
    for axis in axes + anchor_axes:
        for minute in refits:
            axis.axvline(minute, color=MUTED, linewidth=0.8, alpha=0.6)
        _style(axis)
        if axis not in (axes[-1], anchor_axes[-1]):
            axis.tick_params(labelbottom=False)
    for axis in (axes[-1], anchor_axes[-1]):
        axis.set_xlabel(f"{unit} (thin vertical lines: a refit used)", color=MUTED, fontsize=9)
    calibration = samples[-1].get("floor_calibration") or "?"
    figure.suptitle(f"Motive's frame against {calibration}: {len(refits)} refits", x=0.06, ha="left", fontsize=12,
                    color=INK)
    figure.tight_layout()
    return figure


# --- --- --- --- --- LIVE --- --- --- --- ---

def live(record: Path | None) -> None:
    """Plot the relay's samples as they come, redrawn every 2 s; keep them in `record` if given."""
    import matplotlib.pyplot as plt
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String
    samples, lock = [], threading.Lock()
    out = record.open("a") if record else None

    def take(message):
        with lock:
            samples.append(json.loads(message.data))
        if out:
            out.write(message.data + "\n")
            out.flush()

    rclpy.init()
    node = rclpy.create_node("mocap_frame_plot")
    node.create_subscription(String, TOPIC, take, 10)

    def spin():
        """Take the relay's samples until ROS shuts down (Ctrl-C)."""
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, daemon=True).start()
    # ! Shown once and redrawn in place: plt.pause shows the window again each time, raising it over the others.
    plt.rcParams["figure.raise_window"] = False
    figure = plt.figure(figsize=(16, 11), facecolor=SURFACE)
    plt.show(block=False)
    print(f"listening on {TOPIC}; close the window to stop")
    drawn = 0
    try:
        while plt.fignum_exists(figure.number) and rclpy.ok():
            with lock:
                current = list(samples)
            if len(current) > drawn:
                drawn = len(current)
                plot(current, figure)
                figure.canvas.draw_idle()
            figure.canvas.start_event_loop(2.0)
    except KeyboardInterrupt:
        pass


def main() -> None:
    """Plot live, or a recording."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--file", nargs="+", help="recordings or frame log day files (JSON lines of the relay's "
                                                  "samples): files, folders (their *.jsonl) or globs")
    parser.add_argument("--record", type=Path, help="live: keep every sample in this file")
    parser.add_argument("--save", type=Path, help="write the plot to this image instead of showing it")
    args = parser.parse_args()
    if not args.file:
        live(args.record)
        return
    files = sample_files(args.file)
    samples, bad = read_samples(files)
    if not samples:
        sys.exit(f"no samples in {', '.join(map(str, files)) or ' '.join(args.file)}")
    print(f"{len(samples)} samples from {len(files)} files" + (f"; {bad} lines did not read, skipped" if bad else ""))
    print("\n".join(anchor_summary(samples)) or "no per-anchor quality in these samples")
    if args.save:
        import matplotlib
        matplotlib.use("Agg")
    figure = plot(samples)
    if args.save:
        figure.savefig(args.save, dpi=110, facecolor=SURFACE)
        print(f"{len(samples)} samples -> {args.save}")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
