from pathlib import Path
import json
import matplotlib.pyplot as plt


def get_sample_files(dir_path: Path):
    return sorted(dir_path.glob("offset_*.json"))


def load_samples(paths):
    records = []
    for p in paths:
        try:
            with p.open("r", encoding="utf-8") as f:
                data = json.load(f)
            records.append(data)
        except Exception as e:
            print(f"warning: failed to read {p}: {e}")
    return records


def extract_xy(records):
    xs, ys, stalled = [], [], []
    for data in records:
        off = data.get("offset")
        if not off or len(off) < 2:
            print(f"warning: no valid offset in {data.get('name')}")
            continue
        xs.append(off[0])
        ys.append(off[1])
        stalled.append(data.get("motor_stalled"))
    return xs, ys, stalled


def plot_offsets(xs, ys, stalled, out_path: Path | None = None):
    # Determine plot limits
    vals = [abs(v) for v in xs] + [abs(v) for v in ys]
    lim = max(vals) if vals else 1.0
    lim *= 1.05  # small margin

    # Split stalled and non-stalled points
    xs_stalled = [x for x, s in zip(xs, stalled) if s]
    ys_stalled = [y for y, s in zip(ys, stalled) if s]
    xs_non_stalled = [x for x, s in zip(xs, stalled) if not s]
    ys_non_stalled = [y for y, s in zip(ys, stalled) if not s]


    plt.figure(figsize=(6, 6))
    ax = plt.gca()
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_autoscale_on(False)
    ax.axhline(0, color="k", linewidth=0.8)
    ax.axvline(0, color="k", linewidth=0.8)
    plt.scatter(xs_non_stalled, ys_non_stalled, color="red", label="Failure")
    plt.scatter(xs_stalled, ys_stalled, color="green", label="Success")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.xlabel("X Offset")
    plt.ylabel("Y Offset")
    plt.title("X Y Offset Plot")
    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=200)
        print(f"saved plot to {out_path}")
    else:
        plt.show()


def main():
    base_dir = Path(__file__).parent
    files = get_sample_files(base_dir)
    if not files:
        print(f"no files matching 'offset_*.json' in {base_dir}")
        return
    records = load_samples(files)
    xs, ys, labels = extract_xy(records)
    if not xs:
        print("no valid offset points to plot")
        return
    out_file = base_dir / "offset_xy_plot.png"
    plot_offsets(xs, ys, labels, out_path=out_file)


if __name__ == "__main__":
    main()