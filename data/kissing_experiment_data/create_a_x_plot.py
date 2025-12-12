from pathlib import Path
import json
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


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


def extract_data(records):
    # create data frame for all records
    duration, xs, ys, a, b, stalled, trajectory_finished, force_profiles = [], [], [], [], [], [], [], []
    for data in records:
        off = data.get("offset")
        if not off or len(off) < 2:
            print(f"warning: no valid offset in {data.get('name')}")
            continue

        duration.append(data.get("finish_time") - data.get("start_time"))
        xs.append(float(off[0]))
        ys.append(float(off[1]))
        a.append(float(off[2]))
        b.append(float(off[3]))
        stalled.append(bool(data.get("motor_stalled")))
        trajectory_finished.append(bool(data.get("trajectory_finished")))
        raw_wrench_profile = data.get("wrench_profile")
        force_profiles.append(np.array([np.linalg.norm(np.array(wrench)[0:3]) for wrench in raw_wrench_profile]))

    data = pd.DataFrame(data={
        "duration": duration,
        "x": xs,
        "y": ys,
        "a": a,
        "b": b,
        "stalled": stalled,
        "trajectory_finished": trajectory_finished,
        "force_profile": force_profiles
    })
    return data


def plot_offsets_ax(data, out_path: Path | None = None):
    data_offset = [0.0, 0.0, 0.0, 0.0]

    data = data.assign(
        x = lambda df: (df["x"] + data_offset[0]) * 1000,
        y = lambda df: (df["y"] + data_offset[1]) * 1000,
        a = lambda df: np.rad2deg(df["a"] + data_offset[2]),
        b = lambda df: np.rad2deg(df["b"] + data_offset[3])
    )

    # Determine plot limits
    max_a = data["a"].abs().max() * 1.5
    max_x = data["x"].abs().max() * 1.5

    max_x = 15

    data_stalled = data.query("stalled == True")
    data_non_stalled = data.query("stalled == False")

    plt.figure(figsize=(6, 6))
    ax = plt.gca()
    ax.set_xlim(-max_x, max_x)
    ax.set_ylim(-max_a, max_a)
    ax.xaxis.set_inverted(True)
    ax.set_autoscale_on(False)
    ax.axhline(0, color="k", linewidth=0.8)
    ax.axvline(0, color="k", linewidth=0.8)
    plt.scatter(data_non_stalled["x"], data_non_stalled["a"], color="red", label="Failure")
    plt.scatter(data_stalled["x"], data_stalled["a"], color="green", label="Success")
    plt.legend(loc='upper left')
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.xlabel("X Offset [mm]")
    plt.ylabel("A Offset [deg]")
    plt.title("A X Offset Plot")
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
    data = extract_data(records)
    print(f"loaded {len(data)} valid records")
    print(data.dtypes)
    print(data.head())

    data_by0 = data[np.isclose(data["b"], 0.0) & np.isclose(data["y"], 0.0)]
    
    out_file = base_dir / "offset_ax_plot.png"
    plot_offsets(data_by0, out_path=out_file)


if __name__ == "__main__":
    main()