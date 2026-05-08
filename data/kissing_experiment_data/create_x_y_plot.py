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


def plot_offsets(data, out_path: Path | None = None):
    # base offset, also flip x to match image
    data_offset = [0.0, 0.0, 0.0, 0.0]

    data = data.assign(
        x = lambda df: (df["x"] + data_offset[0]) * 1000,
        y = lambda df: (df["y"] + data_offset[1]) * 1000
    )

    # Determine plot limits
    max_x = data["x"].abs().max()
    max_y = data["y"].abs().max()
    lim = max(max_x, max_y) * 1.5  # small margin

    data_stalled = data.query("stalled == True")
    data_non_stalled = data.query("stalled == False")

    img = plt.imread("screw_spiral.jpg")
    im_offset = [0.0, 0.0]

    plt.figure(figsize=(6, 6))
    ax = plt.gca()
    ax.imshow(img, extent=[-16 + im_offset[0], 16 + im_offset[0], -16 + im_offset[1], 16 + im_offset[1]])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.xaxis.set_inverted(True)
    ax.set_autoscale_on(False)
    ax.axhline(0, color="k", linewidth=0.8)
    ax.axvline(0, color="k", linewidth=0.8)
    plt.scatter(data_non_stalled["x"], data_non_stalled["y"], color="red", label="Failure")
    plt.scatter(data_stalled["x"], data_stalled["y"], color="green", label="Success")
    plt.legend(loc='upper left')
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.xlabel("X Offset [mm]")
    plt.ylabel("Y Offset [mm]")
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
    data = extract_data(records)
    print(f"loaded {len(data)} valid records")
    print(data.dtypes)
    print(data.head())

    data_ab0 = data[np.isclose(data["a"], 0.0) & np.isclose(data["b"], 0.0)]
    
    out_file = base_dir / "offset_xy_plot.png"
    plot_offsets(data_ab0, out_path=out_file)


if __name__ == "__main__":
    main()