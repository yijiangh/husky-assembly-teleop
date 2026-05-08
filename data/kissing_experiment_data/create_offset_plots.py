from pathlib import Path
import json
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


marker_success = "o"
marker_failure = "x"
# rgb 112 166 255
color_success = (112/255, 166/255, 255/255)
# rgb 255 50 50
color_failure = (255/255, 50/255, 50/255)

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


def plot_offsets(data, ax):
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
    lim = 10

    data_stalled = data.query("stalled == True")
    data_non_stalled = data.query("stalled == False")

    img = plt.imread("screw_spiral.jpg")
    im_offset = [0.0, 0.0]

    ax.imshow(img, extent=[-16 + im_offset[0], 16 + im_offset[0], -16 + im_offset[1], 16 + im_offset[1]])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.xaxis.set_inverted(True)
    ax.set_autoscale_on(False)
    ax.axhline(0, color="k", linewidth=0.8)
    ax.axvline(0, color="k", linewidth=0.8)
    ax.scatter(data_non_stalled["x"], data_non_stalled["y"], color=color_failure, marker=marker_failure, label="Failure")
    ax.scatter(data_stalled["x"], data_stalled["y"], color=color_success, marker=marker_success, label="Success")
    ax.legend(loc='upper left')
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_xlabel("X Offset [mm]")
    ax.set_ylabel("Y Offset [mm]")
    ax.set_title("Figure 5a: X-Y Offset Plane")

def plot_offsets_ax(data, ax):
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

    ax.set_xlim(-max_x, max_x)
    ax.set_ylim(-max_a, max_a)
    ax.xaxis.set_inverted(True)
    ax.set_autoscale_on(False)
    ax.axhline(0, color="k", linewidth=0.8)
    ax.axvline(0, color="k", linewidth=0.8)
    ax.scatter(data_non_stalled["x"], data_non_stalled["a"], color=color_failure, marker=marker_failure, label="Failure")
    ax.scatter(data_stalled["x"], data_stalled["a"], color=color_success, marker=marker_success, label="Success")
    ax.legend(loc='center right')
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_xlabel("X Offset [mm]")
    ax.set_ylabel("A Offset [deg]")
    ax.set_title("Figure 5b: A-X Offset Plane")

def plot_offsets_bx(data, ax):
    data_offset = [0.0, 0.0, 0.0, 0.0]

    data = data.assign(
        x = lambda df: (df["x"] + data_offset[0]) * 1000,
        y = lambda df: (df["y"] + data_offset[1]) * 1000,
        a = lambda df: np.rad2deg(df["a"] + data_offset[2]),
        b = lambda df: np.rad2deg(df["b"] + data_offset[3])
    )

    # Determine plot limits
    max_b = data["b"].abs().max() * 1.5
    max_x = data["x"].abs().max() * 1.5

    max_x = 15

    data_stalled = data.query("stalled == True")
    data_non_stalled = data.query("stalled == False")

    ax.set_xlim(-max_x, max_x)
    ax.set_ylim(-max_b, max_b)
    ax.xaxis.set_inverted(True)
    ax.set_autoscale_on(False)
    ax.axhline(0, color="k", linewidth=0.8)
    ax.axvline(0, color="k", linewidth=0.8)
    ax.scatter(data_non_stalled["x"], data_non_stalled["b"], color=color_failure, marker=marker_failure, label="Failure")
    ax.scatter(data_stalled["x"], data_stalled["b"], color=color_success, marker=marker_success, label="Success")
    ax.legend(loc='center right')
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_xlabel("X Offset [mm]")
    ax.set_ylabel("B Offset [deg]")
    ax.set_title("Figure 5c: B-X Offset Plane")

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

    plt.figure(figsize=(12, 6))
    ax1 = plt.subplot(1,2,1)
    ax2 = plt.subplot(2,2,2)
    ax3 = plt.subplot(2,2,4)

    data_ab0 = data[np.isclose(data["a"], 0.0) & np.isclose(data["b"], 0.0)]
    plot_offsets(data_ab0, ax=ax1)

    data_by0 = data[np.isclose(data["b"], 0.0) & np.isclose(data["y"], 0.0)]
    plot_offsets_ax(data_by0, ax=ax2)

    data_ay0 = data[np.isclose(data["a"], 0.0) & np.isclose(data["y"], 0.0)]
    plot_offsets_bx(data_ay0, ax=ax3)

    # output
    out_file = base_dir / "offset_plots.png"
    plt.suptitle("Alignment Tolerance")
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(out_file, dpi=300)
    print(f"saved offset plots to {out_file}")


if __name__ == "__main__":
    main()