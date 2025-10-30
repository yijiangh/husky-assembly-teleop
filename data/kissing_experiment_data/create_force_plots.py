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


def plot_force_profiles(data, title, out_path: Path | None = None):
    # sort by distance from force_profiles
    data = data.assign(
        x = lambda df: (df["x"]) * 1000,
        y = lambda df: (df["y"]) * 1000,
        tc_distance=lambda df: df["x"] + df["y"]
    )
    data.sort_values(by="tc_distance", inplace=True)
    
    # create plot of force vs time, colored gradient by offset distance, dashed if not stalled
    plt.figure(figsize=(8, 6))
    for sample in data.itertuples():
        if not sample.stalled:
            plt.plot(np.linspace(0, sample.duration, len(sample.force_profile)), sample.force_profile, linestyle='--', color=plt.cm.coolwarm(0.5 + sample.tc_distance / max(abs(data.tc_distance))), label=f"Offset {sample.tc_distance:.1f}mm", alpha=0.7)
        else:
            plt.plot(np.linspace(0, sample.duration, len(sample.force_profile)), sample.force_profile, linestyle='-', color=plt.cm.coolwarm(0.5 + sample.tc_distance / max(abs(data.tc_distance))), label=f"Offset {sample.tc_distance:.1f}mm", alpha=0.7)

    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.title(title)
    plt.legend(fontsize='small', loc='upper left')
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

    data_x0 = data[np.isclose(data["x"], 0.0) & np.isclose(data["a"], 0.0) & np.isclose(data["b"], 0.0)]
    data_y0 = data[np.isclose(data["y"], 0.0) & np.isclose(data["a"], 0.0) & np.isclose(data["b"], 0.0)]
    
    out_file_x = base_dir / "force_profiles_x_plot.png"
    plot_force_profiles(data_y0, "Force Profiles along X-axis", out_path=out_file_x)
    
    out_file_y = base_dir / "force_profiles_y_plot.png"
    plot_force_profiles(data_x0, "Force Profiles along Y-axis", out_path=out_file_y)


if __name__ == "__main__":
    main()