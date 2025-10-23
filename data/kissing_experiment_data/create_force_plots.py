from pathlib import Path
import json
import matplotlib.pyplot as plt
import numpy as np


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
    duration, xs, ys, stalled, force_profiles = [], [], [], [], []
    for data in records:
        off = data.get("offset")
        if not off or len(off) < 2:
            print(f"warning: no valid offset in {data.get('name')}")
            continue
        duration.append(data.get("finish_time") - data.get("start_time"))
        xs.append(off[0])
        ys.append(off[1])
        stalled.append(data.get("motor_stalled"))
        raw_wrench_profile = data.get("wrench_profile")
        force_profiles.append([np.linalg.norm(np.array(wrench)[0:3]) for wrench in raw_wrench_profile])
    return duration, xs, ys, stalled, force_profiles


def plot_force_profiles(durations, xs, ys, stalled, force_profiles, out_path: Path | None = None):
    # sort by distance from force_profiles
    distances = [np.sqrt(x**2 + y**2) for x, y in zip(xs, ys)]
    sorted_indices = np.argsort(distances)  

    # create plot of force vs time, colored gradient by offset distance, dashed if not stalled
    plt.figure(figsize=(8, 6))
    for idx in sorted_indices:
        profile = force_profiles[idx]
        if not stalled[idx]:
            plt.plot(np.linspace(0, durations[idx], len(profile)), profile, linestyle='--', color=plt.cm.viridis(distances[idx] / max(distances)), label=f"Offset {distances[idx]:.3f}", alpha=0.7)
        else:
            plt.plot(np.linspace(0, durations[idx], len(profile)), profile, linestyle='-', color=plt.cm.viridis(distances[idx] / max(distances)), label=f"Offset {distances[idx]:.3f}", alpha=0.7)

    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.title("Force Over Time")
    plt.legend(fontsize='small', loc='upper right', bbox_to_anchor=(1.15, 1))
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
    duration, xs, ys, stalled, force_profiles = extract_data(records)
    if not xs:
        print("no valid offset points to plot")
        return
    out_file = base_dir / "force_profiles_plot.png"
    plot_force_profiles(duration, xs, ys, stalled, force_profiles, out_path=out_file)


if __name__ == "__main__":
    main()