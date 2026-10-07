"""
Live plot of each mocap rigid body's marker error, as the health panel's mocap chip judges it.

Thin lines are the raw samples, thick lines the mean over MARKER_ERROR_WINDOW of the valid samples, the band
the mean ± one standard deviation over the same window. A cross marks a sample the relay flagged invalid (chip red);
the chip turns amber while the thick line is above the dashed MARKER_ERROR_WARN line.

Run with the venv and ROS sourced; by default it plots every rigid body the relay publishes:

    python3 src/husky-assembly-teleop/scripts/plot_marker_error.py [ids ...] [--span 20]
"""

from __future__ import annotations

import argparse
import math
import re
import threading
import time
from collections import defaultdict, deque

import matplotlib.pyplot as plt
import rclpy
from crl_husky_msgs.msg import MocapRigidBodyPose
from matplotlib.animation import FuncAnimation

from husky_assembly_teleop.robot_interface.mocap import mocap_topic
from husky_assembly_teleop.world.mocap import MARKER_ERROR_WARN, MARKER_ERROR_WINDOW

#: Seconds to wait for topics when looking up which rigid bodies the relay publishes.
DISCOVERY_SEC = 2.0
TOPIC_ID = re.compile(r"^/mocap/rigid_body/id_(\d+)/pose$")


def relay_ids(node) -> list[int]:
    """Rigid-body ids with a relay topic in the graph right now."""
    return sorted(int(m.group(1)) for name, _types in node.get_topic_names_and_types() if (m := TOPIC_ID.match(name)))


def discover_ids(node) -> list[int]:
    """Rigid-body ids with a relay topic, waiting up to DISCOVERY_SEC for the first to show up."""
    deadline = time.monotonic() + DISCOVERY_SEC
    while not relay_ids(node) and time.monotonic() < deadline:
        time.sleep(0.1)
    time.sleep(0.3)  # let the rest arrive
    return relay_ids(node)


def rolling_stats(valid: list[tuple[float, float]]) -> tuple[list[float], list[float], list[float]]:
    """Mean and standard deviation over the trailing MARKER_ERROR_WINDOW at each sample.

    Args:
        valid: (time, error) of the valid samples, oldest first.

    Returns:
        tuple: The samples' times, the rolling means and the rolling standard deviations.
    """
    times, means, stds = [], [], []
    start, total, total_sq = 0, 0.0, 0.0
    for t, value in valid:
        total += value
        total_sq += value * value
        while valid[start][0] <= t - MARKER_ERROR_WINDOW:
            total -= valid[start][1]
            total_sq -= valid[start][1] ** 2
            start += 1
        count = len(times) + 1 - start
        mean = total / count
        times.append(t)
        means.append(mean)
        stds.append(math.sqrt(max(total_sq / count - mean * mean, 0.0)))
    return times, means, stds


def main() -> None:
    """Subscribe to the rigid bodies and redraw the plot five times a second."""
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("ids", type=int, nargs="*", help="rigid-body ids; default: every one the relay publishes")
    parser.add_argument("--span", type=float, default=20.0, help="seconds of history shown")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("plot_marker_error")
    ids = args.ids or discover_ids(node)
    if not ids:
        raise SystemExit("no /mocap/rigid_body/id_*/pose topics found; is mocap_relay running?")

    #: Per id: (ROS time, marker error in mm, pose valid) of each sample.
    samples: dict[int, deque] = defaultdict(lambda: deque(maxlen=50_000))
    lock = threading.Lock()

    def store(rb_id: int):
        def callback(message: MocapRigidBodyPose) -> None:
            now = node.get_clock().now().nanoseconds * 1e-9
            with lock:
                samples[rb_id].append((now, message.marker_error * 1e3, message.pose_valid))
        return callback

    for rb_id in ids:
        node.create_subscription(MocapRigidBodyPose, mocap_topic(rb_id), store(rb_id), 50)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    warn_mm, window = MARKER_ERROR_WARN * 1e3, MARKER_ERROR_WINDOW
    fig, ax = plt.subplots(figsize=(12, 6))
    raw = {rb_id: ax.plot([], [], ".-", ms=2, lw=0.6, alpha=0.4)[0] for rb_id in ids}
    means = {rb_id: ax.plot([], [], "-", lw=2, color=raw[rb_id].get_color(), label=f"id {rb_id}")[0] for rb_id in ids}
    bands: dict[int, object] = {}
    invalid = ax.scatter([], [], marker="x", c="k", s=20, label="pose invalid")
    ax.axhline(warn_mm, color="orange", ls="--", label=f"warn {warn_mm:g} mm")
    ax.set_xlabel("seconds ago")
    ax.set_ylabel("marker error [mm]")
    ax.set_title(f"thin: samples · thick: {window:g} s mean of valid samples · band: ± 1 std")
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)

    def update(_frame):
        now = node.get_clock().now().nanoseconds * 1e-9
        crosses, top = [], warn_mm * 1.5
        with lock:
            recent = {rb_id: [s for s in samples[rb_id] if s[0] > now - args.span] for rb_id in ids}
        for rb_id, points in recent.items():
            raw[rb_id].set_data([t - now for t, _e, _v in points], [e for _t, e, _v in points])
            times, mean, std = rolling_stats([(t, e) for t, e, valid in points if valid])
            ago = [t - now for t in times]
            means[rb_id].set_data(ago, mean)
            if rb_id in bands:
                bands[rb_id].remove()
            bands[rb_id] = ax.fill_between(ago, [m - s for m, s in zip(mean, std)], [m + s for m, s in zip(mean, std)],
                                           color=raw[rb_id].get_color(), alpha=0.2, lw=0)
            crosses += [(t - now, e) for t, e, valid in points if not valid]
            top = max([top] + [e for _t, e, _v in points])
        invalid.set_offsets(crosses or [[math.nan, math.nan]])
        ax.set_xlim(-args.span, 0)
        ax.set_ylim(0, top * 1.1)

    # ! Keep a reference: the animation stops when it is garbage collected.
    animation = FuncAnimation(fig, update, interval=200, cache_frame_data=False)  # noqa: F841
    plt.show()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
