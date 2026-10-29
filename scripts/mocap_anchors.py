"""
List Motive's unlabeled markers that stay put, in the lab frame, to write the lab frame file's anchors.

Motive streams anchors as unlabeled markers, with no name and a temporary id. This listens to the mocap relay's
`/mocap_relay/unlabeled_markers` (each marker averaged over one look, in the lab frame the relay applies, refit
included) and prints every marker seen in nearly every look and staying put; with --json an "anchors" section to paste
into the lab frame file `data/lab/main_frame.json` (`world/lab_frame.py`): keep the anchors, name them, drop the rest,
commit.

    python3 scripts/mocap_anchors.py                    20 s of the relay's looks
    python3 scripts/mocap_anchors.py --json

! Write it right after a floor calibration whose marks agree (the probe's "fixed" chip), with the frame held: the
  anchors then define the lab frame with the marks, and every later floor calibration is fitted to both.

Read-only. Run it from the workspace with the venv and ROS sourced, in the relay's ROS domain, with "Unlabeled Markers"
on in Motive's streaming settings.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

#: The relay's unlabeled markers and its floor calibration's state.
MARKERS_TOPIC, FRAME_TOPIC = "/mocap_relay/unlabeled_markers", "/mocap_relay/frame"
#: A marker seen in fewer looks than this share, or wandering more than this, metres, is no anchor.
SEEN_SHARE, STILL = 0.95, 0.5e-3
#: Markers of two looks this close, metres, are taken for the same marker.
SAME_MARKER = 5e-3


def listen(seconds: float) -> tuple[list[list[tuple]], dict | None]:
    """The relay's looks for `seconds` (each a list of marker positions), and its last `~/frame` announcement."""
    looks, frame, lock = [], [None], threading.Lock()
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_markers(message):
        with lock:
            looks.append([tuple(marker) for marker in json.loads(message.data)["markers"]])

    def on_frame(message):
        with lock:
            frame[0] = json.loads(message.data)

    rclpy.init()
    node = rclpy.create_node("mocap_anchors")
    # * Not latched: only the looks made while listening count, not the one before.
    node.create_subscription(String, MARKERS_TOPIC, on_markers, 10)
    node.create_subscription(String, FRAME_TOPIC, on_frame, latched)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    with lock:
        return list(looks), frame[0]


def still_markers(looks: list[list[tuple]]) -> list[tuple[np.ndarray, float]]:
    """The markers seen in nearly every look and staying put: mean position and spread, metres, sorted by x then y.

    A marker of one look is the same as a track's whose mean is within SAME_MARKER, else a new one.
    """
    tracks: list[list[np.ndarray]] = []
    for markers in looks:
        for position in map(np.asarray, markers):
            near = [track for track in tracks if np.linalg.norm(np.mean(track, axis=0) - position) <= SAME_MARKER]
            if near:
                near[0].append(position)
            else:
                tracks.append([position])
    still = []
    for track in tracks:
        points = np.asarray(track)
        spread = float(np.linalg.norm(points.std(axis=0)))
        if len(points) >= SEEN_SHARE * len(looks) and spread <= STILL:
            still.append((points.mean(axis=0), spread))
    return sorted(still, key=lambda item: (round(item[0][0], 2), item[0][1]))


def main() -> None:
    """List the still markers, or print them as an anchors file."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=20.0, help="how long to listen")
    parser.add_argument("--json", action="store_true", help="print an anchors file of every still marker")
    args = parser.parse_args()
    looks, frame = listen(args.seconds)
    if not looks:
        sys.exit(f"nothing on {MARKERS_TOPIC}: is the relay running in this ROS domain?")
    if frame is None or not frame.get("floor_calibration"):
        sys.exit("the relay applies no floor calibration: its markers are not in the lab frame")
    if frame.get("status") != "held":
        print(f"! the relay's frame is {frame.get('status')}, not held: the markers may be off the lab frame",
              file=sys.stderr)
    still = still_markers(looks)
    source = frame["floor_calibration"] + (" with its refit" if frame.get("refit") else "")
    if args.json:
        print(f"// the lab frame of {source}: z up from the floor plane, metres")
        print(json.dumps({"anchors": {f"M{index + 1}": [round(float(v), 5) for v in position]
                                      for index, (position, _) in enumerate(still)}}, indent=2))
        return
    print(f"{len(still)} still unlabeled markers in {len(looks)} looks, lab frame of {source} (metres):")
    for index, (position, spread) in enumerate(still):
        print(f"  M{index + 1:<3} x {position[0]:7.3f}  y {position[1]:7.3f}  z {position[2]:6.3f}   "
              f"spread {spread * 1e3:.2f} mm")


if __name__ == "__main__":
    main()
