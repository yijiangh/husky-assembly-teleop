"""
Watch Motive's rigid body definitions and report refines, pivot edits, bodies made again and other bodies on an id.

The relay asks the same every few seconds and logs each change (doc/mocap_relay.md); this script tries changes by hand.

! WIP: of the lab test below, 3, 6 and 7 pass on the lab's Motive 3.0.3.1 (NatNet 4.0); the others are not done.
  Reads NatNet 4.0 and 4.1. A body missing for one reading comes back as "new body", not compared with its earlier
  definition: calibration compares against the definition it stored, so it needs no such history.

    python3 scripts/mocap_body_definitions.py watch                        Motive at 192.168.0.28, every 2 s
    python3 scripts/mocap_body_definitions.py watch --ids 1850,1866 --log /tmp/definitions.jsonl
    python3 scripts/mocap_body_definitions.py compare /tmp/definitions.jsonl 3 17   readings 3 and 17 of a log

Read-only: it only asks Motive for its model descriptions, so it can run next to the relay. Run it from the workspace
with the venv and ROS sourced (it uses the relay's NatNet client). Names and streaming ids are not part of a body: the
same markers under another name or id are reported as such, not as a change.

Lab test, one change at a time, noting the numbers each prints:
  1. Leave Motive alone for two minutes: nothing is reported (the same definitions every time).
  2. Rename a body: "renamed" only.
  3. Refine the tool body (Builder, Refine, Apply): "refined".
  4. Move its pivot, then turn it: "pivot moved" each time.
  5. Delete a body and make it again from the same markers, same streaming id: "redefined" (or "refined").
  6. Make a body from other markers under that streaming id: "other body".
  7. Change a body's streaming id: "id A -> B", unchanged.
  8. Restart Motive and load the same project: nothing.
Also note whether "Motive set tracked models changed" appears with each change. Seen: with a body added, removed or
swapped for another, not with a refine, so only reading the definitions catches a refine.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

from husky_assembly_teleop.mocap_relay.body_definitions import DescriptionClient, has_sizes
from husky_assembly_teleop.robot_interface.mocap_definitions import BodyDefinition, match

#: The relay's defaults (mocap_relay/node.py).
SERVER_IP, MULTICAST_ADDRESS = "192.168.0.28", "239.255.42.99"
#: Seconds to wait for Motive's handshake.
CONNECT_TIMEOUT = 5.0


def route_source(server_ip: str) -> str | None:
    """This PC's address on the route to `server_ip`, as mocap.launch.py finds it; None if unknown."""
    try:
        tokens = subprocess.check_output(["ip", "-4", "route", "get", server_ip], text=True).split()
    except (OSError, subprocess.CalledProcessError):
        return None
    return tokens[tokens.index("src") + 1] if "src" in tokens[:-1] else None


def shape_id(body: BodyDefinition) -> str:
    """A short id of the body's marker layout (pairwise distances to 0.1 mm), the same whatever its frame or order."""
    points = body.array()
    distances = np.sort(np.linalg.norm(points[:, None] - points[None, :], axis=2)[np.triu_indices(len(points), 1)])
    return hashlib.sha1(np.round(distances * 1e4).astype(np.int64).tobytes()).hexdigest()[:8]


def table(bodies: dict[int, BodyDefinition]) -> str:
    """The bodies as a table for people."""
    lines = [f"{'id':>6}  {'markers':>7}  {'reach mm':>8}  {'shape':>8}  name"]
    for key, body in sorted(bodies.items()):
        reach = float(np.linalg.norm(body.array(), axis=1).max()) * 1e3 if body.offsets else 0.0
        lines.append(f"{key:>6}  {len(body.offsets):>7}  {reach:>8.2f}  {shape_id(body):>8}  {body.name}")
    return "\n".join(lines)


def to_json(bodies: dict[int, BodyDefinition]) -> list[dict]:
    """The bodies for the log, offsets exact."""
    return [{"id": body.streaming_id, "name": body.name, "offsets": body.offsets, "labels": body.labels}
            for body in bodies.values()]


def from_json(entries: list[dict]) -> dict[int, BodyDefinition]:
    """The bodies of a log line."""
    return {entry["id"]: BodyDefinition(entry["id"], entry["name"], tuple(tuple(xyz) for xyz in entry["offsets"]),
                                        tuple(entry["labels"])) for entry in entries}


def watch(args: argparse.Namespace) -> int:
    """Ask Motive for its definitions every `--every` seconds and report what changed; Ctrl+C ends with a summary."""
    client_ip = args.client_ip or route_source(args.server_ip)
    if client_ip is None:
        print(f"no route to {args.server_ip}: pass --client-ip", file=sys.stderr)
        return 2
    client = DescriptionClient()
    client.set_client_address(client_ip)
    client.set_server_address(args.server_ip)
    client.set_use_multicast(args.multicast)
    client.multicast_address = args.multicast_address
    client.command_port = args.command_port
    if not client.run():
        print("could not open the NatNet sockets", file=sys.stderr)
        return 2
    deadline = time.monotonic() + CONNECT_TIMEOUT
    while not client.connected() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not client.connected():
        print(f"no answer from Motive at {args.server_ip} (client {client_ip}) within {CONNECT_TIMEOUT:g} s",
              file=sys.stderr)
        client.shutdown()
        return 2
    major, minor = client.get_major(), client.get_minor()
    print(f"Motive {'.'.join(map(str, client.get_server_version()))}, NatNet {major}.{minor}"
          f"{' (descriptions carry sizes)' if has_sizes(major, minor) else ''}; client {client_ip}")
    if major < 3:
        print("NatNet before 3 sends no marker offsets: stream NatNet 3 or later from Motive", file=sys.stderr)
        client.shutdown()
        return 2
    ids = {int(token) for token in args.ids.split(",") if token} if args.ids else None
    log = Path(args.log).open("a") if args.log else None
    previous, readings, missed, kinds, jitter, flagged = None, 0, 0, Counter(), 0.0, 0
    try:
        while True:
            client.request()
            replies, waited = [], 0.0
            while not replies and waited < args.every:
                time.sleep(0.05)
                waited += 0.05
                replies = client.take_replies()
            if not replies:
                missed += 1
                print(f"{time.strftime('%H:%M:%S')}  no reply in {args.every:g} s")
                continue
            if client.models_changed > flagged:
                print(f"{time.strftime('%H:%M:%S')}  Motive set tracked models changed in "
                      f"{client.models_changed - flagged} frames")
                flagged = client.models_changed
            stamp, found, problems = replies[-1].time, replies[-1].bodies, replies[-1].problems
            bodies = {body.streaming_id: body for body in found if ids is None or body.streaming_id in ids}
            for problem in problems:
                print(f"{time.strftime('%H:%M:%S')}  {problem}")
            events = [] if previous is None else match(previous, bodies)
            if previous is None:
                print(table(bodies))
            for event in events:
                change = event.change
                if change is not None and change.kind == "unchanged":
                    jitter = max(jitter, change.markers_moved, change.pivot_moved)
                    continue
                kinds[change.kind if change else ("new body" if event.before is None else "body gone")] += 1
                flag = " -> data out of date" if change is not None and change.data_change else ""
                print(f"{time.strftime('%H:%M:%S')}  {event.describe()}{flag}")
            if log is not None:
                log.write(json.dumps({"reading": readings, "time": stamp, "natnet": [major, minor],
                                      "models_changed": client.models_changed, "problems": problems,
                                      "bodies": to_json(bodies),
                                      "events": [event.describe() for event in events]}) + "\n")
                log.flush()
            previous, readings = bodies, readings + 1
            time.sleep(max(0.0, args.every - waited))
    except KeyboardInterrupt:
        pass
    finally:
        client.shutdown()
        if log is not None:
            log.close()
    print(f"\n{readings} readings, {missed} unanswered; reported: {dict(kinds) or 'nothing'}; "
          f"{client.models_changed} frames flagged tracked models changed; largest difference within 'unchanged': "
          f"{jitter * 1e3:.4f} mm")
    return 0


def compare_log(args: argparse.Namespace) -> int:
    """Report what changed between two readings of a `watch --log` file, unchanged bodies included."""
    lines = [json.loads(line) for line in Path(args.log).read_text().splitlines() if line.strip()]
    by_reading = {line["reading"]: line for line in lines}
    missing = [n for n in (args.first, args.second) if n not in by_reading]
    if missing:
        print(f"no reading {missing} in {args.log} (it has {min(by_reading)}..{max(by_reading)})", file=sys.stderr)
        return 2
    first, second = (from_json(by_reading[n]["bodies"]) for n in (args.first, args.second))
    for event in match(first, second):
        print(event.describe())
    return 0


def main() -> int:
    """Parse the command line and run it."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    commands = parser.add_subparsers(dest="command", required=True)
    watching = commands.add_parser("watch", help="report changes to Motive's definitions as they happen")
    watching.add_argument("--server-ip", default=SERVER_IP)
    watching.add_argument("--client-ip", default=None, help="this PC's address towards Motive; found by default")
    watching.add_argument("--multicast", action="store_true", help="as Motive streams; the relay uses unicast")
    watching.add_argument("--multicast-address", default=MULTICAST_ADDRESS)
    watching.add_argument("--command-port", type=int, default=1510)
    watching.add_argument("--every", type=float, default=2.0, help="seconds between readings")
    watching.add_argument("--ids", default="", help="streaming ids to watch, comma separated; all by default")
    watching.add_argument("--log", default="", help="append every reading to this JSON-lines file")
    watching.set_defaults(run=watch)
    comparing = commands.add_parser("compare", help="what changed between two readings of a log")
    comparing.add_argument("log")
    comparing.add_argument("first", type=int)
    comparing.add_argument("second", type=int)
    comparing.set_defaults(run=compare_log)
    args = parser.parse_args()
    return args.run(args)


if __name__ == "__main__":
    sys.exit(main())
