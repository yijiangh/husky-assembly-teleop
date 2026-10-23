#!/bin/bash
# Fake lab: cut or restore one simulated robot's link, as if its WiFi dropped: freezes (SIGSTOP) or wakes (SIGCONT)
# that robot's simulator processes. Nothing is killed, so "up" brings the robot back where it was.
#
# Usage: link.sh alice|belle|cindy down|up
#
# ! Only children of the running fake-lab launch (sim.sh) are touched, never a shell or a tool that merely names the
#   robot's namespace. The fake Motive names no robot and keeps running: it goes on publishing the frozen base where
#   it stands, as Motive tracks a robot whose WiFi dropped.

set -e
case "$1" in
    alice) namespace=a200_0804 ;;
    belle) namespace=a200_0805 ;;
    cindy) namespace=a200_0806 ;;
    *) echo "usage: $0 alice|belle|cindy down|up" >&2; exit 2 ;;
esac
case "$2" in
    down) signal=STOP ;;
    up) signal=CONT ;;
    *) echo "usage: $0 alice|belle|cindy down|up" >&2; exit 2 ;;
esac

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
domain="${FAKE_LAB_DOMAIN:-70}"
# The fake-lab launch on this domain (another one may run on its own domain beside it).
launch=""
for pid in $(pgrep -u "$USER" -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch $repo/launch/sim.launch.py" || true); do
    tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$domain" && launch+="$pid "
done
if [ "$(echo $launch | wc -w)" != 1 ]; then
    echo "link.sh: expected one fake-lab launch (sim.sh) on domain $domain, found: ${launch:-none}" >&2
    exit 1
fi
# The launch's own children whose arguments name this robot's namespace: its base simulator, follower and arm nodes.
pids=""
for pid in $(pgrep -P "${launch% }"); do  # without the trailing space, which pgrep refuses
    tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q "$namespace" && pids+="$pid "
done
if [ -z "$pids" ]; then
    echo "link.sh: no simulator process of $1 ($namespace) under the fake-lab launch" >&2
    exit 1
fi
kill -s "$signal" $pids
echo "link.sh: $1 $2 ($(echo $pids | wc -w) processes)"
