#!/bin/bash
# Fake lab: simulate all three robots (bases and arms) until Ctrl-C, with sim.launch.py next to this file: crl-husky's
# sim_all, but each base shifted from its parked pose in -y, clear of the process's Rhino cabinet; arms at their stow
# pose. Needs crl-husky f138fdb or later (park.yaml, and arm namespaces passed for every robot).
#
# Usage: sim.sh [shift_y:=<metres>]    arguments go to the launch file

set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The fake-lab settings that are set (FAKE_LAB_*), for the clean environment.
settings=(); for name in ${!FAKE_LAB_*}; do settings+=("$name=${!name}"); done

# ! A clean environment: your shell's ROS settings (domain, middleware, the lab workspace) never leak in.
exec env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin TERM="${TERM:-xterm}" \
    LANG="${LANG:-C.UTF-8}" "${settings[@]}" \
    bash --noprofile --norc -c '
        source "$0/env.sh" || exit 1
        mkdir -p "$FAKE_LAB_HOME" && cd "$FAKE_LAB_HOME"  # out of any checkout
        echo "fake lab: alice, belle and cindy on ROS domain $ROS_DOMAIN_ID, Zenoh router 127.0.0.1:$FAKE_LAB_ZENOH_PORT"
        exec ros2 launch "$0/sim.launch.py" "${@:1}"
    ' "$here" "$@"
