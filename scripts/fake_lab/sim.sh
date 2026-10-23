#!/bin/bash
# Fake lab: simulate all three robots (bases and arms) until Ctrl-C, parked, arms at their stow pose, with a fake
# Motive: this checkout's launch/sim.launch.py (robot:=all fake_motive:=true; doc/simulation_manual.md). The relay
# (relay.sh) publishes their poses. Needs crl-husky jg/ros2-next or later (husky_sim publishes the true base pose).
#
# * The fake Motive has Motive's measured latency, jitter and marker errors (simulation_manual.md, "Realistic mocap"):
#   Belle's base reports 1.2 mm, so her mocap chip is amber, as in the lab. No drift, no dropouts.
#
# Usage: sim.sh [<argument>:=<value> ...]    more arguments for sim.launch.py; one given here wins over the default,
#        e.g. sim.sh fake_motive:=false (the true poses), sim.sh dropout:=0.002 dropout_burst_s:=0.5

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
        exec ros2 launch "$FAKE_LAB_REPO/launch/sim.launch.py" robot:=all fake_motive:=true delay_sec:=0.006 \
            noise:="0.000015,0.003 1866:0.00001,0.015" \
            marker_error:="0.0008 1840:0.0005 1850:0.0012 1860:0.0009 1866:0.0003" "${@:1}"
    ' "$here" "$@"
