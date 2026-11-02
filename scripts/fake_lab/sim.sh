#!/bin/bash
# Fake lab: simulate all three robots (bases and arms) until Ctrl-C, parked, arms at their stow pose, with a fake
# Motive: this checkout's launch/sim.launch.py (robot:=all fake_motive:=true; doc/simulation_manual.md). The relay
# (relay.sh) publishes their poses. Needs crl-husky jg/ros2 at 78041cf or later (husky_sim publishes the true base pose, tools are simulated).
#
# * The fake Motive has Motive's measured latency, jitter and marker errors (simulation_manual.md, "Realistic mocap"):
#   Belle's base reports 1.2 mm, so her mocap chip is amber, as in the lab. No drift, no dropouts.
#
# Usage: sim.sh [<argument>:=<value> ...]    more arguments for sim.launch.py; one given here wins over the default,
#        e.g. sim.sh fake_motive:=false (the true poses), sim.sh dropout:=0.002 dropout_burst_s:=0.5

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"

# * In a clean environment (lab_common.sh).
# shellcheck disable=SC2016  # expanded by the clean shell
lab_exec_clean FAKE_LAB_ "$here" '
    # ! A start that died part way leaves its nodes running. A second controller manager of the same name then
    #   answers for an arm at random, and the spawners of the new start set up the wrong one: refuse until stopped.
    nodes="ros2_control_node|/crl_husky|robot_state_publisher|husky_sim_motive|husky_sim_wrench"
    left=$(for pid in $(pgrep -f "$nodes"); do
        if grep -qsxz "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" "/proc/$pid/environ"; then echo "$pid"; fi
    done)
    if [ -n "$left" ]; then
        echo "fake lab: $(echo $left | wc -w) simulator nodes of an earlier start still run on domain" \
            "$ROS_DOMAIN_ID; stop them first with:  kill -INT" $left >&2
        exit 1
    fi
    mkdir -p "$FAKE_LAB_HOME" && cd "$FAKE_LAB_HOME"  # out of any checkout
    echo "fake lab: alice, belle and cindy on ROS domain $ROS_DOMAIN_ID, Zenoh router 127.0.0.1:$FAKE_LAB_ZENOH_PORT"
    exec ros2 launch "$FAKE_LAB_REPO/launch/sim.launch.py" robot:=all fake_motive:=true delay_sec:=0.006 \
        noise:="0.000015,0.003 1866:0.00001,0.015" \
        marker_error:="0.0008 1840:0.0005 1850:0.0012 1860:0.0009 1866:0.0003" "$@"
' "$@"
