#!/bin/bash
# Fake lab: the Zenoh router that the simulated robots and the monitor meet through, until Ctrl-C. In the lab, each
# robot and each PC runs one (crl-husky's crl-zenoh-router service and zenoh_router.sh); here one stands in for all.
# Start it first: a node without a router waits for one.
#
# ! It listens on the loopback only, on the fake lab's own port (env.sh: 7400 + the domain, so 7470), and connects to
#   nothing.

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"

# * In a clean environment (lab_common.sh).
# shellcheck disable=SC2016  # expanded by the clean shell
lab_exec_clean FAKE_LAB_ "$here" '
    mkdir -p "$FAKE_LAB_HOME" && cd "$FAKE_LAB_HOME"  # out of any checkout
    echo "fake lab: Zenoh router for ROS domain $ROS_DOMAIN_ID, on 127.0.0.1:$FAKE_LAB_ZENOH_PORT only"
    # * Queues at the maximum, as the lab routers have them (crl-husky zenoh_router.sh).
    queues="{control:16,real_time:16,interactive_high:16,interactive_low:16,data_high:16,data:16,data_low:16,background:16}"
    export ZENOH_CONFIG_OVERRIDE="listen/endpoints=[\"tcp/127.0.0.1:$FAKE_LAB_ZENOH_PORT\"];connect/endpoints=[];scouting/multicast/enabled=false;transport/link/tx/queue/size=$queues"
    exec ros2 run rmw_zenoh_cpp rmw_zenohd
'
