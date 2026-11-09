# Insertion demo: the settings every demo script and terminal uses: Cindy alone over one Ethernet cable, no mocap and
# no Zenoh router on this laptop. Sourced by base.sh, monitor.sh and tmux.sh, and in the session's checks pane.
#
# ! This reaches the real robot: a node started with it talks to Cindy through her own Zenoh router, over the cable.
#   Only with DEMO_SIM set (sim_tmux.sh sets it) it is the dry run: a simulated Cindy, all on this laptop.
# ! Sourced, also into an interactive shell: no set -e here, and it works under set -u (a script's set -euo pipefail).

#: This checkout and its workspace: scripts/insertion_demo/ -> the repo -> src/ -> the workspace.
DEMO_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEMO_WS="$(cd "$DEMO_REPO/../.." && pwd)"
#: The demo's process, relative to the Drive root: B6, a diagonal bar, into the jig on Cindy's base.
DEMO_PROCESS="data_design_study/261104_SRD_insertion/process"
#: Cindy's mocap rigid body, published standing at the origin by base.sh.
DEMO_BODY=1860
#: The port of the monitor's web page.
DEMO_PORT="${DEMO_PORT:-8080}"
export DEMO_REPO DEMO_WS DEMO_PROCESS DEMO_BODY DEMO_PORT

if [ -n "${DEMO_SIM:-}" ]; then
    # * The dry run (doc/lab/insertion_demo_sim_manual.md): the fake lab's settings (scripts/fake_lab/env.sh) in a
    #   domain of its own, 71 unless DEMO_SIM_DOMAIN names another: every node dials only the dry run's router on the
    #   loopback (sim_tmux.sh starts it), and the Drive root is the scratch copy sim_drive.sh makes.
    # ! No robot can reach the dry run and it reaches no robot; the fake lab's env.sh refuses the robots' domains.
    #: The scratch Drive (drive/) and the logs (logs/).
    DEMO_HOME="${DEMO_HOME:-$HOME/husky_insertion_demo_sim}"
    export DEMO_HOME FAKE_LAB_DOMAIN="${DEMO_SIM_DOMAIN:-71}" FAKE_LAB_HOME="$DEMO_HOME"
    source "$DEMO_REPO/scripts/fake_lab/env.sh" || return 1
    #: The Zenoh router every node dials, as host:port.
    export DEMO_ROUTER="127.0.0.1:$FAKE_LAB_ZENOH_PORT"
else
    #: The logs of the monitor and the panes.
    DEMO_HOME="${DEMO_HOME:-$HOME/husky_insertion_demo}"
    #: Cindy on her wired bridge br0, the arms' network, where the cable goes: read it on her with
    #: `ip -4 addr show br0` (manual 1.3); 192.168.0.115 is her WiFi address, not reachable over the cable.
    DEMO_CINDY_IP="${DEMO_CINDY_IP:-192.168.131.1}"
    #: The Zenoh router every node dials, as host:port: Cindy's own.
    DEMO_ROUTER="$DEMO_CINDY_IP:7447"
    # * The Drive: DEMO_DRIVE if set, else the Insync copy of the shared "2025-03 Husky Assembly" folder.
    if [ -z "${DEMO_DRIVE:-}" ]; then
        for candidate in "$HOME"/Insync/*/"Google Drive - Shared with me/2025-03 Husky Assembly" \
                         "$HOME/Insync/2025-03 Husky Assembly"; do
            [ -d "$candidate" ] && DEMO_DRIVE="$candidate" && break
        done
    fi
    export DEMO_HOME DEMO_CINDY_IP DEMO_ROUTER DEMO_DRIVE
    export HUSKY_DRIVE_ROOT="${DEMO_DRIVE:-}"

    # * ROS: rmw_zenoh in the robots' domain 80, as in the lab. No router on this laptop: every node dials Cindy's
    #   router (crl-zenoh-router on her PC, port 7447, every interface) over the cable, with multicast discovery off.
    # ! As a Zenoh client, not a peer: her router relays everything for a client. Two peers on one router are told to
    #   connect to each other directly, and her nodes listen on her loopback only: nothing of hers would arrive, and
    #   her launch would warn "Unable to connect to any locator of scouted peer … [tcp/127.0.0.1:…]".
    export ROS_DOMAIN_ID=80 RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/$DEMO_ROUTER\"];scouting/multicast/enabled=false"
    unset ZENOH_SESSION_CONFIG_URI

    # * The workspace: its venv, ROS, then its install. Their scripts read unset variables: they are sourced with
    #   set -u off, and it is back on after if it was on.
    demo_nounset=${-//[^u]/}
    set +u
    demo_loaded=yes
    source "$DEMO_WS/venv/bin/activate" || demo_loaded=""
    source /opt/ros/humble/setup.bash || demo_loaded=""
    source "$DEMO_WS/install/setup.bash" || demo_loaded=""
    [ -z "$demo_nounset" ] || set -u
    if [ -z "$demo_loaded" ]; then
        echo "insertion demo: could not load the workspace $DEMO_WS (above)" >&2
        return 1
    fi
fi

# Succeed if the Zenoh router the nodes dial answers; else say what to check.
demo_need_router() {
    timeout 2 bash -c "exec 3<>/dev/tcp/${DEMO_ROUTER%:*}/${DEMO_ROUTER#*:}" 2>/dev/null && return 0
    if [ -n "${DEMO_SIM:-}" ]; then
        echo "insertion demo dry run: no Zenoh router at $DEMO_ROUTER: start the session with sim_tmux.sh" \
            "(its router pane)" >&2
    else
        echo "insertion demo: no Zenoh router at $DEMO_ROUTER: is the cable in, this laptop on her subnet" \
            "(manual 1.3), and her launch running?" >&2
    fi
    return 1
}
