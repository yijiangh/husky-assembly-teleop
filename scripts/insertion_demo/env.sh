# Insertion demo: the settings every demo script and terminal uses: Cindy alone over one Ethernet cable, no mocap and
# no Zenoh router on this laptop. Sourced by base.sh, monitor.sh and tmux.sh, and in the session's checks pane.
#
# ! This reaches the real robot: a node started with it talks to Cindy through her own Zenoh router, over the cable.
# ! Sourced, also into an interactive shell: no set -e here, and it works under set -u (a script's set -euo pipefail).

#: This checkout and its workspace: scripts/insertion_demo/ -> the repo -> src/ -> the workspace.
DEMO_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEMO_WS="$(cd "$DEMO_REPO/../.." && pwd)"
#: The logs of the monitor and the panes.
DEMO_HOME="${DEMO_HOME:-$HOME/husky_insertion_demo}"
#: The demo's process, relative to the Drive root: B6, a diagonal bar, into the jig on Cindy's base.
DEMO_PROCESS="data_design_study/261104_SRD_insertion/process"
#: Cindy on her wired bridge br0, the arms' network, where the cable goes: read it on her with `ip -4 addr show br0`
#: (manual 1.3); 192.168.0.115 is her WiFi address, not reachable over the cable.
DEMO_CINDY_IP="${DEMO_CINDY_IP:-192.168.131.1}"
#: Cindy's mocap rigid body, published standing at the origin by base.sh.
DEMO_BODY=1860
# * The Drive: DEMO_DRIVE if set, else the Insync copy of the shared "2025-03 Husky Assembly" folder.
if [ -z "${DEMO_DRIVE:-}" ]; then
    for candidate in "$HOME"/Insync/*/"Google Drive - Shared with me/2025-03 Husky Assembly" \
                     "$HOME/Insync/2025-03 Husky Assembly"; do
        [ -d "$candidate" ] && DEMO_DRIVE="$candidate" && break
    done
fi
export DEMO_REPO DEMO_WS DEMO_HOME DEMO_PROCESS DEMO_CINDY_IP DEMO_BODY DEMO_DRIVE
export HUSKY_DRIVE_ROOT="${DEMO_DRIVE:-}"

# * ROS: rmw_zenoh in the robots' domain 80, as in the lab. No router on this laptop: every node dials Cindy's router
#   (crl-zenoh-router on her PC, port 7447, every interface) over the cable, with multicast discovery off.
export ROS_DOMAIN_ID=80 RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/$DEMO_CINDY_IP:7447\"];scouting/multicast/enabled=false"
unset ZENOH_SESSION_CONFIG_URI

# * The workspace: its venv, ROS, then its install. Their scripts read unset variables: they are sourced with set -u
#   off, and it is back on after if it was on.
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

# Whether Cindy's Zenoh router answers on the cable (her br0 address, port 7447).
demo_cindy_up() {
    timeout 2 bash -c "exec 3<>/dev/tcp/$DEMO_CINDY_IP/7447" 2>/dev/null
}
