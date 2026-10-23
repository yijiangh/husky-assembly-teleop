#!/bin/bash
# Fake lab: the tmux session "fake_lab" that you and Claude share, then attach to it.
#
# One window "lab" with four panes:
# - top: the three simulated robots and the fake Motive (sim.sh), started at once;
# - middle: the monitor (monitor.sh), typed in but not started. Press Enter once the robots are up (~20 s).
# - bottom left, small: the Zenoh router that every node meets through (router.sh), started first;
# - bottom right, small: the mocap relay (relay.sh), started next.
#
# Claude reads the panes (tmux capture-pane) and never types into them unless you ask. Detach with Ctrl-b d; the
# session keeps running. Running this again only re-attaches. Stop everything with: tmux kill-session -t fake_lab

set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
session="${FAKE_LAB_SESSION:-fake_lab}"
# * The panes' shells come from the tmux server, not from this shell: pass the fake-lab settings that are set.
settings=""
for name in ${!FAKE_LAB_*}; do
    settings+="$name=$(printf %q "${!name}") "
done
if ! tmux has-session -t "$session" 2>/dev/null; then
    tmux new-session -d -s "$session" -n lab -x 220 -y 60
    tmux set-option -t "$session" history-limit 50000
    tmux split-window -v -t "$session:lab"
    tmux split-window -v -l 5 -t "$session:lab.1"
    tmux split-window -h -t "$session:lab.2"
    tmux send-keys -t "$session:lab.2" "$settings$here/router.sh" Enter
    tmux send-keys -t "$session:lab.3" "$settings$here/relay.sh" Enter
    tmux send-keys -t "$session:lab.0" "$settings$here/sim.sh" Enter
    tmux send-keys -t "$session:lab.1" "$settings$here/monitor.sh"
fi
if [ -z "$TMUX" ]; then
    exec tmux attach -t "$session"
else
    echo "already inside tmux: switch with  tmux switch-client -t $session"
fi
