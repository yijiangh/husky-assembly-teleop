#!/usr/bin/env bash
# * Opens the five terminal panes of the three-robot test in tmux (three-robot manual, Part 0).
#
#     ~/ros2_ws/src/husky-assembly-teleop/scripts/husky/open_terminals.sh
#
#   ┌──────────────────────────┬────────────────┐
#   │ 1 MONITOR                │ 2 ALICE (ssh)  │
#   │  settings checks,        ├────────────────┤
#   │  then the monitor        │ 3 BELLE (ssh)  │
#   ├──────────────────────────┼────────────────┤
#   │ 5 CHECKS                 │ 4 CINDY (ssh)  │
#   └──────────────────────────┴────────────────┘
#
# - No tmux session 'husky_real' yet: creates it with this window ('robots') and opens it.
# - The session exists: adds the window to it and switches there. Nothing is closed, so robot
#   launches running in other windows stay up. A 'robots' window that exists already is reused.
# - Each pane's title sits on its top border, and each pane prints what it is for.
# - Everything shown in pane 1 is also written to ~/husky_real/monitor.log.
# - No pane starts with a ROS_DOMAIN_ID: the robot's block in pane 1 sets it (manual Part 0,
#   "Which robot will the monitor drive?").

SESSION=husky_real
WINDOW=robots
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
LOG_DIR="$HOME/husky_real"
mkdir -p "$LOG_DIR"

# Open (attach to, or switch to) the robots window.
open_window() {
    if [ -n "$TMUX" ]; then
        tmux switch-client -t "$SESSION:$WINDOW"
    else
        tmux attach -t "$SESSION:$WINDOW"
    fi
}

if tmux has-session -t "$SESSION" 2>/dev/null \
   && tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$WINDOW"; then
    echo "[open_terminals] '$SESSION' already has the '$WINDOW' window; opening it."
    open_window
    exit 0
fi

# A pane's start command: print its hint, then a normal shell without any ROS_DOMAIN_ID.
pane_cmd() {
    printf "printf '\\\\033[1m%%s\\\\033[0m\\\\n%%s\\\\n\\\\n' %q %q; exec env -u ROS_DOMAIN_ID bash" "$1" "$2"
}

# * ------------------------------------------------------------- pane 1 and the window
CMD1=$(pane_cmd "1 MONITOR" "Settings checks (Part 1), then: paste the robot's block, ros2 daemon stop, ros2 run husky_assembly_teleop husky_monitor.")
if tmux has-session -t "$SESSION" 2>/dev/null; then
    P1=$(tmux new-window -t "$SESSION" -n "$WINDOW" -c "$WS" -P -F '#{pane_id}' "$CMD1")
else
    P1=$(tmux new-session -d -s "$SESSION" -n "$WINDOW" -c "$WS" -x "$(tput cols)" -y "$(tput lines)" \
         -P -F '#{pane_id}' "$CMD1")
fi
[ -n "$P1" ] || { echo "[open_terminals] ! could not create the '$WINDOW' window (see the tmux error above)"; exit 1; }

# * ------------------------------------------------------------- the other four
# Right column (40 % wide): Alice, Belle, Cindy; then pane 5 under pane 1.
P2=$(tmux split-window -h -l 40% -t "$P1" -c "$HOME" -P -F '#{pane_id}' \
     "$(pane_cmd "2 ALICE (ssh)" "ssh administrator@192.168.0.113 (password clearpath), then her launch line (Part 2).")")
P3=$(tmux split-window -v -l 66% -t "$P2" -c "$HOME" -P -F '#{pane_id}' \
     "$(pane_cmd "3 BELLE (ssh)" "ssh administrator@192.168.0.114 (password clearpath), then her launch line (Part 2).")")
P4=$(tmux split-window -v -l 50% -t "$P3" -c "$HOME" -P -F '#{pane_id}' \
     "$(pane_cmd "4 CINDY (ssh)" "ssh administrator@192.168.0.115, then her launch line (Part 2).")")
P5=$(tmux split-window -v -l 35% -t "$P1" -c "$WS" -P -F '#{pane_id}' \
     "$(pane_cmd "5 CHECKS" "scripts/husky/check_robots.sh, joint / finger echoes, base driving, recovery. Never 'export ROS_DOMAIN_ID' here.")")

# * ------------------------------------------------------------- titles and log
tmux set-option -w -t "$SESSION:$WINDOW" pane-border-status top
# ! The ssh shells would otherwise rename the panes to their own prompt.
tmux set-option -w -t "$SESSION:$WINDOW" allow-set-title off
tmux select-pane -t "$P1" -T "1 MONITOR"
tmux select-pane -t "$P2" -T "2 ALICE (ssh)"
tmux select-pane -t "$P3" -T "3 BELLE (ssh)"
tmux select-pane -t "$P4" -T "4 CINDY (ssh)"
tmux select-pane -t "$P5" -T "5 CHECKS"
# Pane 5's ROS env (no domain): the commands there carry their robot's domain inline.
tmux send-keys -t "$P5" "source $WS/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" Enter
tmux pipe-pane -o -t "$P1" "cat >> '$LOG_DIR/monitor.log'"
tmux select-pane -t "$P1"

echo "[open_terminals] window '$WINDOW' ready in session '$SESSION' (pane 1 logged to $LOG_DIR/monitor.log)."
open_window
