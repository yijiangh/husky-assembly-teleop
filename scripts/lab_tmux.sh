#!/bin/bash
# Open a tmux session with one pane per component of the real or the fake lab (doc/lab/real_lab_short_manual.md,
# doc/lab/fake_lab_short_manual.md). Each pane has run `crl_ros_source <domain>` (the fake lab: and its router setting)
# and has its start command typed in, not run:
# read or edit it, then press Enter. Ctrl-b d detaches and everything keeps running; running this again re-attaches.
#
# Usage: lab_tmux.sh real
#        lab_tmux.sh fake <scratch drive>    the fake lab's copy of the Drive (fake_lab_short_manual.md 1.4)

set -e
monitor="ros2 launch husky_assembly_teleop monitor.launch.py robots:=\"alice belle cindy\" \
plugins:=\"assembly_steps robot_control base_planner_panel obstacles\""

# * Each pane: its name, then the command typed into it; `|now` after it runs it at once (a robot's login: it starts
#   nothing on her, and her launch waits for her arms). The keepalives survive a short Wi-Fi drop.
ssh_opts="-o ConnectTimeout=5 -o ServerAliveInterval=10 -o ServerAliveCountMax=30"
case "$1" in
    real) session=real_lab setup="crl_ros_source 80" panes=(
              "monitor|$monitor process_directory:=data_design_study/261103_SRD"
              "relay|ros2 launch husky_assembly_teleop mocap.launch.py"
              "checks|ros2 node list"
              "alice|ssh $ssh_opts administrator@192.168.0.113|now"
              "belle|ssh $ssh_opts administrator@192.168.0.114|now"
              "cindy|ssh $ssh_opts administrator@192.168.0.115|now") ;;
    fake) [ -d "$2" ] || { echo "usage: $0 fake <scratch drive>: an existing folder" >&2; exit 2; }
          drive=$(printf %q "$(cd "$2" && pwd)")
          # ! The fake lab's own router on the loopback, connected to nothing; every pane's nodes connect only to it.
          session=fake_lab
          setup="crl_ros_source 70; export ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/127.0.0.1:7470\"];scouting/multicast/enabled=false'"
          panes=(
              "router|ZENOH_CONFIG_OVERRIDE='listen/endpoints=[\"tcp/127.0.0.1:7470\"];connect/endpoints=[];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd"
              "monitor|$monitor process_directory:=data_design_study/261008_2ModulesDemo_purged_process drive_root:=$drive"
              "relay|ros2 launch husky_assembly_teleop mocap.launch.py sim:=true drive_root:=$drive"
              "robots|ros2 launch husky_assembly_teleop sim.launch.py robot:=all fake_motive:=true"
              "checks|ros2 node list") ;;
    *) echo "usage: $0 real | fake <scratch drive>" >&2; exit 2 ;;
esac

if ! tmux has-session -t "=$session" 2>/dev/null; then
    ids=()
    for i in "${!panes[@]}"; do
        if [ "$i" = 0 ]; then
            ids+=("$(tmux new-session -d -s "$session" -x 220 -y 60 -P -F '#{pane_id}')")
        else
            ids+=("$(tmux split-window -t "$session" -P -F '#{pane_id}')")
            tmux select-layout -t "$session" tiled
        fi
        tmux set-option -p -t "${ids[i]}" @name "${panes[i]%%|*}"
    done
    # * The name in each pane's top border: a pane option, since a shell or ssh overwrites the pane title.
    tmux set-option -t "$session" pane-border-status top
    tmux set-option -t "$session" pane-border-format " #{@name} "
    tmux set-option -t "$session" mouse on
    sleep 1  # the shells' prompts first, so the typed command shows after them
    for i in "${!panes[@]}"; do
        command="${panes[i]#*|}"
        tmux send-keys -t "${ids[i]}" "$setup" Enter "${command%|now}"
        [ "$command" = "${command%|now}" ] || tmux send-keys -t "${ids[i]}" Enter
    done
    tmux select-pane -t "${ids[0]}"
fi
if [ -n "$TMUX" ]; then
    tmux switch-client -t "=$session"
else
    tmux attach -t "=$session"
fi
