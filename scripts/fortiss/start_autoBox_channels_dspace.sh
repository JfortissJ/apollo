#!/bin/bash

tmux new -d
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/autobox_bridge/launch/autobox_bridge_chassis.launch' C-m 
tmux split-window -v
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/autobox_bridge/launch/autobox_bridge_control.launch' C-m 
tmux split-window -v
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/autobox_bridge/launch/autobox_bridge_localization.launch' C-m 
tmux split-window -v
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/autobox_bridge/launch/autobox_bridge_trajectory.launch' C-m 
tmux split-window -h
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/bridge/launch/bridge_sender_to_autobox_dspace.launch' C-m
tmux attach
