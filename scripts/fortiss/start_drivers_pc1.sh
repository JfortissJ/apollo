#!/bin/bash

# A launch script to start all the drivers on car pc1

cd /apollo # make sure the absolut paths are correct
python scripts/fortiss/set_cyber_ip.py

tmux new -d
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/drivers/imar/launch/imar.launch' C-m
tmux split-window -h
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/drivers/velodyne/launch/velodyne32_fortuna.launch' C-m
tmux split-window -v
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/localization/launch/rtk_localization.launch' C-m
tmux split-window -v
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'sudo ip link set can2 type can bitrate 500000' C-m
tmux send-keys 'sudo ip link set up can2' C-m
tmux send-keys 'cyber_launch start modules/canbus/launch/canbus_fortuna.launch' C-m
tmux select-pane -t 0
tmux split-window -v
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/transform/launch/static_transform_fortiss.launch' C-m
tmux split-window -v
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/drivers/camera/launch/dashcam.launch' C-m
tmux select-layout tiled
tmux attach
