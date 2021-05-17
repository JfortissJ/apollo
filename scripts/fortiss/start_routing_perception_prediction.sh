#!/bin/bash

cd /apollo # make sure the absolut paths are correct
python scripts/fortiss/set_cyber_ip.py

tmux new -d
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'bash scripts/bootstrap.sh start' C-m 
tmux send-keys 'cyber_monitor' C-m
tmux split-window -h
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/perception/production/launch/perception_fortiss.launch' C-m
tmux split-window -v -p 66
tmux send-keys 'source cyber/setup.bash' C-m
#tmux send-keys 'cyber_launch start modules/tools/prediction/fake_prediction/fake_prediction.launch' C-m
tmux send-keys 'cyber_launch start modules/prediction/launch/prediction.launch' C-m
tmux split-window -v
tmux send-keys 'source cyber/setup.bash' C-m
tmux send-keys 'cyber_launch start modules/routing/launch/routing.launch' C-m
tmux attach