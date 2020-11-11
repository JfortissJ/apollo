# Usage: sudo bash ptp_laptop_slave.sh
# Starts ptp in fortiss laptop which is used as a slave node
# For more info, please visit https://git.fortiss.org/fav/apollo35/-/blob/dev_fortiss/docs/fortiss/distributed_cyber_setup.md
cd ~/linuxptp/

tmux new -d
tmux send-keys './ptp4l -i enp0s31f6 -m -s' C-m 

tmux split-window -v
tmux send-keys './phc2sys -a -r -m' C-m

tmux select-layout tiled
tmux attach
