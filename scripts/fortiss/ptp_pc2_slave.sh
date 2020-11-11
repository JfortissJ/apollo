# Usage: sudo bash ptp_pc2_slave.sh
# Starts ptp in PC2 which is used as a slave node
# For more info, please visit https://git.fortiss.org/fav/apollo35/-/blob/dev_fortiss/docs/fortiss/distributed_cyber_setup.md
cd ~/linuxptp/

tmux new -d
tmux send-keys './ptp4l -i enp4s0 -m -s' C-m 

tmux split-window -v
tmux send-keys './phc2sys -a -r -m' C-m

tmux select-layout tiled
tmux attach
