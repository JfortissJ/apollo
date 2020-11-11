# Usage: sudo bash ptp_pc1_master.sh
# Starts ptp in PC1 which is used as master node
# For more info, please visit https://git.fortiss.org/fav/apollo35/-/blob/dev_fortiss/docs/fortiss/distributed_cyber_setup.md
cd ~/linuxptp/
./ptp4l -i enp0s31f6 -m
