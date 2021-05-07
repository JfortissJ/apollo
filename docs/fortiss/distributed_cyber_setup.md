
# Time Synchronization using Linux PTP (recommended by apollo)

## Installation

* Follow https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_setup_dual_ipc.md

## Using PTP 
* Note the ethernet interface on the fortiss laptops and PC1 is: enp0s31f6 (So launching is done eg. via `sudo ./ptp4l -i enp0s31f6 -m`)
* linuxptp does not have to be installed on the system (but could be using the make install target), just launch it from the compiled source folder
* A good readme: https://docs.fedoraproject.org/en-US/fedora/rawhide/system-administrators-guide/servers/Configuring_PTP_Using_ptp4l/ 


### Master

* Make PC1 the master (it also runs the ntp master for the ros setup)
* sudo ./ptp4l -i enp0s31f6 -m
* You should see something like:
`
ptp4l[4224.743]: selected /dev/ptp0 as PTP clock
ptp4l[4224.745]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[4224.745]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[4232.559]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[4232.559]: selected local clock c85b76.fffe.9bd4d9 as best master <-- Hardware Address of this PC's (PC1) ethernet interface
ptp4l[4232.559]: port 1: assuming the grand master role
`

### Slave

* Make all other PCs slaves (two processes)
* sudo ./ptp4l -i enp0s31f6 -m -s
* sudo ./phc2sys -a -r -m
* 


* You should see something like:
`
ptp4l[3688.994]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[3688.994]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[3695.303]: selected local clock c85b76.fffe.9bd4d9 as best master
ptp4l[3701.754]: selected local clock c85b76.fffe.9bd4d9 as best master
ptp4l[3709.029]: selected local clock c85b76.fffe.9bd4d9 as best master
ptp4l[3709.093]: port 1: new foreign master c85b76.fffe.bc16b3-1
ptp4l[3713.537]: selected best master clock c85b76.fffe.bc16b3 <-- Hardware Adress of the remote PC's (PC1) ethernet interface
ptp4l[3713.537]: port 1: LISTENING to UNCALIBRATED on RS_SLAVE
ptp4l[3715.759]: master offset      27836 s0 freq   -1171 path delay     24961
ptp4l[3716.870]: master offset      27975 s1 freq   -1046 path delay     24904
ptp4l[3717.981]: master offset        583 s2 freq    -463 path delay     24904
ptp4l[3717.981]: port 1: UNCALIBRATED to SLAVE on MASTER_CLOCK_SELECTED
ptp4l[3719.092]: master offset       -919 s2 freq   -1790 path delay     24904
ptp4l[3720.203]: master offset        511 s2 freq    -636 path delay     24848
ptp4l[3721.315]: master offset        386 s2 freq    -607 path delay     24764
ptp4l[3722.426]: master offset        301 s2 freq    -577 path delay     24764
ptp4l[3723.537]: master offset        215 s2 freq    -572 path delay     24764
ptp4l[3724.649]: master offset       -510 s2 freq   -1233 path delay     24779
ptp4l[3725.760]: master offset        588 s2 freq    -288 path delay     24764
`
`
phc2sys[3727.450]: reconfiguring after port state change
phc2sys[3727.450]: selecting CLOCK_REALTIME for synchronization
phc2sys[3727.450]: selecting enp0s31f6 as the master clock
phc2sys[3727.450]: CLOCK_REALTIME phc offset -36566920429550 s0 freq    -559 delay      0
phc2sys[3728.451]: CLOCK_REALTIME phc offset -36566920429824 s1 freq    -833 delay      0
phc2sys[3729.451]: CLOCK_REALTIME phc offset       776 s2 freq     -57 delay      0
phc2sys[3730.451]: CLOCK_REALTIME phc offset       456 s2 freq    -144 delay      0
phc2sys[3731.451]: CLOCK_REALTIME phc offset      -391 s2 freq    -854 delay      0
phc2sys[3732.451]: CLOCK_REALTIME phc offset      -584 s2 freq   -1165 delay      0
phc2sys[3733.452]: CLOCK_REALTIME phc offset      -898 s2 freq   -1654 delay      0
phc2sys[3734.452]: CLOCK_REALTIME phc offset       571 s2 freq    -454 delay      0
phc2sys[3735.452]: CLOCK_REALTIME phc offset      -198 s2 freq   -1052 delay      0
`

