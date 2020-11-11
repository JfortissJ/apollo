# Apollo Autobox Control

This project contributes 
- a C-implemenation of Apollo's bridge module to be evaluated on a dSpace Micro Autobox rapid prototyping platform, see modules/bridge_c,
- a trajectory tracking controller for Apollo's ADCTrajectories,
- the CAN interfacing for a Passat GTE vehicle,
as a Simulink model.

# Prerequisites
- Install the protoc-c compiler for windows either building protobuf-c from source via cmake (https://github.com/protobuf-c/protobuf-c) or install eg. Msys2 (https://www.msys2.org/) and install a prebuild version. Then execute the tools/generate_proto_c.bat to generate the *.pb-c.c/h files from the proto definitions. The hack solution: generate the files in Linux and copy them to Windows.
- Install MATLAB, Simulink, Stateflow
- Intall the dSpace toolchain (we use the CAN Blockset and the UDP Ethernet Blockset)

# Setup
- Matlab root path shall be modules/autobox_control
- Execute startup.m
- Build the model for dSapce via build_rti1401.m

# Details
Details on the methodology can be found in our paper:
> T. Kessler et al., "Bridging the Gap between Open Source Software and Vehicle Hardware for Autonomous Driving," 2019 IEEE Intelligent Vehicles Symposium (IV), Paris, France, 2019, pp. 1612-1619, doi: 10.1109/IVS.2019.8813784.

