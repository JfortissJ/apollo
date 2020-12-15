# List of fortiss Code Contributions

| Module | Description | Path | License |
| --- | --- | --- | --- | --- |
| Autobox Trajectory Tracking Controller | A trajectory tracking controller for trajectories in apollo's ADCTrajectory format, CAN actuation interfaces for the VW Passat, UDP communication to the PC running apollo for a dSpace Micro Autobox platform written in Simulink and C. | modules/autobox_control | LGPLv2 |
| Autobox Brigde | An apollo module removing unnecessary parts for the Autobox Controller from messages to keep the UDP package size samll. | modules/autobox_bridge | Apache 2 |
| bridge_c | A pure-C implementation of apollo's bridge component (to be used on the dSpace plattform) | modules/bridge_c | LGPLv2 |
| Fortuna Canbus | An extension for apollo's canbus component for some of the interfaces from the VW Passat. The new vehicle fortuna publishes an adapted chassis_detail msg. | modues/canbus, modules/vehicles/fortuna | Apache 2 |
| VW production radar perception | An apollo component taking raw radar detection objects (from the canbus component) and outputs apollo PreFused objects. | modules/perception/onboard/component | Apache 2 |
| IMAR Driver | An apollo component publishing GPS and INS message from an imar inat dgps system based on imar's SDK. | modules/drivers/imar | Apache 2 |
| Opendrive Map Converter | A tool to convert standard XODR maps into apollo's special open drive format | tools/opendrive_to_apollo | LGPLv2 |

# Further Adaptions to apollo
- Velodyne 32 layer configuration
- Fortunas VW Passat GTE vehicle parameter configration
- Fortunas static transformations
- SimControl: Start at a parameter-defined location on the map
- Bash scripts to synchronize the (car) PCs, start all necessary components (scripts/fortiss)
- Fortuna vehicle configuration for Dreamview

# Further Documentation
see docs/fortiss
