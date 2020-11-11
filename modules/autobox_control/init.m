
% Initialized Busses
VehicleDataCan;
CustomerCan;
CreateTrajectoryPointBus;
InterfaceCommandsBus;
ControlCommandsBus;
SteeringInterfaceCommands;
CreateLocalizationBus;
CreateTrajectoryBus;
CreateTrackingErrorBus;
CreateControllerParamsBus;
CreateControlCommandBus;
clear Trajectory PathPoint ipaths

% Initialize Parameters
LoadControllerParameters;

% Generate Bus defaults
default_vehicle_state_bus = Simulink.Bus.createMATLABStruct('vehicle_data_can_bus');
default_customer_can_bus = Simulink.Bus.createMATLABStruct('customer_can_bus');
default_localization_bus = Simulink.Bus.createMATLABStruct('LocalizationToAutobox');
default_trajectory_bus = Simulink.Bus.createMATLABStruct('ControlTrajectory');
default_control_commands_bus = Simulink.Bus.createMATLABStruct('ControlCommands');
default_interface_commands_bus = Simulink.Bus.createMATLABStruct('InterfaceCommands');
default_control_command_apollo_bus = Simulink.Bus.createMATLABStruct('ControlCommandToAutobox');

% build sfunctions
mexBuild_chassis
mexBuild_control
mexBuild_localization
mexBuild_trajectory
mexBuild_dSpace_bridge_converter
