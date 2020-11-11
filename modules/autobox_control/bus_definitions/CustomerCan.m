%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Apollo Autobox Control
% Copyright (C) 2020 fortiss GmbH
% Authors: Tobias Kessler, Jianjie Lin, Julian Bernhard, Klemens Esterle,
% Patrick Hart
%
% This library is free software; you can redistribute it and/or modify it
% under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation; either version 3 of the License, or (at 
% your option) any later version.
%
% This library is distributed in the hope that it will be useful, but 
% WITHOUT ANY WARRANTY; without even the implied warranty of 
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser 
% General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this library; if not, write to the Free Software Foundation,
% Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function CustomerCan() 
% CUSTOMERCAN initializes a set of bus objects in the MATLAB base workspace 

% Bus object: customer_can_bus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Acceleration_Interface';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'Acceleration_Interface';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'EmergencyBrake_Interface';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'EmergencyBrake_Interface';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Gateway_Config';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'Gateway_Config';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Gateway_States';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'Gateway_States';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Gear_Interface';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'Gear_Interface';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'ParkDecel_Interface_1';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'ParkDecel_Interface_1';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'ParkDecel_Interface_2';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'ParkDecel_Interface_2';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'Steering_Interface';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'Steering_Interface';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'Throttle_Interface';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'Throttle_Interface';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'VIN_Data';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'VIN_Data';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'Vehicle_Data';
elems(11).Dimensions = 1;
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'Vehicle_Data';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'Vehicle_Interface';
elems(12).Dimensions = 1;
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'Vehicle_Interface';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

customer_can_bus = Simulink.Bus;
customer_can_bus.HeaderFile = '';
customer_can_bus.Description = '';
customer_can_bus.DataScope = 'Auto';
customer_can_bus.Alignment = -1;
customer_can_bus.Elements = elems;
clear elems;
assignin('base','customer_can_bus', customer_can_bus);

% Bus object: Acceleration_Interface 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'AccelerationRequest';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'AcousticDriverHint';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'uint8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'ActivationRequest';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'boolean';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'CRC';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'CancelRequest';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'boolean';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'ClearanceAI';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'boolean';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'ClearanceStopDistance';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'boolean';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'Counter';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'uint8';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'OpticalDriverHint';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'boolean';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'StopDistanceRequest';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

Acceleration_Interface = Simulink.Bus;
Acceleration_Interface.HeaderFile = '';
Acceleration_Interface.Description = '';
Acceleration_Interface.DataScope = 'Auto';
Acceleration_Interface.Alignment = -1;
Acceleration_Interface.Elements = elems;
clear elems;
assignin('base','Acceleration_Interface', Acceleration_Interface);

% Bus object: EmergencyBrake_Interface 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'ActivationRequest';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'CRC';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'CancelRequest';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'ClearanceEBI';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Counter';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'DecelerationRequest';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

EmergencyBrake_Interface = Simulink.Bus;
EmergencyBrake_Interface.HeaderFile = '';
EmergencyBrake_Interface.Description = '';
EmergencyBrake_Interface.DataScope = 'Auto';
EmergencyBrake_Interface.Alignment = -1;
EmergencyBrake_Interface.Elements = elems;
clear elems;
assignin('base','EmergencyBrake_Interface', EmergencyBrake_Interface);

% Bus object: Gateway_Config 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Coding_AI';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Coding_EBI';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Coding_GI';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Coding_PDI';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Coding_SI1';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'Coding_SI2';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'Coding_THI';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'int8';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'Coding_VI';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'int8';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'SoftwareVersionMajor';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'int8';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'SoftwareVersionMinor';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'int8';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'SoftwareVersionRevision';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'int8';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

Gateway_Config = Simulink.Bus;
Gateway_Config.HeaderFile = '';
Gateway_Config.Description = '';
Gateway_Config.DataScope = 'Auto';
Gateway_Config.Alignment = -1;
Gateway_Config.Elements = elems;
clear elems;
assignin('base','Gateway_Config', Gateway_Config);

% Bus object: Gateway_States 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'AIState';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'AngleDeviationLimitationSI';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'boolean';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'AngleGradientLimitationSI';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'boolean';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'AngleLimitationSI';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'boolean';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'AngleLimitationStatus';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'boolean';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'CRC';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'uint8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'Counter';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'uint8';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'EBIState';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'uint8';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'GIState';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'uint8';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'GWState';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'uint8';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'GatewayClearanceAI';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'boolean';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'GatewayClearanceEBI';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'boolean';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'GatewayClearanceSI';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'boolean';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'LimitationsReceivedSI';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'uint8';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'MsgTimeoutDisplay';
elems(15).Dimensions = [1 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'boolean';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

elems(16) = Simulink.BusElement;
elems(16).Name = 'MsgTimeoutPwrMgnt';
elems(16).Dimensions = [1 1];
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'boolean';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = '';
elems(16).Description = '';

elems(17) = Simulink.BusElement;
elems(17).Name = 'PDIState';
elems(17).Dimensions = [1 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'uint8';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = '';
elems(17).Description = '';

elems(18) = Simulink.BusElement;
elems(18).Name = 'SIState';
elems(18).Dimensions = [1 1];
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'uint8';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = '';
elems(18).Description = '';

elems(19) = Simulink.BusElement;
elems(19).Name = 'THIState';
elems(19).Dimensions = [1 1];
elems(19).DimensionsMode = 'Fixed';
elems(19).DataType = 'uint8';
elems(19).SampleTime = -1;
elems(19).Complexity = 'real';
elems(19).Min = [];
elems(19).Max = [];
elems(19).DocUnits = '';
elems(19).Description = '';

elems(20) = Simulink.BusElement;
elems(20).Name = 'VIState';
elems(20).Dimensions = [1 1];
elems(20).DimensionsMode = 'Fixed';
elems(20).DataType = 'uint8';
elems(20).SampleTime = -1;
elems(20).Complexity = 'real';
elems(20).Min = [];
elems(20).Max = [];
elems(20).DocUnits = '';
elems(20).Description = '';

Gateway_States = Simulink.Bus;
Gateway_States.HeaderFile = '';
Gateway_States.Description = '';
Gateway_States.DataScope = 'Auto';
Gateway_States.Alignment = -1;
Gateway_States.Elements = elems;
clear elems;
assignin('base','Gateway_States', Gateway_States);

% Bus object: Gear_Interface 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'ActivationRequest';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'CRC';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'CancelRequest';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'ClearanceGI';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Counter';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'GearSelection';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

Gear_Interface = Simulink.Bus;
Gear_Interface.HeaderFile = '';
Gear_Interface.Description = '';
Gear_Interface.DataScope = 'Auto';
Gear_Interface.Alignment = -1;
Gear_Interface.Elements = elems;
clear elems;
assignin('base','Gear_Interface', Gear_Interface);

% Bus object: ParkDecel_Interface_1 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'ActivationRequest';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'CRC';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'CancelRequest';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'ClearancePDI';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Counter';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'TorqueRequest';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

ParkDecel_Interface_1 = Simulink.Bus;
ParkDecel_Interface_1.HeaderFile = '';
ParkDecel_Interface_1.Description = '';
ParkDecel_Interface_1.DataScope = 'Auto';
ParkDecel_Interface_1.Alignment = -1;
ParkDecel_Interface_1.Elements = elems;
clear elems;
assignin('base','ParkDecel_Interface_1', ParkDecel_Interface_1);

% Bus object: ParkDecel_Interface_2 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'ActivationRequest';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'CRC';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'CancelRequest';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'ClearancePDI';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Counter';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'DecelerationRequest';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

ParkDecel_Interface_2 = Simulink.Bus;
ParkDecel_Interface_2.HeaderFile = '';
ParkDecel_Interface_2.Description = '';
ParkDecel_Interface_2.DataScope = 'Auto';
ParkDecel_Interface_2.Alignment = -1;
ParkDecel_Interface_2.Elements = elems;
clear elems;
assignin('base','ParkDecel_Interface_2', ParkDecel_Interface_2);

% Bus object: Steering_Interface 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'ActivationRequest';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'boolean';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'AdvancedSteeringAngleLimitation';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'boolean';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Amplitude';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'CRC';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'CancelRequest';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'boolean';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'ClearanceSI';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'uint8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'Counter';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'uint8';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'Frequency';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'uint8';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'SteeringAngleRequest';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'SteeringAngleRequestSign';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'boolean';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'SteeringTorqueRequest';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'SteeringTorqueRequestSign';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'boolean';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

Steering_Interface = Simulink.Bus;
Steering_Interface.HeaderFile = '';
Steering_Interface.Description = '';
Steering_Interface.DataScope = 'Auto';
Steering_Interface.Alignment = -1;
Steering_Interface.Elements = elems;
clear elems;
assignin('base','Steering_Interface', Steering_Interface);

% Bus object: Throttle_Interface 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'ActivationRequest';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'CRC';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'CancelRequest';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'ClearanceTHI';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Counter';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'ThrottleRequest';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

Throttle_Interface = Simulink.Bus;
Throttle_Interface.HeaderFile = '';
Throttle_Interface.Description = '';
Throttle_Interface.DataScope = 'Auto';
Throttle_Interface.Alignment = -1;
Throttle_Interface.Elements = elems;
clear elems;
assignin('base','Throttle_Interface', Throttle_Interface);

% Bus object: VIN_Data 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'VIN_01_MUX';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'VIN_1';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'VIN_10';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'VIN_11';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'VIN_12';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'VIN_13';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'VIN_14';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'int8';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'VIN_15';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'int8';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'VIN_16';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'int8';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'VIN_17';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'int8';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'VIN_2';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'int8';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'VIN_3';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'int8';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'VIN_4';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'int8';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'VIN_5';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'int8';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'VIN_6';
elems(15).Dimensions = [1 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'int8';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

elems(16) = Simulink.BusElement;
elems(16).Name = 'VIN_7';
elems(16).Dimensions = [1 1];
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'int8';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = '';
elems(16).Description = '';

elems(17) = Simulink.BusElement;
elems(17).Name = 'VIN_8';
elems(17).Dimensions = [1 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'int8';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = '';
elems(17).Description = '';

elems(18) = Simulink.BusElement;
elems(18).Name = 'VIN_9';
elems(18).Dimensions = [1 1];
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'int8';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = '';
elems(18).Description = '';

VIN_Data = Simulink.Bus;
VIN_Data.HeaderFile = '';
VIN_Data.Description = '';
VIN_Data.DataScope = 'Auto';
VIN_Data.Alignment = -1;
VIN_Data.Elements = elems;
clear elems;
assignin('base','VIN_Data', VIN_Data);

% Bus object: Vehicle_Data 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'DriverBraking';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'boolean';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'GearLeverPos';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'uint8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'SteeringAngle';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'SteeringAngleSign';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'boolean';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'ThrottleSetpoint';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'VehicleVelocity';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

Vehicle_Data = Simulink.Bus;
Vehicle_Data.HeaderFile = '';
Vehicle_Data.Description = '';
Vehicle_Data.DataScope = 'Auto';
Vehicle_Data.Alignment = -1;
Vehicle_Data.Elements = elems;
clear elems;
assignin('base','Vehicle_Data', Vehicle_Data);

% Bus object: Vehicle_Interface 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'ACCButton';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'int8';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'ActivationRequest';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int8';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'CRC';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'CancelRequest';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int8';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'ClearanceVI';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int8';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'Counter';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int8';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'FPKSwitch';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'int8';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'HMISwitch';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'int8';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'HUDSwitch';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'int8';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'MFSButtonFirstPressed';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'int8';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'MFSButtonSecondPressed';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'int8';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'MFSTipDown';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'int8';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'MFSTipUp';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'int8';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'TransmissionVehicleDataMessage';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'int8';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'TurnIndicatorRequest';
elems(15).Dimensions = [1 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'int8';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

Vehicle_Interface = Simulink.Bus;
Vehicle_Interface.HeaderFile = '';
Vehicle_Interface.Description = '';
Vehicle_Interface.DataScope = 'Auto';
Vehicle_Interface.Alignment = -1;
Vehicle_Interface.Elements = elems;
clear elems;
assignin('base','Vehicle_Interface', Vehicle_Interface);

