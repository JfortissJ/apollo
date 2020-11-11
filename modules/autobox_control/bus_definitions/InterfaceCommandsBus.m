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

function InterfaceCommandsBus() 
% INTERFACECOMMANDSBUS initializes a set of bus objects in the MATLAB base workspace 

% Bus object: InterfaceCommands 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'AccelerationInterfaceCommands';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'Bus: AccelerationInterfaceCommands';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'SteeringInterfaceCommands';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'SteeringInterfaceCommands';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

InterfaceCommands = Simulink.Bus;
InterfaceCommands.HeaderFile = '';
InterfaceCommands.Description = '';
InterfaceCommands.DataScope = 'Auto';
InterfaceCommands.Alignment = -1;
InterfaceCommands.Elements = elems;
clear elems;
assignin('base','InterfaceCommands', InterfaceCommands);

% Bus object: AccelerationInterfaceCommands 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Cancel';
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
elems(2).Name = 'Activation';
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
elems(3).Name = 'Clearance';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'boolean';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

AccelerationInterfaceCommands = Simulink.Bus;
AccelerationInterfaceCommands.HeaderFile = '';
AccelerationInterfaceCommands.Description = '';
AccelerationInterfaceCommands.DataScope = 'Auto';
AccelerationInterfaceCommands.Alignment = -1;
AccelerationInterfaceCommands.Elements = elems;
clear elems;
assignin('base','AccelerationInterfaceCommands', AccelerationInterfaceCommands);

% Bus object: SteeringInterfaceCommands 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Cancel';
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
elems(2).Name = 'Activation';
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
elems(3).Name = 'Clearance';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'uint8';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

SteeringInterfaceCommands = Simulink.Bus;
SteeringInterfaceCommands.HeaderFile = '';
SteeringInterfaceCommands.Description = '';
SteeringInterfaceCommands.DataScope = 'Auto';
SteeringInterfaceCommands.Alignment = -1;
SteeringInterfaceCommands.Elements = elems;
clear elems;
assignin('base','SteeringInterfaceCommands', SteeringInterfaceCommands);

