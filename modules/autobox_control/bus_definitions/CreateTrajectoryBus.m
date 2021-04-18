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

function CreateTrajectoryBus() 
% CREATETRAJECTORYBUS initializes a set of bus objects in the MATLAB base workspace 

% Bus object: ControlTrajectory 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'n_trajectory_point';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint32';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'x';
elems(2).Dimensions = 100;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = sprintf('m');
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'y';
elems(3).Dimensions = 100;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = sprintf('m');
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'z';
elems(4).Dimensions = 100;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = sprintf('m');
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'theta';
elems(5).Dimensions = 100;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'kappa';
elems(6).Dimensions = 100;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = sprintf('curvature on the x-y planning');

elems(7) = Simulink.BusElement;
elems(7).Name = 'dkappa';
elems(7).Dimensions = 100;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = sprintf('derivative of kappa');

elems(8) = Simulink.BusElement;
elems(8).Name = 'ddkappa';
elems(8).Dimensions = 100;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = sprintf('derivative of derivative of kappa');

elems(9) = Simulink.BusElement;
elems(9).Name = 's';
elems(9).Dimensions = 100;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = sprintf('accumulated distance from beginning of the path');

elems(10) = Simulink.BusElement;
elems(10).Name = 'v';
elems(10).Dimensions = 100;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = sprintf('m/s');
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'a';
elems(11).Dimensions = 100;
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = sprintf('m/s/s');
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'relative_time';
elems(12).Dimensions = 100;
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'double';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = sprintf('s');
elems(12).Description = sprintf('relative time from beginning of the trajectory');

elems(13) = Simulink.BusElement;
elems(13).Name = 'timestamp_sec';
elems(13).Dimensions = 1;
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'double';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = sprintf('s');
elems(13).Description = sprintf('Message publishing time in seconds.');

ControlTrajectory = Simulink.Bus;
ControlTrajectory.HeaderFile = '';
ControlTrajectory.Description = '';
ControlTrajectory.DataScope = 'Auto';
ControlTrajectory.Alignment = -1;
ControlTrajectory.Elements = elems;
clear elems;
assignin('base','ControlTrajectory', ControlTrajectory);

