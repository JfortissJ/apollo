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

function CreateLocalizationBus() 
% CREATELOCALIZATIONBUS initializes a set of bus objects in the MATLAB base workspace 

% Bus object: LocalizationToAutobox 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Pose';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'Bus: Pose';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'MeasurementTime';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Uncertainty';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'Bus: Uncertainty';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

LocalizationToAutobox = Simulink.Bus;
LocalizationToAutobox.HeaderFile = '';
LocalizationToAutobox.Description = '';
LocalizationToAutobox.DataScope = 'Auto';
LocalizationToAutobox.Alignment = -1;
LocalizationToAutobox.Elements = elems;
clear elems;
assignin('base','LocalizationToAutobox', LocalizationToAutobox);

% Bus object: Point3D 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'y';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'z';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

Point3D = Simulink.Bus;
Point3D.HeaderFile = '';
Point3D.Description = '';
Point3D.DataScope = 'Auto';
Point3D.Alignment = -1;
Point3D.Elements = elems;
clear elems;
assignin('base','Point3D', Point3D);

% Bus object: PointENU 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'y';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'z';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

PointENU = Simulink.Bus;
PointENU.HeaderFile = '';
PointENU.Description = '';
PointENU.DataScope = 'Auto';
PointENU.Alignment = -1;
PointENU.Elements = elems;
clear elems;
assignin('base','PointENU', PointENU);

% Bus object: Pose 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Position';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'Bus: PointENU';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = sprintf('Position of the vehicle reference point (VRP) in the map reference frame.\nThe VRP is the center of rear axle.');

elems(2) = Simulink.BusElement;
elems(2).Name = 'Orientation';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'Bus: Quaternion';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = sprintf('A quaternion that represents the rotation from the IMU coordinate\n(Right/Forward/Up) to the world coordinate (East/North/Up).');

elems(3) = Simulink.BusElement;
elems(3).Name = 'LinearVelocity';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'Bus: Point3D';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = sprintf('Linear velocity of the VRP in the map reference frame.\nEast/north/up in meters per second.');

elems(4) = Simulink.BusElement;
elems(4).Name = 'LinearAcceleration';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'Bus: Point3D';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = sprintf('Linear acceleration of the VRP in the map reference frame.\nEast/north/up in meters per second.');

elems(5) = Simulink.BusElement;
elems(5).Name = 'AngularVelocity';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'Bus: Point3D';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = sprintf('Angular velocity of the vehicle in the map reference frame.\nAround east/north/up axes in radians per second.');

elems(6) = Simulink.BusElement;
elems(6).Name = 'Heading';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = sprintf('The heading is zero when the car is facing East and positive when facing North.');

elems(7) = Simulink.BusElement;
elems(7).Name = 'LinearAccelerationVrf';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'Bus: Point3D';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = sprintf('Linear acceleration of the VRP in the vehicle reference frame.\nRight/forward/up in meters per square second.');

elems(8) = Simulink.BusElement;
elems(8).Name = 'AngularVelocityVrf';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'Bus: Point3D';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = sprintf('Angular velocity of the VRP in the vehicle reference frame.\nAround right/forward/up axes in radians per second.');

elems(9) = Simulink.BusElement;
elems(9).Name = 'EulerAngles';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'Bus: Point3D';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = sprintf('Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y.\nin world coordinate (East/North/Up)\nThe roll, in (-pi/2, pi/2), corresponds to a rotation around the y-axis.\nThe pitch, in [-pi, pi), corresponds to a rotation around the x-axis.\nThe yaw, in [-pi, pi), corresponds to a rotation around the z-axis.\nThe direction of rotation follows the right-hand rule.');

Pose = Simulink.Bus;
Pose.HeaderFile = '';
Pose.Description = '';
Pose.DataScope = 'Auto';
Pose.Alignment = -1;
Pose.Elements = elems;
clear elems;
assignin('base','Pose', Pose);

% Bus object: Quaternion 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'qx';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'qy';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'qz';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'qw';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

Quaternion = Simulink.Bus;
Quaternion.HeaderFile = '';
Quaternion.Description = '';
Quaternion.DataScope = 'Auto';
Quaternion.Alignment = -1;
Quaternion.Elements = elems;
clear elems;
assignin('base','Quaternion', Quaternion);

% Bus object: Uncertainty 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'PositionStdDev';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'Bus: Point3D';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = sprintf('Standard deviation of position, east/north/up in meters.');

elems(2) = Simulink.BusElement;
elems(2).Name = 'OrientationStdDev';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'Bus: Point3D';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = sprintf('Standard deviation of quaternion qx/qy/qz, unitless.');

elems(3) = Simulink.BusElement;
elems(3).Name = 'LinearVelocityStdDev';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'Bus: Point3D';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = sprintf('Standard deviation of linear velocity, east/north/up in meters per second.');

elems(4) = Simulink.BusElement;
elems(4).Name = 'LinearAccelerationStdDev';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'Bus: Point3D';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = sprintf('Standard deviation of linear acceleration, right/forward/up in meters per square second.');

elems(5) = Simulink.BusElement;
elems(5).Name = 'AngularVelocityStdDev';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'Bus: Point3D';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = sprintf('Standard deviation of angular velocity, right/forward/up in radians per second.');

Uncertainty = Simulink.Bus;
Uncertainty.HeaderFile = '';
Uncertainty.Description = '';
Uncertainty.DataScope = 'Auto';
Uncertainty.Alignment = -1;
Uncertainty.Elements = elems;
clear elems;
assignin('base','Uncertainty', Uncertainty);

