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

function [trajectory, trajectory_vector, localization, localization_vector, vehicle_data_can, vehicle_data_can_vector] = getRecordedStateAtIndex(cda, idx)

% Better: only output motion and steering can
% Could also be dynamic...

TRAJ_SIZE = 1000;
trajectory.n_trajectory_point = cda.res.Trajectory.n_trajectory_point(idx);
trajectory.timestamp_sec = cda.res.Trajectory.timestamp_sec(idx);
for i=1:TRAJ_SIZE
    number = ['i',num2str(i-1,'%03d')];
    trajectory.a(i) = cda.res.Trajectory.a.(number)(idx);
    trajectory.ddkappa(i) = cda.res.Trajectory.ddkappa.(number)(idx);
    trajectory.dkappa(i) = cda.res.Trajectory.dkappa.(number)(idx);
    trajectory.kappa(i) = cda.res.Trajectory.kappa.(number)(idx); 
    trajectory.relative_time(i) = cda.res.Trajectory.relative_time.(number)(idx);
    trajectory.s(i) = cda.res.Trajectory.s.(number)(idx);
    trajectory.theta(i) = cda.res.Trajectory.theta.(number)(idx); 
    trajectory.v(i) = cda.res.Trajectory.v.(number)(idx);
    trajectory.x(i) = cda.res.Trajectory.x.(number)(idx);
    trajectory.y(i) = cda.res.Trajectory.y.(number)(idx);
    trajectory.z(i) = 0;%cda.res.Trajectory.z.(number)(idx);
end
trajectory_vector = [double(trajectory.n_trajectory_point), double(trajectory.timestamp_sec), trajectory.a, trajectory.ddkappa, ...
    trajectory.dkappa, trajectory.kappa, trajectory.relative_time, trajectory.s, trajectory.theta, ...
    trajectory.v, trajectory.x, trajectory.y, trajectory.z];

localization.MeasurementTime = cda.res.Localization.MeasurementTime(idx);
localization.AngularVelocity.x = 0;%cda.res.Localization.Pose.AngularVelocity.x(idx);
localization.AngularVelocity.y = 0;%cda.res.Localization.Pose.AngularVelocity.y(idx);
localization.AngularVelocity.z = 0;%cda.res.Localization.Pose.AngularVelocity.z(idx);
localization.AngularVelocityVrf.x = 0;%0*cda.res.Localization.Pose.AngularVelocity.x(idx); %cda.res.Localization.Pose.AngularVelocityVrf.x(idx);
localization.AngularVelocityVrf.y = 0;%0*cda.res.Localization.Pose.AngularVelocity.x(idx); %cda.res.Localization.Pose.AngularVelocityVrf.y(idx);
localization.AngularVelocityVrf.z = 0;%0*cda.res.Localization.Pose.AngularVelocity.x(idx); %cda.res.Localization.Pose.AngularVelocityVrf.z(idx);
localization.EulerAngles.x = cda.res.Localization.Pose.EulerAngles.x(idx);
localization.EulerAngles.y = cda.res.Localization.Pose.EulerAngles.y(idx);
localization.EulerAngles.z = cda.res.Localization.Pose.EulerAngles.z(idx);
localization.Heading = cda.res.Localization.Pose.Heading(idx);
localization.LinearAcceleration.x = cda.res.Localization.Pose.LinearAcceleration.x(idx);
localization.LinearAcceleration.y = cda.res.Localization.Pose.LinearAcceleration.y(idx);
localization.LinearAcceleration.z = 0;%cda.res.Localization.Pose.LinearAcceleration.z(idx);
localization.LinearAccelerationVrf.x = 0;%0*cda.res.Localization.Pose.LinearAcceleration.x(idx); %cda.res.Localization.Pose.LinearAccelerationVrf.x(idx);
localization.LinearAccelerationVrf.y = 0;%0*cda.res.Localization.Pose.LinearAcceleration.y(idx); %cda.res.Localization.Pose.LinearAccelerationVrf.y(idx);
localization.LinearAccelerationVrf.z = 0;%0*cda.res.Localization.Pose.LinearAcceleration.z(idx); %cda.res.Localization.Pose.LinearAccelerationVrf.z(idx);
localization.LinearVelocity.x = cda.res.Localization.Pose.LinearVelocity.x(idx);
localization.LinearVelocity.y = cda.res.Localization.Pose.LinearVelocity.y(idx);
localization.LinearVelocity.z = 0;%cda.res.Localization.Pose.LinearVelocity.z(idx);
localization.Orientation.qw = 0;%cda.res.Localization.Pose.Orientation.qw(idx);
localization.Orientation.qx = 0;%cda.res.Localization.Pose.Orientation.qx(idx);
localization.Orientation.qy = 0;%cda.res.Localization.Pose.Orientation.qy(idx);
localization.Orientation.qz = 0;%cda.res.Localization.Pose.Orientation.qz(idx);
localization.Position.x = cda.res.Localization.Pose.Position.x(idx);
localization.Position.y = cda.res.Localization.Pose.Position.y(idx);
localization.Position.z = 0;%cda.res.Localization.Pose.Position.z(idx);
localization_vector = [localization.MeasurementTime, localization.AngularVelocity.z, localization.AngularVelocity.y, localization.AngularVelocity.z, ...
    localization.AngularVelocityVrf.x, localization.AngularVelocityVrf.y, localization.AngularVelocityVrf.z,...
    localization.EulerAngles.x, localization.EulerAngles.y, localization.EulerAngles.z, localization.Heading, ...
    localization.LinearAcceleration.x, localization.LinearAcceleration.y, localization.LinearAcceleration.z, ...
    localization.LinearAccelerationVrf.x, localization.LinearAccelerationVrf.y, localization.LinearAccelerationVrf.z, ...
    localization.LinearVelocity.x, localization.LinearVelocity.y, localization.LinearVelocity.z, ...
    localization.Orientation.qw, localization.Orientation.qx, localization.Orientation.qy, localization.Orientation.qz, ...
    localization.Position.x, localization.Position.y, localization.Position.z];


% TODO has to be uncommented again!!
%vehicle_data_can = Simulink.Bus.createMATLABStruct('vehicle_data_can_bus');
vehicle_data_can.Motion.CRC = 0;%0* cda.res.VehicleDataCan.Motion.LateralAcceleration(idx); %cda.res.VehicleDataCan.Motion.CRC(idx);
vehicle_data_can.Motion.Counter = 0;%0*cda.res.VehicleDataCan.Motion.LateralAcceleration(idx); %cda.res.VehicleDataCan.Motion.Counter(idx);
vehicle_data_can.Motion.LateralAcceleration = 0;%cda.res.VehicleDataCan.Motion.LateralAcceleration(idx);
vehicle_data_can.Motion.LongitudinalAcceleration = 0;%cda.res.VehicleDataCan.Motion.LongitudinalAcceleration(idx);
vehicle_data_can.Motion.VehicleVelocity = 0;%cda.res.VehicleDataCan.Motion.VehicleVelocity(idx);
vehicle_data_can.Motion.YawRate = 0;%cda.res.VehicleDataCan.Motion.YawRate(idx);
vehicle_data_can.Motion.YawRateSign = 0;%cda.res.VehicleDataCan.Motion.YawRateSign(idx);
vehicle_data_can.Steering.CRC = 0;%0*cda.res.VehicleDataCan.Motion.LateralAcceleration(idx); %cda.res.VehicleDataCan.Steering.CRC(idx);
vehicle_data_can.Steering.Counter = 0;%0*cda.res.VehicleDataCan.Motion.LateralAcceleration(idx); %cda.res.VehicleDataCan.Steering.Counter(idx);
vehicle_data_can.Steering.SteeringWheelAngle = 0;%cda.res.VehicleDataCan.Steering.SteeringWheelAngle(idx);
vehicle_data_can.Steering.SteeringWheelAngleSign = 0;%cda.res.VehicleDataCan.Steering.SteeringWheelAngleSign(idx);
vehicle_data_can.Steering.SteeringWheelSpeed = 0;%cda.res.VehicleDataCan.Steering.SteeringWheelSpeed(idx);
vehicle_data_can.Steering.SteeringWheelSpeedSign = 0;%cda.res.VehicleDataCan.Steering.SteeringWheelSpeedSign(idx);
vehicle_data_can.Steering.SteeringWheelTorque = 0;%cda.res.VehicleDataCan.Steering.SteeringWheelTorque(idx);
vehicle_data_can.Steering.SteeringWheelTorqueSign = 0;%cda.res.VehicleDataCan.Steering.SteeringWheelTorqueSign(idx);
vehicle_data_can_vector = [double(vehicle_data_can.Motion.CRC), double(vehicle_data_can.Motion.Counter), vehicle_data_can.Motion.LateralAcceleration, ...
    vehicle_data_can.Motion.LongitudinalAcceleration, vehicle_data_can.Motion.VehicleVelocity, vehicle_data_can.Motion.YawRate, double(vehicle_data_can.Motion.YawRateSign), ...
    double(vehicle_data_can.Steering.CRC), double(vehicle_data_can.Steering.Counter), vehicle_data_can.Steering.SteeringWheelAngle, double(vehicle_data_can.Steering.SteeringWheelAngleSign), ...
    double(vehicle_data_can.Steering.SteeringWheelSpeed), double(vehicle_data_can.Steering.SteeringWheelSpeedSign), ...
    vehicle_data_can.Steering.SteeringWheelTorque, double(vehicle_data_can.Steering.SteeringWheelTorqueSign)];



end