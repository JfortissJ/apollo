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

%function [vehicle_data_can_bus, customer_can_bus, default_vehicle_state_bus, default_customer_can_bus] = dbc2busobject()
function [default_vehicle_state_bus, default_customer_can_bus] = dbc2busobject()
% BUG!!! Data types in the matlab canDatabase function are always int!


dbv = canDatabase('dbc\VehicleData_CAN_TFD7-17008-03_fortiss_GTE.dbc');
dbc = canDatabase('dbc\Customer_CAN_TFD7-17008-03_fortiss_GTE.dbc');

default_vehicle_state_bus = dbc2can_internal(dbv);
default_customer_can_bus = dbc2can_internal(dbc);

default_vehicle_state_bus.Motion.CRC = uint8(0);
default_vehicle_state_bus.Motion.Counter = uint8(0);
default_vehicle_state_bus.Motion.YawRateSign = boolean(0);
default_vehicle_state_bus.Motion.YawRate = double(0);
default_vehicle_state_bus.Motion.LongitudinalAcceleration = double(0);
default_vehicle_state_bus.Motion.LateralAcceleration = double(0);
default_vehicle_state_bus.Motion.VehicleVelocity = double(0);

default_vehicle_state_bus.Steering.CRC = uint8(0);
default_vehicle_state_bus.Steering.Counter = uint8(0);
default_vehicle_state_bus.Steering.SteeringWheelAngleSign = boolean(0);
default_vehicle_state_bus.Steering.SteeringWheelSpeedSign = boolean(0);
default_vehicle_state_bus.Steering.SteeringWheelTorqueSign = boolean(0);
default_vehicle_state_bus.Steering.SteeringWheelAngle = double(0);
default_vehicle_state_bus.Steering.SteeringWheelSpeed = uint16(0);
default_vehicle_state_bus.Steering.SteeringWheelTorque = double(0);


default_customer_can_bus.Gateway_States.AIState = uint8(0);
default_customer_can_bus.Gateway_States.AngleDeviationLimitationSI = boolean(0);
default_customer_can_bus.Gateway_States.AngleGradientLimitationSI = boolean(0);
default_customer_can_bus.Gateway_States.AngleLimitationSI = boolean(0);
default_customer_can_bus.Gateway_States.AngleLimitationStatus = boolean(0);
default_customer_can_bus.Gateway_States.CRC = uint8(0);
default_customer_can_bus.Gateway_States.Counter = uint8(0);
default_customer_can_bus.Gateway_States.EBIState = uint8(0);
default_customer_can_bus.Gateway_States.GWState = uint8(0);
default_customer_can_bus.Gateway_States.GIState = uint8(0);
default_customer_can_bus.Gateway_States.GatewayClearanceAI = boolean(0);
default_customer_can_bus.Gateway_States.GatewayClearanceEBI = boolean(0);
default_customer_can_bus.Gateway_States.GatewayClearanceSI = boolean(0);
default_customer_can_bus.Gateway_States.LimitationsReceivedSI = uint8(0);
default_customer_can_bus.Gateway_States.MsgTimeoutDisplay = boolean(0);
default_customer_can_bus.Gateway_States.MsgTimeoutPwrMgnt = boolean(0);
default_customer_can_bus.Gateway_States.PDIState = uint8(0);
default_customer_can_bus.Gateway_States.SIState = uint8(0);
default_customer_can_bus.Gateway_States.THIState = uint8(0);
default_customer_can_bus.Gateway_States.VIState = uint8(0);

default_customer_can_bus.Vehicle_Data.DriverBraking = boolean(0);
default_customer_can_bus.Vehicle_Data.GearLeverPos = uint8(0);
default_customer_can_bus.Vehicle_Data.SteeringAngle = double(0);
default_customer_can_bus.Vehicle_Data.SteeringAngleSign = boolean(0);
default_customer_can_bus.Vehicle_Data.ThrottleSetpoint = double(0);
default_customer_can_bus.Vehicle_Data.VehicleVelocity = double(0);

default_customer_can_bus.Steering_Interface.ActivationRequest = boolean(0);
default_customer_can_bus.Steering_Interface.AdvancedSteeringAngleLimitation = boolean(0);
default_customer_can_bus.Steering_Interface.Amplitude = double(0);
default_customer_can_bus.Steering_Interface.CRC = uint8(0);
default_customer_can_bus.Steering_Interface.CancelRequest = boolean(0);
default_customer_can_bus.Steering_Interface.ClearanceSI = uint8(0);
default_customer_can_bus.Steering_Interface.Counter = uint8(0);
default_customer_can_bus.Steering_Interface.Frequency = uint8(0);
default_customer_can_bus.Steering_Interface.SteeringAngleRequest = double(0);
default_customer_can_bus.Steering_Interface.SteeringAngleRequestSign = boolean(0);
default_customer_can_bus.Steering_Interface.SteeringTorqueRequest = double(0);
default_customer_can_bus.Steering_Interface.SteeringTorqueRequestSign = boolean(0);

default_customer_can_bus.Acceleration_Interface.AccelerationRequest = double(0);
default_customer_can_bus.Acceleration_Interface.AcousticDriverHint = uint8(0);
default_customer_can_bus.Acceleration_Interface.ActivationRequest = boolean(0);
default_customer_can_bus.Acceleration_Interface.CRC = uint8(0);
default_customer_can_bus.Acceleration_Interface.CancelRequest = boolean(0);
default_customer_can_bus.Acceleration_Interface.ClearanceAI = boolean(0);
default_customer_can_bus.Acceleration_Interface.ClearanceStopDistance = boolean(0);
default_customer_can_bus.Acceleration_Interface.Counter = uint8(0);
default_customer_can_bus.Acceleration_Interface.OpticalDriverHint = uint8(0);
default_customer_can_bus.Acceleration_Interface.StopDistanceRequest = double(0);

%busInfov = Simulink.Bus.createObject(default_vehicle_state_bus);
%busInfoc = Simulink.Bus.createObject(default_customer_can_bus);

%vehicle_data_can_bus = evalin('base',busInfov.busName);
%customer_can_bus = evalin('base',busInfoc.busName);

end



function s = dbc2can_internal(db)


s = struct();

messageinfo = db.messageInfo;

for i=1:length(db.messageInfo)
   
    message = messageinfo(i).Name;
    
    for j=1:length(messageinfo(i).SignalInfo)
       
        signal = messageinfo(i).SignalInfo(j).Name;
        type = messageinfo(i).SignalInfo(j).Class;

        s.(message).(signal) = int8(0);
        
    end
    
end

end