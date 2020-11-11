/******************************************************************************
 * Apollo Autobox Control
 * Copyright (C) 2020 fortiss GmbH
 * Authors: Tobias Kessler, Jianjie Lin
 * 
 * This library is free software; you can redistribute it and/or modify it 
 * under the terms of the GNU Lesser General Public License as published by the 
 * Free Software Foundation; either version 2.1 of the License, or (at your 
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License 
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License 
 * along with this library; if not, write to the Free Software Foundation, 
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *****************************************************************************/

#define APOLLO_CNABUS_CHASIS_HEADER_SEQUA_NUM 0
#define APOLLO_CNABUS_CHASIS_HEADER_TIME_SEQ 1
#define APOLLO_CNABUS_CHASIS_ENGINE_STARTED 2
#define APOLLO_CNABUS_CHASIS_ENGINE_RPM 3
#define APOLLO_CNABUS_CHASIS_ODOMETER_M 4
#define APOLLO_CNABUS_CHASIS_FUEL_RANGE_M 5
#define APOLLO_CNABUS_CHASIS_THROTTLE_PERCENTAGE 6
#define APOLLO_CNABUS_CHASIS_BRAKE_PERCENTAGE 7
#define APOLLO_CNABUS_CHASIS_STEERING_PERCENTAGE 8
#define APOLLO_CNABUS_CHASIS_STEERING_TORQUE_NM 9
#define APOLLO_CNABUS_CHASIS_PARKING_BRAKE 10

#include "common/macro.h"

void initialize_outputsPort_CanBus_Chassis(SimStruct* S)
{
	if(!ssSetNumOutputPorts(S, 11))
	{
		return;
	}
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_HEADER_SEQUA_NUM, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_HEADER_TIME_SEQ, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_ENGINE_STARTED, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_ENGINE_RPM, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_ODOMETER_M, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_FUEL_RANGE_M, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_THROTTLE_PERCENTAGE, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_BRAKE_PERCENTAGE, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_STEERING_PERCENTAGE, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_STEERING_TORQUE_NM, 1);
	ssSetOutputPortWidth(S, APOLLO_CNABUS_CHASIS_PARKING_BRAKE, 1);
	
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_HEADER_SEQUA_NUM, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_HEADER_TIME_SEQ, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_ENGINE_STARTED, SS_BOOLEAN);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_ENGINE_RPM, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_ODOMETER_M, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_FUEL_RANGE_M, SS_INT32);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_THROTTLE_PERCENTAGE, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_BRAKE_PERCENTAGE, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_STEERING_PERCENTAGE, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_STEERING_TORQUE_NM, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CNABUS_CHASIS_PARKING_BRAKE, SS_BOOLEAN);
}

void fill_outputsPort_CanBus_Chassis(SimStruct* S, Apollo__Canbus__ChassisToAutoboxBridge* apollo_msg)
{
	double* port_header_Sequa_num = (double*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_HEADER_SEQUA_NUM);
	double* port_header_time_seq = (double*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_HEADER_TIME_SEQ);
	bool* port_chassis_engine_started = (bool*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_ENGINE_STARTED);
	double* port_chassis_engin_rpm = (double*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_ENGINE_RPM);
	double* port_chassis_odometer = (double*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_ODOMETER_M);
	int32_t* port_chassis_fuel_range_m = (int32_t*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_FUEL_RANGE_M);
	double* port_chassis_throttle_percentage = (double*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_THROTTLE_PERCENTAGE);
	double* port_chassis_brake_percentage = (double*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_BRAKE_PERCENTAGE);
	double* port_chassis_steering_percentage = (double*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_STEERING_PERCENTAGE);
	double* port_chassis_steering_torque_nm = (double*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_STEERING_TORQUE_NM);
	bool* port_chassis_parking_brake = (bool*) ssGetOutputPortSignal(S, APOLLO_CNABUS_CHASIS_PARKING_BRAKE);
	FORTISS_INFO_PRINTF("has sequemce num: %i \n", apollo_msg->header->has_sequence_num);
	if(apollo_msg->header->has_sequence_num)
	{
		*port_header_Sequa_num = apollo_msg->header->sequence_num;
	}
	if(apollo_msg->header->has_timestamp_sec)
	{
		double time;
		DOUBLE_CAST(time, &apollo_msg->header->timestamp_sec);
		*port_header_time_seq = time;
		//SWAP64(port_header_time_seq,apollo_msg->header->timestamp_sec);
		
	}
	if(apollo_msg->has_engine_started)
	{
		
		//SWAP64(port_chassis_engine_started,apollo_msg->engine_started);
		*port_chassis_engine_started = apollo_msg->engine_started;
		
		
	}
	if(apollo_msg->has_engine_rpm)
	{
		double engine_rpm;
		DOUBLE_CAST(engine_rpm, &apollo_msg->engine_rpm);
		*port_chassis_engin_rpm = engine_rpm;
		//SWAP64(port_chassis_engin_rpm,apollo_msg->engine_rpm);
	}
	
	if(apollo_msg->has_odometer_m)
	{
		//SWAP64(port_chassis_odometer,apollo_msg->odometer_m);
		double odometer_m;
		DOUBLE_CAST(odometer_m, &apollo_msg->odometer_m);
		*port_chassis_odometer = odometer_m;
	}
	
	if(apollo_msg->has_fuel_range_m)
	{
		
		
		*port_chassis_fuel_range_m = apollo_msg->fuel_range_m;
		
		
	}
	if(apollo_msg->has_throttle_percentage)
	{
		//SWAP64(port_chassis_throttle_percentage,apollo_msg->throttle_percentage);
		double throttle_percentage;
		DOUBLE_CAST(throttle_percentage, &apollo_msg->throttle_percentage);
		*port_chassis_throttle_percentage = throttle_percentage;
	}
	if(apollo_msg->has_brake_percentage)
	{
		//SWAP64(port_chassis_brake_percentage,apollo_msg->brake_percentage);
		double brake_percentage;
		DOUBLE_CAST(brake_percentage, &apollo_msg->brake_percentage);
		*port_chassis_brake_percentage = brake_percentage;
	}
	
	if(apollo_msg->has_steering_percentage)
	{
		//SWAP64(port_chassis_steering_percentage,apollo_msg->steering_percentage);
		double steering_percentage;
		DOUBLE_CAST(steering_percentage, &apollo_msg->steering_percentage);
		*port_chassis_steering_percentage = steering_percentage;
	}
	if(apollo_msg->has_steering_torque_nm)
	{
		//SWAP64(port_chassis_steering_torque_nm,apollo_msg->steering_torque_nm);
		double steering_torque_nm;
		DOUBLE_CAST(steering_torque_nm, &apollo_msg->steering_torque_nm);
		*port_chassis_steering_torque_nm = steering_torque_nm;
	}
	if(apollo_msg->has_parking_brake)
	{
		*port_chassis_parking_brake = apollo_msg->parking_brake;
		
	}
	apollo__canbus__chassis_to_autobox_bridge__free_unpacked(apollo_msg, NULL);
}
