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

#define APOLLO_CONTROL_HEADER_SEQUA_NUM 0
#define APOLLO_CONTROL_HEADER_TIMESTAMP_SEC 1
#define APOLLO_CONTROL_THROTTLE 2
#define APOLLO_CONTROL_BRAKE 3
#define APOLLO_CONTROL_STEERING_RATE 4
#define APOLLO_CONTROL_STEERING_TARGET 5
#define APOLLO_CONTROL_ACCELERATION 6

#include "common/macro.h"

void initialize_outputsPort_Control_COMMAND(SimStruct* S)
{
	if(!ssSetNumOutputPorts(S, 7))
	{
		return;
	}
	ssSetOutputPortWidth(S, APOLLO_CONTROL_HEADER_SEQUA_NUM, 1);
	ssSetOutputPortWidth(S, APOLLO_CONTROL_HEADER_TIMESTAMP_SEC, 1);
	ssSetOutputPortWidth(S, APOLLO_CONTROL_THROTTLE, 1);
	ssSetOutputPortWidth(S, APOLLO_CONTROL_BRAKE, 1);
	ssSetOutputPortWidth(S, APOLLO_CONTROL_STEERING_RATE, 1);
	ssSetOutputPortWidth(S, APOLLO_CONTROL_STEERING_TARGET, 1);
	ssSetOutputPortWidth(S, APOLLO_CONTROL_ACCELERATION, 1);
	
	ssSetOutputPortDataType(S, APOLLO_CONTROL_HEADER_SEQUA_NUM, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CONTROL_HEADER_TIMESTAMP_SEC, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CONTROL_THROTTLE, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CONTROL_BRAKE, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CONTROL_STEERING_RATE, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CONTROL_STEERING_TARGET, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_CONTROL_ACCELERATION, SS_DOUBLE);
}

void fill_outputsPort_Control_COMMAND(SimStruct* S, Apollo__Control__ControlCommandToAutoboxBridge* apollo_msg)
{
	double* port_header_Sequa_num = (double*) ssGetOutputPortSignal(S, APOLLO_CONTROL_HEADER_SEQUA_NUM);
	double* port_header_teamstem_sec = (double*) ssGetOutputPortSignal(S, APOLLO_CONTROL_HEADER_TIMESTAMP_SEC);
	double* port_control_throttle = (double*) ssGetOutputPortSignal(S, APOLLO_CONTROL_THROTTLE);
	double* port_control_brake = (double*) ssGetOutputPortSignal(S, APOLLO_CONTROL_BRAKE);
	double* port_control_stearing_rate = (double*) ssGetOutputPortSignal(S, APOLLO_CONTROL_STEERING_RATE);
	double* port_control_steering_target = (double*) ssGetOutputPortSignal(S, APOLLO_CONTROL_STEERING_TARGET);
	double* port_control_acceleration = (double*) ssGetOutputPortSignal(S, APOLLO_CONTROL_ACCELERATION);
	if(apollo_msg->header->has_sequence_num)
	{
		DOUBLE_CAST(*port_header_Sequa_num, &apollo_msg->header->sequence_num);
	}
	
	if(apollo_msg->header->has_timestamp_sec)
	{
		DOUBLE_CAST(*port_header_teamstem_sec, &apollo_msg->header->timestamp_sec);
	}
	if(apollo_msg->has_throttle)
	{
		DOUBLE_CAST(*port_control_throttle, &apollo_msg->throttle);
	}
	if(apollo_msg->has_brake)
	{
		DOUBLE_CAST(*port_control_brake, &apollo_msg->brake);
	}
	
	if(apollo_msg->has_steering_rate)
	{
		DOUBLE_CAST(*port_control_stearing_rate, &apollo_msg->steering_rate);
	}
	if(apollo_msg->has_steering_target)
	{
		DOUBLE_CAST(*port_control_steering_target, &apollo_msg->steering_target);
	}
	if(apollo_msg->has_acceleration)
	{
		DOUBLE_CAST(*port_control_acceleration, &apollo_msg->acceleration);
	}
	apollo__control__control_command_to_autobox_bridge__free_unpacked(apollo_msg, NULL);
}
