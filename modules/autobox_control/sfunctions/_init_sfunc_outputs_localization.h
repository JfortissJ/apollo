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

#define APOLLO_POSE_ORIENTATION_QY 0
#define APOLLO_POSE_ORIENTATION_QX 1
#define APOLLO_POSE_ORIENTATION_QZ 2
#define APOLLO_POSE_ORIENTATION_QW 3
#define APOLLO_POSE_LINEAR_VELOCITY_Y 4
#define APOLLO_POSE_LINEAR_VELOCITY_X 5
#define APOLLO_POSE_LINEAR_VELOCITY_Z 6
#define APOLLO_POSE_LINEAR_ACCELERATION_Y 7
#define APOLLO_POSE_LINEAR_ACCELERATION_X 8
#define APOLLO_POSE_LINEAR_ACCELERATION_Z 9
#define APOLLO_POSE_POSITION_Y 10
#define APOLLO_POSE_POSITION_X 11
#define APOLLO_POSE_POSITION_Z 12
#define APOLLO_POSE_ANGULAR_VELOCITY_Y 13
#define APOLLO_POSE_ANGULAR_VELOCITY_X 14
#define APOLLO_POSE_ANGULAR_VELOCITY_Z 15
#define APOLLO_POSE_HEADING 16
#define APOLLO_MEASUREMENT_TIME 17

#include "common/macro.h"

void initialize_outputs_localization(SimStruct* S)
{
	
	if(!ssSetNumOutputPorts(S, 18)) return;
	
	ssSetOutputPortWidth(S, APOLLO_POSE_ORIENTATION_QY, 1); // pose->orientation->qy
	ssSetOutputPortWidth(S, APOLLO_POSE_ORIENTATION_QX, 1); // pose->orientation->qx
	ssSetOutputPortWidth(S, APOLLO_POSE_ORIENTATION_QZ, 1); // pose->orientation->qz
	ssSetOutputPortWidth(S, APOLLO_POSE_ORIENTATION_QW, 1); // pose->orientation->qw
	ssSetOutputPortWidth(S, APOLLO_POSE_LINEAR_VELOCITY_Y, 1); // pose->linear_velocity->y
	ssSetOutputPortWidth(S, APOLLO_POSE_LINEAR_VELOCITY_X, 1); // pose->linear_velocity->x
	ssSetOutputPortWidth(S, APOLLO_POSE_LINEAR_VELOCITY_Z, 1); // pose->linear_velocity->z
	ssSetOutputPortWidth(S, APOLLO_POSE_LINEAR_ACCELERATION_Y, 1); // pose->linear_acceleration->y
	ssSetOutputPortWidth(S, APOLLO_POSE_LINEAR_ACCELERATION_X, 1); // pose->linear_acceleration->x
	ssSetOutputPortWidth(S, APOLLO_POSE_LINEAR_ACCELERATION_Z, 1); // pose->linear_acceleration->z
	ssSetOutputPortWidth(S, APOLLO_POSE_POSITION_Y, 1); // pose->position->y
	ssSetOutputPortWidth(S, APOLLO_POSE_POSITION_X, 1); // pose->position->x
	ssSetOutputPortWidth(S, APOLLO_POSE_POSITION_Z, 1); // pose->position->z
	ssSetOutputPortWidth(S, APOLLO_POSE_ANGULAR_VELOCITY_Y, 1); // pose->angular_velocity->y
	ssSetOutputPortWidth(S, APOLLO_POSE_ANGULAR_VELOCITY_X, 1); // pose->angular_velocity->x
	ssSetOutputPortWidth(S, APOLLO_POSE_ANGULAR_VELOCITY_Z, 1); // pose->angular_velocity->z
	ssSetOutputPortWidth(S, APOLLO_POSE_HEADING, 1); // pose->heading
	ssSetOutputPortWidth(S, APOLLO_MEASUREMENT_TIME, 1); // measurement_time
	
	ssSetOutputPortDataType(S, APOLLO_POSE_ORIENTATION_QY, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_ORIENTATION_QX, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_ORIENTATION_QZ, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_ORIENTATION_QW, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_LINEAR_VELOCITY_Y, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_LINEAR_VELOCITY_X, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_LINEAR_VELOCITY_Z, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_LINEAR_ACCELERATION_Y, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_LINEAR_ACCELERATION_X, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_LINEAR_ACCELERATION_Z, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_POSITION_Y, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_POSITION_X, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_POSITION_Z, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_ANGULAR_VELOCITY_Y, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_ANGULAR_VELOCITY_X, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_ANGULAR_VELOCITY_Z, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_POSE_HEADING, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_MEASUREMENT_TIME, SS_DOUBLE);
}


void fill_outputs_localization(SimStruct* S, Apollo__Localization__LocalizationToAutoboxBridge* apollo_msg)
{
	uint32_t i;
	
	double* port_pose_orientation_qy = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_ORIENTATION_QY);
	double* port_pose_orientation_qx = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_ORIENTATION_QX);
	double* port_pose_orientation_qz = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_ORIENTATION_QZ);
	double* port_pose_orientation_qw = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_ORIENTATION_QW);
	double* port_pose_linear_velocity_y = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_LINEAR_VELOCITY_Y);
	double* port_pose_linear_velocity_x = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_LINEAR_VELOCITY_X);
	double* port_pose_linear_velocity_z = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_LINEAR_VELOCITY_Z);
	double* port_pose_linear_acceleration_y = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_LINEAR_ACCELERATION_Y);
	double* port_pose_linear_acceleration_x = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_LINEAR_ACCELERATION_X);
	double* port_pose_linear_acceleration_z = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_LINEAR_ACCELERATION_Z);
	double* port_pose_position_y = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_POSITION_Y);
	double* port_pose_position_x = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_POSITION_X);
	double* port_pose_position_z = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_POSITION_Z);
	double* port_pose_angular_velocity_y = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_ANGULAR_VELOCITY_Y);
	double* port_pose_angular_velocity_x = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_ANGULAR_VELOCITY_X);
	double* port_pose_angular_velocity_z = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_ANGULAR_VELOCITY_Z);
	double* port_pose_heading = (double*) ssGetOutputPortSignal(S, APOLLO_POSE_HEADING);
	double* port_measurement_time = (double*) ssGetOutputPortSignal(S, APOLLO_MEASUREMENT_TIME);
	*port_pose_orientation_qy = 0;
	*port_pose_orientation_qx = 0;
	*port_pose_orientation_qz = 0;
	*port_pose_orientation_qw = 0;
	*port_pose_linear_velocity_y = 0;
	*port_pose_linear_velocity_x = 0;
	*port_pose_linear_velocity_z = 0;
	*port_pose_linear_acceleration_y = 0;
	*port_pose_linear_acceleration_x = 0;
	*port_pose_linear_acceleration_z = 0;
	*port_pose_position_y = 0;
	*port_pose_position_x = 0;
	*port_pose_position_z = 0;
	*port_pose_angular_velocity_y = 0;
	*port_pose_angular_velocity_x = 0;
	*port_pose_angular_velocity_z = 0;
	*port_pose_heading = 0;
	*port_measurement_time = 0;

//!!!!ADD FILLING MANUALLY !!!!
	
	if(apollo_msg->has_measurement_time)
	{
		DOUBLE_CAST(*port_measurement_time, &apollo_msg->measurement_time);
	}
	
	if(apollo_msg->pose->orientation->has_qy)
	{
		DOUBLE_CAST(*port_pose_orientation_qy, &apollo_msg->pose->orientation->qy);
	}
	
	if(apollo_msg->pose->orientation->has_qx)
	{
		DOUBLE_CAST(*port_pose_orientation_qx, &apollo_msg->pose->orientation->qx);
	}
	
	if(apollo_msg->pose->orientation->has_qz)
	{
		DOUBLE_CAST(*port_pose_orientation_qz, &apollo_msg->pose->orientation->qz);
	}
	
	if(apollo_msg->pose->orientation->has_qw)
	{
		DOUBLE_CAST(*port_pose_orientation_qw, &apollo_msg->pose->orientation->qw);
	}
	
	if(apollo_msg->pose->linear_velocity->has_y)
	{
		DOUBLE_CAST(*port_pose_linear_velocity_y, &apollo_msg->pose->linear_velocity->y);
	}
	
	if(apollo_msg->pose->linear_velocity->has_x)
	{
		DOUBLE_CAST(*port_pose_linear_velocity_x, &apollo_msg->pose->linear_velocity->x);
		
	}
	
	if(apollo_msg->pose->linear_velocity->has_z)
	{
		DOUBLE_CAST(*port_pose_linear_velocity_z, &apollo_msg->pose->linear_velocity->z);
	}
	
	if(apollo_msg->pose->linear_acceleration->has_y)
	{
		DOUBLE_CAST(*port_pose_linear_acceleration_y, &apollo_msg->pose->linear_acceleration->y);
	}
	
	if(apollo_msg->pose->linear_acceleration->has_x)
	{
		DOUBLE_CAST(*port_pose_linear_acceleration_x, &apollo_msg->pose->linear_acceleration->x);
	}
	
	if(apollo_msg->pose->linear_acceleration->has_z)
	{
		DOUBLE_CAST(*port_pose_linear_acceleration_z, &apollo_msg->pose->linear_acceleration->z);
	}
	
	if(apollo_msg->pose->position->has_y)
	{
		DOUBLE_CAST(*port_pose_position_y, &apollo_msg->pose->position->y);
	}
	
	if(apollo_msg->pose->position->has_x)
	{
		DOUBLE_CAST(*port_pose_position_x, &apollo_msg->pose->position->x);
	}
	
	if(apollo_msg->pose->position->has_z)
	{
		DOUBLE_CAST(*port_pose_position_z, &apollo_msg->pose->position->z);
	}
	
	if(apollo_msg->pose->angular_velocity->has_y)
	{
		DOUBLE_CAST(*port_pose_angular_velocity_y, &apollo_msg->pose->angular_velocity->y);
	}
	
	if(apollo_msg->pose->angular_velocity->has_x)
	{
		DOUBLE_CAST(*port_pose_angular_velocity_x, &apollo_msg->pose->angular_velocity->x);
	}
	
	if(apollo_msg->pose->angular_velocity->has_z)
	{
		DOUBLE_CAST(*port_pose_angular_velocity_z, &apollo_msg->pose->angular_velocity->z);
	}
	
	if(apollo_msg->pose->has_heading)
	{
		DOUBLE_CAST(*port_pose_heading, &apollo_msg->pose->heading);
	}
	apollo__localization__localization_to_autobox_bridge__free_unpacked(apollo_msg, NULL);
// BUG: measurement_time field is invalid! we do not use this but timestamp_sec
//if(ptr_localization_unpacked->has_measurement_time)
//{  
//    SWAP64(port_measurement_time ,ptr_localization_unpacked->measurement_time);
//    FORTISS_INFO_PRINTF(S,">> loca measurement time first observation: %f. \n", *port_measurement_time);
//}
//else
//{
//    FORTISS_INFO_PRINTF(S,">> loca measurement time first observation: has measurement time == false \n");
//}

}
