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

#define APOLLO_TRAJECTORY_HEADER_TIMESTAMP_SEC 0
#define APOLLO_TRAJECTORY_RELATIVE_TIME 1
#define APOLLO_TRAJECTORY_GEAR 2
#define APOLLO_TRAJECTORY_TOTAL_PATH_LENGTH 3
#define APOLLO_TRAJECTORY_TOTAL_PATH_TIME 4
#define APOLLO_TRAJECTORY_ESTOP_IS_ESTOP 5
#define APOLLO_TRAJECTORY_X 6
#define APOLLO_TRAJECTORY_Y 7
#define APOLLO_TRAJECTORY_Z 8
#define APOLLO_TRAJECTORY_ACCELERATION_S 9
#define APOLLO_TRAJECTORY_CURVATURE 10
#define APOLLO_TRAJECTORY_THETA 11
#define APOLLO_TRAJECTORY_CURVATURE_CHANGE_RATE 12
#define APOLLO_TRAJECTORY_SPEED 13
#define APOLLO_TRAJECTORY_ACCUMULATED_S 14
#define APOLLO_TRAJECTORY_N_POINTS 15
#define APOLLO_TRAJECTORY_DDKAPPA 16

#define BUS_MAX_ARRAY_SIZE 100

#include "common/macro.h"

void initialize_outputs_trajectory(SimStruct* S)
{
	
	if(!ssSetNumOutputPorts(S, 17)) return;
	
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_HEADER_TIMESTAMP_SEC, 1); // header->timestamp_sec
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_RELATIVE_TIME, BUS_MAX_ARRAY_SIZE); // ->relative_time
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_GEAR, 1); // ->gear
	
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_TOTAL_PATH_LENGTH, 1); // ->total_path_length
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_TOTAL_PATH_TIME, 1); // ->total_path_time
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_ESTOP_IS_ESTOP, 1); // estop->is_estop
	
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_X, BUS_MAX_ARRAY_SIZE); // ->x
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_Y, BUS_MAX_ARRAY_SIZE); // ->y
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_Z, BUS_MAX_ARRAY_SIZE); // ->z
	
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_ACCELERATION_S, BUS_MAX_ARRAY_SIZE); // ->acceleration_s
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_CURVATURE, BUS_MAX_ARRAY_SIZE); // ->curvature
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_THETA, BUS_MAX_ARRAY_SIZE); // ->theta
	
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_CURVATURE_CHANGE_RATE, BUS_MAX_ARRAY_SIZE); // ->curvature_change_rate
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_SPEED, BUS_MAX_ARRAY_SIZE); // ->speed
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_ACCUMULATED_S, BUS_MAX_ARRAY_SIZE); // ->accumulated_s
	
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_N_POINTS, 1); // ->n_points
	ssSetOutputPortWidth(S, APOLLO_TRAJECTORY_DDKAPPA, BUS_MAX_ARRAY_SIZE); // ->accumulated_s
	
	
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_RELATIVE_TIME, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_GEAR, SS_INT32);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_TOTAL_PATH_LENGTH, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_ACCELERATION_S, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_CURVATURE, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_Y, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_HEADER_TIMESTAMP_SEC, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_CURVATURE_CHANGE_RATE, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_TOTAL_PATH_TIME, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_THETA, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_ESTOP_IS_ESTOP, SS_BOOLEAN);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_X, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_Z, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_SPEED, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_ACCUMULATED_S, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_DDKAPPA, SS_DOUBLE);
	ssSetOutputPortDataType(S, APOLLO_TRAJECTORY_N_POINTS, SS_UINT32);
}

void fill_outputs_trajectory(SimStruct* S, Apollo__Planning__ADCTrajectoryToAutoboxBridge* apollo_msg)
{
	uint32_t i;
	double* port_relative_time = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_RELATIVE_TIME);
	int32_t* port_gear = (int32_t*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_GEAR);
	double* port_total_path_length = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_TOTAL_PATH_LENGTH);
	double* port_acceleration_s = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_ACCELERATION_S);
	double* port_curvature = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_CURVATURE);
	double* port_y = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_Y);
	double* port_header_timestamp_sec = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_HEADER_TIMESTAMP_SEC);
	double* port_curvature_change_rate = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_CURVATURE_CHANGE_RATE);
	double* port_ddkappa = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_DDKAPPA);
	double* port_total_path_time = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_TOTAL_PATH_TIME);
	double* port_theta = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_THETA);
	bool* port_estop_is_estop = (bool*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_ESTOP_IS_ESTOP);
	double* port_x = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_X);
	double* port_z = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_Z);
	double* port_speed = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_SPEED);
	double* port_accumulated_s = (double*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_ACCUMULATED_S);
	uint32_t* port_n_points = (uint32_t*) ssGetOutputPortSignal(S, APOLLO_TRAJECTORY_N_POINTS);

	
	if(apollo_msg->header->has_timestamp_sec)
	{
		double temp;
		DOUBLE_CAST(temp, &apollo_msg->header->timestamp_sec);
		*port_header_timestamp_sec = temp;
	}

	*port_n_points = (uint32_t) apollo_msg->n_trajectory_point;
	*port_gear = -1000;
	*port_total_path_length = -1000;
	*port_total_path_time = -1000;
	*port_estop_is_estop = 0;
// Zero bus
	for (i = 0; i < BUS_MAX_ARRAY_SIZE; ++i)
	{
		port_relative_time[i] = 0;
		port_acceleration_s[i] = 0;
		port_curvature[i] = 0;
		port_x[i] = 0;
		port_y[i] = 0;
		port_z[i] = 0;
		port_curvature_change_rate[i] = 0;
		port_ddkappa[i] = 0;
		port_theta[i] = 0;
		port_speed[i] = 0;
		port_accumulated_s[i] = 0;
	}
	for (i = 0; i < apollo_msg->n_trajectory_point && i < BUS_MAX_ARRAY_SIZE; ++i)
	{
		if(apollo_msg->trajectory_point[i]->has_relative_time)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->relative_time);
			port_relative_time[i] = temp;
		}
		if(apollo_msg->trajectory_point[i]->has_a)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->a);
			port_acceleration_s[i] = temp;
		}
		if(apollo_msg->trajectory_point[i]->path_point->has_kappa)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->path_point->kappa);
			port_curvature[i] = temp;
		}
		if(apollo_msg->trajectory_point[i]->path_point->has_x)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->path_point->x);
			port_x[i] = temp;
		}
		if(apollo_msg->trajectory_point[i]->path_point->has_y)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->path_point->y);
			port_y[i] = temp;
		}
		
		if(apollo_msg->trajectory_point[i]->path_point->has_z)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->path_point->z);
			port_z[i] = temp;
		}
		
		if(apollo_msg->trajectory_point[i]->path_point->has_dkappa)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->path_point->dkappa);
			port_curvature_change_rate[i] = temp;
		}
		
		if(apollo_msg->trajectory_point[i]->path_point->has_ddkappa)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->path_point->ddkappa);
			port_ddkappa[i] = temp;
		}
		
		if(apollo_msg->trajectory_point[i]->path_point->has_theta)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->path_point->theta);
			port_theta[i] = temp;
		}
		
		if(apollo_msg->trajectory_point[i]->has_v)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->v);
			port_speed[i] = temp;
		}
		
		if(apollo_msg->trajectory_point[i]->path_point->has_s)
		{
			double temp;
			DOUBLE_CAST(temp, &apollo_msg->trajectory_point[i]->path_point->s);
			port_accumulated_s[i] = temp;
		}
		
	}
	
	apollo__planning__adctrajectory_to_autobox_bridge__free_unpacked(apollo_msg, NULL);
	
}



























