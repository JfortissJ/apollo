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

#define S_FUNCTION_NAME AutoBoxBridgeControlDeserializedSFunction_MultipleFrame
#define S_FUNCTION_LEVEL 2

#define DSPACE_UDP_RECEIVE_SIZE_UINT8 302
#define DSPACE_UDP_RECEIVE_SIZE_UINT32 DSPACE_UDP_RECEIVE_SIZE_UINT8*4

#define INPUT_DATA 0  //
#define INPUT_SIZE 1  //
#define INPUT_STATUS 2  //
#define PWORK_WIDTH 1

#include "simstruc.h"

#include "fortiss_definitions.h"
#include "client/bridge_receiver_autobox_control.h"
#include "client/bridge_receiver_preprocessing.h"
#include "modules/autobox_bridge/proto/autobox_control.pb-c.h"
#include "_init_sfunc_outputs_control.h"

#if defined(_DS1401)
extern SimStruct *sim_struct_ptr;
#else
SimStruct* sim_struct_ptr;
#endif

static void mdlInitializeSizes(SimStruct* S)
{
	int iPort;
	ssSetNumSFcnParams(S, 0);
	if(ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
	{
		return;
	}
	if(!ssSetNumInputPorts(S, 3))
	{
		return;
	}
	// Specify the width of an input port
	ssSetInputPortWidth(S, INPUT_DATA, DSPACE_UDP_RECEIVE_SIZE_UINT32);
	ssSetInputPortWidth(S, INPUT_SIZE, 1);
	ssSetInputPortWidth(S, INPUT_STATUS, 1);
	// Set the data type of an input port
	ssSetInputPortDataType(S, INPUT_DATA, SS_UINT8);
	ssSetInputPortDataType(S, INPUT_SIZE, SS_UINT16);
	ssSetInputPortDataType(S, INPUT_STATUS, SS_BOOLEAN);
	for (iPort = 0; iPort < 3; iPort++)
	{
		// Specify that the signal elements entering a port must be contiguous
		ssSetInputPortRequiredContiguous(S, iPort, true); /*direct input signal access*/
		// Set ports to no direct feedthrough state
		ssSetInputPortDirectFeedThrough(S, iPort, 1);
	}
	
	// TODO message orientierted
	initialize_outputsPort_Control_COMMAND(S);
	// Specify the number of sample times that an S-Function block has
	ssSetNumSampleTimes(S, 1);
	
	/* Take care when specifying exception free code - see sfuntmpl.doc */
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
	
	ssSetNumPWork(S, PWORK_WIDTH); // pointer work vector
	// ssSetNumIWork(S,IWORK_WIDTH); // integer work vector
}

static void mdlInitializeSampleTimes(SimStruct* S)
{
	ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
	ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
#if defined(MDL_START)

static void mdlStart(SimStruct* S)
{
	
	// \!todo: specify output ports ... specify trajectory bus
	BridgeProtoDiserializedBuf* proto_buf = malloc(sizeof(BridgeProtoDiserializedBuf));
	proto_buf->has_init_ = false;
	proto_buf->is_ready_diser = false;
    proto_buf->has_max_total_frames_ = false;
	ssSetPWorkValue(S, 0, proto_buf);
	
}

#endif

/*
 * Update the PWORK
 * */
#define MDL_UPDATE
#if defined(MDL_UPDATE)

static void mdlUpdate(SimStruct* S, int_T tid)
{
	const bool* input_status = (bool*) ssGetInputPortSignal(S, INPUT_STATUS);
	if(*input_status)
	{
		const char* buf = (char*) ssGetInputPortSignal(S, INPUT_DATA);
		const uint16_T* size = (const uint16_T*) ssGetInputPortSignal(S, INPUT_SIZE);
		BridgeProtoDiserializedBuf* proto_buf = (BridgeProtoDiserializedBuf*) ssGetPWorkValue(S, 0);
		if(*size > 0)
		{
			bool is_ready_diser = handle_message_Check(proto_buf, buf, &proto_buf->has_init_, *size);
		}
	}
}

#endif

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct* S, int_T tid)
{
	BridgeProtoDiserializedBuf* proto_buf = (BridgeProtoDiserializedBuf*) ssGetPWorkValue(S, 0);
	if(proto_buf->is_ready_diser && proto_buf->has_max_total_frames_)
	{
		Apollo__Control__ControlCommandToAutoboxBridge* apollo_msg = handle_message_control(proto_buf);
		if(apollo_msg != NULL)
		{
			fill_outputsPort_Control_COMMAND(S, apollo_msg);
		}
		else
		{
			FORTISS_INFO_PRINTF("\n-------apollo_messg is NULL-------------------\n");
		}
		proto_buf->has_init_ = false;
		proto_buf->is_ready_diser = false;
        proto_buf->has_max_total_frames_ = false;
		free(proto_buf->status_list_);
		free(proto_buf->proto_buf_);
	}
	else if(!proto_buf->is_ready_diser && proto_buf->has_max_total_frames_)
	{
		FORTISS_INFO_PRINTF("\n-------has maximum total frame but error-----!Control!----------\n");
		proto_buf->has_init_ = false;
		proto_buf->is_ready_diser = false;
        proto_buf->has_max_total_frames_ = false;
		free(proto_buf->status_list_);
		free(proto_buf->proto_buf_);
	}
}

static void mdlTerminate(SimStruct* S)
{
	// TODO free
	ssSetPWorkValue(S, 0, NULL);
}
/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else

#include "cg_sfun.h" /* Code generation registration function */

#endif
