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

#define S_FUNCTION_NAME dspace_bridge_converter
#define S_FUNCTION_LEVEL 2

#define DSPACE_UDP_RECEIVE_SIZE_UINT8 302
#define DSPACE_UDP_RECEIVE_SIZE_UINT32 DSPACE_UDP_RECEIVE_SIZE_UINT8*4

#define INPUT_DATA 0
#define INPUT_SIZE 1
#define OUTPUT_DATA 0
#define OUTPUT_SIZE 1
#define INPUT_NEW 2
#define OUTPUT_NEW 2

#include "simstruc.h"

#if defined(_DS1401)
extern SimStruct* sim_struct_ptr;
#else
SimStruct* sim_struct_ptr;
#endif

#include "fortiss_definitions.h"

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
  ssSetInputPortWidth(S, INPUT_DATA, DSPACE_UDP_RECEIVE_SIZE_UINT8);
  ssSetInputPortWidth(S, INPUT_SIZE, 1);
  ssSetInputPortWidth(S, INPUT_NEW, 1);
  // Set the data type of an input port
  ssSetInputPortDataType(S, INPUT_DATA, SS_UINT32);
  ssSetInputPortDataType(S, INPUT_SIZE, SS_INT32);
  ssSetInputPortDataType(S, INPUT_NEW, SS_BOOLEAN);
  
  for (iPort = 0; iPort < 3; iPort++)
  {
    // Specify that the signal elements entering a port must be contiguous
    ssSetInputPortRequiredContiguous(S, iPort,
                                     true); /*direct input signal access*/
    // Set ports to no direct feedthrough state
    ssSetInputPortDirectFeedThrough(S, iPort, 1);
  }
  
  // TODO message orientierted
  if(!ssSetNumOutputPorts(S, 3))
  {
    return;
  }
  ssSetOutputPortWidth(S, OUTPUT_DATA, DSPACE_UDP_RECEIVE_SIZE_UINT32);
  ssSetOutputPortWidth(S, OUTPUT_SIZE, 1);
  ssSetOutputPortWidth(S, OUTPUT_NEW, 1);
  ssSetOutputPortDataType(S, OUTPUT_DATA, SS_UINT8);
  ssSetOutputPortDataType(S, OUTPUT_SIZE, SS_UINT16);
  ssSetOutputPortDataType(S, OUTPUT_NEW, SS_BOOLEAN);
  
  // Specify the number of sample times that an S-Function block has
  ssSetNumSampleTimes(S, 1);
  
  /* Take care when specifying exception free code - see sfuntmpl.doc */
  ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct* S)
{
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct* S, int_T tid)
{
  const uint32_T* buf = (uint32_T*) ssGetInputPortSignal(S, INPUT_DATA);
  const uint32_T* size = (const uint32_T*) ssGetInputPortSignal(S, INPUT_SIZE);
  uint8_T* output_data = (uint8_T*) ssGetOutputPortSignal(S, OUTPUT_DATA);
  uint16_T* output_size = (uint16_T*) ssGetOutputPortSignal(S, OUTPUT_SIZE);
  const bool* input_new = (bool*) ssGetInputPortSignal(S, INPUT_NEW);
  bool* output_new = (bool*) ssGetOutputPortSignal(S, OUTPUT_NEW);
  int_T width = ssGetInputPortWidth(S, INPUT_DATA);
  if(*size > 0)
  {
    uint32_T i;
    for (i = 0; i < width; ++i)
    { // TODO we only checkt with a hardcoded width of 256 (aka DSPACE_UDP_RECEIVE_SIZE_UINT8) but this should also work.
      *(output_data + 4 * i) = (buf[i] & 0x000000ff);
      *(output_data + 1 + 4 * i) = (buf[i] & 0x0000ff00) >> 8;
      *(output_data + 2 + 4 * i) = (buf[i] & 0x00ff0000) >> 16;
      *(output_data + 3 + 4 * i) = (buf[i] & 0xff000000) >> 24;
    }
    *output_size = *size;
  }
  *output_new = *input_new;
}

static void mdlTerminate(SimStruct* S)
{
  // TODO free
}
/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else

#include "cg_sfun.h" /* Code generation registration function */

#endif