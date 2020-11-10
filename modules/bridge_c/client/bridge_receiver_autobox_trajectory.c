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
#include "bridge_receiver_autobox_trajectory.h"
#include "fortiss_definitions.h"

//#define PRINTBUF
Apollo__Planning__ADCTrajectoryToAutoboxBridge* handle_message_planning(BridgeProtoDiserializedBuf* proto_buf)
{
	Apollo__Planning__ADCTrajectoryToAutoboxBridge* apollo_msg = NULL;
#ifdef PRINTBUF
	for(int i=0; i<proto_buf->total_size_; ++i)
	  {
		  //FORTISS_INFO_PRINTF("i= %i, value= %u\n",i,(uint8_t)proto_buf->proto_buf_[i]);
		  FORTISS_INFO_PRINTF("%u, ",(uint8_t)proto_buf->proto_buf_[i]);
		  if(i%1024==0)
		  {
			  printf("\n");
		  }
	  }
		  printf("\n");
#endif
	apollo_msg = apollo__planning__adctrajectory_to_autobox_bridge__unpack(NULL, proto_buf->total_size_, (uint8_t*) (proto_buf->proto_buf_));
	
	if(apollo_msg == NULL)
	{
		
		return NULL;
	}
	else
	{
		return apollo_msg;
	}
}
