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
#include "bridge_receiver_preprocessing.h"
#include "fortiss_definitions.h"

bool handle_message_Check(BridgeProtoDiserializedBuf* proto_buf, const char* total_buf, bool* hasinit, uint32_t frame_buffer_size)
{
	APOLLO_BRIDGE_HEADER bridgeHeader;
	bool hasHeader = APOLLO_BRIDGE_Header_Handle(&bridgeHeader, total_buf);
	if(!hasHeader)
	{
		FORTISS_INFO_PRINTF("APollo_BRIDGE_HEADER can not be correctly decoded\n");
		return false;
	}
	if(APOLLO_BRIDGE_HEADER_GetFrameBufferSize(&bridgeHeader) != frame_buffer_size)
	{
		FORTISS_INFO_PRINTF("The receving data buffer size is not equal to the real data buffer size\n");
		return false;
	}
	if(!*hasinit)
	{
		bool basicInit = APOLLO_BRIDGE_DiserializeInit(proto_buf, &bridgeHeader);
		bool statusInit = BridgeProto_Diserialized_Buf_Status_Initialize(proto_buf);
		*hasinit = true;
	}
	bool hasValid = APOLLO_BRIDGE_Buffer(proto_buf, &bridgeHeader, total_buf);
	if(!hasValid)
	{
		FORTISS_INFO_PRINTF("APOLLO_BRIDGE_BUffer failed to get the valid information\n");
		return false;
	}
	return BridgeProto_Diserialized_Buf_IsReadyDiserialize(proto_buf);
}
