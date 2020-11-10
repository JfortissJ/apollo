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
#include "bridge_buffer.h"
#include "fortiss_definitions.h"

bool APOLLO_BRIDGE_Header_Handle(APOLLO_BRIDGE_HEADER* bridgeHeader, const char* total_buf)
{
	int i;
	char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1];
	for (size_t j = 0; j < sizeof(BRIDGE_HEADER_FLAG) + 1; ++j)
	{
		header_flag[j] = 0;
	}
	size_t offset = 0;
	// step 1: get the Header Flag which should be ApolloBridgeHeader
	memcpy(header_flag, total_buf, HEADER_FLAG_SIZE);
	if(strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0)
	{
		return false;
	}
	offset += sizeof(BRIDGE_HEADER_FLAG) + 1;
	char header_size_buf[sizeof(hsize) + 1];
	for (size_t k = 0; k < sizeof(hsize) + 1; ++k)
	{
		header_size_buf[k] = 0;
	}
	// step2:  move the cursor to the next position, and get the information of header_size
	const char* cursor = total_buf + offset;
	memcpy(header_size_buf, cursor, sizeof(hsize));
	hsize header_size = UINT32_CAST(header_size_buf);
	if(header_size > APOLLO_BRIDGE_MARCO_FRAME_SIZE)
	{
		
		return false;
	}
	offset += sizeof(hsize) + 1;
	APOLLO_BRIDGE_HEADER_Item_Init(bridgeHeader);
	// step3: get the header Frame situation
	size_t buf_size = header_size - offset;
	cursor = total_buf + offset;
	if(!APOLLO_BRIDGE_HEADER_Diserialize(bridgeHeader, cursor, buf_size))
	{
		printf("APOLLO_BRIDGE_HEADER_Diserialize failed\n");
		return false;
	}
	bridgeHeader->header_size = header_size;
	return true;
}

bool APOLLO_BRIDGE_DiserializeInit(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	bool hasInit =
			BridgeProto_Diserialized_Buf_Initialize(bridgeProtoDiserializedBuf, *bridgeHeader);
	if(!hasInit)
	{
		FORTISS_INFO_PRINTF("hasInit is false!\n");
		return false;
	}
	return true;
}

bool APOLLO_BRIDGE_Buffer(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER* bridgeHeader, const char* proto_message_buf)
{
	
	// move the cursor after the header_size
	const char* cursor = proto_message_buf + bridgeHeader->header_size;
	char* buf = BridgeProto_Diserialized_Buf_GetBuf(bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER_GetFramePos(bridgeHeader));
	
	memcpy(buf, cursor, APOLLO_BRIDGE_HEADER_GetFrameSize(bridgeHeader));
	BridgeProto_Diserialized_Buf_UpdateStatus(bridgeProtoDiserializedBuf, (APOLLO_BRIDGE_HEADER_GetIndex(bridgeHeader)));
	return true;
}
