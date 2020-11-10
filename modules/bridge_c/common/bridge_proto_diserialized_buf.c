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
#include "bridge_proto_diserialized_buf.h"
#include "fortiss_definitions.h"

bool BridgeProto_Diserialized_Buf_Initialize(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER bridgeHeader)
{
	bridgeProtoDiserializedBuf->is_ready_diser = false;
	bridgeProtoDiserializedBuf->sequence_num_ = 0;
	bridgeProtoDiserializedBuf->proto_name_ = "";
	bridgeProtoDiserializedBuf->total_size_ = APOLLO_BRIDGE_HEADER_GetMsgSize(&bridgeHeader);
	bridgeProtoDiserializedBuf->total_frames_ = APOLLO_BRIDGE_HEADER_GetTotalFrames(&bridgeHeader);
	if(bridgeProtoDiserializedBuf->total_frames_ == 0)
	{
		return false;
	}
	return true;
}

bool BridgeProto_Diserialized_Buf_Status_Initialize(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf)
{
	int status_size = (int) (bridgeProtoDiserializedBuf->total_frames_ / INT_BITS + ((bridgeProtoDiserializedBuf->total_frames_ % INT_BITS) ? 1 : 0));
	bridgeProtoDiserializedBuf->status_size_ = status_size;
	bridgeProtoDiserializedBuf->status_list_ = (uint32_t*) (malloc(status_size * sizeof(uint32_t)));
	for (int i = 0; i < status_size; i++)
	{
		bridgeProtoDiserializedBuf->status_list_[i] = 0;
	}
	
	bridgeProtoDiserializedBuf->proto_buf_ = (char*) malloc(bridgeProtoDiserializedBuf->total_size_ * sizeof(char));
	return true;
}

bool BridgeProto_Diserialized_Buf_IsReadyDiserialize(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf)
{
	return bridgeProtoDiserializedBuf->is_ready_diser;
}

void BridgeProto_Diserialized_Buf_UpdateStatus(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, uint32_t frame_index)
{
	size_t status_size = bridgeProtoDiserializedBuf->status_size_;
	if(status_size == 0)
	{
		bridgeProtoDiserializedBuf->is_ready_diser = false;
		return;
	}
	
	uint32_t status_index = frame_index / INT_BITS;
	bridgeProtoDiserializedBuf->status_list_[status_index] |= (1 << (frame_index % INT_BITS));
	//FORTISS_INFO_PRINTF("total_frame %i frame_index %i, status_index %i\n",bridgeProtoDiserializedBuf->total_frames_,frame_index,status_index);
	bridgeProtoDiserializedBuf->has_max_total_frames_ = bridgeProtoDiserializedBuf->total_frames_ - 1 == frame_index ? true : false;
	for (size_t i = 0; i < status_size; i++)
	{
		//FORTISS_INFO_PRINTF("Statue_size[%d] is %i \n",i, bridgeProtoDiserializedBuf->status_list_[i]);
		if(i == status_size - 1)
		{
			if(bridgeProtoDiserializedBuf->status_list_[i] == (uint32_t) ((1 << bridgeProtoDiserializedBuf->total_frames_ % INT_BITS) - 1))
			{
				bridgeProtoDiserializedBuf->is_ready_diser = true;
			}
			else
			{
				bridgeProtoDiserializedBuf->is_ready_diser = false;
				break;
			}
		}
		else
		{
			if(bridgeProtoDiserializedBuf->status_list_[i] != 0xffffffff)
			{
				bridgeProtoDiserializedBuf->is_ready_diser = false;
				break;
			}
			bridgeProtoDiserializedBuf->is_ready_diser = true;
		}
	}
}

bool BridgeProto_Diserialized_Buf_IsTheProto(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER bridgeHeader)
{
	if(strcmp(bridgeProtoDiserializedBuf->proto_name_, APOLLO_BRIDGE_HEADER_GetMsgName(&bridgeHeader)) == 0 &&
	   bridgeProtoDiserializedBuf->sequence_num_ == APOLLO_BRIDGE_HEADER_GetMsgID(&bridgeHeader))
	{
		return true;
	}
	return false;
}

char* BridgeProto_Diserialized_Buf_GetBuf(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, size_t offset)
{
	return bridgeProtoDiserializedBuf->proto_buf_ + offset;
}

uint32_t BridgeProto_Diserialized_Buf_GetMsgID(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf)
{
	return bridgeProtoDiserializedBuf->sequence_num_;
}

char* BridgeProto_Diserialized_Buf_GetMsgName(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf)
{
	return bridgeProtoDiserializedBuf->proto_name_;
}

