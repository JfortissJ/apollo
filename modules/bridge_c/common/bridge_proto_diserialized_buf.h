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
#ifndef MODULES_BRIDGE_C_COMMON_BRIDE_PROTO_DISERAILIZED_BUF_H
#define MODULES_BRIDGE_C_COMMON_BRIDE_PROTO_DISERAILIZED_BUF_H

#include <string.h>
#include "bridge_header.h"
#include "macro.h"

static const uint32_t INT_BITS = (uint32_t) (sizeof(uint32_t) * 8);
#define MAX_MSG_SIZE 1024
typedef struct _BridgeProtoDiserializedBuf BridgeProtoDiserializedBuf;
struct _BridgeProtoDiserializedBuf
{
	char* proto_buf_;
	bool is_ready_diser;
	uint32_t* status_list_;
	size_t total_size_;
	size_t total_frames_;
	char* proto_name_;
	uint32_t sequence_num_;
	uint32_t status_size_;
	bool has_init_;
	bool has_max_total_frames_;
};

bool BridgeProto_Diserialized_Buf_Initialize(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER bridgeHeader);

bool BridgeProto_Diserialized_Buf_Status_Initialize(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf);

bool BridgeProto_Diserialized_Buf_IsReadyDiserialize(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf);

void BridgeProto_Diserialized_Buf_UpdateStatus(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, uint32_t frame_index);

bool BridgeProto_Diserialized_Buf_IsTheProto(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER bridgeHeader);

char* BridgeProto_Diserialized_Buf_GetBuf(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, size_t offset);

uint32_t BridgeProto_Diserialized_Buf_GetMsgID(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf);

char* BridgeProto_Diserialized_Buf_GetMsgName(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf);

#endif
