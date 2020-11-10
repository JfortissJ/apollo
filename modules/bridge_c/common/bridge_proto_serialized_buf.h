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
#ifndef APOLLO_CLIENT_C_BRIDGE_PROTO_SERIALIZED_BUF_H
#define APOLLO_CLIENT_C_BRIDGE_PROTO_SERIALIZED_BUF_H

#include <string.h>
#include "bridge_header.h"
#include "macro.h"

typedef struct _BridgeProtoSerializedBuf BridgeProtoSerializedBuf;
typedef struct _Buf Buf;

struct _Buf
{
	char* buf_;
	size_t buf_len_;
};
struct _BridgeProtoSerializedBuf
{
	Buf* frame;
	char* msg_name;
	bsize msg_len;
	char* protobuf;
	uint32_t total_frames;
	uint32_t sequence_num;
	double timestamp_sec;
	
};

bool BridgeProto_Serialized_Buf_SetValue(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf, char* protobuf, bsize msg_len, char* msg_name,
										 uint32_t sequence_num, double timeStamp_sec);

bool BridgeProto_Serialized_Buf_Serialize(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf);

void BridgeProto_Serialized_Buf_Free(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf);


char* BridgeProto_Serialized_Buf_GetSerializedBuf(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf, size_t index);

size_t BridgeProto_Serialized_Buf_GetSerializedBufCount(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf, size_t index);

size_t BridgeProto_Serialized_Buf_GetSerializedBufSize(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf, size_t index);

#endif //APOLLO_CLIENT_C_BRIDGE_PROTO_SERIALIZED_BUF_H
