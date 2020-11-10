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
#include "bridge_proto_serialized_buf.h"
#include "protoc_genfiles/modules/autobox_bridge/proto/autobox_trajectory.pb-c.h"

bool BridgeProto_Serialized_Buf_SetValue(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf, char* protobuf, bsize msg_len, char* msg_name,
										 uint32_t sequence_num, double timeStamp_sec)
{
	bridgeProtoSerializedBuf->msg_name = msg_name;
	bridgeProtoSerializedBuf->msg_len = msg_len;
	bridgeProtoSerializedBuf->sequence_num = sequence_num;
	bridgeProtoSerializedBuf->timestamp_sec = timeStamp_sec;
	bridgeProtoSerializedBuf->total_frames = UINT32_CAST(bridgeProtoSerializedBuf->msg_len / APOLLO_BRIDGE_MARCO_FRAME_SIZE
														 + bridgeProtoSerializedBuf->msg_len % APOLLO_BRIDGE_MARCO_FRAME_SIZE ? 1 : 0);
	bridgeProtoSerializedBuf->protobuf = (char*) malloc(ARRAY_SIZE(protobuf)); //TODO
	bridgeProtoSerializedBuf->protobuf = protobuf;
	bridgeProtoSerializedBuf->frame = (Buf*) malloc(sizeof(Buf) * bridgeProtoSerializedBuf->total_frames);
	return true;
}

bool BridgeProto_Serialized_Buf_Serialize(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf)
{
	
	bsize offset = 0;
	bsize frame_index = 0;
	while (offset < bridgeProtoSerializedBuf->msg_len)
	{
		bsize left = bridgeProtoSerializedBuf->msg_len - frame_index * APOLLO_BRIDGE_MARCO_FRAME_SIZE;
		bsize copy_size = (left > APOLLO_BRIDGE_MARCO_FRAME_SIZE ? APOLLO_BRIDGE_MARCO_FRAME_SIZE : left);
		APOLLO_BRIDGE_HEADER bridgeHeader;
		APOLLO_BRIDGE_HEADER_SetHeaderVer(&bridgeHeader, 0);
		APOLLO_BRIDGE_HEADER_SetMsgName(&bridgeHeader, bridgeProtoSerializedBuf->msg_name);
		APOLLO_BRIDGE_HEADER_SetMsgID(&bridgeHeader, bridgeProtoSerializedBuf->sequence_num);
		APOLLO_BRIDGE_HEADER_SetTimeStamp(&bridgeHeader, bridgeProtoSerializedBuf->timestamp_sec);
		APOLLO_BRIDGE_HEADER_SetMsgSize(&bridgeHeader, bridgeProtoSerializedBuf->msg_len);
		APOLLO_BRIDGE_HEADER_SetTotalFrames(&bridgeHeader, bridgeProtoSerializedBuf->total_frames);
		APOLLO_BRIDGE_HEADER_SetFrameSize(&bridgeHeader, copy_size);
		APOLLO_BRIDGE_HEADER_SetFramePos(&bridgeHeader, frame_index * APOLLO_BRIDGE_MARCO_FRAME_SIZE);
		hsize header_size = APOLLO_BRIDGE_HEADER_GetHeaderSize(&bridgeHeader);
		Buf buf;
		buf.buf_ = (char*) malloc((copy_size + header_size) * sizeof(char));
		buf.buf_len_ = copy_size + header_size;
		APOLLO_BRIDGE_HEADER_Serialize(&bridgeHeader, buf.buf_, buf.buf_len_);
		memcpy(buf.buf_ + header_size, bridgeProtoSerializedBuf->protobuf + frame_index * APOLLO_BRIDGE_MARCO_FRAME_SIZE, copy_size);
		bridgeProtoSerializedBuf->frame[frame_index] = buf;
		frame_index++;
		offset += copy_size;
	}
	return true;
}

void BridgeProto_Serialized_Buf_Free(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf)
{
	free(bridgeProtoSerializedBuf->protobuf);
	for (size_t i = 0; i < bridgeProtoSerializedBuf->total_frames; ++i)
	{
		free(bridgeProtoSerializedBuf->frame[i].buf_);
	}
}

char* BridgeProto_Serialized_Buf_GetSerializedBuf(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf, size_t index)
{
	return bridgeProtoSerializedBuf->frame[index].buf_;
}

size_t BridgeProto_Serialized_Buf_GetSerializedBufCount(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf, size_t index)
{
	return bridgeProtoSerializedBuf->total_frames;
}

size_t BridgeProto_Serialized_Buf_GetSerializedBufSize(BridgeProtoSerializedBuf* bridgeProtoSerializedBuf, size_t index)
{
	return bridgeProtoSerializedBuf->frame[index].buf_len_;
}
