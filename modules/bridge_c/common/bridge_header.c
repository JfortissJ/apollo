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
#include "bridge_header.h"
#include "fortiss_definitions.h"

char* APOLLO_BRIDGE_HEADER_SerializeBasicType_CHAR(
		APOLLO_BRIDGE_HEADER* bridgeHeader, const char* value, char* buf,
		size_t size)
{
	size_t S = sizeof(BRIDGE_HEADER_FLAG);
	if(!buf || size < S)
	{
		return NULL;
	}
	char* res = buf;
	memcpy(res, value, S);
	res[S] = '\n';
	res += S + 1;
	return res;
}

void APOLLO_BRIDGE_HEADER_Item_Init(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	APOLLO_BRIDGE_HeaderItem_UINT32_T_SetType(&bridgeHeader->header_ver_,
											  Header_Ver);
	APOLLO_BRIDGE_HeaderItem_STRING_SetType(&bridgeHeader->msg_name_, Msg_Name);
	APOLLO_BRIDGE_HeaderItem_UINT32_T_SetType(&bridgeHeader->msg_id_, Msg_ID);
	APOLLO_BRIDGE_HeaderItem_BSIZE_SetType(&bridgeHeader->msg_size_, Msg_Size);
	APOLLO_BRIDGE_HeaderItem_UINT32_T_SetType(&bridgeHeader->total_frames_,
											  Msg_Frames);
	APOLLO_BRIDGE_HeaderItem_BSIZE_SetType(&bridgeHeader->frame_size_,
										   Frame_Size);
	APOLLO_BRIDGE_HeaderItem_BSIZE_SetType(&bridgeHeader->frame_pos_, Frame_Pos);
	APOLLO_BRIDGE_HeaderItem_UINT32_T_SetType(&bridgeHeader->index_, Frame_Index);
	APOLLO_BRIDGE_HeaderItem_DOUBLE_SetType(&bridgeHeader->time_stamp_,
											Time_Stamp);
	bridgeHeader->header_body_size_ = 0;
}

char* APOLLO_BRIDGE_HEADER_SerializeHeaderFlag(
		APOLLO_BRIDGE_HEADER* bridgeHeader, char* buf, size_t size)
{
	if(!buf || size == 0)
	{
		return NULL;
	}
	return APOLLO_BRIDGE_HEADER_SerializeBasicType_CHAR(
			bridgeHeader, BRIDGE_HEADER_FLAG, buf, size);
}

bool APOLLO_BRIDGE_HEADER_Diserialize(APOLLO_BRIDGE_HEADER* bridgeHeader,
									  const char* buf, size_t buf_size)
{
	const char* cursor = buf;
	
	int i = (int) (buf_size);
	uint32_t check_i = 0;
	
	while (i > 0)
	{
		HType type = UINT32_CAST(cursor);
		if(type >= Header_Tail || type < 0)
		{
			cursor += sizeof(HType) + 1;
			bsize size = UINT32_CAST(cursor);
			cursor += sizeof(bsize) + size + 2;
			i -= (int) (sizeof(HType) + sizeof(bsize) + size + 3);
			continue;
		}
		
		size_t value_size = 0;
		switch (type)
		{
			case Header_Ver:
				cursor = APOLLO_BRIDGE_HeaderItem_UINT32_T_DiserializeItem(
						&bridgeHeader->header_ver_, cursor, &value_size);
				break;
			case Msg_Name:
				cursor = APOLLO_BRIDGE_HeaderItem_STRING_DiserializeItem(
						&bridgeHeader->msg_name_, cursor, &value_size);
				break;
			case Msg_ID:
				cursor = APOLLO_BRIDGE_HeaderItem_UINT32_T_DiserializeItem(
						&bridgeHeader->msg_id_, cursor, &value_size);
				break;
			case Msg_Size:
				cursor = APOLLO_BRIDGE_HeaderItem_BSIZE_DiserializeItem(
						&bridgeHeader->msg_size_, cursor, &value_size);
				break;
			case Msg_Frames:
				cursor = APOLLO_BRIDGE_HeaderItem_UINT32_T_DiserializeItem(
						&bridgeHeader->total_frames_, cursor, &value_size);
				break;
			case Frame_Size:
				cursor = APOLLO_BRIDGE_HeaderItem_BSIZE_DiserializeItem(
						&bridgeHeader->frame_size_, cursor, &value_size);
				break;
			case Frame_Pos:
				cursor = APOLLO_BRIDGE_HeaderItem_BSIZE_DiserializeItem(
						&bridgeHeader->frame_pos_, cursor, &value_size);
				break;
			case Frame_Index:
				cursor = APOLLO_BRIDGE_HeaderItem_UINT32_T_DiserializeItem(
						&bridgeHeader->index_, cursor, &value_size);
				break;
			case Time_Stamp:
				cursor = APOLLO_BRIDGE_HeaderItem_DOUBLE_DiserializeItem(
						&bridgeHeader->time_stamp_, cursor, &value_size);
				break;
			
			default:
				break;
		}
		check_i = check_i + 1;
		i -= (int) (value_size);
	}
	return true;
}

bool APOLLO_BRIDGE_HEADER_IsAvailable(APOLLO_BRIDGE_HEADER* bridgeHeader,
									  const char* buf)
{
	if(!buf)
	{
		return false;
	}
	if(memcmp(BRIDGE_HEADER_FLAG, buf, sizeof(BRIDGE_HEADER_FLAG) - 1) != 0)
	{
		return false;
	}
	return true;
}

uint32_t APOLLO_BRIDGE_HEADER_GetHeaderVer(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->header_ver_.value_;
}

hsize APOLLO_BRIDGE_HEADER_GetHeaderSize(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return (hsize) (bridgeHeader->header_body_size_ + HEADER_FLAG_SIZE +
					sizeof(hsize) + 2);
}

bsize APOLLO_BRIDGE_HEADER_GetHeaderBodySize(
		APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->header_body_size_;
}

char* APOLLO_BRIDGE_HEADER_GetMsgName(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->msg_name_.value_;
}

uint32_t APOLLO_BRIDGE_HEADER_GetMsgID(APOLLO_BRIDGE_HEADER* brdidgeHeader)
{
	return brdidgeHeader->msg_id_.value_;
}

uint32_t APOLLO_BRIDGE_HEADER_GetTotalFrames(
		APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->total_frames_.value_;
}

uint32_t APOLLO_BRIDGE_HEADER_GetIndex(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->index_.value_;
}

double APOLLO_BRIDGE_HEADER_GetTimeStamp(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->time_stamp_.value_;
}

bsize APOLLO_BRIDGE_HEADER_GetMsgSize(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->msg_size_.value_;
}

bsize APOLLO_BRIDGE_HEADER_GetFrameSize(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->frame_size_.value_;
}

bsize APOLLO_BRIDGE_HEADER_GetFramePos(APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->frame_pos_.value_;
}

bsize APOLLO_BRIDGE_HEADER_GetFrameBufferSize(
		APOLLO_BRIDGE_HEADER* bridgeHeader)
{
	return bridgeHeader->frame_pos_.value_ +
		   APOLLO_BRIDGE_HEADER_GetHeaderSize(bridgeHeader);
}

void APOLLO_BRIDGE_HEADER_SetHeaderVer(APOLLO_BRIDGE_HEADER* bridgeHeader,
									   uint32_t header_ver)
{
	bridgeHeader->header_ver_.value_ = header_ver;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + sizeof(uint32_t));
}

void APOLLO_BRIDGE_HEADER_SetMsgName(APOLLO_BRIDGE_HEADER* bridgeHeader,
									 const char* msg_name)
{
	bridgeHeader->msg_name_.value_ = (char*) msg_name;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + strlen(msg_name) +
					 1);  // sizeof(msg_name)/sizeof(*msg_name)
}

void APOLLO_BRIDGE_HEADER_SetMsgID(APOLLO_BRIDGE_HEADER* bridgeHeader,
								   uint32_t msg_id)
{
	bridgeHeader->msg_id_.value_ = msg_id;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + sizeof(uint32_t));
}

void APOLLO_BRIDGE_HEADER_SetTotalFrames(APOLLO_BRIDGE_HEADER* bridgeHeader,
										 uint32_t total_frames)
{
	bridgeHeader->total_frames_.value_ = total_frames;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + sizeof(uint32_t));
}

void APOLLO_BRIDGE_HEADER_SetFrameSize(APOLLO_BRIDGE_HEADER* bridgeHeader,
									   bsize frame_size)
{
	bridgeHeader->frame_size_.value_ = frame_size;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + sizeof(bsize));
}

void APOLLO_BRIDGE_HEADER_SetFramePos(APOLLO_BRIDGE_HEADER* bridgeHeader,
									  bsize frame_pos)
{
	bridgeHeader->frame_pos_.value_ = frame_pos;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + sizeof(bsize));
}

void APOLLO_BRIDGE_HEADER_SetIndex(APOLLO_BRIDGE_HEADER* bridgeHeader,
								   uint32_t index)
{
	bridgeHeader->index_.value_ = index;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + sizeof(uint32_t));
}

void APOLLO_BRIDGE_HEADER_SetTimeStamp(APOLLO_BRIDGE_HEADER* bridgeHeader,
									   double time_stamp)
{
	bridgeHeader->time_stamp_.value_ = time_stamp;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + sizeof(double));
}

void APOLLO_BRIDGE_HEADER_SetMsgSize(APOLLO_BRIDGE_HEADER* bridgeHeader,
									 bsize msg_size)
{
	bridgeHeader->msg_size_.value_ = msg_size;
	bridgeHeader->header_body_size_ +=
			(hsize) (Item_Header_Size + 1 + sizeof(bsize));
}

char* APOLLO_BRIDGE_HEADER_SerializeBasicType_HSIZE(APOLLO_BRIDGE_HEADER* bridgeHeader, const hsize* value, char* buf, size_t size)
{
	size_t S = sizeof(hsize);
	if(!buf || size < S)
	{
		return NULL;
	}
	char* res = buf;
	memcpy(res, value, S);
	res[S] = '\n';
	res += S + 1;
	return res;
}

bool APOLLO_BRIDGE_HEADER_DiserializeBasicType_CHAR(APOLLO_BRIDGE_HEADER* bridgeHeader, char* value, const char* buf)
{
	size_t S = sizeof(BRIDGE_HEADER_FLAG);
	if(!buf)
	{
		return false;
	}
	char* temp;
	temp = malloc(S * sizeof(char));
	memcpy(temp, buf, S);
	*value = *((char*) temp);
	return true;
}

bool APOLLO_BRIDGE_HEADER_DiserializeBasicType_HSIZE(APOLLO_BRIDGE_HEADER* bridgeHeader, hsize* value, const char* buf)
{
	size_t S = sizeof(hsize);
	if(!buf)
	{
		return false;
	}
	char* temp;
	temp = malloc(S * sizeof(char));
	memcpy(temp, buf, S);
	*value = UINT32_CAST(temp);
	return true;
}

char* APOLLO_BRIDGE_HEADER_SerializeHeaderSize(APOLLO_BRIDGE_HEADER* bridgeHeader, char* buf, size_t size)
{
	hsize header_size = APOLLO_BRIDGE_HEADER_GetHeaderSize(bridgeHeader);
	return APOLLO_BRIDGE_HEADER_SerializeBasicType_HSIZE(bridgeHeader,
														 &header_size, buf, size);
}

bool APOLLO_BRIDGE_HEADER_Serialize(APOLLO_BRIDGE_HEADER* bridgeHeader, char* buf, size_t size)
{
	if(!buf || size == 0)
	{
		return false;
	}
	char* cursor = buf;
	char* p_header_size = NULL;
	cursor = APOLLO_BRIDGE_HEADER_SerializeHeaderFlag(bridgeHeader, cursor, size);
	p_header_size = cursor;
	cursor += sizeof(hsize) + 1;
	cursor = APOLLO_BRIDGE_HeaderItem_UINT32_T_SerializeItem(&bridgeHeader->header_ver_, cursor, size);  // header_ver_.SerializeItem(cursor, size);
	cursor = APOLLO_BRIDGE_HeaderItem_STRING_SerializeItem(&bridgeHeader->msg_name_, cursor, size);
	cursor = APOLLO_BRIDGE_HeaderItem_UINT32_T_SerializeItem(&bridgeHeader->msg_id_, cursor, size);
	cursor = APOLLO_BRIDGE_HeaderItem_BSIZE_SerializeItem(&bridgeHeader->msg_size_, cursor, size);
	cursor = APOLLO_BRIDGE_HeaderItem_UINT32_T_SerializeItem(&bridgeHeader->total_frames_, cursor, size);
	cursor = APOLLO_BRIDGE_HeaderItem_BSIZE_SerializeItem(&bridgeHeader->frame_size_, cursor, size);
	cursor = APOLLO_BRIDGE_HeaderItem_BSIZE_SerializeItem(&bridgeHeader->frame_pos_, cursor, size);
	cursor = APOLLO_BRIDGE_HeaderItem_UINT32_T_SerializeItem(&bridgeHeader->index_, cursor, size);
	cursor = APOLLO_BRIDGE_HeaderItem_DOUBLE_SerializeItem(&bridgeHeader->time_stamp_, cursor, size);
	
	if(!APOLLO_BRIDGE_HEADER_SerializeHeaderSize(bridgeHeader, p_header_size, size))
	{
		return false;
	}
	return true;
}
