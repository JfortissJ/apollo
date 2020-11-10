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
#ifndef MODULES_BRIDEG_C_COMMON_HEARDER_H
#define MODULES_BRIDEG_C_COMMON_HEARDER_H

#include "bridge_header_item.h"
#include <stdbool.h>
#include <string.h>
#include "macro.h"

typedef uint32_t hsize;
static const char BRIDGE_HEADER_FLAG[] = "ApolloBridgeHeader";
static const size_t HEADER_FLAG_SIZE = sizeof(BRIDGE_HEADER_FLAG);
static const size_t Item_Header_Size = sizeof(HType) + sizeof(bsize) + 2;
typedef struct _APOLLO_BRIDGE_HEADER APOLLO_BRIDGE_HEADER;
struct _APOLLO_BRIDGE_HEADER
{
	APOLLO_BRIDGE_HeaderItem_UINT32_T header_ver_;
	APOLLO_BRIDGE_HeaderItem_STRING msg_name_;
	APOLLO_BRIDGE_HeaderItem_UINT32_T msg_id_;
	APOLLO_BRIDGE_HeaderItem_BSIZE msg_size_; // the total protobuf_size
	APOLLO_BRIDGE_HeaderItem_UINT32_T total_frames_;
	APOLLO_BRIDGE_HeaderItem_BSIZE frame_size_; // each frame size
	APOLLO_BRIDGE_HeaderItem_BSIZE frame_pos_;
	APOLLO_BRIDGE_HeaderItem_UINT32_T index_;
	APOLLO_BRIDGE_HeaderItem_DOUBLE time_stamp_;
	hsize header_body_size_;
	hsize header_size;
};

char* APOLLO_BRIDGE_HEADER_SerializeBasicType_CHAR(APOLLO_BRIDGE_HEADER* bridgeHeader, const char* value, char* buf, size_t size);

void APOLLO_BRIDGE_HEADER_Item_Init(APOLLO_BRIDGE_HEADER* bridgeHeader);

char* APOLLO_BRIDGE_HEADER_SerializeHeaderFlag(APOLLO_BRIDGE_HEADER* bridgeHeader, char* buf, size_t size);

bool APOLLO_BRIDGE_HEADER_Diserialize(APOLLO_BRIDGE_HEADER* bridgeHeader, const char* buf, size_t buf_size);


bool APOLLO_BRIDGE_HEADER_IsAvailable(APOLLO_BRIDGE_HEADER* bridgeHeader, const char* buf);

uint32_t APOLLO_BRIDGE_HEADER_GetHeaderVer(APOLLO_BRIDGE_HEADER* bridgeHeader);

hsize APOLLO_BRIDGE_HEADER_GetHeaderSize(APOLLO_BRIDGE_HEADER* bridgeHeader);

bsize APOLLO_BRIDGE_HEADER_GetHeaderBodySize(APOLLO_BRIDGE_HEADER* bridgeHeader);

char* APOLLO_BRIDGE_HEADER_GetMsgName(APOLLO_BRIDGE_HEADER* bridgeHeader);

uint32_t APOLLO_BRIDGE_HEADER_GetMsgID(APOLLO_BRIDGE_HEADER* brdidgeHeader);

uint32_t APOLLO_BRIDGE_HEADER_GetTotalFrames(APOLLO_BRIDGE_HEADER* bridgeHeader);

uint32_t APOLLO_BRIDGE_HEADER_GetIndex(APOLLO_BRIDGE_HEADER* bridgeHeader);

double APOLLO_BRIDGE_HEADER_GetTimeStamp(APOLLO_BRIDGE_HEADER* bridgeHeader);

bsize APOLLO_BRIDGE_HEADER_GetMsgSize(APOLLO_BRIDGE_HEADER* bridgeHeader);

bsize APOLLO_BRIDGE_HEADER_GetFrameSize(APOLLO_BRIDGE_HEADER* bridgeHeader);

bsize APOLLO_BRIDGE_HEADER_GetFramePos(APOLLO_BRIDGE_HEADER* bridgeHeader);

bsize APOLLO_BRIDGE_HEADER_GetFrameBufferSize(APOLLO_BRIDGE_HEADER* bridgeHeader);


void APOLLO_BRIDGE_HEADER_SetHeaderVer(APOLLO_BRIDGE_HEADER* bridgeHeader, uint32_t header_ver);


void APOLLO_BRIDGE_HEADER_SetMsgName(APOLLO_BRIDGE_HEADER* bridgeHeader, const char* msg_name);

void APOLLO_BRIDGE_HEADER_SetMsgID(APOLLO_BRIDGE_HEADER* bridgeHeader, uint32_t msg_id);

void APOLLO_BRIDGE_HEADER_SetTotalFrames(APOLLO_BRIDGE_HEADER* bridgeHeader, uint32_t total_frames);


void APOLLO_BRIDGE_HEADER_SetFrameSize(APOLLO_BRIDGE_HEADER* bridgeHeader, bsize frame_size);


void APOLLO_BRIDGE_HEADER_SetFramePos(APOLLO_BRIDGE_HEADER* bridgeHeader, bsize frame_pos);

void APOLLO_BRIDGE_HEADER_SetIndex(APOLLO_BRIDGE_HEADER* bridgeHeader, uint32_t index);

void APOLLO_BRIDGE_HEADER_SetTimeStamp(APOLLO_BRIDGE_HEADER* bridgeHeader, double time_stamp);

void APOLLO_BRIDGE_HEADER_SetMsgSize(APOLLO_BRIDGE_HEADER* bridgeHeader, bsize msg_size);


char* APOLLO_BRIDGE_HEADER_SerializeBasicType_HSIZE(APOLLO_BRIDGE_HEADER* bridgeHeader, const hsize* value, char* buf, size_t size);

bool APOLLO_BRIDGE_HEADER_DiserializeBasicType_CHAR(APOLLO_BRIDGE_HEADER* bridgeHeader, char* value, const char* buf);

bool APOLLO_BRIDGE_HEADER_DiserializeBasicType_HSIZE(APOLLO_BRIDGE_HEADER* bridgeHeader, hsize* value, const char* buf);


char* APOLLO_BRIDGE_HEADER_SerializeHeaderSize(APOLLO_BRIDGE_HEADER* bridgeHeader, char* buf, size_t size);


bool APOLLO_BRIDGE_HEADER_Serialize(APOLLO_BRIDGE_HEADER* bridgeHeader, char* buf, size_t size);


#endif
