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
#ifndef MODULES_BRIDGE_C_COMMON_HEADER_ITEM_H
#define MODULES_BRIDGE_C_COMMON_HEADER_ITEM_H

#include <string.h>

#ifdef linux

#include <unistd.h>

#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef _DS1401 //dspace
#include <brtenv.h>
#include <dstypes.h>
typedef UInt32 uint32_t; 
typedef UInt8 uint8_t;
#else

#include <stdint.h>

#endif

#include <stddef.h>
#include "macro.h"

typedef uint32_t bsize;
typedef struct _APOLLO_BRIDGE_HeaderItem_STRING APOLLO_BRIDGE_HeaderItem_STRING;
typedef struct _APOLLO_BRIDGE_HeaderItem_UINT32_T APOLLO_BRIDGE_HeaderItem_UINT32_T;
typedef struct _APOLLO_BRIDGE_HeaderItem_BSIZE APOLLO_BRIDGE_HeaderItem_BSIZE;
typedef struct _APOLLO_BRIDGE_HeaderItem_DOUBLE APOLLO_BRIDGE_HeaderItem_DOUBLE;
typedef enum _HType
{
	Header_Ver,
	Msg_Name,
	Msg_ID,
	Msg_Size,
	Msg_Frames,
	Frame_Size,
	Frame_Pos,
	Frame_Index,
	Time_Stamp,
	Header_Tail,
} HType;

struct _APOLLO_BRIDGE_HeaderItem_STRING
{
	HType hytpe;
	char* value_;
};
struct _APOLLO_BRIDGE_HeaderItem_UINT32_T
{
	HType hytpe;
	uint32_t value_;
};
struct _APOLLO_BRIDGE_HeaderItem_BSIZE
{
	HType hytpe;
	bsize value_;
};
struct _APOLLO_BRIDGE_HeaderItem_DOUBLE
{
	HType hytpe;
	double value_;
};

void APOLLO_BRIDGE_HeaderItem_STRING_SetValue(APOLLO_BRIDGE_HeaderItem_STRING* msg, const char* buf);

size_t APOLLO_BRIDGE_HeaderItem_STRING_ValueSize(APOLLO_BRIDGE_HeaderItem_STRING* msg);

HType APOLLO_BRIDGE_HeaderItem_STRING_GetType(APOLLO_BRIDGE_HeaderItem_STRING* msg);

void APOLLO_BRIDGE_HeaderItem_STRING_SetType(APOLLO_BRIDGE_HeaderItem_STRING* msg, HType t);

char* APOLLO_BRIDGE_HeaderItem_STRING_GetValuePtr(APOLLO_BRIDGE_HeaderItem_STRING* msg);

char* APOLLO_BRIDGE_HeaderItem_STRING_SerializeItem(APOLLO_BRIDGE_HeaderItem_STRING* msg, char* buf, size_t buf_size);


const char* APOLLO_BRIDGE_HeaderItem_STRING_DiserializeItem(APOLLO_BRIDGE_HeaderItem_STRING* msg, const char* buf, size_t* diserialized_size);


void APOLLO_BRIDGE_HeaderItem_UINT32_T_SetValue(APOLLO_BRIDGE_HeaderItem_UINT32_T* msg, const char* buf);

size_t APOLLO_BRIDGE_HeaderItem_UINT32_T_ValueSize(APOLLO_BRIDGE_HeaderItem_UINT32_T* msg);

HType APOLLO_BRIDGE_HeaderItem_UINT32_T_GetType(APOLLO_BRIDGE_HeaderItem_UINT32_T* msg);

void APOLLO_BRIDGE_HeaderItem_UINT32_T_SetType(APOLLO_BRIDGE_HeaderItem_UINT32_T* msg, HType t);

const uint32_t* APOLLO_BRIDGE_HeaderItem_UINT32_T_GetValuePtr(APOLLO_BRIDGE_HeaderItem_UINT32_T* msg);

char* APOLLO_BRIDGE_HeaderItem_UINT32_T_SerializeItem(APOLLO_BRIDGE_HeaderItem_UINT32_T* msg, char* buf, size_t buf_size);


const char* APOLLO_BRIDGE_HeaderItem_UINT32_T_DiserializeItem(APOLLO_BRIDGE_HeaderItem_UINT32_T* msg, const char* buf, size_t* diserialized_size);


void APOLLO_BRIDGE_HeaderItem_BSIZE_SetValue(APOLLO_BRIDGE_HeaderItem_BSIZE* msg, const char* buf);

size_t APOLLO_BRIDGE_HeaderItem_BSIZE_ValueSize(APOLLO_BRIDGE_HeaderItem_BSIZE* msg);

HType APOLLO_BRIDGE_HeaderItem_BSIZE_GetType(APOLLO_BRIDGE_HeaderItem_BSIZE* msg);

void APOLLO_BRIDGE_HeaderItem_BSIZE_SetType(APOLLO_BRIDGE_HeaderItem_BSIZE* msg, HType t);

const bsize* APOLLO_BRIDGE_HeaderItem_BSIZE_GetValuePtr(APOLLO_BRIDGE_HeaderItem_BSIZE* msg);

char* APOLLO_BRIDGE_HeaderItem_BSIZE_SerializeItem(APOLLO_BRIDGE_HeaderItem_BSIZE* msg, char* buf, size_t buf_size);


const char* APOLLO_BRIDGE_HeaderItem_BSIZE_DiserializeItem(APOLLO_BRIDGE_HeaderItem_BSIZE* msg, const char* buf, size_t* diserialized_size);


void APOLLO_BRIDGE_HeaderItem_DOUBLE_SetValue(APOLLO_BRIDGE_HeaderItem_DOUBLE* msg, const char* buf);

size_t APOLLO_BRIDGE_HeaderItem_DOUBLE_ValueSize(APOLLO_BRIDGE_HeaderItem_DOUBLE* msg);

HType APOLLO_BRIDGE_HeaderItem_DOUBLE_GetType(APOLLO_BRIDGE_HeaderItem_DOUBLE* msg);

void APOLLO_BRIDGE_HeaderItem_DOUBLE_SetType(APOLLO_BRIDGE_HeaderItem_DOUBLE* msg, HType t);

const double* APOLLO_BRIDGE_HeaderItem_DOUBLE_GetValuePtr(APOLLO_BRIDGE_HeaderItem_DOUBLE* msg);

char* APOLLO_BRIDGE_HeaderItem_DOUBLE_SerializeItem(APOLLO_BRIDGE_HeaderItem_DOUBLE* msg, char* buf, size_t buf_size);

const char* APOLLO_BRIDGE_HeaderItem_DOUBLE_DiserializeItem(APOLLO_BRIDGE_HeaderItem_DOUBLE* msg, const char* buf, size_t* diserialized_size);


#endif
