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
#include "bridge_header_item.h"
#include "fortiss_definitions.h"

void APOLLO_BRIDGE_HeaderItem_STRING_SetValue(
		APOLLO_BRIDGE_HeaderItem_STRING* msg, const char* buf)
{
	if(!buf)
	{
		return;
	}
	msg->value_ = (char*) buf;
}

size_t APOLLO_BRIDGE_HeaderItem_STRING_ValueSize(
		APOLLO_BRIDGE_HeaderItem_STRING* msg)
{
	return strlen(msg->value_);
}

HType APOLLO_BRIDGE_HeaderItem_STRING_GetType(APOLLO_BRIDGE_HeaderItem_STRING* msg)
{
	return msg->hytpe;
}

void APOLLO_BRIDGE_HeaderItem_STRING_SetType(APOLLO_BRIDGE_HeaderItem_STRING* msg, HType t)
{
	msg->hytpe = t;
}

char* APOLLO_BRIDGE_HeaderItem_STRING_GetValuePtr(APOLLO_BRIDGE_HeaderItem_STRING* msg)
{
	return msg->value_;
}

char* APOLLO_BRIDGE_HeaderItem_STRING_SerializeItem(APOLLO_BRIDGE_HeaderItem_STRING* msg, char* buf, size_t buf_size)
{
	if(!buf || buf_size == 0 ||
	   buf_size < (size_t) (sizeof(msg->hytpe) + APOLLO_BRIDGE_HeaderItem_STRING_ValueSize(msg) + 3))
	{
		return NULL;
	}
	char* res = buf;
	size_t item_size = APOLLO_BRIDGE_HeaderItem_STRING_ValueSize(msg);
	
	memcpy(res, &msg->hytpe, sizeof(HType));
	res[sizeof(HType)] = ':';
	res = res + sizeof(HType) + 1;
	
	memcpy(res, &item_size, sizeof(size_t));
	res[sizeof(bsize)] = ':';
	res = res + sizeof(bsize) + 1;
	
	memcpy(res, APOLLO_BRIDGE_HeaderItem_STRING_GetValuePtr(msg),
		   APOLLO_BRIDGE_HeaderItem_STRING_ValueSize(msg));
	res[APOLLO_BRIDGE_HeaderItem_STRING_ValueSize(msg)] = '\n';
	res += APOLLO_BRIDGE_HeaderItem_STRING_ValueSize(msg) + 1;
	return res;
}

const char* APOLLO_BRIDGE_HeaderItem_STRING_DiserializeItem(APOLLO_BRIDGE_HeaderItem_STRING* msg, const char* buf, size_t* diserialized_size)
{
	if(!buf || !diserialized_size)
	{
		return NULL;
	}
	const char* res = buf;
	
	char p_type[sizeof(HType)] = {0};
	memcpy(p_type, buf, sizeof(HType));
	// HType type = *((HType*)(p_type));
	HType type = UINT32_CAST(p_type);
	if(type != msg->hytpe)
	{
		return NULL;
	}
	res += sizeof(HType) + 1;
	*diserialized_size += sizeof(HType) + 1;
	
	char p_size[sizeof(bsize)] = {0};
	memcpy(p_size, res, sizeof(bsize));
	// bsize size = *((bsize*) p_size);
	bsize size = UINT32_CAST(p_size);
	res += sizeof(bsize) + 1;
	*diserialized_size += sizeof(bsize) + 1;
	
	APOLLO_BRIDGE_HeaderItem_STRING_SetValue(msg, res);
	res += size + 1;
	*diserialized_size += size + 1;
	return res;
}

void APOLLO_BRIDGE_HeaderItem_UINT32_T_SetValue(
		APOLLO_BRIDGE_HeaderItem_UINT32_T* msg, const char* buf)
{
	if(!buf)
	{
		return;
	}
	// msg->value_ = *((const uint32_t*) buf);
	msg->value_ = UINT32_CAST(buf);
}

size_t APOLLO_BRIDGE_HeaderItem_UINT32_T_ValueSize(
		APOLLO_BRIDGE_HeaderItem_UINT32_T* msg)
{
	return sizeof(msg->value_);
}

HType APOLLO_BRIDGE_HeaderItem_UINT32_T_GetType(
		APOLLO_BRIDGE_HeaderItem_UINT32_T* msg)
{
	return msg->hytpe;
}

void APOLLO_BRIDGE_HeaderItem_UINT32_T_SetType(
		APOLLO_BRIDGE_HeaderItem_UINT32_T* msg, HType t)
{
	msg->hytpe = t;
}

const uint32_t* APOLLO_BRIDGE_HeaderItem_UINT32_T_GetValuePtr(
		APOLLO_BRIDGE_HeaderItem_UINT32_T* msg)
{
	return &(msg->value_);
}

char* APOLLO_BRIDGE_HeaderItem_UINT32_T_SerializeItem(
		APOLLO_BRIDGE_HeaderItem_UINT32_T* msg, char* buf, size_t buf_size)
{
	if(!buf || buf_size == 0 ||
	   buf_size <
	   (size_t) (sizeof(msg->hytpe) +
				 APOLLO_BRIDGE_HeaderItem_UINT32_T_ValueSize(msg) + 3))
	{
		return NULL;
	}
	char* res = buf;
	size_t item_size = APOLLO_BRIDGE_HeaderItem_UINT32_T_ValueSize(msg);
	
	memcpy(res, &msg->hytpe, sizeof(HType));
	res[sizeof(HType)] = ':';
	res = res + sizeof(HType) + 1;
	
	memcpy(res, &item_size, sizeof(size_t));
	res[sizeof(bsize)] = ':';
	res = res + sizeof(bsize) + 1;
	
	memcpy(res, APOLLO_BRIDGE_HeaderItem_UINT32_T_GetValuePtr(msg),
		   APOLLO_BRIDGE_HeaderItem_UINT32_T_ValueSize(msg));
	res[APOLLO_BRIDGE_HeaderItem_UINT32_T_ValueSize(msg)] = '\n';
	res += APOLLO_BRIDGE_HeaderItem_UINT32_T_ValueSize(msg) + 1;
	return res;
}

const char* APOLLO_BRIDGE_HeaderItem_UINT32_T_DiserializeItem(APOLLO_BRIDGE_HeaderItem_UINT32_T* msg, const char* buf, size_t* diserialized_size)
{
	if(!buf || !diserialized_size)
	{
		return NULL;
	}
	const char* res = buf;
	
	char p_type[sizeof(HType)] = {0};
	memcpy(p_type, buf, sizeof(HType));
	// HType type = *((HType*)p_type);
	HType type = UINT32_CAST(p_type);
	if(type != msg->hytpe)
	{
		return NULL;
	}
	res += sizeof(HType) + 1;
	*diserialized_size += sizeof(HType) + 1;
	char p_size[sizeof(bsize)] = {0};
	memcpy(p_size, res, sizeof(bsize));
	// bsize size = *((bsize*) (p_size));
	bsize size = UINT32_CAST(p_size);
	res += sizeof(bsize) + 1;
	*diserialized_size += sizeof(bsize) + 1;
	APOLLO_BRIDGE_HeaderItem_UINT32_T_SetValue(msg, res);
	res += size + 1;
	*diserialized_size += size + 1;
	return res;
}

void APOLLO_BRIDGE_HeaderItem_BSIZE_SetValue(
		APOLLO_BRIDGE_HeaderItem_BSIZE* msg, const char* buf)
{
	if(!buf)
	{
		return;
	}
	// msg->value_ = *((const bsize*) buf);
	msg->value_ = UINT32_CAST(buf);
}

size_t APOLLO_BRIDGE_HeaderItem_BSIZE_ValueSize(
		APOLLO_BRIDGE_HeaderItem_BSIZE* msg)
{
	return sizeof(msg->value_);
}

HType APOLLO_BRIDGE_HeaderItem_BSIZE_GetType(
		APOLLO_BRIDGE_HeaderItem_BSIZE* msg)
{
	return msg->hytpe;
}

void APOLLO_BRIDGE_HeaderItem_BSIZE_SetType(APOLLO_BRIDGE_HeaderItem_BSIZE* msg,
											HType t)
{
	msg->hytpe = t;
}

const bsize* APOLLO_BRIDGE_HeaderItem_BSIZE_GetValuePtr(
		APOLLO_BRIDGE_HeaderItem_BSIZE* msg)
{
	return &(msg->value_);
}

char* APOLLO_BRIDGE_HeaderItem_BSIZE_SerializeItem(
		APOLLO_BRIDGE_HeaderItem_BSIZE* msg, char* buf, size_t buf_size)
{
	if(!buf || buf_size == 0 ||
	   buf_size < (size_t) (sizeof(msg->hytpe) +
							APOLLO_BRIDGE_HeaderItem_BSIZE_ValueSize(msg) + 3))
	{
		return NULL;
	}
	char* res = buf;
	size_t item_size = APOLLO_BRIDGE_HeaderItem_BSIZE_ValueSize(msg);
	
	memcpy(res, &msg->hytpe, sizeof(HType));
	res[sizeof(HType)] = ':';
	res = res + sizeof(HType) + 1;
	
	memcpy(res, &item_size, sizeof(size_t));
	res[sizeof(bsize)] = ':';
	res = res + sizeof(bsize) + 1;
	
	memcpy(res, APOLLO_BRIDGE_HeaderItem_BSIZE_GetValuePtr(msg),
		   APOLLO_BRIDGE_HeaderItem_BSIZE_ValueSize(msg));
	res[APOLLO_BRIDGE_HeaderItem_BSIZE_ValueSize(msg)] = '\n';
	res += APOLLO_BRIDGE_HeaderItem_BSIZE_ValueSize(msg) + 1;
	return res;
}

const char* APOLLO_BRIDGE_HeaderItem_BSIZE_DiserializeItem(
		APOLLO_BRIDGE_HeaderItem_BSIZE* msg, const char* buf,
		size_t* diserialized_size)
{
	if(!buf || !diserialized_size)
	{
		return NULL;
	}
	const char* res = buf;
	
	char p_type[sizeof(HType)] = {0};
	memcpy(p_type, buf, sizeof(HType));
	// HType type = *((HType*)p_type);
	HType type = UINT32_CAST(p_type);
	if(type != msg->hytpe)
	{
		return NULL;
	}
	res += sizeof(HType) + 1;
	*diserialized_size += sizeof(HType) + 1;
	
	char p_size[sizeof(bsize)] = {0};
	memcpy(p_size, res, sizeof(bsize));
	// bsize size = *((bsize*) (p_size));
	bsize size = UINT32_CAST(p_size);
	res += sizeof(bsize) + 1;
	*diserialized_size += sizeof(bsize) + 1;
	APOLLO_BRIDGE_HeaderItem_BSIZE_SetValue(msg, res);
	res += size + 1;
	*diserialized_size += size + 1;
	return res;
}

void APOLLO_BRIDGE_HeaderItem_DOUBLE_SetValue(
		APOLLO_BRIDGE_HeaderItem_DOUBLE* msg, const char* buf)
{
	if(!buf)
	{
		return;
	}
	// msg->value_ = *((const double*)buf);
	double swapvalue;
	DOUBLE_CAST(swapvalue, buf);
	msg->value_ = swapvalue;
}

size_t APOLLO_BRIDGE_HeaderItem_DOUBLE_ValueSize(
		APOLLO_BRIDGE_HeaderItem_DOUBLE* msg)
{
	return sizeof(msg->value_);
}

HType APOLLO_BRIDGE_HeaderItem_DOUBLE_GetType(
		APOLLO_BRIDGE_HeaderItem_DOUBLE* msg)
{
	return msg->hytpe;
}

void APOLLO_BRIDGE_HeaderItem_DOUBLE_SetType(
		APOLLO_BRIDGE_HeaderItem_DOUBLE* msg, HType t)
{
	msg->hytpe = t;
}

const double* APOLLO_BRIDGE_HeaderItem_DOUBLE_GetValuePtr(
		APOLLO_BRIDGE_HeaderItem_DOUBLE* msg)
{
	return &(msg->value_);
}

char* APOLLO_BRIDGE_HeaderItem_DOUBLE_SerializeItem(
		APOLLO_BRIDGE_HeaderItem_DOUBLE* msg, char* buf, size_t buf_size)
{
	if(!buf || buf_size == 0 ||
	   buf_size < (size_t) (sizeof(msg->hytpe) +
							APOLLO_BRIDGE_HeaderItem_DOUBLE_ValueSize(msg) + 3))
	{
		return NULL;
	}
	char* res = buf;
	size_t item_size = APOLLO_BRIDGE_HeaderItem_DOUBLE_ValueSize(msg);
	
	memcpy(res, &msg->hytpe, sizeof(HType));
	res[sizeof(HType)] = ':';
	res = res + sizeof(HType) + 1;
	
	memcpy(res, &item_size, sizeof(size_t));
	res[sizeof(bsize)] = ':';
	res = res + sizeof(bsize) + 1;
	
	memcpy(res, APOLLO_BRIDGE_HeaderItem_DOUBLE_GetValuePtr(msg),
		   APOLLO_BRIDGE_HeaderItem_DOUBLE_ValueSize(msg));
	res[APOLLO_BRIDGE_HeaderItem_DOUBLE_ValueSize(msg)] = '\n';
	res += APOLLO_BRIDGE_HeaderItem_DOUBLE_ValueSize(msg) + 1;
	return res;
}

const char* APOLLO_BRIDGE_HeaderItem_DOUBLE_DiserializeItem(
		APOLLO_BRIDGE_HeaderItem_DOUBLE* msg, const char* buf,
		size_t* diserialized_size)
{
	if(!buf || !diserialized_size)
	{
		return NULL;
	}
	const char* res = buf;
	
	char p_type[sizeof(HType)] = {0};
	memcpy(p_type, buf, sizeof(HType));
	// HType type = *((HType*)p_type);
	HType type = UINT32_CAST(p_type);
	if(type != msg->hytpe)
	{
		return NULL;
	}
	res += sizeof(HType) + 1;
	*diserialized_size += sizeof(HType) + 1;
	
	char p_size[sizeof(bsize)] = {0};
	memcpy(p_size, res, sizeof(bsize));
	// bsize size = *((bsize*) (p_size));
	bsize size = UINT32_CAST(p_size);
	res += sizeof(bsize) + 1;
	*diserialized_size += sizeof(bsize) + 1;
	
	APOLLO_BRIDGE_HeaderItem_DOUBLE_SetValue(msg, res);
	res += size + 1;
	*diserialized_size += size + 1;
	return res;
}
