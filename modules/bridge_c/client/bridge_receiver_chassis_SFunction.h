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
#ifndef MODULES_BRIDGE_C_CLIENT_CHASSIS_SFUNCTION_H
#define MODULES_BRIDGE_C_CLIENT_CHASSIS_SFUNCTION_H

#include "modules/bridge_c/common/bridge_buffer.h"
#include "modules/bridge_c/common/bridge_header.h"
#include "modules/bridge_c/common/bridge_proto_diserialized_buf_stanadard.h"
#include "modules/bridge_c/common/macro.h"
#include "protoc_genfiles/modules/canbus/proto/chassis.pb-c.h"

Apollo__Canbus__Chassis* handle_message(const char* total_buf)
{
	/* char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1];
	for (size_t j = 0; j < sizeof(BRIDGE_HEADER_FLAG) + 1; ++j) {
	  header_flag[j] = 0;
	}
	size_t offset = 0;
	memcpy(header_flag, total_buf, HEADER_FLAG_SIZE);
	if (strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0) {
	  return NULL;
	}
	offset += sizeof(BRIDGE_HEADER_FLAG) + 1;
	char header_size_buf[sizeof(hsize) + 1];
	for (size_t k = 0; k < sizeof(hsize) + 1; ++k) {
	  header_size_buf[k] = 0;
	}
	const char* cursor = total_buf + offset;
	memcpy(header_size_buf, cursor, sizeof(hsize));
	hsize header_size = *((hsize*)(header_size_buf));
	if (header_size > APOLLO_BRIDGE_MARCO_FRAME_SIZE) {
	  return NULL;
	}
	offset += sizeof(hsize) + 1;
	APOLLO_BRIDGE_HEADER header;
	APOLLO_BRIDGE_HEADER_Item_Init(&header);
 
	size_t buf_size = header_size - offset;
	cursor = total_buf + offset;
	if (!APOLLO_BRIDGE_HEADER_Diserialize(&header, cursor, buf_size)) {
	  return NULL;
	}
	  bool hasInit = BridgeProto_Diserialized_Buf_Initialize(&proto_buf, header);
	if (!hasInit) {
	  return NULL;
	}
	cursor = total_buf + header_size;
	char* buf = BridgeProto_Diserialized_Buf_GetBuf(
		&proto_buf, APOLLO_BRIDGE_HEADER_GetFramePos(&header));
  
	memcpy(buf, cursor, APOLLO_BRIDGE_HEADER_GetFrameSize(&header));
	BridgeProto_Diserialized_Buf_UpdateStatus(
		&proto_buf, APOLLO_BRIDGE_HEADER_GetIndex(&header));
	 */
	
	BridgeProtoDiserializedBuf proto_buf;
	bool hasValid = APOLLO_BRIDGE_Buffer(&proto_buf, total_buf);
	if(!hasValid)
	{
		return NULL;
	}
	if(BridgeProto_Diserialized_Buf_IsReadyDiserialize(&proto_buf))
	{
		Apollo__Canbus__Chassis* apollo_canbus_chasis_msg = NULL;
		apollo_canbus_chasis_msg = apollo__canbus__chassis__unpack(
				NULL, proto_buf.total_size_, (uint8_t*) (proto_buf.proto_buf_));
		if(apollo_canbus_chasis_msg == NULL)
		{
			return NULL;
		}
		else
		{
			return apollo_canbus_chasis_msg;
		}
	}
}

#endif
