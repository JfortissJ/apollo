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
#ifndef APOLLO_BRIDGE_C_BUFFER_CHECKING_H
#define APOLLO_BRIDGE_C_BUFFER_CHECKING_H
#include "bridge_header.h"
#include "macro.h"
#include "modules/bridge_c/common/bridge_proto_diserialized_buf.h"

bool APOLLO_BRIDGE_Buffer(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER* bridgeHeader, const char* proto_message_buf);
bool APOLLO_BRIDGE_Header_Handle(APOLLO_BRIDGE_HEADER* bridgeHeader, const char* total_buf);
bool APOLLO_BRIDGE_DiserializeInit(BridgeProtoDiserializedBuf* bridgeProtoDiserializedBuf, APOLLO_BRIDGE_HEADER* bridgeHeader);
#endif
