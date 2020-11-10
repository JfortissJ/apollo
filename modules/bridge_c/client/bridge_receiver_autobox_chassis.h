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
#include "modules/bridge_C/common/bridge_proto_diserialized_buf.h"
#include "modules/bridge_c/common/macro.h"
#include "protoc_genfiles/modules/autobox_bridge/proto/autobox_chassis.pb-c.h"
#include "fortiss_definitions.h"

Apollo__Canbus__ChassisToAutoboxBridge* handle_message_chassis(BridgeProtoDiserializedBuf* proto_buf);

#endif
