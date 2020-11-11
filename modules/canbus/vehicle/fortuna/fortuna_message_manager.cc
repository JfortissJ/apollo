/******************************************************************************
 * Copyright 2020 fortiss GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus/vehicle/fortuna/fortuna_message_manager.h"

#include "modules/canbus/vehicle/fortuna/protocol/powertrain_1_rpt_0x04.h"
#include "modules/canbus/vehicle/fortuna/protocol/motion_rpt_0x01.h"
#include "modules/canbus/vehicle/fortuna/protocol/assistance_2_rpt_0xe.h"
#include "modules/canbus/vehicle/fortuna/protocol/steering_rpt_0x02.h"
#include "modules/canbus/vehicle/fortuna/protocol/front_object_1_rpt_0x31.h"
#include "modules/canbus/vehicle/fortuna/protocol/front_object_2_rpt_0x32.h"
#include "modules/canbus/vehicle/fortuna/protocol/front_object_3_rpt_0x33.h"
#include "modules/canbus/vehicle/fortuna/protocol/front_object_4_rpt_0x34.h"
#include "modules/canbus/vehicle/fortuna/protocol/rear_object_1_rpt_0x41.h"
#include "modules/canbus/vehicle/fortuna/protocol/rear_object_2_rpt_0x42.h"
#include "modules/canbus/vehicle/fortuna/protocol/rear_object_3_rpt_0x43.h"
#include "modules/canbus/vehicle/fortuna/protocol/rear_object_4_rpt_0x44.h"
#include "modules/canbus/vehicle/fortuna/protocol/rear_object_5_rpt_0x45.h"
#include "modules/canbus/vehicle/fortuna/protocol/rear_object_6_rpt_0x46.h"

namespace apollo {
namespace canbus {
namespace fortuna {

FortunaMessageManager::FortunaMessageManager() {
  // Control Messages

  // Report Messages
  // Add Protocols in vector of ProtocolData to be received
  // The MessageManager Parse method calls the GetMutableDataByID method. 
  // The latter calls the Parse method of each protocol in order to
  // read in the CANBUS data and set the correspodning Chassis (detail) values
  AddRecvProtocolData<Assistance2rpt0xe, true>(); 
  AddRecvProtocolData<Motionrpt0x01, true>();
  AddRecvProtocolData<Steeringrpt0x02, true>();
  AddRecvProtocolData<Powertrain1rpt0x04, true>();
  AddRecvProtocolData<FrontObject1rpt0x31, true>();
  AddRecvProtocolData<FrontObject2rpt0x32, true>();
  AddRecvProtocolData<FrontObject3rpt0x33, true>();
  AddRecvProtocolData<FrontObject4rpt0x34, true>();
  AddRecvProtocolData<RearObject1rpt0x41, true>();
  AddRecvProtocolData<RearObject2rpt0x42, true>();
  AddRecvProtocolData<RearObject3rpt0x43, true>();
  AddRecvProtocolData<RearObject4rpt0x44, true>();
  AddRecvProtocolData<RearObject5rpt0x45, true>();
  AddRecvProtocolData<RearObject6rpt0x46, true>();
}

FortunaMessageManager::~FortunaMessageManager() {}

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
