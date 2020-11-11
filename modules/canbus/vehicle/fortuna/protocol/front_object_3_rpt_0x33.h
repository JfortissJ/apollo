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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace fortuna {

class FrontObject3rpt0x33 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  FrontObject3rpt0x33();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'CRC',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int checksum(const std::uint8_t* bytes, int32_t length) const;
  
  // config detail: {'name': 'RelVelocityX', 'offset': -128.0,
  // 'precision': 0.25, 'len': 10, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 12, 'type': 'double', 'order': 'intel', 'physical_unit':
  // ''}
  // Relative velocity of the nearest object ahead on right lane in direction of X.
  double rel_velocity_x(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RelPosY', 'offset': -32.0, 'precision': 0.0625,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 22,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  // Position of the nearest object (object center) ahead on right lane in 
  // direction of Y relative to the longitudinal axis of the ego vehicle.
  double rel_pos_y(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RelPosX', 'offset': 0.0, 'precision': 0.0625,
  // 'len': 12, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  // Position of the nearest object (closest point) ahead on right lane in 
  // direction of X relative to the rear axle of the ego vehicle. The distance 
  // is based on the current course of the ego vehicle.
  double rel_pos_x(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ID', 'offset': 0.0,
  // 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit':
  // ''}
  // ID of FrontObject
  int32_t id(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'FusedState', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 50, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  // State of radar and camera object fusion for FrontObject
  bool fused_state(const std::uint8_t* bytes, const int32_t length) const;

};

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
