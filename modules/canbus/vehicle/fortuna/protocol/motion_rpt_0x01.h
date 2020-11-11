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

class Motionrpt0x01 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Motionrpt0x01();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:

 // config detail: {'description': 'Motion checksum', 'offset': 0.0,
 // 'precision': 1.0, 'len': 8, 'name': 'motion_checksum',
 // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
 // 'int', 'order': 'intel', 'physical_unit': ''}
 int motion_checksum(const std::uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'YAW_RATE_SIGN', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit':
  // ''}
  bool yaw_rate_sign(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'YAW_RATE', 'offset': 0.0, 'precision': 0.01,
  // 'len': 14, 'is_signed_var': False, 'physical_range': '[0|16382]', 'bit': 16,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double yaw_rate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LONGITUDINAL _ACCELERATION', 'offset': -16, 
  // 'precision': 0.03125, 'len': 10, 'is_signed_var': False, 'physical_range': 
  //'[0|0]', 'bit': 30, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double longitudinal_acceleration(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LATERAL_ACCELERATION', 'offset': -1.27,
  // 'precision': 0.01, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[0|0]', 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit':
  // ''}
  double lateral_acceleration(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'VEHICLE_VELOCITY', 'offset': 0.0,
  // 'precision': 0.01, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double vehicle_velocity(const std::uint8_t* bytes,
                            const int32_t length) const;

};

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
