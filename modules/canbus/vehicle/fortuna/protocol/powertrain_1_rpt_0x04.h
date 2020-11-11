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

class Powertrain1rpt0x04 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Powertrain1rpt0x04();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'Powertrain 1 checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'powertrain1_checksum',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int powertrain1_checksum(const std::uint8_t* bytes, int32_t length) const;
  
  // config detail: {'name': 'GEAR', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 12, 'type': 'int', 'order': 'intel', 'physical_unit':
  // ''}
  int32_t gear(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'THROTTLE_PEDAL_POSITION', 'offset': 0.0, 'precision': 0.4,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 16,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double throttle_pedal_position(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ENGINE_SPEED', 'offset': 0.0, 'precision': 0.25,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double engine_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ACTUAL_TORQUE_ENGINE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit':
  // ''}
  int32_t actual_torque_engine(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'name': 'ENGINE_RUNNING', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 51, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool engine_running(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'name': 'REQUESTED_TORQUE_ENGINE', 'offset': -1527.0,
  // 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 52, 'type': 'double', 'order': 'intel', 'physical_unit':
  // ''}
  double requested_torque_engine(const std::uint8_t* bytes,
                                const int32_t length) const;

};

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
