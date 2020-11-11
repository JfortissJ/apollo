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

class Steeringrpt0x02 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Steeringrpt0x02();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  
  // config detail: {'description': 'checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'CRC',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int checksum(const std::uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'SteeringWheelAngleSign', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool steering_wheel_angle_sign(const std::uint8_t* bytes, const int32_t length) const;
  
  // config detail: {'name': 'SteeringWheelSpeedSign', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool steering_wheel_speed_sign(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SteeringWheelTorqueSign', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool steering_wheel_torque_sign(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SteeringWheelTorque', 'offset': 0.0,
  // 'precision': 0.01, 'len': 10, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 15, 'type': 'double', 'order': 'intel', 'physical_unit':
  // ''}
  double steering_wheel_torque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SteeringWheelAngle', 'offset': 0.0,
  // 'precision': 0.1, 'len': 13, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 25, 'type': 'double', 'order': 'intel', 'physical_unit':
  // ''}
  double steering_wheel_angle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SteeringWheelSpeed', 'offset': 0.0,
  // 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 38, 'type': 'double', 'order': 'intel', 'physical_unit':
  // ''}
  double steering_wheel_speed(const std::uint8_t* bytes, const int32_t length) const;

};

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
