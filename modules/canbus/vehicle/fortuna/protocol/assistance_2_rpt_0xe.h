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

class Assistance2rpt0xe : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Assistance2rpt0xe();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'Assistance 2 checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'assistance2_checksum',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int assistance2_checksum(const std::uint8_t* bytes, int32_t length) const;
  
  // config detail: {'name': 'BRAKE PRESSURE', 'offset': -30.0, 'precision': 0.3,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|1022]', 'bit': 12,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double brake_pressure(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BRAKE PERCENTAGE', 'offset': -30.0, 'precision': 0.3,
  // 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 12,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double brake_percentage(const std::uint8_t* bytes, const int32_t length) const;

};

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
