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

#include "modules/canbus/vehicle/fortuna/protocol/assistance_2_rpt_0xe.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fortuna {

using ::apollo::drivers::canbus::Byte;

Assistance2rpt0xe::Assistance2rpt0xe() {}
const int32_t Assistance2rpt0xe::ID = 0xe;

void Assistance2rpt0xe::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis_detail) const {
  // apollo.canbus.chassis.brake_percentage   
  // via apollo.canbus.chassis_detail.gas.throttle_input                        
  chassis_detail->mutable_brake()->set_brake_input(brake_percentage(bytes, length));
}

// config detail: {'description': 'Assistance 2 checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'assistance2_checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Assistance2rpt0xe::assistance2_checksum(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'BRAKE PRESSURE', 'offset': -30.0, 'precision': 0.3,
// 'len': 10, 'is_signed_var': False, 'physical_range': '[0|1022]', 'bit': 12,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Assistance2rpt0xe::brake_pressure(const std::uint8_t* bytes, 
                                      const int32_t length) const{
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 6);
  
  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);

  x <<= 4;
  x |= t;

  double ret = x * 0.3 + -30.0;
  return ret;

}

// config detail: {'name': 'BRAKE PERCENTAGE', 'offset': -30.0, 'precision': 0.3,
// 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 12,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Assistance2rpt0xe::brake_percentage(const std::uint8_t* bytes, 
                                      const int32_t length) const{
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 6);
  
  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);

  x <<= 4;
  x |= t;

  // use 1022 bar as the max brake pressure to calculate the
  // brake percentage 
  double ret = ( (x * 0.3 + -30.0) / 137.0 ) * 100;
  return ret;

}

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
