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

#include "modules/canbus/vehicle/fortuna/protocol/motion_rpt_0x01.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fortuna {

using ::apollo::drivers::canbus::Byte;

Motionrpt0x01::Motionrpt0x01() {}
const int32_t Motionrpt0x01::ID = 0x01;

void Motionrpt0x01::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis_detail) const {
  // apollo.canbus.chassis.speed_mps will be set via
  // apollo.canbus.chasssis_detail.vehicle_spd
  // fortuna canbus has vehicle speed in km/h so
  // we have to convert it to m/s
  double vehicle_speed_mps_ =  (vehicle_velocity(bytes, length)*5.0)/18.0; 
  chassis_detail->mutable_vehicle_spd()->set_vehicle_spd(vehicle_speed_mps_);                           
  
}

// config detail: {'description': 'Motion checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'motion_checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Motionrpt0x01::motion_checksum(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'YAW_RATE_SIGN', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit':
// ''}
bool Motionrpt0x01::yaw_rate_sign(const std::uint8_t* bytes, 
                              const int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;

}

// config detail: {'name': 'YAW_RATE', 'offset': 0.0, 'precision': 0.01,
// 'len': 14, 'is_signed_var': False, 'physical_range': '[0|16382]', 'bit': 16,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Motionrpt0x01::yaw_rate(const std::uint8_t* bytes, 
                        const int32_t length) const {

  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0,6);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  double ret = x * 0.01;
  return ret;                        

}

// config detail: {'name': 'LONGITUDINAL _ACCELERATION', 'offset': -16, 
// 'precision': 0.03125, 'len': 10, 'is_signed_var': False, 'physical_range': 
//'[0|0]', 'bit': 30, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Motionrpt0x01::longitudinal_acceleration(const std::uint8_t* bytes, 
                                         const int32_t length) const {

  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;

  double ret = x * 0.03125 + -16;
  return ret;                                         
                                          
}

// config detail: {'name': 'LATERAL_ACCELERATION', 'offset': -1.27,
// 'precision': 0.01, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[0|0]', 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit':
// ''}
double Motionrpt0x01::lateral_acceleration(const std::uint8_t* bytes,
                                   const int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.01 + -1.27;
  return ret;
}

// config detail: {'name': 'VEHICLE_VELOCITY', 'offset': 0.0,
// 'precision': 0.01, 'len': 16, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Motionrpt0x01::vehicle_velocity(const std::uint8_t* bytes,
                    const int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;
  // oveflow checking
  // as in lexus Accelauxrpt300::raw_pedal_pos
  x <<= 16;
  x >>= 16;

  double ret = x * 0.01;
  return ret; 

}

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
