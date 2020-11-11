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

#include "modules/canbus/vehicle/fortuna/protocol/powertrain_1_rpt_0x04.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fortuna {

using ::apollo::drivers::canbus::Byte;

Powertrain1rpt0x04::Powertrain1rpt0x04() {}
const int32_t Powertrain1rpt0x04::ID = 0x04;

void Powertrain1rpt0x04::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis_detail) const {

  // use this to set apollo.canbus.chassis.gear_location 
  // via apollo.canbus.chassis_detail.gear.gear_state                          
  int32_t gear_ = gear(bytes, length);
  switch (gear_) {
    case 0x00: // Fortuna: Intermediate
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NONE);
      break;
    case 0x01: // Fortuna: Init
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NONE);
      break;
    case 0x05: // Fortuna: Park
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_PARKING);
      break;
    case 0x06: // Fortuna: Reverse
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_REVERSE);
      break;
    case 0x07: // Fortuna: Neutral
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NEUTRAL);
      break;  
    case 0x08: // Fortuna: Drive
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_DRIVE);
      break;
    case 0x09: // Fortuna: Sport
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_DRIVE);
      break;
    case 0xA: // Fortuna: 10-Efficient
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_DRIVE);
      break;
    case 0xD: // Fortuna: 13-TipInS
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NONE);
      break;
    case 0xE: // Fortuna: 15-TipInD
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NONE);
      break;              
    default:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_INVALID);
      break;
  }
    
  // apollo.canbus.chassis.throttle_percentage   
  // via apollo.canbus.chassis_detail.gas.throttle_input                        
  chassis_detail->mutable_gas()->set_throttle_input(throttle_pedal_position(bytes, length));
  
  // apollo.canbus.chassis.engine_started
  // via apollo.canbus.chassis_detail.ems.engine_state
  
  // initially set it to invalid, before reading canbus
  chassis_detail->mutable_ems()->set_engine_state(Ems_Type_INVALID);

  if (engine_running(bytes, length)){
    chassis_detail->mutable_ems()->set_engine_state(Ems_Type_RUNNING);
  }
  else { 
    chassis_detail->mutable_ems()->set_engine_state(Ems_Type_STOP);
  }
    
}

// config detail: {'description': 'Powertrain 1 checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'powertrain1_checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Powertrain1rpt0x04::powertrain1_checksum(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'gear', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 12, 'type': 'int', 'order': 'intel', 'physical_unit':
// ''}
int32_t Powertrain1rpt0x04::gear(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 1);
  int32_t ret = t0.get_byte(4, 4);

  return ret;
}

// config detail: {'name': 'THROTTLE_PEDAL_POSITION', 'offset': 0.0, 'precision': 0.4,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 16,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Powertrain1rpt0x04::throttle_pedal_position(const std::uint8_t* bytes, 
                                             const int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.4;
  return ret;
    
}

// config detail: {'name': 'ENGINE_SPEED', 'offset': 0.0, 'precision': 0.25,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Powertrain1rpt0x04::engine_speed(const std::uint8_t* bytes, 
                                 const int32_t length) const {
                                      
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;
  // oveflow checking
  // as in lexus Accelauxrpt300::raw_pedal_pos
  x <<= 16;
  x >>= 16;

  double ret = x * 0.25;
  return ret;

}

// config detail: {'name': 'ACTUAL_TORQUE_ENGINE', 'offset': 0.0,
// 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit':
// ''}
int32_t Powertrain1rpt0x04::actual_torque_engine(const std::uint8_t* bytes,
                                           const int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 3);
  
  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  int32_t ret = x;
  return ret;

}
                                  

// config detail: {'name': 'ENGINE_RUNNING', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 51, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Powertrain1rpt0x04::engine_running(const std::uint8_t* bytes,
                                 const int32_t length) const {

  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;

}

// config detail: {'name': 'REQUESTED_TORQUE_ENGINE', 'offset': -1527.0,
// 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 52, 'type': 'double', 'order': 'intel', 'physical_unit':
// ''}
double Powertrain1rpt0x04::requested_torque_engine(const std::uint8_t* bytes,
                                             const int32_t length) const{

  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);
  
  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);


  x <<= 4;
  x |= t;

  double ret = x + -1527.0;
  return ret;

}

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
