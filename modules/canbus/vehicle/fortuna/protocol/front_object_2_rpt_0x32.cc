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

#include "modules/canbus/vehicle/fortuna/protocol/front_object_2_rpt_0x32.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fortuna {

using ::apollo::drivers::canbus::Byte;

FrontObject2rpt0x32::FrontObject2rpt0x32() {}
const int32_t FrontObject2rpt0x32::ID = 0x32;

void FrontObject2rpt0x32::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis_detail) const {
    chassis_detail->mutable_fortuna()->mutable_front_object_2()->set_rel_velocity_x(rel_velocity_x(bytes, length));                           
    chassis_detail->mutable_fortuna()->mutable_front_object_2()->set_rel_pos_y(rel_pos_y(bytes, length));                           
    chassis_detail->mutable_fortuna()->mutable_front_object_2()->set_rel_pos_x(rel_pos_x(bytes, length));                           
    chassis_detail->mutable_fortuna()->mutable_front_object_2()->set_id(id(bytes, length));                           
    if(fused_state(bytes, length)){
        chassis_detail->mutable_fortuna()->mutable_front_object_2()->set_fused_state(Front_object_2_Fused_stateType_OBJECT_FUSED);                           
    }
    else{
        chassis_detail->mutable_fortuna()->mutable_front_object_2()->set_fused_state(Front_object_2_Fused_stateType_NOT_FUSED);
    }
    
}

// config detail: {'description': 'checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int FrontObject2rpt0x32::checksum(const std::uint8_t* bytes, 
                                     int32_t length) const {
    Byte t0(bytes);
    int32_t x = t0.get_byte(0, 8);

    int ret = x;
    return ret;
}
  
// config detail: {'name': 'RelVelocityX', 'offset': -128.0,
// 'precision': 0.25, 'len': 10, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 12, 'type': 'double', 'order': 'intel', 'physical_unit':
// ''}
// Relative velocity of the nearest object ahead on left lane in direction of X.
double FrontObject2rpt0x32::rel_velocity_x(const std::uint8_t* bytes, 
                                         const int32_t length) const{

    Byte t0(bytes + 2);
    int32_t x = t0.get_byte(0, 6);
  
    Byte t1(bytes + 1);
    int32_t t = t1.get_byte(4, 4);


    x <<= 4;
    x |= t;

    double ret = x * 0.25 + -128.0;
    return ret;                                             

}

// config detail: {'name': 'RelPosY', 'offset': -32.0, 'precision': 0.0625,
// 'len': 10, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 22,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
// Position of the nearest object (object center) ahead on left lane in 
// direction of Y relative to the longitudinal axis of the ego vehicle.
double FrontObject2rpt0x32::rel_pos_y(const std::uint8_t* bytes, 
                             const int32_t length) const{
  
    Byte t0(bytes + 3);
    int32_t x = t0.get_byte(0, 8);
  
    Byte t1(bytes + 2);
    int32_t t = t1.get_byte(6, 2);


    x <<= 2;
    x |= t;

    double ret = x * 0.0625 + -32.0;
    return ret;                                  

}

// config detail: {'name': 'RelPosX', 'offset': 0.0, 'precision': 0.0625,
// 'len': 12, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
// Position of the nearest object (closest point) ahead on left lane in 
// direction of X relative to the rear axle of the ego vehicle. The distance 
// is based on the current course of the ego vehicle.
double FrontObject2rpt0x32::rel_pos_x(const std::uint8_t* bytes, const int32_t length) const{
    
    Byte t0(bytes + 5);
    int32_t x = t0.get_byte(0, 4);
  
    Byte t1(bytes + 4);
    int32_t t = t1.get_byte(0, 8);
    
    x <<= 8;
    x |= t;
    
    double ret = x * 0.0625;
    return ret; 

}

// config detail: {'name': 'ID', 'offset': 0.0,
// 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit':
// ''}
// ID of FrontObject
int32_t FrontObject2rpt0x32::id(const std::uint8_t* bytes, 
                             const int32_t length) const{
    
    Byte t0(bytes + 6);
    int32_t x = t0.get_byte(0, 2);
  
    Byte t1(bytes + 5);
    int32_t t = t1.get_byte(4, 4);
    
    x <<= 4;
    x |= t;
        
    int32_t ret = x;
    return ret;
}

// config detail: {'name': 'FusedState', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 50, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// State of radar and camera object fusion for FrontObject
bool FrontObject2rpt0x32::fused_state(const std::uint8_t* bytes, 
                                const int32_t length) const{
    
    Byte t0(bytes + 6);
    int32_t x = t0.get_byte(2, 1);

    bool ret = x;
    return ret;

}


}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo