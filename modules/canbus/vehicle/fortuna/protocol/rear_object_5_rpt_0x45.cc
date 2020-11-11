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

#include "modules/canbus/vehicle/fortuna/protocol/rear_object_5_rpt_0x45.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fortuna {

using ::apollo::drivers::canbus::Byte;

RearObject5rpt0x45::RearObject5rpt0x45() {}
const int32_t RearObject5rpt0x45::ID = 0x45;

void RearObject5rpt0x45::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis_detail) const {
    if(rear_tracked(bytes, length)){
        chassis_detail->mutable_fortuna()->mutable_rear_object_5()->set_rear_tracked(Rear_object_5_Rear_trackedType_OBJECT_MEASURED);                             
    }
    else{
        chassis_detail->mutable_fortuna()->mutable_rear_object_5()->set_rear_tracked(Rear_object_5_Rear_trackedType_OBJECT_TRACKED);                             
    }

    chassis_detail->mutable_fortuna()->mutable_rear_object_5()->set_rear_id(rear_id(bytes, length));
    chassis_detail->mutable_fortuna()->mutable_rear_object_5()->set_rear_pos_y(rear_pos_y(bytes, length));
    chassis_detail->mutable_fortuna()->mutable_rear_object_5()->set_rear_rel_velocity_x(rear_rel_velocity_x(bytes, length));
    chassis_detail->mutable_fortuna()->mutable_rear_object_5()->set_rear_pos_x(rear_pos_x(bytes, length));
    
}

// config detail: {'description': 'checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'CRC',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int RearObject5rpt0x45::checksum(const std::uint8_t* bytes, 
                                 int32_t length) const{
    
    Byte t0(bytes);
    int32_t x = t0.get_byte(0, 8);

    int ret = x;
    return ret;
}

// config detail: {'name': 'RearTracked', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// Tracking state of the rear object.
bool RearObject5rpt0x45::rear_tracked(const std::uint8_t* bytes, 
                                 const int32_t length) const{
    
    Byte t0(bytes + 1);
    int32_t x = t0.get_byte(4, 1);

    bool ret = x;
    return ret;

}

// config detail: {'name': 'RearID', 'offset': 0.0,
// 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'physical_range':
// '[0|30]', 'bit': 13, 'type': 'int', 'order': 'intel', 'physical_unit':
// ''}
// ID of RearObject
int32_t RearObject5rpt0x45::rear_id(const std::uint8_t* bytes, 
                             const int32_t length) const{
    
    Byte t0(bytes + 2);
    int32_t x = t0.get_byte(0, 2);
  
    Byte t1(bytes + 1);
    int32_t t = t1.get_byte(5, 3);
    
    x <<= 3;
    x |= t;
    
    int32_t ret = x;
    return ret;
}
  
// config detail: {'name': 'RearPosY', 'offset': -16.0, 'precision': 0.125,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 18,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
// Position of the nearest rear object (closests point) on right lane in 
// direction of Y relative to the longitudinal axis of the ego vehicle.
double RearObject5rpt0x45::rear_pos_y(const std::uint8_t* bytes, 
                                 const int32_t length) const{

    Byte t0(bytes + 3);
    int32_t x = t0.get_byte(0, 2);
  
    Byte t1(bytes + 2);
    int32_t t = t1.get_byte(2, 6);


    x <<= 6;
    x |= t;

    double ret = x * 0.125 + -16.0;
    return ret;      
}

// config detail: {'name': 'RearRelVelocityX', 'offset': -37.5,
// 'precision': 0.25, 'len': 9, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 26, 'type': 'double', 'order': 'intel', 'physical_unit':
// ''}
// Relative velocity of the nearest rear object on right lane in direction of X.
double RearObject5rpt0x45::rear_rel_velocity_x(const std::uint8_t* bytes, 
                                         const int32_t length) const{
    
    Byte t0(bytes + 4);
    int32_t x = t0.get_byte(0, 3);
  
    Byte t1(bytes + 3);
    int32_t t = t1.get_byte(2, 6);


    x <<= 6;
    x |= t;

    double ret = x * 0.25 + -37.5;
    return ret;      
}

// config detail: {'name': 'RearPosX', 'offset': -102.5, 'precision': 0.25,
// 'len': 9, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 35,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
// Position of the nearest rear object (closests point) on right lane in 
// direction of X relative to the rear axle of the ego vehicle.
double RearObject5rpt0x45::rear_pos_x(const std::uint8_t* bytes, 
                                 const int32_t length) const{
    
    Byte t0(bytes + 5);
    int32_t x = t0.get_byte(0, 4);
  
    Byte t1(bytes + 4);
    int32_t t = t1.get_byte(3, 5);


    x <<= 5;
    x |= t;

    double ret = x * 0.25 + -102.5;
    return ret;      
}


}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo