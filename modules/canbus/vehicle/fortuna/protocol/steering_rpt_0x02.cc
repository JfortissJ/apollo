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

#include "modules/canbus/vehicle/fortuna/protocol/steering_rpt_0x02.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace fortuna {

using ::apollo::drivers::canbus::Byte;

Steeringrpt0x02::Steeringrpt0x02() {}
const int32_t Steeringrpt0x02::ID = 0x02;

void Steeringrpt0x02::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis_detail) const {
                     
    if(steering_wheel_angle_sign(bytes, length)){
        chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_angle_sign(Steering_Steering_signType_STEERING_RIGHT);                           
    }
    else{
        chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_angle_sign(Steering_Steering_signType_STEERING_LEFT);   
    }

    if(steering_wheel_speed_sign(bytes, length)){
        chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_speed_sign(Steering_Steering_signType_STEERING_RIGHT);                           
    }
    else{
        chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_speed_sign(Steering_Steering_signType_STEERING_LEFT);   
    }

    if(steering_wheel_torque_sign(bytes, length)){
        chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_torque_sign(Steering_Steering_signType_STEERING_RIGHT);                           
    }
    else{
        chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_torque_sign(Steering_Steering_signType_STEERING_LEFT);   
    }

    chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_torque(steering_wheel_torque(bytes, length));
    chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_angle(steering_wheel_angle(bytes, length));
    chassis_detail->mutable_fortuna()->mutable_steering()->set_steering_wheel_speed(steering_wheel_speed(bytes, length));

}

// config detail: {'description': 'checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Steeringrpt0x02::checksum(const std::uint8_t* bytes, 
                                     int32_t length) const {
    Byte t0(bytes);
    int32_t x = t0.get_byte(0, 8);

    int ret = x;
    return ret;
}

// config detail: {'name': 'SteeringWheelAngleSign', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Steeringrpt0x02::steering_wheel_angle_sign(const std::uint8_t* bytes, 
                                             const int32_t length) const{
    Byte t0(bytes + 1);
    int32_t x = t0.get_byte(4, 1);

    bool ret = x;
    return ret;

}
  
// config detail: {'name': 'SteeringWheelSpeedSign', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Steeringrpt0x02::steering_wheel_speed_sign(const std::uint8_t* bytes, 
                                         const int32_t length) const{
    Byte t0(bytes + 1);
    int32_t x = t0.get_byte(5, 1);

    bool ret = x;
    return ret;

}

// config detail: {'name': 'SteeringWheelTorqueSign', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Steeringrpt0x02::steering_wheel_torque_sign(const std::uint8_t* bytes,
                                         const int32_t length) const{
    Byte t0(bytes + 1);
    int32_t x = t0.get_byte(6, 1);

    bool ret = x;
    return ret;
}

// config detail: {'name': 'SteeringWheelTorque', 'offset': 0.0,
// 'precision': 0.01, 'len': 10, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 15, 'type': 'double', 'order': 'intel', 'physical_unit':
// ''}
double Steeringrpt0x02::steering_wheel_torque(const std::uint8_t* bytes, 
                                     const int32_t length) const{
    Byte t0(bytes + 3);
    int32_t x = t0.get_byte(0, 1);
  
    Byte t1(bytes + 2);
    int32_t t = t1.get_byte(0, 8);

    Byte t2(bytes + 1);
    int32_t u = t2.get_byte(7, 1);


    x <<= 8;
    x |= t;
    // TODO: Check if this merge of three bytes is correct
    x <<= 1;
    x |= u;

    double ret = x * 0.01;
    return ret;                   
}

// config detail: {'name': 'SteeringWheelAngle', 'offset': 0.0,
// 'precision': 0.1, 'len': 13, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 25, 'type': 'double', 'order': 'intel', 'physical_unit':
// ''}
double Steeringrpt0x02::steering_wheel_angle(const std::uint8_t* bytes, 
                                         const int32_t length) const{
    Byte t0(bytes + 4);
    int32_t x = t0.get_byte(0, 6);
  
    Byte t1(bytes + 3);
    int32_t t = t1.get_byte(1, 7);


    x <<= 7;
    x |= t;

    double ret = x * 0.1;
    return ret;       
}

// config detail: {'name': 'SteeringWheelSpeed', 'offset': 0.0,
// 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 38, 'type': 'double', 'order': 'intel', 'physical_unit':
// ''}
double Steeringrpt0x02::steering_wheel_speed(const std::uint8_t* bytes, 
                                     const int32_t length) const{
    Byte t0(bytes + 5);
    int32_t x = t0.get_byte(0, 7);
  
    Byte t1(bytes + 4);
    int32_t t = t1.get_byte(6, 2);


    x <<= 2;
    x |= t;

    double ret = x * 0.5;
    return ret;     

}

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo