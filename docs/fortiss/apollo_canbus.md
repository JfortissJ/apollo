# Installation
* docker has to be (re)build with the canutils and the socketcan driver!
* if no peak driver is installed it is sufficient to bringup the can interfaces usinng ip link ...

## On PC1 of the car
* For the apollo node, we need the socketcan driver and NOT the peak can character device driver.
* There are bash scripts to switch between character devive and socketcan usage: see modules/tools/canbus

# Troubleshooting
* check candump can0 or another can channel if you see the raw can data, if not the driver is not installed correctly
* Note that for SocketCan the can IDs are random every reboot! So you might have to bringup another can channel and launch the component also with another channel!

# How to bring up the usb socket can adapter

* see https://en.wikipedia.org/wiki/SocketCAN
* `sudo ip link set can0 type can bitrate 500000`
* `sudo ip link set up can0`


# Brief description of the Apollo CANBUS component functionality along with information on creating a new Apollo vehicle which can be used by this component.

## Introduction
The Apollo CANBUS module accepts and executes control module commands and collects the car's chassis status as feedback to control.

## Input
  * Control commands

## Output
  * Chassis status
  * Chassis detailed status

The chassis protobuf file (modules/canbus/proto/chassis.proto) contains all the basic messages that are used for the control of a vehicle, like GearPosition, DrivingMode, ThrottlePercentage etc. The chassis detail file (modules/canbus/proto/chassis_detail.proto) contains message fields regarding specific parts of the vehicle like the battery but can also hold extra information for basic parts like the Gas or the Brake system. Since not all vehicles have the same data read / written through their CAN bus, **the Chassis detail is vehicle-specific**. The default chassis_detail.proto file is based on the Lincoln vehicle which is the one mainly used by the Apollo team. It can be extented by creating a new vehicle-specific file (e.g. lexus.proto) and importing this file in the chassis_detail.proto. 

## Implementation

The major components in canbus module are:
  * **Vehicle**: the vehicle itself, including its controller and message manager. An Apollo Vehicle consists of three main components (see https://git.fortiss.org/fav/apollo35/blob/dev_fortiss/docs/howto/how_to_add_a_new_vehicle.md): 

    * **Message Manager**: The message manager handles sending and receiving data through the canbus. The data are read or written via protocols which are vehicle-specific and must be created in modules/canbus/vehicle/vehicle_name/protocol/. 
      * *Receive-protocols*: These are the protocols that read data from the canbus. Each protocol file has a Parse method that is used to write the information that it received from the canbus into the **chassis_detail** status. The protocol files can be created based on a .dbc file in order to access specific CAN frames of a vehicle. For example, the Fortuna dbc file provided by IAV GmbH has a frame named Powetrain_1 and whose ID is 0x4. Through this 8-byte long CAN frame are transmitted signals like the ThrottlePedalPosition and the EngineSpeed. The protocol file /modules/canbus/vehicle/fortuna/protocol/powertrain_1_0x04.cc contains methods to read the data by accessing the specific bits as given in the .dbc file. Let's have a closer look in the following method that reads the EngineSpeed data from the canbus: 
          
          **dbc file desciption:**
          ```
          | Byte No. | Bit No. | Length (Bit) | Byte Order |     Name    | Factor | Offset |
          |:--------:|:-------:|:------------:|:----------:|:-----------:|:------:|:------:|
          |     3    |    0    |      16      |  i (Intel) | EngineSpeed |  0.25  |    0   |
          ```

          **Method to read the above signal:**
          ```
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
             x <<= 16;
             x >>= 16;
             
             double ret = x * 0.25;
             return ret;
          }
          ```
        
           The first thing to notice before writing such a method is the **byte order**. There are only two options: big endian (a.k.a. **Motorola order**) and little endian (**Intel order**). With Motorola order, the bytes must be read in normal order while with Intel order they must be read inversely. 
           Then, we look into which specific bytes we must read. The dbc file specifies that the EngineSpeed signal starts at the 0-bit of Byte 3 (starting from 0 this is the 4th byte in the CAN frame). Because we follow a bitwise order to read the whole 8-byte long CAN frame (from bit 0  to bit 63), this means that the EngineSpeed signal starts at **bit 24**.
           Subsequently, we write the method to parse this signal, making sure to take care of its **Length**. In this example, we must read 16 bits (2 Bytes), starting from bit 24, with inverse (Intel) order. Therefore, we first read byte 4 and then byte 3. Then, we merge them via left shift and an OR operation.
           Finally, we must **multiply the result by the factor and add the offset**. In this example, we have a factor of 0.25 and no offset.    
           Once all the signal-specific methods are written as above, the next step is to write the Parse method of this protocol. This method calls the signal-specific methods and writes the canbus data into the chassis_detail.
     
      * *Send-protocols*: These are the protocols that send commands to the canbus. For example the Lincoln vehicle has a gear_66.cc protocol file with methods to set the gear in a  specific state by transimtting the relevant value in the canbus. Please note that
      when creating methods that send signals to the canbus, we have to **substract th offset from the result and then divide by the factor**. 

    * **Controller**: The vehicle controller has a Chassis object on which it can write all the messages used to control a vehicle. It receives commands from the Apollo control module and writes the Chassis fields like the DrivingMode, GearPosition or the VehicleSpeed. Furthermore, it can read all the data in the Chassis detail by calling the message_manager method **GetSensorData(&chassis_detail)**. This can be used as a feedback to to set Chassis fields based on the Chassis_detail data which are written from the specific protocols described above. For example, the Lincoln vehicle controller sets the throttle_percentage field of the Chassis via the throttle_output field of the Chassis-detail: 

       ```
       if (chassis_detail.has_gas() && chassis_detail.gas().has_throttle_output()) {
          chassis_.set_throttle_percentage(static_cast<float>(chassis_detail.gas().throttle_output()));
       } 
       else {
          chassis_.set_throttle_percentage(0);
       }
       ``` 

    * **Factory**: The vehicle factory is used to create the Controller and Message Manager objects for a vehicle.


  * **CAN Client** - CAN client has been moved to `/modules/drivers/canbus` since it is shared by different sensors utilizing the canbus protocol. It depends on the CAN card that is installed on a vehilce. For the fortuna vehicle we use the SocketCanRaw client which is offered by Apollo since the PEAK PCI card in the vehicle supports SocketCAN.

  ## Canbus component + Vehicle Controller + Vehicle Message Manager

  The canbus component uses the vehicle factory in order to create a specific vehicle object. Then it publishes the Chassis data as they are written from the VehicleController and MessageManager. As described above, the VehicleController has a VehicleController::Chassis object to which it sets all the control-specific Chassis fields like GearPosition, VehicleSpeed etc. These fields can then be sent to the canbus as signals by calling methods of the *send-protocols*. 
  The MessageManager reads data from the canbus via the *receive-protocols* and sets the relevant Chassis-detail fields, which can then be accesed by the contoller. In addition, it manages all the *send-protocols* which can be used to send the Chassis and Chassis-detail fields as canbus signals. 
  The CANBUS component can then publish all this information as seen below:

  ```
  void CanbusComponent::PublishChassis() {
  Chassis chassis = vehicle_controller_->chassis();
  common::util::FillHeader(node_->Name(), &chassis);
  chassis_writer_->Write(std::make_shared<Chassis>(chassis));
  ADEBUG << chassis.ShortDebugString();
  }

  void CanbusComponent::PublishChassisDetail() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  ADEBUG << chassis_detail.ShortDebugString();
  chassis_detail_writer_->Write(
      std::make_shared<ChassisDetail>(chassis_detail));
  }
  ```



