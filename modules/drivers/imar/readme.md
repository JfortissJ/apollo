# IMAR Apollo Node

* Publish both, IMU and GPS messages
* To be used in combination with the rtk localization node
* Publish the messages in an endless cyber = ok while loop, without a check if new data is available from the imar sdk

* Alternative approach we did not implement: publish localization and dynamic world transformation messages directly from here

# Build Instructions
This code relies on the SDK from IMAR. We do not have an appropritate license to include this SDK here. Therefore, 
1) Copy the files from the SDK you get from IMAR to the imar_sdk subfolder
2) Uncomment the BUILD files modules/drivers/imar/BUILD and modules/drivers/imar/imar_sdk/BUILD

# Node Parameters

## Imar SDK Connection
  optional string ip_address = 1;
  optional uint32 tcp_port = 2;
  optional uint32 udp_port = 3;

## Apollo topics (fitting Baidu's INS/GPS modules)
  optional string localization_topic = 4;
  optional string imu_topic = 5;
  optional string imar_status_topic = 6;
  optional string ins_status_topic = 7;

## Lever Arm between inat device and vehicle coordinate system (center point rear axle)
  optional double x_lever = 8;
  optional double y_lever = 9;
  optional double z_lever = 10;

## Artificial additional offset to the position to shift to loca 
  optional double x_offset = 11;
  optional double y_offset = 12;
  optional double z_offset = 13;