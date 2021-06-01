/******************************************************************************
 * Copyright 2020 fortiss GmbH
 * Authors: Tobias Kessler, Jianjie Lin
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

#include "modules/autobox_bridge/autobox_bridge_component_chassis.h"

#include <time.h>
namespace apollo {
namespace autobox_bridge {

using apollo::common::time::Clock;
using apollo::cyber::proto::RoleAttributes;
using apollo::drivers::AutoboxBridgeConf;

AutoBoxBridgeComponent_CHASSIS::AutoBoxBridgeComponent_CHASSIS()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CANBUS),
      check_timeouts_(false) {}

bool AutoBoxBridgeComponent_CHASSIS::Init() {
  // Get Parameters
  AutoboxBridgeConf abx_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &abx_config)) {
    monitor_logger_buffer_.ERROR("Unable to load autobox gateway conf file: " +
                                 config_file_path_);
    return false;
  }
  check_timeouts_ = abx_config.check_timeouts();
  cyber::ReaderConfig reader_config_traj;
  // Estabilish cyber connection chassis command
  cyber::ReaderConfig reader_config_chassis_cmd;
  reader_config_chassis_cmd.channel_name = abx_config.chassis_channel_name();
  reader_config_chassis_cmd.pending_queue_size = 100;
  chassis_cmd_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
      reader_config_chassis_cmd,
      [this](const std::shared_ptr<apollo::canbus::Chassis>& chassis_) {
        std::lock_guard<std::mutex> lock(chassis_cmd_mutex_);
        chassis_cmd_->CopyFrom(*chassis_);
      });
  monitor_logger_buffer_.INFO("Started AutoBoxUDP connection.");

  // Initialized stored trajectory and localization and control command
  chassis_cmd_ = std::make_shared<apollo::canbus::Chassis>();
  chassis_cmd_->mutable_header()->set_timestamp_sec(0.0);

  chassis_writer_ = node_->CreateWriter<apollo::canbus::ChassisToAutoboxBridge>(
      "/apollo/autobox_bridge/chassis");

  monitor_logger_buffer_.INFO("AutoBridgeComponent successfully initialized");

  return true;
}
bool AutoBoxBridgeComponent_CHASSIS::Proc() {
  std::lock_guard<std::mutex> lock(chassis_cmd_mutex_);
  publishChassis();
  return true;
}

void AutoBoxBridgeComponent_CHASSIS::publishChassis() {
  apollo::canbus::ChassisToAutoboxBridge chassistoAutoboxBridge;
  srand(time(NULL));
  int count = 10000;
  int i = rand() % count;
  float total = static_cast<float>(count);
  float hundred = 100.00;
  auto pb_msg = std::make_shared<apollo::canbus::Chassis>();
  double timestamp_ = Clock::NowInSeconds();
  float coefficient = static_cast<float>(i);
  chassistoAutoboxBridge.mutable_header()->set_sequence_num(i);
  chassistoAutoboxBridge.mutable_header()->set_timestamp_sec(timestamp_);
  chassistoAutoboxBridge.set_engine_started(true);
  chassistoAutoboxBridge.set_engine_rpm(static_cast<float>(coefficient * 2.0));
  chassistoAutoboxBridge.set_odometer_m(coefficient);
  chassistoAutoboxBridge.set_fuel_range_m(100);
  chassistoAutoboxBridge.set_throttle_percentage(coefficient * hundred / total);
  chassistoAutoboxBridge.set_brake_percentage(coefficient * hundred / total);
  chassistoAutoboxBridge.set_steering_percentage(static_cast<double>(coefficient * hundred / total));
  chassistoAutoboxBridge.set_steering_torque_nm(coefficient);
  chassistoAutoboxBridge.set_parking_brake(i % 2);
  chassis_writer_->Write(chassistoAutoboxBridge);
}
}  // namespace autobox_bridge
}  // namespace apollo
