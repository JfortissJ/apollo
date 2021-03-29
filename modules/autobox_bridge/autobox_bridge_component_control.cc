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

#include "modules/autobox_bridge/autobox_bridge_component_control.h"

#include <time.h>
namespace apollo {
namespace autobox_bridge {

using apollo::common::time::Clock;
using apollo::control::ControlCommand;
using apollo::cyber::proto::RoleAttributes;
using apollo::drivers::AutoboxBridgeConf;

AutoBoxBridgeComponent_CONTROL::AutoBoxBridgeComponent_CONTROL()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CANBUS),
      new_control_cmd_(false),
      check_timeouts_(false) {}

bool AutoBoxBridgeComponent_CONTROL::Init() {
  // Get Parameters
  AutoboxBridgeConf abx_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &abx_config)) {
    monitor_logger_buffer_.ERROR("Unable to load autobox gateway conf file: " +
                                 config_file_path_);
    return false;
  }
  check_timeouts_ = abx_config.check_timeouts();
  // Establish cyber connection control command
  cyber::ReaderConfig reader_config_control_cmd;
  reader_config_control_cmd.channel_name = abx_config.controlcmd_channel_name();
  reader_config_control_cmd.pending_queue_size = 100;
  control_cmd_reader_ = node_->CreateReader<apollo::control::ControlCommand>(
      reader_config_control_cmd,
      [&](const std::shared_ptr<apollo::control::ControlCommand>&
              control_cmd_msg) { controlCmdMsgCallback(control_cmd_msg); });

  monitor_logger_buffer_.INFO("Started AutoBoxUDP connection.");

  // Initialized stored trajectory and localization and control command
  control_cmd_ = std::make_shared<apollo::control::ControlCommand>();
  control_cmd_->mutable_header()->set_timestamp_sec(0.0);
  control_cmd_writer_ =
      node_->CreateWriter<apollo::control::ControlCommandToAutoboxBridge>(
          "/apollo/autobox_bridge/control");
  monitor_logger_buffer_.INFO("AutoBridgeComponent successfully initialized");

  return true;
}
bool AutoBoxBridgeComponent_CONTROL::Proc() {
  std::lock_guard<std::mutex> lock(control_cmd_mutex_);
  if (new_control_cmd_) {
    publishControl();
    new_control_cmd_ = false;
  }

  return true;
}
void AutoBoxBridgeComponent_CONTROL::controlCmdMsgCallback(
    const std::shared_ptr<apollo::control::ControlCommand>& control_cmd_msg) {
  // AINFO << "Got a control command at t= "
  //       << apollo::cyber::Time::Now().ToSecond();
  if (checkControlCmdMsg(control_cmd_msg)) {
    std::lock_guard<std::mutex> lock(control_cmd_mutex_);
    control_cmd_->CopyFrom(*control_cmd_msg);
    new_control_cmd_ = true;
  }
}
bool AutoBoxBridgeComponent_CONTROL::checkControlCmdMsg(
    const std::shared_ptr<apollo::control::ControlCommand>& control_cmd_msg) {
  // msg is available
  if (control_cmd_msg == NULL) {
    AERROR << "Received empty control command msg!";
    return false;
  }
  if (!control_cmd_msg->has_header()) {
    AERROR << "control command message invalid! header is missing!";
    return false;
  }
  if (!control_cmd_msg->has_throttle()) {
    AERROR << "control command messgae invalid. throttle  is missing";
    return false;
  }
  if (!control_cmd_msg->has_brake()) {
    AERROR << "control command messgae invalid. brake  is missing";
    return false;
  }
  if (std::isnan(control_cmd_msg->acceleration()) ||
      std::isnan(control_cmd_msg->brake()) ||
      std::isnan(control_cmd_msg->throttle()) ||
      std::isnan(control_cmd_msg->steering_rate()) ||
      std::isnan(control_cmd_msg->steering_target())) {
    AERROR << "Control Command message invalid! at least one field is NaN!";
    return false;
  }
  // msg is new (compared to the old, stored one)
  if (control_cmd_->mutable_header()->timestamp_sec() >
      control_cmd_msg->mutable_header()->timestamp_sec()) {
    AERROR << "control command msg is older than previously received control "
              "command!";
    AERROR << "Old control command: "
           << control_cmd_->mutable_header()->timestamp_sec();
    AERROR << "New control command: "
           << control_cmd_msg->mutable_header()->timestamp_sec();
    return false;
  }
  return true;
}

void AutoBoxBridgeComponent_CONTROL::publishControl() {
  apollo::control::ControlCommandToAutoboxBridge controlcommandtoAutoBoxBridge;
  controlcommandtoAutoBoxBridge.mutable_header()->CopyFrom(
      control_cmd_->header());
  controlcommandtoAutoBoxBridge.set_brake(control_cmd_->brake());
  controlcommandtoAutoBoxBridge.set_throttle(control_cmd_->throttle());
  controlcommandtoAutoBoxBridge.set_steering_rate(
      control_cmd_->steering_rate());

  controlcommandtoAutoBoxBridge.set_steering_target(
      control_cmd_->steering_target());
  controlcommandtoAutoBoxBridge.set_acceleration(control_cmd_->acceleration());
  control_cmd_writer_->Write(controlcommandtoAutoBoxBridge);
  // controlcommandtoAutoBoxBridge.set_acceleration(10);
  // controlcommandtoAutoBoxBridge.set_steering_rate(0.5);
  // controlcommandtoAutoBoxBridge.set_steering_target(5);
}

}  // namespace autobox_bridge
}  // namespace apollo
