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

#include "modules/autobox_bridge/autobox_bridge_component_localization.h"

#include <time.h>
namespace apollo {
namespace autobox_bridge {

using apollo::common::time::Clock;
using apollo::cyber::proto::RoleAttributes;
using apollo::drivers::AutoboxBridgeConf;
using apollo::localization::LocalizationEstimate;

AutoBoxBridgeComponent_LOCALIZATION::AutoBoxBridgeComponent_LOCALIZATION()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CANBUS),
      new_localization_(false),
      check_timeouts_(false) {}

bool AutoBoxBridgeComponent_LOCALIZATION::Init() {
  // Get Parameters
  AutoboxBridgeConf abx_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &abx_config)) {
    monitor_logger_buffer_.ERROR("Unable to load autobox gateway conf file: " +
                                 config_file_path_);
    return false;
  }
  check_timeouts_ = abx_config.check_timeouts();
   // Establish cyber connection localization
  cyber::ReaderConfig reader_config_loca;
  reader_config_loca.channel_name = abx_config.localization_channel_name();
  reader_config_loca.pending_queue_size = 100;
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      reader_config_loca,
      [&](const std::shared_ptr<LocalizationEstimate>& localization_msg) {
        localizationMsgCallback(localization_msg);
      });

   monitor_logger_buffer_.INFO("Started AutoBoxUDP connection.");

  // Initialized stored trajectory and localization and control command
   localization_ =
      std::make_shared<apollo::localization::LocalizationEstimate>();
  localization_->mutable_header()->set_timestamp_sec(0.0);


  localization_writer_ =
      node_->CreateWriter<apollo::localization::LocalizationToAutoboxBridge>(
          "/apollo/autobox_bridge/localization");
  monitor_logger_buffer_.INFO("AutoBridgeComponent successfully initialized");

  return true;
}
bool AutoBoxBridgeComponent_LOCALIZATION::Proc() {
  publishLocalization();
  return true;
}


void AutoBoxBridgeComponent_LOCALIZATION::localizationMsgCallback(
    const std::shared_ptr<apollo::localization::LocalizationEstimate>&
        localization_msg) {
  AINFO << "Got a localization at t= " << apollo::cyber::Time::Now().ToSecond();
  if (checkLocalizationMsg(localization_msg)) {
    //localization_mutex_.lock();
    std::lock_guard<std::mutex> lock(localization_mutex_);
    localization_->CopyFrom(*localization_msg);
    new_localization_ = true;
    //localization_mutex_.unlock();
  }
}



bool AutoBoxBridgeComponent_LOCALIZATION::checkLocalizationMsg(
    const std::shared_ptr<apollo::localization::LocalizationEstimate>&
        localization_msg) {
  // msg is available
  if (localization_msg == NULL) {
    AERROR << "Received empty localization msg!";
    return false;
  }

  // msg is valid
  if (!(localization_msg->has_header() && localization_msg->has_pose() &&
        localization_msg->mutable_pose()->has_position() &&
        localization_msg->mutable_pose()->has_orientation() &&
        localization_msg->mutable_pose()->has_linear_velocity() &&
        localization_msg->mutable_pose()->has_linear_acceleration())) {
    AERROR << "Localization message invalid! at least one field is missing!";
    return false;
  }
  if (std::isnan(localization_msg->mutable_pose()->mutable_position()->x()) ||
      std::isnan(localization_msg->mutable_pose()->mutable_position()->y()) ||
      std::isnan(localization_msg->mutable_pose()->mutable_position()->z()) ||
      std::isnan(
          localization_msg->mutable_pose()->mutable_orientation()->qw()) ||
      std::isnan(
          localization_msg->mutable_pose()->mutable_orientation()->qx()) ||
      std::isnan(
          localization_msg->mutable_pose()->mutable_orientation()->qy()) ||
      std::isnan(
          localization_msg->mutable_pose()->mutable_orientation()->qz())) {
    AERROR << "Localization message invalid! at least one field is NaN!";
    return false;
  }

  // msg is new (compared to the old, stored one)
  if (localization_->mutable_header()->timestamp_sec() >
      localization_msg->mutable_header()->timestamp_sec()) {
    AERROR
        << "Localization msg is older than previously received localization!";
    AERROR << "Old Loca: " << localization_->mutable_header()->timestamp_sec();
    AERROR << "New Loca: "
           << localization_msg->mutable_header()->timestamp_sec();
    return false;
  }

  // msg timed out
  //const double timedelta = 100.0;  // todo define a reasonable value here!
  //if (check_timeouts_ &&
  //    cyber::Time::Now().ToSecond() -
  //            localization_msg->mutable_header()->timestamp_sec() >
  //        timedelta) {  // check if timestamp is recent
  //  AERROR_EVERY(100) << "Localization message timed out.";
  //  return false;
 // }

  // no error
  return true;
}

void AutoBoxBridgeComponent_LOCALIZATION::publishLocalization() {
  apollo::localization::LocalizationToAutoboxBridge localizationToAutoboxBridge;
  localizationToAutoboxBridge.mutable_header()->CopyFrom(localization_->header());
  localizationToAutoboxBridge.set_measurement_time(localization_->measurement_time());
  localizationToAutoboxBridge.mutable_pose()->CopyFrom(localization_->pose());
  localizationToAutoboxBridge.mutable_uncertainty()->CopyFrom(localization_->uncertainty());
  localization_writer_->Write(localizationToAutoboxBridge);
}


}  // namespace autobox_bridge
}  // namespace apollo
