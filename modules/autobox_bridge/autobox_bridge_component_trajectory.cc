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

#include "modules/autobox_bridge/autobox_bridge_component_trajectory.h"

#include <time.h>
namespace apollo {
namespace autobox_bridge {

using apollo::common::time::Clock;
using apollo::cyber::proto::RoleAttributes;
using apollo::drivers::AutoboxBridgeConf;
using apollo::planning::ADCTrajectory;

AutoBoxBridgeComponent_TRAJECTORY::AutoBoxBridgeComponent_TRAJECTORY()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CANBUS),
      new_trajectory_(false),
      check_timeouts_(false) {}

bool AutoBoxBridgeComponent_TRAJECTORY::Init() {
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
  reader_config_traj.channel_name = abx_config.trajectory_channel_name();
  reader_config_traj.pending_queue_size = 100;
  trajectory_reader_ = node_->CreateReader<ADCTrajectory>(
      reader_config_traj,
      [&](const std::shared_ptr<ADCTrajectory>& trajectory_msg) {
        trajectoryMsgCallback(trajectory_msg);
      });

  monitor_logger_buffer_.INFO("Started AutoBoxUDP connection.");

  // Initialized stored trajectory and localization and control command
  trajectory_ = std::make_shared<ADCTrajectory>();
  trajectory_->mutable_header()->set_timestamp_sec(0.0);
  trajectory_writer_ =
      node_->CreateWriter<apollo::planning::ADCTrajectoryToAutoboxBridge>(
          "/apollo/autobox_bridge/adctrajectory");
  monitor_logger_buffer_.INFO("AutoBridgeComponent successfully initialized");

  return true;
}
bool AutoBoxBridgeComponent_TRAJECTORY::Proc() {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (new_trajectory_) {
    publishTrajectory();
    new_trajectory_ = false;
  }

  return true;
}
void AutoBoxBridgeComponent_TRAJECTORY::trajectoryMsgCallback(
    const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory_msg) {
  // AINFO << "Got a trajectory at t= " <<
  // apollo::cyber::Time::Now().ToSecond();
  if (checkTrajectoryMsg(trajectory_msg)) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_->CopyFrom(*trajectory_msg);
    new_trajectory_ = true;
  }
}

bool AutoBoxBridgeComponent_TRAJECTORY::checkTrajectoryMsg(
    const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory_msg) {
  // msg is available
  if (trajectory_msg == NULL) {
    AERROR << "Received empty trajectory msg!";
    return false;
  }

  // msg is new (compared to the old, stored one)
  if (trajectory_->mutable_header()->timestamp_sec() >
      trajectory_msg->mutable_header()->timestamp_sec()) {
    AERROR << "Trajectory msg is older than previously received trajectory!";
    AERROR << "old traj " << trajectory_->mutable_header()->timestamp_sec();
    AERROR << "new traj " << trajectory_msg->mutable_header()->timestamp_sec();
    return false;
  }

  // msg is valid
  if (!trajectory_msg->estop().is_estop() &&
      trajectory_msg->trajectory_point_size() == 0) {
    AERROR_EVERY(100) << "planning has no trajectory point.";
    return false;
  }

  return true;
}

void AutoBoxBridgeComponent_TRAJECTORY::publishTrajectory() {
  apollo::planning::ADCTrajectoryToAutoboxBridge adctrajectorytoAutoboxBridge;
  adctrajectorytoAutoboxBridge.mutable_header()->CopyFrom(
      trajectory_->header());
  std::vector<common::TrajectoryPoint> pb_trajectory_;
  pb_trajectory_.assign(trajectory_->trajectory_point().begin(),
                        trajectory_->trajectory_point().end());
  for (int i = 0; i < trajectory_->trajectory_point().size(); i++) {
    apollo::common::TrajectoryPoint* pt_traj_points =
        adctrajectorytoAutoboxBridge.add_trajectory_point();
    pt_traj_points->CopyFrom(pb_trajectory_[i]);
  }
  trajectory_writer_->Write(adctrajectorytoAutoboxBridge);
}

}  // namespace autobox_bridge
}  // namespace apollo
