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

#pragma once

#include <mutex>
#include <thread>

#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "cyber/timer/timer.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/autobox_bridge/proto/autobox_bridge_conf.pb.h"
#include "modules/autobox_bridge/proto/autobox_trajectory.pb.h"


namespace apollo {
namespace autobox_bridge {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class AutoBoxBridgeComponent_TRAJECTORY final: public apollo::cyber::TimerComponent{
 public:

  AutoBoxBridgeComponent_TRAJECTORY();

  bool Init() override;
  bool Proc() override;
 private:

  bool checkTrajectoryMsg(const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory_msg);
  void trajectoryMsgCallback(const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory_msg);

  void  publishTrajectory();

  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  
  std::shared_ptr<apollo::planning::ADCTrajectory> trajectory_; //the last received traj

  bool new_trajectory_;

  std::shared_ptr<apollo::cyber::Reader<apollo::planning::ADCTrajectory>> trajectory_reader_ = nullptr;

  std::shared_ptr<cyber::Writer<apollo::planning::ADCTrajectoryToAutoboxBridge>> trajectory_writer_;

  std::mutex trajectory_mutex_;

  bool check_timeouts_;

};

CYBER_REGISTER_COMPONENT(AutoBoxBridgeComponent_TRAJECTORY)

}  // namespace autobox_bridge
}  // namespace apollo
