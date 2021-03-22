/******************************************************************************
 * Copyright 2021 fortiss GmbH
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

#include <memory>

#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/fake_obstacle/proto/fake_obstacle_conf.pb.h"

namespace apollo {
namespace fake_obstacle {

class FakeObstacleComponent : public apollo::cyber::TimerComponent {
 public:
  FakeObstacleComponent();

  ~FakeObstacleComponent();

 public:
  bool Init() override;

  bool Proc() override;

 private:
  bool InitReaders();
  bool UpdateReferenceLine();
  bool FillPerceptionObstacles();

 private:
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_ = nullptr;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>> localization_reader_ = nullptr;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_ = nullptr;
  std::shared_ptr<cyber::Writer<perception::PerceptionObstacles>> obstacle_writer_ = nullptr;

  FakeObstacleConf fake_obst_config_;
  std::mutex mutex_;
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  routing::RoutingResponse latest_routing_;
  canbus::Chassis latest_chassis_;
  localization::LocalizationEstimate latest_localization_;

  std::unique_ptr<planning::ReferenceLineProvider> reference_line_provider_;
  std::list<planning::ReferenceLine> last_reference_lines_;

  const hdmap::HDMap* hdmap_ = nullptr;
  
  double s_init_obstacle_;
  double t_init_obstacle_;
  bool obstacle_has_started_;
};

CYBER_REGISTER_COMPONENT(FakeObstacleComponent)

}  // namespace fake_obstacle
}  // namespace apollo
