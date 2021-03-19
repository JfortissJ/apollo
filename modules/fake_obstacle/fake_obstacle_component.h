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
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace fake_obstacle {

/**
 * @class FakeObstacleComponent
 *
 * @brief fake obstacle module main class, it processes routing data to compute
 * a fake dynamic obstacle.
 */
class FakeObstacleComponent final
    : public cyber::Component<routing::RoutingResponse> {
 public:
  FakeObstacleComponent();

  ~FakeObstacleComponent();

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<routing::RoutingResponse>& routing) override;

 private:
  bool InitReaders();
  bool UpdateReferenceLine(
      const std::shared_ptr<routing::RoutingResponse>& routing);

 private:
  // std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Writer<perception::PerceptionObstacles>>
      obstacle_writer_;

  std::mutex mutex_;

  // routing::RoutingResponse latest_routing_;
  canbus::Chassis latest_chassis_;
  localization::LocalizationEstimate latest_localization_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  std::unique_ptr<planning::ReferenceLineProvider> reference_line_provider_;
  std::list<planning::ReferenceLine> last_reference_lines_;

  const hdmap::HDMap* hdmap_ = nullptr;
};

CYBER_REGISTER_COMPONENT(FakeObstacleComponent)

}  // namespace fake_obstacle
}  // namespace apollo
