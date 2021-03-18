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
#include "modules/localization/proto/localization.pb.h"
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
    : public cyber::Component<localization::LocalizationEstimate> {
 public:
  FakeObstacleComponent() = default;

  ~FakeObstacleComponent() = default;

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  void OnRouting(const std::shared_ptr<routing::RoutingResponse>& routing);

 private:
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> obstacle_writer_;

  std::mutex mutex_;

  routing::RoutingResponse latest_routing_;
  localization::LocalizationEstimate latest_localization_;
};

CYBER_REGISTER_COMPONENT(FakeObstacleComponent)

}  // namespace fake_obstacle
}  // namespace apollo
