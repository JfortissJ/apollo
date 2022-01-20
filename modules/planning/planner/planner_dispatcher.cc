/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planner/planner_dispatcher.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/public_road/public_road_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#ifdef USE_PLANNER_MIQP
#include "modules/planning/planner/miqp/miqp_planner.h"
#endif
#include "modules/planning/planner/bark_rl_wrapper/bark_rl_planner.h"
#include "modules/planning/planner/reference_tracking/reference_tracking_planner.h"

namespace apollo {
namespace planning {

void PlannerDispatcher::RegisterPlanners() {
  planner_factory_.Register(
      PlannerType::RTK, []() -> Planner* { return new RTKReplayPlanner(); });
  planner_factory_.Register(PlannerType::PUBLIC_ROAD, []() -> Planner* {
    return new PublicRoadPlanner();
  });
  planner_factory_.Register(PlannerType::LATTICE,
                            []() -> Planner* { return new LatticePlanner(); });

  planner_factory_.Register(PlannerType::NAVI,
                            []() -> Planner* { return new NaviPlanner(); });
#ifdef USE_PLANNER_MIQP
  planner_factory_.Register(PlannerType::MIQP,
                            []() -> Planner* { return new MiqpPlanner(); });
#endif
  planner_factory_.Register(PlannerType::BARK_RL,
                            []() -> Planner* { return new BarkRlPlanner(); });
  planner_factory_.Register(PlannerType::REFERENCE_TRACKING,
                            []() -> Planner* { return new ReferenceTrackingPlanner(); });
}

}  // namespace planning
}  // namespace apollo
