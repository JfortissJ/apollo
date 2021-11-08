/******************************************************************************
 * Copyright 2021 fortiss GmbH
 * Authors: Tobias Kessler, Klemens Esterle
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

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {
namespace fortiss {

enum class PlannerState {
  DRIVING_TRAJECTORY = 0,
  START_TRAJECTORY = 1,
  STOP_TRAJECTORY = 2,
  STANDSTILL_TRAJECTORY = 3
};

struct MapOffset {
  MapOffset(double x, double y) : x(x), y(y) {}
  double x;
  double y;
};

struct RoadBoundaries {
  std::vector<apollo::common::math::Vec2d> left;
  std::vector<apollo::common::math::Vec2d> right;
};

RoadBoundaries ToLeftAndRightBoundary(ReferenceLineInfo* reference_line_info);

bool EnvironmentCollision(const RoadBoundaries& road_bounds,
                          const DiscretizedTrajectory& ego_trajectory);

std::vector<const Obstacle*> FilterNonVirtualObstacles(
    const std::vector<const Obstacle*>& obstacles);

void ConvertToPolyPts(const RoadBoundaries& road_bounds,
                      const MapOffset& map_offset, double poly_pts[]);

std::vector<apollo::common::PathPoint> ToDiscretizedReferenceLine(
    ReferenceLineInfo* reference_line_info, double stop_dist,
    double cutoff_dist_ref_after_stop);

std::pair<bool, DiscretizedTrajectory> SmoothTrajectory(
    const DiscretizedTrajectory& traj_in,
    const common::TrajectoryPoint& planning_init_point, const char logdir[],
    const MapOffset& map_offset, int subsampling);

void CreateStandstillTrajectory(
    const common::TrajectoryPoint& planning_init_point,
    ReferenceLineInfo* reference_line_info);

double CalculateSDistanceToStop(ReferenceLineInfo* reference_line_info,
                                bool brake_for_inlane);

PlannerState DeterminePlannerState(const double planning_init_v,
                                   ReferenceLineInfo* reference_line_info,
                                   double& stop_dist,
                                   const double destin_dist_thresh,
                                   const double standstill_velocity_thresh,
                                   const double minimum_valid_speed);

void FillTimeDerivativesInApolloTrajectory(DiscretizedTrajectory& traj);

}  // namespace fortiss
}  // namespace planning
}  // namespace apollo
