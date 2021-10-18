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

/**
 * @file
 **/

#include "modules/planning/planner/bark_rl_wrapper/bark_rl_planner.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/logger/logger_util.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/fortiss_common.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/constraint_checker/constraint_checker.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::planning::DiscretizedTrajectory;
using apollo::planning::fortiss::MapOffset;

BarkRlPlanner::BarkRlPlanner() {}

common::Status BarkRlPlanner::Init(const PlanningConfig& config) {
  minimum_valid_speed_planning_ = 1.0;   // below our model is invalid
  standstill_velocity_threshold_ = 0.1;  // set velocity hard to zero below this

  config_ = config;
  if (!config_.has_bark_rl_planner_config()) {
    AERROR << "Please provide miqp planner parameter file! " +
                  config_.DebugString();
    return Status(ErrorCode::PLANNING_ERROR,
                  "miqp planner parameters missing!");
  } else {
    AINFO << "MIQP Planner Configuration: "
          << config_.bark_rl_planner_config().DebugString();
  }

  return common::Status::OK();
}

void BarkRlPlanner::Stop() {}

Status BarkRlPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  const double timestep = Clock::NowInSeconds();
  AINFO << std::setprecision(15)
        << "############## BARK-RL Planner called at t = " << timestep;
  double current_time = timestep;
  const double start_time = timestep;
  const MapOffset map_offset(config_.bark_rl_planner_config().pts_offset_x(),
                             config_.bark_rl_planner_config().pts_offset_y());

  double stop_dist;
  fortiss::PlannerState planner_status = fortiss::DeterminePlannerState(
      planning_init_point.v(), reference_line_info, stop_dist,
      config_.bark_rl_planner_config().destination_distance_stop_threshold(),
      standstill_velocity_threshold_, minimum_valid_speed_planning_);

  if (planner_status == fortiss::PlannerState::STANDSTILL_TRAJECTORY) {
    fortiss::CreateStandstillTrajectory(planning_init_point,
                                        reference_line_info);
    return Status::OK();
  }

  // Obtain a reference line and transform it to the PathPoint format.
  reference_line_info->set_is_on_reference_line();
  std::vector<PathPoint> discrete_reference_line =
      fortiss::ToDiscretizedReferenceLine(
          reference_line_info, stop_dist,
          config_.bark_rl_planner_config().cutoff_distance_reference_after_stop());

  // Map
  // fortiss::RoadBoundaries road_bounds;
  // road_bounds = fortiss::ToLeftAndRightBoundary(reference_line_info);

  // Initial State
  // TODO

  // // Target velocity
  double vDes;
  const double dist_start_slowdown =
      config_.bark_rl_planner_config().distance_start_slowdown();
  const double dist_stop_before =
      config_.bark_rl_planner_config().distance_stop_before();
  if ((stop_dist - dist_stop_before < dist_start_slowdown) &&
      (planner_status != fortiss::PlannerState::START_TRAJECTORY)) {
    vDes = 0;
  } else {
    vDes = FLAGS_default_cruise_speed;
  }

  // Obstacles as obstacles
  if (config_.bark_rl_planner_config().consider_obstacles()) {
    bool success = ProcessObstacles(
        frame->obstacles(), planning_init_point.relative_time());
    if (success) {
      AERROR << "Processing of obstacles failed";
      return Status(ErrorCode::PLANNING_ERROR, "processing of obstacles failed!");
    }
  }

  // // Plan
  // DiscretizedTrajectory apollo_traj;
  // if (planner_status == fortiss::PlannerState::START_TRAJECTORY ||
  //     planner_status == fortiss::PlannerState::STOP_TRAJECTORY) {
  //   AERROR << "Start/Stop Trajectory, using reference instead of miqp
  //   solution"; GetRawCLastReferenceTrajectoryCMiqpPlaner(
  //       planner_, egoCarIdx_, planning_init_point.relative_time(), traj,
  //       size);
  //   apollo_traj = RawCTrajectoryToApolloTrajectory(traj, size, false);
  // } else {
  //   current_time = Clock::NowInSeconds();
  //   bool success = PlanCBarkRlPlanner(planner_, timestep);
  //   AINFO << "Miqp planning Time [s] = "
  //         << (Clock::NowInSeconds() - current_time);
  //   current_time = Clock::NowInSeconds();

  //   // Planning failed
  //   if (!success) {
  //     AINFO << "Planning failed";
  //     return Status(ErrorCode::PLANNING_ERROR, "miqp planner failed!");
  //   }

  //   // Get trajectory from miqp planner
  //   AINFO << "Planning Success!";
  //   // trajectories shall start at t=0 with an offset of
  //   // planning_init_point.relative_time()
  //   GetRawCMiqpTrajectoryCBarkRlPlanner(
  //       planner_, egoCarIdx_, planning_init_point.relative_time(), traj,
  //       size);
  //   apollo_traj = RawCTrajectoryToApolloTrajectory(traj, size, true);
  // }

  // if (config_.bark_rl_planner_config().minimum_percentage_valid_miqp_points() *
  //         config_.bark_rl_planner_config().nr_steps() >
  //     apollo_traj.size()) {
  //   AERROR << "Trajectory has too many invalid points, setting error state";
  //   return Status(ErrorCode::PLANNING_ERROR, "invalid points!");
  // }

  // // Check resulting trajectory for collision with obstacles
  // if (config_.bark_rl_planner_config().consider_obstacles()) {
  //   const auto& vehicle_config =
  //       common::VehicleConfigHelper::Instance()->GetConfig();
  //   const double ego_length = vehicle_config.vehicle_param().length();
  //   const double ego_width = vehicle_config.vehicle_param().width();
  //   const double ego_back_edge_to_center =
  //       vehicle_config.vehicle_param().back_edge_to_center();
  //   auto obstacles_non_virtual =
  //   FilterNonVirtualObstacles(frame->obstacles()); const bool
  //   obstacle_collision = CollisionChecker::InCollision(
  //       obstacles_non_virtual, apollo_traj, ego_length, ego_width,
  //       ego_back_edge_to_center);
  //   if (obstacle_collision) {
  //     AERROR << "Planning success but collision with obstacle!";
  //   }
  // }

  // // Check resulting trajectory for collision with environment
  // if (config_.bark_rl_planner_config().use_environment_polygon()) {
  //   if (fortiss::EnvironmentCollision(road_bounds, apollo_traj)) {
  //     AERROR << "Planning success but collision with environment!";
  //   }
  // }

  // // Planning success -> publish trajectory
  // Status return_status;
  // if (config_.bark_rl_planner_config().use_smoothing()) {
  //   auto smoothed_apollo_trajectory = fortiss::SmoothTrajectory(
  //       apollo_traj, planning_init_point, logdir_.c_str(), map_offset);
  //   if (smoothed_apollo_trajectory.first) {
  //     reference_line_info->SetTrajectory(smoothed_apollo_trajectory.second);
  //     reference_line_info->SetCost(0);
  //     reference_line_info->SetDrivable(true);
  //     return_status = Status::OK();
  //   } else {
  //     return_status = Status(ErrorCode::PLANNING_ERROR, "Smoothing failed!");
  //   }
  // } else {
  //   reference_line_info->SetTrajectory(apollo_traj);
  //   reference_line_info->SetCost(0);
  //   reference_line_info->SetDrivable(true);
  //   return_status = Status::OK();
  // }

  // AINFO << "MIQP Planner postprocess took [s]: "
  //       << (Clock::NowInSeconds() - current_time);
  // AINFO << "BarkRlPlanner::PlanOnReferenceLine() took "
  //       << (Clock::NowInSeconds() - start_time);

  // return return_status;
}

}  // namespace planning
}  // namespace apollo
