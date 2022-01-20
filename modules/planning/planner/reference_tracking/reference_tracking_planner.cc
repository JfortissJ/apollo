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

#include "modules/planning/planner/reference_tracking/reference_tracking_planner.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/logger/logger_util.h"
#include "cyber/time/rate.h"
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
using apollo::common::math::PathMatcher;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::cyber::Rate;
using apollo::planning::DiscretizedTrajectory;
using apollo::planning::fortiss::MapOffset;

ReferenceTrackingPlanner::ReferenceTrackingPlanner() {
  logdir_ += "/apollo/data/log/";
}

common::Status ReferenceTrackingPlanner::Init(const PlanningConfig& config) {
  minimum_valid_speed_planning_ = 1.0;   // below our model is invalid
  standstill_velocity_threshold_ = 0.1;  // set velocity hard to zero below this

  LOG(INFO) << "Writing ReferenceTrackingPlanner Logs to " << logdir_.c_str();
  config_ = config;
  if (!config_.has_bark_rl_planner_config()) {
    AERROR << "Please provide ReferenceTrackingPlanner parameter file! " +
                  config_.DebugString();
    return Status(ErrorCode::PLANNING_ERROR,
                  "ReferenceTrackingPlanner parameters missing!");
  } else {
    AINFO << "ReferenceTrackingPlanner Configuration: "
          << config_.bark_rl_planner_config().DebugString();
  }

  return common::Status::OK();
}

void ReferenceTrackingPlanner::Stop() {}

Status ReferenceTrackingPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  const double timestep = Clock::NowInSeconds();
  AINFO << std::setprecision(15)
        << "############## Smoother-only planning called at t = " << timestep;
  double current_time = timestep;
  const double start_time = timestep;
  const MapOffset map_offset(config_.bark_rl_planner_config().pts_offset_x(),
                             config_.bark_rl_planner_config().pts_offset_y());

  double stop_dist;
  bool brake_for_inline_while_driving = true;
  fortiss::PlannerState planner_status = fortiss::DeterminePlannerState(
      planning_init_point.v(), reference_line_info, stop_dist,
      config_.bark_rl_planner_config().destination_distance_stop_threshold(),
      standstill_velocity_threshold_, minimum_valid_speed_planning_,
      brake_for_inline_while_driving);

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
          config_.bark_rl_planner_config()
              .cutoff_distance_reference_after_stop());

  // Map
  fortiss::RoadBoundaries road_bounds;
  road_bounds = fortiss::ToLeftAndRightBoundary(reference_line_info);

  // Target velocity
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

  // create reference traj from reference line and reference speed
  double acc_max = 3;    // TODO parameter!!
  double acc_decel = 3;  // TODO parameter!!
  DiscretizedTrajectory reference_traj;
  reference_traj.reserve(config_.bark_rl_planner_config().nr_steps());
  reference_traj.push_back(planning_init_point);

  // GetNearestPointAndS
  PathPoint point_on_line = PathMatcher::MatchToPath(
      discrete_reference_line, planning_init_point.path_point().x(),
      planning_init_point.path_point().y());
  double s = point_on_line.s();
  const double dt = config_.bark_rl_planner_config().ts();
  double t =
      planning_init_point.relative_time();  // TODO should be zero, is it zero??

  // start from 1 as planning_init_point is already first point of the traj
  for (int i = 1; i < config_.bark_rl_planner_config().nr_steps(); ++i) {
    t += dt;

    const double v0 = reference_traj.at(i - 1).v();
    const double vinterval = 0.5;
    if (v0 < vDes - vinterval) {
      s = s + v0 * dt + 0.5 * acc_max * dt * dt;  // accelerate
    } else if (v0 > vDes + vinterval) {
      s = s + fmax(0.0, v0 * dt - 0.5 * acc_decel * dt * dt);  // decelerate
    } else {
      s = s + vDes * dt;  // keep speed
    }

    // GetPointAtS
    TrajectoryPoint next_traj_pt;
    next_traj_pt.mutable_path_point()->CopyFrom(
        PathMatcher::MatchToPath(discrete_reference_line, s));
    next_traj_pt.set_v(vDes);  // TODO
    next_traj_pt.set_a(0.0);   // TODO
    next_traj_pt.set_da(0.0);  // TODO
    next_traj_pt.set_relative_time(t);
    reference_traj.push_back(next_traj_pt);
  }

  // call smoother
  Status return_status;
  int subsampling = 0;
  auto smoothed_apollo_trajectory =
      fortiss::SmoothTrajectory(reference_traj, planning_init_point,
                                logdir_.c_str(), map_offset, subsampling);
  if (smoothed_apollo_trajectory.first) {
    reference_line_info->SetTrajectory(smoothed_apollo_trajectory.second);
    reference_line_info->SetCost(0);
    reference_line_info->SetDrivable(true);
    return_status = Status::OK();
  } else {
    return_status = Status(ErrorCode::PLANNING_ERROR, "Smoothing failed!");
  }

  // Check resulting trajectory for collision with obstacles
  if (config_.bark_rl_planner_config().consider_obstacles()) {
    const auto& vehicle_config =
        common::VehicleConfigHelper::Instance()->GetConfig();
    const double ego_length = vehicle_config.vehicle_param().length();
    const double ego_width = vehicle_config.vehicle_param().width();
    const double ego_back_edge_to_center =
        vehicle_config.vehicle_param().back_edge_to_center();
    auto obstacles_non_virtual =
        fortiss::FilterNonVirtualObstacles(frame->obstacles());
    const bool obstacle_collision = CollisionChecker::InCollision(
        obstacles_non_virtual, smoothed_apollo_trajectory.second, ego_length,
        ego_width, ego_back_edge_to_center);
    if (obstacle_collision) {
      AERROR << "Planning success but collision with obstacle!";
    }
  }

  // Check resulting trajectory for collision with environment
  if (config_.bark_rl_planner_config().use_environment_polygon()) {
    if (fortiss::EnvironmentCollision(road_bounds,
                                      smoothed_apollo_trajectory.second)) {
      AERROR << "Planning success but collision with environment!";
    }
  }

  AINFO << "ReferenceTrackingPlanner::PlanOnReferenceLine() took "
        << (Clock::NowInSeconds() - start_time);

  return return_status;
}

}  // namespace planning
}  // namespace apollo
