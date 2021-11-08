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
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::cyber::Rate;
using apollo::planning::DiscretizedTrajectory;
using apollo::planning::fortiss::MapOffset;

BarkRlPlanner::BarkRlPlanner() { logdir_ += "/apollo/data/log/"; }

common::Status BarkRlPlanner::Init(const PlanningConfig& config) {
  minimum_valid_speed_planning_ = 1.0;   // below our model is invalid
  standstill_velocity_threshold_ = 0.1;  // set velocity hard to zero below this

  LOG(INFO) << "Writing BarkRlPlanner Logs to " << logdir_.c_str();
  config_ = config;
  if (!config_.has_bark_rl_planner_config()) {
    AERROR << "Please provide BarkRlPlanner parameter file! " +
                  config_.DebugString();
    return Status(ErrorCode::PLANNING_ERROR,
                  "BarkRlPlanner parameters missing!");
  } else {
    AINFO << "BarkRlPlanner Configuration: "
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

  // Obstacles as obstacles
  std::vector<BarkObstacle> bark_obstacles;
  if (config_.bark_rl_planner_config().consider_obstacles()) {
    bark_obstacles = ConvertToBarkObstacles(
        frame->obstacles(), planning_init_point.relative_time());
  }

  ApolloToBarkMsg bark_request;
  bark_request.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
  bark_request.mutable_planning_init_point()->CopyFrom(planning_init_point);
  bark_request.set_velocity_desired(vDes);
  *bark_request.mutable_reference_line() = {discrete_reference_line.begin(),
                                            discrete_reference_line.end()};
  *bark_request.mutable_obstacles() = {bark_obstacles.begin(),
                                       bark_obstacles.end()};

  // send ApolloToBarkMsg message
  AINFO << "Sending ApolloToBarkMsg msg:" << bark_request.DebugString();
  apollo_to_bark_msg_writer_->Write(bark_request);

  // Plan ... wait for trajectory from bark ml
  Rate rate(1/receiver_wait_in_sec_);
  double waited_period = 0;
  bool received_reponse = false;
  while (waited_period <= bark_timeout_) {
    {
      std::lock_guard<std::mutex> lock(*mutex_);
      if (bark_response_ &&
          bark_response_->mutable_header()->timestamp_sec() >=
              bark_request.mutable_header()->timestamp_sec()) {
        AINFO << "Received BarkResponse:" << bark_response_->DebugString();
        received_reponse = true;
        break;
      }
    }
    rate.Sleep();
    waited_period += receiver_wait_in_sec_;
    AERROR << "Already waited for :" << waited_period << "s";
  }

  DiscretizedTrajectory apollo_traj;
  if (!received_reponse) {
    AERROR << "Did not receive a reponse from BARK";
    return Status(ErrorCode::PLANNING_ERROR, "bark interfacing failed!");
  } else {
    apollo_traj = DiscretizedTrajectory(bark_response_->planned_trajectory());
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
        obstacles_non_virtual, apollo_traj, ego_length, ego_width,
        ego_back_edge_to_center);
    if (obstacle_collision) {
      AERROR << "Planning success but collision with obstacle!";
    }
  }

  // Check resulting trajectory for collision with environment
  if (config_.bark_rl_planner_config().use_environment_polygon()) {
    if (fortiss::EnvironmentCollision(road_bounds, apollo_traj)) {
      AERROR << "Planning success but collision with environment!";
    }
  }

  // Planning success -> publish trajectory
  Status return_status;
  int subsampling = 0;
  if (config_.bark_rl_planner_config().use_smoothing()) {
    auto smoothed_apollo_trajectory = fortiss::SmoothTrajectory(
        apollo_traj, planning_init_point, logdir_.c_str(), map_offset, subsampling);
    if (smoothed_apollo_trajectory.first) {
      reference_line_info->SetTrajectory(smoothed_apollo_trajectory.second);
      reference_line_info->SetCost(0);
      reference_line_info->SetDrivable(true);
      return_status = Status::OK();
    } else {
      return_status = Status(ErrorCode::PLANNING_ERROR, "Smoothing failed!");
    }
  } else {
    reference_line_info->SetTrajectory(apollo_traj);
    reference_line_info->SetCost(0);
    reference_line_info->SetDrivable(true);
    return_status = Status::OK();
  }

  AINFO << "BarkRlPlanner::PlanOnReferenceLine() took "
        << (Clock::NowInSeconds() - start_time);

  return return_status;
}

void BarkRlPlanner::SetBarkInterfacePointers(
    const std::shared_ptr<cyber::Writer<ApolloToBarkMsg>>& request_writer,
    BarkResponse* response, std::mutex* mutex) {
  apollo_to_bark_msg_writer_ = request_writer;
  bark_response_ = response;
  mutex_ = mutex;
}

std::vector<BarkObstacle> BarkRlPlanner::ConvertToBarkObstacles(
    const std::vector<const Obstacle*>& obstacles, double timestep) const {
  std::vector<BarkObstacle> bark_obstacles;
  // TODO: fill in obstacle message
  return bark_obstacles;
}

}  // namespace planning
}  // namespace apollo
