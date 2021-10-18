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

#include "modules/planning/common/fortiss_common.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/smoothers/trajectory_smoother_nlopt.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {
namespace fortiss {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::planning::DiscretizedTrajectory;

RoadBoundaries ToLeftAndRightBoundary(ReferenceLineInfo* reference_line_info) {
  std::vector<Vec2d> left_points, right_points;
  const hdmap::RouteSegments& segments = reference_line_info->Lanes();
  for (const auto& seg : segments) {
    const apollo::hdmap::LaneInfoConstPtr lane_info = seg.lane;

    for (auto& segment :
         (lane_info->lane().left_boundary().curve().segment())) {
      for (auto& p : (segment.line_segment().point())) {
        left_points.emplace_back(p.x(), p.y());
      }
    }
    for (auto& segment :
         (lane_info->lane().right_boundary().curve().segment())) {
      for (auto& p : (segment.line_segment().point())) {
        right_points.emplace_back(p.x(), p.y());
      }
    }
  }
  RoadBoundaries bounds;
  bounds.left = left_points;
  bounds.right = right_points;
  return bounds;
}

//! @note copied from apollo's CollisionChecker::InCollision() function
bool EnvironmentCollision(const RoadBoundaries& road_bounds,
                          const DiscretizedTrajectory& ego_trajectory) {
  // append reversed right points to the left points
  auto left_pts = road_bounds.left;
  left_pts.insert(left_pts.end(), road_bounds.right.rbegin(),
                  road_bounds.right.rend());
  // create polygon from the point vector
  Polygon2d envpoly(left_pts);

  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double ego_length = vehicle_config.vehicle_param().length();
  const double ego_width = vehicle_config.vehicle_param().width();
  const double ego_back_edge_to_center =
      vehicle_config.vehicle_param().back_edge_to_center();

  for (size_t i = 0; i < ego_trajectory.NumOfPoints(); ++i) {
    const auto& ego_point =
        ego_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));
    const auto ego_theta = ego_point.path_point().theta();

    Box2d ego_box({ego_point.path_point().x(), ego_point.path_point().y()},
                  ego_theta, ego_length, ego_width);

    // correct the inconsistency of reference point and center point
    // TODO(all): move the logic before constructing the ego_box
    double shift_distance = ego_length / 2.0 - ego_back_edge_to_center;
    Vec2d shift_vec(shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta));
    ego_box.Shift(shift_vec);
    Polygon2d carpoly = Polygon2d(ego_box);

    if (!envpoly.Contains(carpoly)) {
      AERROR << "Collision found at idx = " << i;
      return true;
    }
  }
  return false;
}

void ConvertToPolyPts(const RoadBoundaries& road_bounds,
                      const MapOffset& map_offset, double poly_pts[]) {
  int i = 0;
  for (auto it = road_bounds.left.begin(); it != road_bounds.left.end(); ++it) {
    poly_pts[2 * i] = it->x() - map_offset.x;
    poly_pts[2 * i + 1] = it->y() - map_offset.y;
    i++;
  }
  for (auto it = road_bounds.right.rbegin(); it != road_bounds.right.rend();
       ++it) {
    poly_pts[2 * i] = it->x() - map_offset.x;
    poly_pts[2 * i + 1] = it->y() - map_offset.y;
    i++;
  }
}

std::vector<PathPoint> ToDiscretizedReferenceLine(
    ReferenceLineInfo* reference_line_info, double stop_distance,
    double cutoff_dist_ref_after_stop) {
  const double s_vehicle = reference_line_info->AdcSlBoundary().end_s();
  const PlanningTarget& planning_target =
      reference_line_info->planning_target();
  double s_stop_for_obstacle = stop_distance + s_vehicle;
  AINFO << "s_stop_for_obstacle: " << s_stop_for_obstacle;
  AINFO << "stop_distance: " << stop_distance;
  AINFO << "s_vehicle: " << s_vehicle;

  // ref_points start at beginning of road, not at pose of vehicle
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point :
       reference_line_info->reference_line().reference_points()) {
    PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_theta(ref_point.heading());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());

    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();
      double dy = path_point.y() - path_points.back().y();
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.set_s(s);

    if (planning_target.has_stop_point() &&
        (s > s_stop_for_obstacle + cutoff_dist_ref_after_stop)) {
      AINFO << "cutting off reference after s:" << s;
      break;
    }
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

// TODO: maybe define proto file with params for fortiss planner
std::pair<bool, DiscretizedTrajectory> SmoothTrajectory(
    const DiscretizedTrajectory& traj_in,
    const common::TrajectoryPoint& planning_init_point, const char logdir[],
    const MapOffset& map_offset) {
  int subsampling = 3;
  TrajectorySmootherNLOpt tsm =
      TrajectorySmootherNLOpt(logdir, map_offset.x, map_offset.y);
  tsm.InitializeProblem(subsampling, traj_in, planning_init_point);
  AINFO << "Planning init point is " << planning_init_point.DebugString();
  tsm.Optimize();
  auto traj = tsm.GetOptimizedTrajectory();
  if (tsm.ValidateSmoothingSolution()) {
    for (size_t idx = 0; idx < traj.size(); ++idx) {
      AINFO << "Smoothed trajectory at idx = " << idx << " : "
            << traj.at(idx).DebugString();
    }
    return {true, traj};
  } else {
    AERROR << "Trajectory smoothing not valid or failed!";
    return {false, traj};
  }
}

void CreateStandstillTrajectory(const TrajectoryPoint& planning_init_point,
                                ReferenceLineInfo* reference_line_info) {
  DiscretizedTrajectory standstill_trajectory;
  TrajectoryPoint trajectory_point;
  trajectory_point.mutable_path_point()->set_x(
      planning_init_point.path_point().x());
  trajectory_point.mutable_path_point()->set_y(
      planning_init_point.path_point().y());
  trajectory_point.mutable_path_point()->set_s(0.0);
  trajectory_point.mutable_path_point()->set_theta(
      planning_init_point.path_point().theta());
  trajectory_point.set_v(0.0);  // set to zero!
  trajectory_point.set_a(0.0);  // TODO better set to something negative?
  trajectory_point.set_relative_time(planning_init_point.relative_time());
  standstill_trajectory.AppendTrajectoryPoint(trajectory_point);

  reference_line_info->SetTrajectory(standstill_trajectory);
  reference_line_info->SetCost(0);  // TODO necessary?
  reference_line_info->SetDrivable(true);

  AINFO << "Setting Standstill trajectory at point t = "
        << planning_init_point.relative_time()
        << " x = " << planning_init_point.path_point().x()
        << " y = " << planning_init_point.path_point().y();
}

double CalculateSDistanceToStop(ReferenceLineInfo* reference_line_info,
                                bool brake_for_inlane) {
  double stop_distance = reference_line_info->SDistanceToDestination();
  AINFO << "Goal distance is " << stop_distance;
  for (const Obstacle* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    bool in_lane = reference_line_info->reference_line().IsOnLane(
        obstacle->PerceptionSLBoundary());
    AINFO << "obstacle " << obstacle->Id()
          << " perception line sl boundary: s_start["
          << obstacle->PerceptionSLBoundary().start_s()
          << "], HasTrajectory: " << obstacle->HasTrajectory()
          << ", IsLaneBlocking: " << obstacle->IsLaneBlocking()
          << ", InLane: " << in_lane;
    if ((!obstacle->HasTrajectory() && obstacle->IsLaneBlocking()) ||
        (!obstacle->HasTrajectory() && in_lane && brake_for_inlane)) {
      // Calculation similar to ReferenceLineInfo::SDistanceToDestination()
      double d_i = obstacle->PerceptionSLBoundary().start_s() -
                   reference_line_info->AdcSlBoundary().end_s();
      if (d_i >= 0) {  // obstacle is in front of the car
        stop_distance = std::min(stop_distance, d_i);
      }
    }
  }
  return stop_distance;
}

PlannerState DeterminePlannerState(const double planning_init_v,
                                   ReferenceLineInfo* reference_line_info,
                                   double& stop_dist,
                                   const double destin_dist_thresh,
                                   const double standstill_velocity_thresh,
                                   const double minimum_valid_speed) {
  PlannerState status;
  // issue hard stop trajectory without optimization if velocity is
  // low enough and goal is nearer than this

  // calculate the stop distance under the assumption to not brake for every
  // obstacle in the lane
  bool brake_for_inlane = false;
  stop_dist = CalculateSDistanceToStop(reference_line_info, brake_for_inlane);

  if (stop_dist < destin_dist_thresh) {  // Approaching end or at end
    if (planning_init_v <= standstill_velocity_thresh) {  // Standstill at end
      status = PlannerState::STANDSTILL_TRAJECTORY;
    } else if (planning_init_v <=
               minimum_valid_speed) {  // Create stopping traj
      status = PlannerState::STOP_TRAJECTORY;
      // overwriting start stop_distance!!
      bool brake_for_inlane = true;
      stop_dist =
          CalculateSDistanceToStop(reference_line_info, brake_for_inlane);
    } else {  // Let miqp optimizier plan the stopping traj
      status = PlannerState::DRIVING_TRAJECTORY;
    }
  } else {  // Driving or want to start driving
    if (planning_init_v <= minimum_valid_speed) {  // Low speed -> modify start
      status = PlannerState::START_TRAJECTORY;
      // overwriting start stop_distance!!
      bool brake_for_inlane =
          false;  // to be prone against obstacles that slightly collide with
                  // lane (referenceLineGenerator cannot handle close obstacles
                  // well -> would not stop otherwise)
      stop_dist =
          CalculateSDistanceToStop(reference_line_info, brake_for_inlane);
    } else {  // Driving: default case
      status = PlannerState::DRIVING_TRAJECTORY;
    }
  }
  AINFO << "Planner status is: " << static_cast<int>(status)
        << " v_init = " << planning_init_v << " stop dist = " << stop_dist;

  return status;
}

void FillTimeDerivativesInApolloTrajectory(DiscretizedTrajectory& traj) {
  if (traj.size() < 2) {  // no derivatives possible
    return;
  }
  for (size_t i = 0; i < traj.size() - 1; ++i) {
    double diff_t = (traj[i + 1].relative_time() - traj[i].relative_time());

    double diff_v = (traj[i + 1].v() - traj[i].v());
    double a = diff_v / diff_t;
    traj[i].set_a(a);

    double diff_kappa = (traj[i + 1].mutable_path_point()->kappa() -
                         traj[i].mutable_path_point()->kappa());
    double dkappa = diff_kappa / diff_t;
    traj[i].mutable_path_point()->set_dkappa(dkappa);
  }
  traj[traj.size() - 1].set_a(0.0);
  traj[traj.size() - 1].mutable_path_point()->set_dkappa(0.0);
}

}  // namespace fortiss
}  // namespace planning
}  // namespace apollo