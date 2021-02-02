/******************************************************************************
 * Copyright 2020 fortiss GmbH
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

#include "modules/planning/planner/miqp/miqp_planner.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/constraint_checker/constraint_checker.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::PathMatcher;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::planning::DiscretizedTrajectory;

static const double X_OFFSET = 692000;     // TODO: move to proto file
static const double Y_OFFSET = 5.339e+06;  // TODO: move to proto file

namespace {

std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points,
    const PlanningTarget& planning_target) {
  double s = 0.0;

  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
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

    // if (planning_target.has_stop_point() &&
    //     (s > planning_target.stop_point().s())) {
    //   AERROR << "cutting off reference after s:" << s;
    //   break;
    // }
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

std::pair<std::vector<Vec2d>, std::vector<Vec2d>> ToLeftAndRightBoundary(
    ReferenceLineInfo* reference_line_info) {
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
  return std::make_pair(left_points, right_points);
}

}  // namespace

common::Status MiqpPlanner::Init(const PlanningConfig& config) {
  MiqpPlannerSettings settings = DefaultSettings();
  planner_ = NewCMiqpPlannerSettings(settings);
  firstrun_ = true;
  egoCarIdx_ = -1;  // invalid
  ActivateDebugFileWriteCMiqpPlanner(planner_, "/apollo/data/log",
                                     "miqp_planner_");
  config_ = config;
  if (!config_.has_miqp_planner_config()) {
    AERROR << "Please provide miqp planner parameter file! " +
                  config_.DebugString();
    return Status(ErrorCode::PLANNING_ERROR,
                  "miqp planner parameters missing!");
  } else {
    AINFO << "MIQP Planner Configuration: "
          << config_.miqp_planner_config().DebugString();
  }
  return common::Status::OK();
}

void MiqpPlanner::Stop() { DelCMiqpPlanner(planner_); }

Status MiqpPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  const double timestep = Clock::NowInSeconds();
  AERROR << std::setprecision(15)
         << "---------- PlanOnReferenceLine() of MIQP planner called at "
            "timestep = "
         << timestep << " ----------";
  double current_time = timestep;

  // Initialized raw C trajectory output
  const int N = GetNCMiqpPlanner(planner_);
  double traj[TRAJECTORY_SIZE * N];
  int size;

  // Obtain a reference line and transform it to the PathPoint format.
  reference_line_info->set_is_on_reference_line();
  std::vector<PathPoint> discrete_reference_line = ToDiscretizedReferenceLine(
      reference_line_info->reference_line().reference_points(),
      reference_line_info->planning_target());

  // Reference line to raw c format
  const int ref_size =
      discrete_reference_line.size();  // aka N optimization support points
  double ref[ref_size * 2];
  for (int i = 0; i < ref_size; ++i) {
    PathPoint refPoint = discrete_reference_line.at(i);
    // AERROR << refPoint.x() - X_OFFSET<< ", " << refPoint.y() - Y_OFFSET;
    ref[2 * i] = refPoint.x() - X_OFFSET;
    ref[2 * i + 1] = refPoint.y() - Y_OFFSET;
  }

  // Map
if (config_.miqp_planner_config().use_environment_polygon()) {
  std::vector<Vec2d> left_pts, right_pts;
  std::tie(left_pts, right_pts) = ToLeftAndRightBoundary(reference_line_info);
  const int poly_size = left_pts.size() + right_pts.size();
  double poly_pts[poly_size * 2];
  ConvertToPolyPts(left_pts, right_pts, poly_pts);
  UpdateConvexifiedMapCMiqpPlaner(planner_, poly_pts, poly_size);
}

  AINFO << "ReferenceLine Time = "
        << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  double initial_state[6];
  ConvertToInitialStateSecondOrder(planning_init_point, initial_state);

  bool track_ref_pos;
  double vDes;
  double deltaSDes;

  const double dist_start_slowdown = 15.0;  // TODO: move to proto file
  const double dist_stop_before = 5;        // TODO: move to proto file

  auto distGoal = reference_line_info->SDistanceToDestination();
  if (distGoal - dist_stop_before < dist_start_slowdown) {
    track_ref_pos = false;
    vDes = 0;
    deltaSDes = std::max(0.0, distGoal - dist_stop_before);
    AERROR << "Close to goal, tracking velocity instead of pts, distGoal:"
           << distGoal;
  } else {
    track_ref_pos = true;
    vDes = FLAGS_default_cruise_speed;
    deltaSDes = 5;  // TODO: move to proto file
  }

  // Add/update ego car
  if (firstrun_) {
    egoCarIdx_ = AddCarCMiqpPlanner(planner_, initial_state, ref, ref_size,
                                    vDes, deltaSDes, timestep, track_ref_pos);
    firstrun_ = false;
    AINFO << "Added ego car Time = "
          << (Clock::NowInSeconds() - current_time) * 1000;
  } else {
    UpdateCarCMiqpPlanner(planner_, egoCarIdx_, initial_state, ref, ref_size,
                          timestep, track_ref_pos);
    UpdateDesiredVelocityCMiqpPlanner(planner_, egoCarIdx_, vDes, deltaSDes);
    AINFO << "Update ego car Time = "
          << (Clock::NowInSeconds() - current_time) * 1000;
  }
  current_time = Clock::NowInSeconds();

  // Plan
  bool success = PlanCMiqpPlanner(planner_, timestep);
  AINFO << "Miqp planning Time = "
        << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // Planning failed
  if (!success) {
    AERROR << "Planning failed";
    // TODO Generate some error trajectory
    return Status(ErrorCode::PLANNING_ERROR, "miqp planner failed!");
  }

  // Planning success -> publish trajectory
  AINFO << "Planning Success!";
  // trajectories shall start at t=0 with an offset of
  // planning_init_point.relative_time()
  GetRawCMiqpTrajectoryCMiqpPlanner(
      planner_, egoCarIdx_, planning_init_point.relative_time(), traj, size);
  DiscretizedTrajectory apollo_traj =
      RawCTrajectoryToApolloTrajectory(traj, size);
  reference_line_info->SetTrajectory(apollo_traj);
  reference_line_info->SetCost(0);  // TODO necessary?
  reference_line_info->SetDrivable(true);

  AINFO << "MIQP Planner postprocess took: "
        << (Clock::NowInSeconds() - current_time) * 1000;

  // // debug outputs:
  // int r = size;
  // int c = TRAJECTORY_SIZE;
  // for (int i = 0; i < r; ++i) {
  //   for (int j = 0; j < c; ++j) {
  //     std::cout << traj[i * c + j] << "\t\t";
  //   }
  //   std::cout << std::endl;
  // }

  return Status::OK();
}

apollo::planning::DiscretizedTrajectory
MiqpPlanner::BarkTrajectoryToApolloTrajectory(double traj[], int size) {
  const int TIME_POSITION = 0;
  const int X_POSITION = 1;
  const int Y_POSITION = 2;
  const int THETA_POSITION = 3;
  const int VEL_POSITION = 4;
  const int MIN_STATE_SIZE = 5;

  double s = 0.0f;
  double lastx = traj[0 + X_POSITION];
  double lasty = traj[0 + Y_POSITION];

  DiscretizedTrajectory apollo_trajectory;
  for (int trajidx = 0; trajidx < size; ++trajidx) {
    double time = traj[trajidx * MIN_STATE_SIZE + TIME_POSITION];
    double x = traj[trajidx * MIN_STATE_SIZE + X_POSITION] + X_OFFSET;
    double y = traj[trajidx * MIN_STATE_SIZE + Y_POSITION] + Y_OFFSET;
    double theta = traj[trajidx * MIN_STATE_SIZE + THETA_POSITION];
    double v = traj[trajidx * MIN_STATE_SIZE + VEL_POSITION];
    s += sqrt(pow(x - lastx, 2) + pow(y - lasty, 2));

    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_x(x);
    trajectory_point.mutable_path_point()->set_y(y);
    trajectory_point.mutable_path_point()->set_s(s);
    trajectory_point.mutable_path_point()->set_theta(theta);
    // trajectory_point.mutable_path_point()->set_kappa(kappa); //TODO
    trajectory_point.set_v(v);
    // trajectory_point.set_a(a); //TODO
    trajectory_point.set_relative_time(time);
    apollo_trajectory.AppendTrajectoryPoint(trajectory_point);

    lastx = x;
    lasty = y;
  }

  return apollo_trajectory;
}

apollo::planning::DiscretizedTrajectory
MiqpPlanner::RawCTrajectoryToApolloTrajectory(double traj[], int size) {
  double s = 0.0f;
  double lastx = traj[0 + TRAJECTORY_X_IDX];
  double lasty = traj[0 + TRAJECTORY_Y_IDX];

  DiscretizedTrajectory apollo_trajectory;
  for (int trajidx = 0; trajidx < size; ++trajidx) {
    const double time = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_TIME_IDX];
    const double x =
        traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_X_IDX] + X_OFFSET;
    const double y =
        traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_Y_IDX] + Y_OFFSET;
    const double vx = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_VX_IDX];
    const double vy = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_VY_IDX];
    const double ax = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_AX_IDX];
    const double ay = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_AY_IDX];
    // const double ux = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_UX_IDX];
    // const double uy = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_UY_IDX];
    const double theta = atan2(vy, vx);
    const double v = vx / cos(theta);
    const double a = ax / cos(theta);
    s += sqrt(pow(x - lastx, 2) + pow(y - lasty, 2));
    const double kappa =
        (vx * ay - ax * vy) / (pow((vx * vx + vy * vy), 3 / 2));

    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_x(x);
    trajectory_point.mutable_path_point()->set_y(y);
    trajectory_point.mutable_path_point()->set_s(s);
    trajectory_point.mutable_path_point()->set_theta(theta);
    trajectory_point.mutable_path_point()->set_kappa(kappa);
    // trajectory_point.mutable_path_point()->set_dkappa(dkappa);
    // trajectory_point.mutable_path_point()->set_dkappa(ddkappa);
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    // trajectory_point.set_da(jerk);
    trajectory_point.set_relative_time(time);
    apollo_trajectory.AppendTrajectoryPoint(trajectory_point);

    lastx = x;
    lasty = y;
  }

  return apollo_trajectory;
}

void MiqpPlanner::ConvertToInitialStateSecondOrder(
    const TrajectoryPoint& planning_init_point, double initial_state[]) {
  // Intial position to raw c format
  AERROR << "planning_init_point = "
         << " x:" << planning_init_point.path_point().x()
         << ", y:" << planning_init_point.path_point().y()
         << ", v:" << planning_init_point.v()
         << ", a:" << planning_init_point.a()
         << ", theta:" << planning_init_point.path_point().theta();

  double theta = planning_init_point.path_point().theta();
  // cplex throws an exception if vel=0
  double vel = std::max(planning_init_point.v(), 0.1);

  initial_state[0] = planning_init_point.path_point().x() - X_OFFSET;
  initial_state[1] = vel * cos(theta);
  initial_state[2] = planning_init_point.a() * cos(theta);
  initial_state[3] = planning_init_point.path_point().y() - Y_OFFSET;
  initial_state[4] = vel * sin(theta);
  initial_state[5] = planning_init_point.a() * sin(theta);
  AERROR << std::setprecision(15)
         << "initial state in miqp = x:" << initial_state[0]
         << ", xd:" << initial_state[1] << ", xdd:" << initial_state[2]
         << ", y:" << initial_state[3] << ", yd:" << initial_state[4]
         << ", ydd:" << initial_state[5];
}

void MiqpPlanner::ConvertToPolyPts(const std::vector<Vec2d>& left_pts,
                                   const std::vector<Vec2d>& right_pts,
                                   double poly_pts[]) {
  int i = 0;
  for (auto it = left_pts.begin(); it != left_pts.end(); ++it) {
    poly_pts[2 * i] = it->x() - X_OFFSET;
    poly_pts[2 * i + 1] = it->y() - Y_OFFSET;
    i++;
  }
  for (auto it = right_pts.rbegin(); it != right_pts.rend(); ++it) {
    poly_pts[2 * i] = it->x() - X_OFFSET;
    poly_pts[2 * i + 1] = it->y() - Y_OFFSET;
    i++;
  }
}

MiqpPlannerSettings MiqpPlanner::DefaultSettings() {
  MiqpPlannerSettings s = MiqpPlannerSettings();
  auto& conf = config_.miqp_planner_config();

  if (conf.has_nr_regions()) {
    s.nr_regions = conf.nr_regions();
  } else {
    s.nr_regions = 16;
  }
  s.nr_steps = 20;
  s.nr_neighbouring_possible_regions = 1;
  s.ts = 0.25;
  s.max_solution_time = 10;
  s.relative_mip_gap_tolerance = 0.1;
  s.mipdisplay = 2;
  s.mipemphasis = 1;
  s.relobjdif = 0.7;
  s.cutpass = 0;
  s.probe = 0;
  s.repairtries = 0;
  s.rinsheur = 0;
  s.varsel = 0;
  s.mircuts = 0;
  s.precision = 12;
  s.constant_agent_safety_distance_slack = 3;
  s.minimum_region_change_speed = 2;
  s.lambda = 0.5;
  s.wheelBase = common::VehicleConfigHelper::Instance()
                    ->GetConfig()
                    .vehicle_param()
                    .wheel_base();
  float collision_radius_add;
  if (conf.has_collision_radius_add()) {
    collision_radius_add = conf.collision_radius_add();
  } else {
    collision_radius_add = 0.0;
  }
  s.collisionRadius = common::VehicleConfigHelper::Instance()
                              ->GetConfig()
                              .vehicle_param()
                              .width() /
                          2 +
                      collision_radius_add;
  s.slackWeight = 30;
  if (conf.has_jerk_weight()) {
    s.jerkWeight = conf.jerk_weight();
  } else {
    s.jerkWeight = 1.0;
  }
  if (conf.has_position_weight()) {
    s.positionWeight = conf.position_weight();
  } else {
    s.positionWeight = 2.0;
  }
  if (conf.has_velocity_weight()) {
    s.velocityWeight = conf.velocity_weight();
  } else {
    s.velocityWeight = 0.0;
  }
  s.acclerationWeight = 0;
  s.simplificationDistanceMap = 0.2;
  s.refLineInterpInc = 0.2;
  s.scaleVelocityForReferenceLongerHorizon = 2.0;
  s.cplexModelpath =
      "../bazel-bin/modules/planning/libplanning_component.so.runfiles/"
      "miqp_planner/cplex_modfiles/";
  s.useSos = false;
  s.useBranchingPriorities = true;
  s.warmstartType = MiqpPlannerWarmstartType::BOTH_WARMSTART_STRATEGIES;
  return s;
}

}  // namespace planning
}  // namespace apollo
