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

static const double X_OFFSET = 692000;
static const double Y_OFFSET = 5.339e+06;

namespace {

std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
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
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

std::pair<std::vector<Vec2d>, std::vector<Vec2d>> ToLeftAndRightBoundary(
    ReferenceLineInfo* reference_line_info) {
  const hdmap::RouteSegments& lanes = reference_line_info->Lanes();
  const apollo::hdmap::LaneInfoConstPtr lane_info =
      lanes.at(0).lane;  // for now, we only use first lane element

  std::vector<Vec2d> left_points;
  for (auto& segment : (lane_info->lane().left_boundary().curve().segment())) {
    for (auto& p : (segment.line_segment().point())) {
      left_points.emplace_back(p.x(), p.y());
    }
  }
  std::vector<Vec2d> right_points;
  for (auto& segment : (lane_info->lane().right_boundary().curve().segment())) {
    for (auto& p : (segment.line_segment().point())) {
      right_points.emplace_back(p.x(), p.y());
    }
  }
  return std::make_pair(left_points, right_points);
}

}  // namespace

Status MiqpPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  const double timestep = Clock::NowInSeconds();
  AERROR << "---------- PlanOnReferenceLine() of MIQP planner called at "
            "timestep = "
         << timestep << " ----------";
  double current_time = timestep;

  // Initialize miqp planner
  MiqpPlannerSettings settings = DefaultSettings();
  CMiqpPlanner planner = NewCMiqpPlannerSettings(settings);
  ActivateDebugFileWriteCMiqpPlanner(planner, "/apollo/data/log", "test_");

  // Initialized raw C trajectory output
  const int N = GetNCMiqpPlanner(planner);
  double traj[TRAJECTORY_SIZE * N];
  int size;

  // Obtain a reference line and transform it to the PathPoint format.
  reference_line_info->set_is_on_reference_line();
  std::vector<PathPoint> discrete_reference_line = ToDiscretizedReferenceLine(
      reference_line_info->reference_line().reference_points());

  // Reference line to raw c format
  const int ref_size =
      discrete_reference_line.size();  // aka N optimization support points
  double ref[ref_size * 2];
  for (int i = 0; i < ref_size; ++i) {
    PathPoint refPoint = discrete_reference_line.at(i);
    // AERROR << refPoint.x() << ", " << refPoint.y();
    ref[2 * i] = refPoint.x() - X_OFFSET;
    ref[2 * i + 1] = refPoint.y() - Y_OFFSET;
  }

  // Map
  std::vector<Vec2d> left_pts, right_pts;
  std::tie(left_pts, right_pts) = ToLeftAndRightBoundary(reference_line_info);

  const int poly_size = left_pts.size() + right_pts.size();
  double poly_pts[poly_size * 2];

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

  UpdateConvexifiedMapCMiqpPlaner(planner, poly_pts, poly_size);

  AINFO << "ReferenceLine Time = "
        << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // Intial position to raw c format
  AERROR << "planning_init_point = v:" << planning_init_point.v()
         << ", theta:" << planning_init_point.path_point().theta();
  double initial_state[6];
  double theta = planning_init_point.path_point().theta();
  double vel = std::max(planning_init_point.v(),
                        0.1);  // cplex throws an exception if vel=0
  initial_state[0] = planning_init_point.path_point().x() - X_OFFSET;
  initial_state[1] = vel * cos(theta);
  initial_state[2] = planning_init_point.a() * cos(theta);  // is that correct?
  initial_state[3] = planning_init_point.path_point().y() - Y_OFFSET;
  initial_state[4] = vel * sin(theta);
  initial_state[5] = planning_init_point.a() * sin(theta);  // is that correct?
  AERROR << "initial state miqp = x:" << initial_state[0]
         << ", xd:" << initial_state[1] << ", xdd:" << initial_state[2]
         << ", y:" << initial_state[3] << ", yd:" << initial_state[4]
         << ", ydd:" << initial_state[5];
  double vDes = FLAGS_default_cruise_speed;

  // Add ego car
  int idx =
      AddCarCMiqpPlanner(planner, initial_state, ref, ref_size, vDes, timestep);
  AINFO << "Added ego car Time = "
        << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // Plan
  bool success = PlanCMiqpPlanner(planner, timestep);
  AINFO << "Miqp planning Time = "
        << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // Planning failed
  if (!success) {
    AERROR << "Planning failed";
    return Status(ErrorCode::PLANNING_ERROR, "miqp planner failed!");

    // This code snipped from apollo could be an alternative
    // if (FLAGS_enable_backup_trajectory) {
    //   AERROR << "Use backup trajectory";
    //   BackupTrajectoryGenerator backup_trajectory_generator(
    //       init_s, init_d, planning_init_point.relative_time(),
    //       std::make_shared<CollisionChecker>(collision_checker),
    //       &trajectory1d_generator);
    //   DiscretizedTrajectory trajectory =
    //       backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);
    //   reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
    //   reference_line_info->SetTrajectory(trajectory);
    //   reference_line_info->SetDrivable(true);
    //   return Status::OK();
    // } else {
    //   reference_line_info->SetCost(std::numeric_limits<double>::infinity());
    // }
  }

  // Planning success -> publish trajectory
  AINFO << "Planning Success!";
  // trajectories shall start at t=0 with an offset of
  // planning_init_point.relative_time()
  GetRawCMiqpTrajectoryCMiqpPlanner(
      planner, idx, planning_init_point.relative_time(), traj, size);
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

  DelCMiqpPlanner(planner);
  return Status::OK();

  // // 2. compute the matched point of the init planning point on the reference
  // // line.
  // PathPoint matched_point = PathMatcher::MatchToPath(
  //     *ptr_reference_line, planning_init_point.path_point().x(),
  //     planning_init_point.path_point().y());

  // // Get instance of collision checker and constraint checker
  // CollisionChecker collision_checker(frame->obstacles(), init_s[0],
  // init_d[0],
  //                                    *ptr_reference_line,
  //                                    reference_line_info,
  //                                    ptr_path_time_graph);

  // // check collision with other obstacles
  // if (collision_checker.InCollision(combined_trajectory)) {
  //   ++collision_failure_count;
  //   continue;
  // }
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
    const double v = sqrt(pow(vx, 2) + pow(vy, 2));
    const double a = sqrt(pow(ax, 2) + pow(ay, 2));
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

MiqpPlannerSettings MiqpPlanner::DefaultSettings() {
  MiqpPlannerSettings s = MiqpPlannerSettings();
  s.nr_regions = 16;
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
  const float collision_radius_add = 0.3;
  s.collisionRadius = common::VehicleConfigHelper::Instance()
                              ->GetConfig()
                              .vehicle_param()
                              .width() /
                          2 +
                      collision_radius_add;
  s.slackWeight = 30;
  s.jerkWeight = 1;
  s.positionWeight = 2;
  s.velocityWeight = 0;
  s.acclerationWeight = 0;
  s.simplificationDistanceMap = 0.2;
  s.refLineInterpInc = 0.2;
  s.scaleVelocityForReferenceLongerHorizon = 2.0;
  s.cplexModelpath =
      "../bazel-bin/modules/planning/libplanning_component.so.runfiles/"
      "miqp_planner/cplex_modfiles/";
  s.useSos = false;
  s.useBranchingPriorities = true;
  s.warmstartType =
      MiqpPlannerWarmstartType::BOTH_WARMSTART_STRATEGIES;  // Receding Horizon
                                                            // Warmstart does
                                                            // not work TODO
  return s;
}

}  // namespace planning
}  // namespace apollo
