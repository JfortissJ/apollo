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
#include "src/miqp_planner_c_api.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::PathMatcher;
using apollo::common::time::Clock;
using apollo::planning::DiscretizedTrajectory;

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

}  // namespace

Status MiqpPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  AERROR << "PlanOnReferenceLine() of MIQP planner called!";
  double start_time = Clock::NowInSeconds();
  double current_time = start_time;

  CMiqpPlanner planner = NewCMiqpPlanner();
  ActivateDebugFileWriteCMiqpPlanner(planner, "/apollo/data/log", "test_");

  const int N = GetNCMiqpPlanner(planner);
  double traj[TRAJECTORY_SIZE * N];
  int size;

  reference_line_info->set_is_on_reference_line();
  // 1. obtain a reference line and transform it to the PathPoint format.
  std::vector<PathPoint> discrete_reference_line = ToDiscretizedReferenceLine(
      reference_line_info->reference_line().reference_points());

  const int ref_size =
      discrete_reference_line.size();  // aka N optimization support points

  double ref[ref_size * 2];
  for (int i = 0; i < ref_size; ++i) {
    PathPoint refPoint = discrete_reference_line.at(i);
    // AERROR << refPoint.x() << ", " << refPoint.y();
    ref[2 * i] = refPoint.x();
    ref[2 * i + 1] = refPoint.y();
  }

  AERROR << "ReferenceLine Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  AERROR << "planning_init_point = v:" << planning_init_point.v()
          << ", theta:" << planning_init_point.path_point().theta();
  double initial_state[6];
  double theta = planning_init_point.path_point().theta();
  double vel = std::max(planning_init_point.v(),
                        0.1);  // cplex throws an exception if vel=0
  initial_state[0] = planning_init_point.path_point().x();
  initial_state[1] = vel * cos(theta);
  initial_state[2] =
      planning_init_point.a() * cos(theta);  // is that correct?
  initial_state[3] = planning_init_point.path_point().y();
  initial_state[4] = vel * sin(theta);
  initial_state[5] =
      planning_init_point.a() * sin(theta);  // is that correct?

  AERROR << "initial state = x:" << initial_state[0]
          << ", xd:" << initial_state[1] << ", xdd:" << initial_state[2]
          << ", y:" << initial_state[3] << ", yd:" << initial_state[4]
          << ", ydd:" << initial_state[5];

  double vDes = 5;
  double timestep = planning_init_point.relative_time();
  AERROR << "Planning timestep = " << timestep;
  int idx =
      AddCarCMiqpPlanner(planner, initial_state, ref, ref_size, vDes, timestep);

  AERROR << "Added ego car Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  bool success = PlanCMiqpPlanner(planner, timestep);

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

  GetRawCMiqpTrajectoryCMiqpPlanner(planner, idx, timestep, traj, size);
  DiscretizedTrajectory apollo_traj =
      RawCTrajectoryToApolloTrajectory(traj, size);
  reference_line_info->SetTrajectory(apollo_traj);
  reference_line_info->SetCost(0);  // TODO necessary?
  reference_line_info->SetDrivable(true);

  DelCMiqpPlanner(planner);

  ADEBUG << "MIQP Planner took [s]: "
         << (Clock::NowInSeconds() - current_time) * 1000;

  // debug outputs:
  int r = size;
  int c = TRAJECTORY_SIZE;
  for (int i = 0; i < r; ++i) {
    for (int j = 0; j < c; ++j) {
      std::cout << traj[i * c + j] << "\t";
    }
    std::cout << std::endl;
  }

  return Status::OK();

  // reference_line_info->set_is_on_reference_line();
  // // 1. obtain a reference line and transform it to the PathPoint format.
  // auto ptr_reference_line =
  //     std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
  //         reference_line_info->reference_line().reference_points()));

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
    double x = traj[trajidx * MIN_STATE_SIZE + X_POSITION];
    double y = traj[trajidx * MIN_STATE_SIZE + Y_POSITION];
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
    const double x = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_X_IDX];
    const double y = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_Y_IDX];
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
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(time);
    apollo_trajectory.AppendTrajectoryPoint(trajectory_point);

    lastx = x;
    lasty = y;
  }

  return apollo_trajectory;
}

}  // namespace planning
}  // namespace apollo
