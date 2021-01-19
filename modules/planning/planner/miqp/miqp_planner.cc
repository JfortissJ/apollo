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

#include "src/miqp_planner_c_api.h"
// #include "bark/commons/params/params_c_api.h"
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
using apollo::common::time::Clock;


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

  {
    AERROR << "00000000000000000000000000000";
    CMiqpPlanner planner = newCMiqpPlanner();
    AERROR << "11111111111111111111111";

    double start_time = Clock::NowInSeconds();
    double current_time = start_time;

    reference_line_info->set_is_on_reference_line();
    // 1. obtain a reference line and transform it to the PathPoint format.
    std::vector<PathPoint> discrete_reference_line = ToDiscretizedReferenceLine(
            reference_line_info->reference_line().reference_points());

    const int ref_size = discrete_reference_line.size(); // aka N optimization support points

    double ref[ref_size * 2];
    for (int i = 0; i < ref_size; ++i) {
      PathPoint refPoint = discrete_reference_line.at(i);
      // AERROR << refPoint.x() << ", " << refPoint.y();
      ref[2*i] = refPoint.x();
      ref[2*i+1] = refPoint.y();
    }

    AERROR << "ReferenceLine Time = "
          << (Clock::NowInSeconds() - current_time) * 1000;
    current_time = Clock::NowInSeconds();

    // double ref[ref_size * 2] = {0, 0, 5, 0, 30, 0};
    // double initial_state[6] = {, 0, 0, 1, 0.01, 0};
    double initial_state[6];
    double theta = planning_init_point.path_point().theta();
    initial_state[0] = planning_init_point.path_point().x();
    initial_state[1] = planning_init_point.v() * cos(theta);
    initial_state[2] = planning_init_point.a() * cos(theta); // is that correct?
    initial_state[3] = planning_init_point.path_point().y();
    initial_state[4] = planning_init_point.v() * sin(theta);
    initial_state[5] = planning_init_point.a() * sin(theta); // is that correct?
    
    double vDes = 5;
    double timestep = 0.0;
    int idx = AddCarCMiqpPlanner(planner, initial_state, ref, ref_size, vDes,
                                 timestep);
    AERROR << "222222222222222222222222222222";
    PlanCMiqpPlanner(planner, timestep);
    AERROR << "3333333333333333333333333333333";
    double traj[5 * 20];  // TODO this is a hack!
    int size;
    GetCTrajectoryCMiqpPlanner(planner, idx, timestep, traj, size);
    AERROR << "444444444444444444444444";
    int r = size;
    int c = 5;  // StateDefinition::MIN_STATE_SIZE
    for (int i = 0; i < r; ++i) {
      for (int j = 0; j < c; ++j) {
        std::cout << traj[i * c + j] << "\t";
      }
      std::cout << std::endl;
    }
    DelCMiqpPlanner(planner);
    AERROR << "222222222222222222222222222222";
  }

  // CSetterParams csp = newCSetterParams(false);
  // setRealCSetterParams(csp, "Miqp::CollisionRadius", 0.9);
  // double test = getRealCSetterParams(csp, "Miqp::CollisionRadius", "", 1.1);
  // delCSetterParams(csp);
  // AERROR << "Test get param: " << test;

  double start_time = Clock::NowInSeconds();
  double current_time = start_time;

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

  ADEBUG << "Decision_Time = " << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

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

  // if (num_lattice_traj > 0) {
  //   ADEBUG << "Planning succeeded";
  //   num_planning_succeeded_cycles += 1;
  //   reference_line_info->SetDrivable(true);
  //   return Status::OK();
  // } else {
  //   AERROR << "Planning failed";
  //   if (FLAGS_enable_backup_trajectory) {
  //     AERROR << "Use backup trajectory";
  //     BackupTrajectoryGenerator backup_trajectory_generator(
  //         init_s, init_d, planning_init_point.relative_time(),
  //         std::make_shared<CollisionChecker>(collision_checker),
  //         &trajectory1d_generator);
  //     DiscretizedTrajectory trajectory =
  //         backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

  //     reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
  //     reference_line_info->SetTrajectory(trajectory);
  //     reference_line_info->SetDrivable(true);
  //     return Status::OK();

  //   } else {
  //     reference_line_info->SetCost(std::numeric_limits<double>::infinity());
  //   }
  //   return Status(ErrorCode::PLANNING_ERROR, "No feasible trajectories");
  // }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
