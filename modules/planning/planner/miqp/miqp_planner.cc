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

#include "bark/commons/params/params.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::PathMatcher;
using apollo::common::time::Clock;


Status MiqpPlanner::Plan(const TrajectoryPoint& planning_start_point,
                            Frame* frame,
                            ADCTrajectory* ptr_computed_trajectory) {
  size_t success_line_count = 0;
  size_t index = 0;
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (index != 0) {
      reference_line_info.SetPriorityCost(
          FLAGS_cost_non_priority_reference_line);
    } else {
      reference_line_info.SetPriorityCost(0.0);
    }
    auto status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    if (status != Status::OK()) {
      if (reference_line_info.IsChangeLanePath()) {
        AERROR << "Planner failed to change lane to "
               << reference_line_info.Lanes().Id();
      } else {
        AERROR << "Planner failed to " << reference_line_info.Lanes().Id();
      }
    } else {
      success_line_count += 1;
    }
    ++index;
  }

  if (success_line_count > 0) {
    return Status::OK();
  }
  return Status(ErrorCode::PLANNING_ERROR,
                "Failed to plan on any reference line.");
}

Status MiqpPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {

      auto bp = bark::commons::Params();
      bark::commons::Params::Point2d_test a(1,1);
      bark::commons::Params::Point2d_test b(2,2);
      bark::commons::Params::Point2d_test c = bp.Distance(a,b);


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
  // CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0],
  //                                    *ptr_reference_line, reference_line_info,
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
