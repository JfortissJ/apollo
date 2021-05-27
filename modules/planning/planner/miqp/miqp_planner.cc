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
#include "cyber/logger/logger_util.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/planner/miqp/trajectory_smoother_nlopt.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::PathMatcher;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::planning::DiscretizedTrajectory;

namespace {

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

MiqpPlanner::MiqpPlanner() {
  // from cyber/logger/log_file_object.cc
  struct ::tm tm_time;
  const time_t timestamp = static_cast<time_t>(Clock::NowInSeconds());
  localtime_r(&timestamp, &tm_time);
  std::ostringstream time_pid_stream;
  time_pid_stream.fill('0');
  time_pid_stream << 1900 + tm_time.tm_year << std::setw(2)
                  << 1 + tm_time.tm_mon << std::setw(2) << tm_time.tm_mday
                  << '-' << std::setw(2) << tm_time.tm_hour << std::setw(2)
                  << tm_time.tm_min << std::setw(2) << tm_time.tm_sec << '.'
                  << apollo::cyber::logger::GetMainThreadPid();
  const std::string& time_pid_string = time_pid_stream.str();
  logdir_ += "/apollo/data/log/";
  // logdir_ += time_pid_string;
}

common::Status MiqpPlanner::Init(const PlanningConfig& config) {
  MiqpPlannerSettings settings = DefaultSettings();
  planner_ = NewCMiqpPlannerSettings(settings);
  firstrun_ = true;                      // add car only in first run
  egoCarIdx_ = -1;                       // set invalid
  minimum_valid_speed_planning_ = 1.0;   // below our model is invalid
  standstill_velocity_threshold_ = 0.1;  // set velocity hard to zero below this
  minimum_valid_speed_vx_vy_ = 0.5;  // below this individual speed threshold
                                     // for vx and vy the model is invalid

  LOG(INFO) << "Writing MIQP Planner Logs to " << logdir_.c_str();
  char logdir_cstr[logdir_.length()];
  strcpy(logdir_cstr, logdir_.c_str());
  char name_prefix_cstr[14];
  strcpy(name_prefix_cstr, "miqp_planner_");
  ActivateDebugFileWriteCMiqpPlanner(planner_, logdir_cstr, name_prefix_cstr);

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
  AINFO << std::setprecision(15)
        << "############## MIQP Planner called at t = " << timestep;
  double current_time = timestep;
  const double start_time = timestep;

  double stop_dist;
  PlannerState planner_status = DeterminePlannerState(
      planning_init_point.v(), reference_line_info, stop_dist);

  if (planner_status == STANDSTILL_TRAJECTORY) {
    CreateStandstillTrajectory(planning_init_point, reference_line_info);
    return Status::OK();
  }

  // Initialized raw C trajectory output
  const int N = GetNCMiqpPlanner(planner_);
  double traj[TRAJECTORY_SIZE * N];
  int size;

  // Obtain a reference line and transform it to the PathPoint format.
  reference_line_info->set_is_on_reference_line();
  std::vector<PathPoint> discrete_reference_line =
      ToDiscretizedReferenceLine(reference_line_info, stop_dist);

  // Reference line to raw c format
  const int ref_size =
      discrete_reference_line.size();  // aka N optimization support points
  AINFO << "Reference Line has " << ref_size << " points";
  double ref[ref_size * 2];
  for (int i = 0; i < ref_size; ++i) {
    PathPoint refPoint = discrete_reference_line.at(i);
    // AINFO << refPoint.x() - config_.miqp_planner_config().pts_offset_x()<< ",
    // " << refPoint.y() - config_.miqp_planner_config().pts_offset_y();
    ref[2 * i] = refPoint.x() - config_.miqp_planner_config().pts_offset_x();
    ref[2 * i + 1] =
        refPoint.y() - config_.miqp_planner_config().pts_offset_y();
  }
  AINFO << "ReferenceLine Time [s] = "
        << (Clock::NowInSeconds() - current_time);
  current_time = Clock::NowInSeconds();

  // Map
  std::vector<Vec2d> left_pts, right_pts;
  if (config_.miqp_planner_config().use_environment_polygon()) {
    current_time = Clock::NowInSeconds();
    std::tie(left_pts, right_pts) = ToLeftAndRightBoundary(reference_line_info);
    const int poly_size = left_pts.size() + right_pts.size();
    double poly_pts[poly_size * 2];
    ConvertToPolyPts(left_pts, right_pts, poly_pts);
    UpdateConvexifiedMapCMiqpPlaner(planner_, poly_pts, poly_size);
    AINFO << "Map Processing Time [s] = "
          << (Clock::NowInSeconds() - current_time);
  }

  // Initial State
  double initial_state[6];
  ConvertToInitialStateSecondOrder(planning_init_point, initial_state);

  // Target velocity
  bool track_ref_pos;
  double vDes;
  double deltaSDes;
  const double dist_start_slowdown =
      config_.miqp_planner_config().distance_start_slowdown();
  const double dist_stop_before =
      config_.miqp_planner_config().distance_stop_before();
  if ((stop_dist - dist_stop_before < dist_start_slowdown) &&
      (planner_status != PlannerState::START_TRAJECTORY)) {
    track_ref_pos = false;  // only relevant for miqp
    vDes = 0;
    deltaSDes = std::max(0.0, stop_dist - dist_stop_before);
  } else if ((stop_dist - dist_stop_before < dist_start_slowdown) &&
             (planner_status == PlannerState::START_TRAJECTORY)) {
    track_ref_pos = false;  // only relevant for miqp
    vDes = FLAGS_default_cruise_speed;
    deltaSDes = std::max(0.0, stop_dist - dist_stop_before);
  } else {
    track_ref_pos = true;
    vDes = FLAGS_default_cruise_speed;
    deltaSDes = config_.miqp_planner_config().delta_s_desired();
  }

  // Add/update ego car
  if (firstrun_) {
    current_time = Clock::NowInSeconds();
    egoCarIdx_ = AddCarCMiqpPlanner(planner_, initial_state, ref, ref_size,
                                    vDes, deltaSDes, timestep, track_ref_pos);
    firstrun_ = false;
    AINFO << "Added ego car, Time [s] = "
          << (Clock::NowInSeconds() - current_time);
  } else {
    current_time = Clock::NowInSeconds();
    UpdateCarCMiqpPlanner(planner_, egoCarIdx_, initial_state, ref, ref_size,
                          timestep, track_ref_pos);
    AINFO << "Update ego car Time [s] = "
          << (Clock::NowInSeconds() - current_time);
    current_time = Clock::NowInSeconds();
    UpdateDesiredVelocityCMiqpPlanner(planner_, egoCarIdx_, vDes, deltaSDes);
    AINFO << "UpdateDesiredVelocityCMiqpPlanner Time [s] = "
          << (Clock::NowInSeconds() - current_time);
  }

  // if (planner_status == STOP_TRAJECTORY) {
  //   CreateStopTrajectory(planning_init_point, reference_line_info);
  //   return Status::OK();
  // }

  // Obstacles as obstacles
  if (config_.miqp_planner_config().consider_obstacles()) {
    // TODO: is that correct or should make use of relative time?
    current_time = Clock::NowInSeconds();
    bool success = ProcessObstacles(frame->obstacles(),
                                    planning_init_point.relative_time());
    AINFO << "Processing Obstacles took Time [s] = "
          << (Clock::NowInSeconds() - current_time);
    if (!success) {
      AERROR << "Processing of obstacles failed";
      return Status(ErrorCode::PLANNING_ERROR,
                    "processing of obstacles failed!");
    }
  }

  // Plan
  DiscretizedTrajectory apollo_traj;
  if (planner_status == START_TRAJECTORY || planner_status == STOP_TRAJECTORY) {
    AERROR << "Start/Stop Trajectory, using reference instead of miqp solution";
    GetRawCLastReferenceTrajectoryCMiqpPlaner(
        planner_, egoCarIdx_, planning_init_point.relative_time(), traj, size);
    apollo_traj = RawCTrajectoryToApolloTrajectory(traj, size, false);
  } else {
    current_time = Clock::NowInSeconds();
    bool success = PlanCMiqpPlanner(planner_, timestep);
    AINFO << "Miqp planning Time [s] = "
          << (Clock::NowInSeconds() - current_time);
    current_time = Clock::NowInSeconds();

    // Planning failed
    if (!success) {
      AINFO << "Planning failed";
      return Status(ErrorCode::PLANNING_ERROR, "miqp planner failed!");
    }

    // Get trajectory from miqp planner
    AINFO << "Planning Success!";
    // trajectories shall start at t=0 with an offset of
    // planning_init_point.relative_time()
    GetRawCMiqpTrajectoryCMiqpPlanner(
        planner_, egoCarIdx_, planning_init_point.relative_time(), traj, size);
    apollo_traj = RawCTrajectoryToApolloTrajectory(traj, size, true);
  }

  if (config_.miqp_planner_config().minimum_percentage_valid_miqp_points() *
          config_.miqp_planner_config().nr_steps() >
      apollo_traj.size()) {
    AERROR << "Trajectory has too many invalid points, setting error state";
    return Status(ErrorCode::PLANNING_ERROR, "invalid points!");
  }

  // if (ThetaChangeLargerThan(apollo_traj, 0.5)) {
  //   AERROR
  //       << "Very high theta changes on this trajectory, setting error
  //       state.";
  //   return Status(ErrorCode::PLANNING_ERROR, "high theta changes!");
  // }
  // CutoffTrajectoryAtV(apollo_traj, minimum_valid_speed_planning_);
  // TODO not sure what is the best value for cutoff:
  // minimum_valid_speed_planning_ might be too agressive and might drops valid
  // results but no more invalidities, standstill_velocity_threshold_ is might
  // be not enough to cover all solver invalidities

  // // debug outputs:
  // int r = size;
  // int c = TRAJECTORY_SIZE;
  // for (int i = 0; i < r; ++i) {
  //   for (int j = 0; j < c; ++j) {
  //     std::cout << traj[i * c + j] << ", ";
  //   }
  //   std::cout << std::endl;
  // }

  // Check resulting trajectory for collision with obstacles
  if (config_.miqp_planner_config().consider_obstacles()) {
    const auto& vehicle_config =
        common::VehicleConfigHelper::Instance()->GetConfig();
    const double ego_length = vehicle_config.vehicle_param().length();
    const double ego_width = vehicle_config.vehicle_param().width();
    const double ego_back_edge_to_center =
        vehicle_config.vehicle_param().back_edge_to_center();
    auto obstacles_non_virtual = FilterNonVirtualObstacles(frame->obstacles());
    const bool obstacle_collision = CollisionChecker::InCollision(
        obstacles_non_virtual, apollo_traj, ego_length, ego_width,
        ego_back_edge_to_center);
    if (obstacle_collision) {
      AERROR << "Planning success but collision with obstacle!";
      // return Status(ErrorCode::PLANNING_ERROR,
      //               "miqp trajectory colliding with obstacles");
    }
  }

  // Check resulting trajectory for collision with environment
  if (config_.miqp_planner_config().use_environment_polygon()) {
    if (EnvironmentCollision(left_pts, right_pts, apollo_traj)) {
      AERROR << "Planning success but collision with environment!";
      // return Status(ErrorCode::PLANNING_ERROR,
      //               "miqp trajectory colliding with environment");
    }
  }

  // Planning success -> publish trajectory
  Status return_status;
  if (config_.miqp_planner_config().use_smoothing()) {
    auto smoothed_apollo_trajectory =
        SmoothTrajectory(apollo_traj, planning_init_point);
    if (smoothed_apollo_trajectory.first) {
      reference_line_info->SetTrajectory(smoothed_apollo_trajectory.second);
      reference_line_info->SetCost(0);  // TODO necessary?
      reference_line_info->SetDrivable(true);
      return_status = Status::OK();
    } else {
      return_status = Status(ErrorCode::PLANNING_ERROR, "Smoothing failed!");
    }
  } else {
    reference_line_info->SetTrajectory(apollo_traj);
    reference_line_info->SetCost(0);  // TODO necessary?
    reference_line_info->SetDrivable(true);
    return_status = Status::OK();
  }

  AINFO << "MIQP Planner postprocess took [s]: "
        << (Clock::NowInSeconds() - current_time);
  AINFO << "MiqpPlanner::PlanOnReferenceLine() took "
        << (Clock::NowInSeconds() - start_time);

  return return_status;
}

std::vector<PathPoint> MiqpPlanner::ToDiscretizedReferenceLine(
    ReferenceLineInfo* reference_line_info, double stop_distance) {
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
        (s >
         s_stop_for_obstacle + config_.miqp_planner_config()
                                   .cutoff_distance_reference_after_stop())) {
      AINFO << "cutting off reference after s:" << s;
      break;
    }
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

void MiqpPlanner::FillTimeDerivativesInApolloTrajectory(
    DiscretizedTrajectory& traj) const {
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

DiscretizedTrajectory MiqpPlanner::RawCTrajectoryToApolloTrajectory(
    double traj[], int size, bool low_speed_check) {
  double s = 0.0f;
  double lastx =
      traj[0 + TRAJECTORY_X_IDX] + config_.miqp_planner_config().pts_offset_x();
  double lasty =
      traj[0 + TRAJECTORY_Y_IDX] + config_.miqp_planner_config().pts_offset_y();

  DiscretizedTrajectory apollo_trajectory;
  for (int trajidx = 0; trajidx < size; ++trajidx) {
    const double time = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_TIME_IDX];
    const double x = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_X_IDX] +
                     config_.miqp_planner_config().pts_offset_x();
    const double y = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_Y_IDX] +
                     config_.miqp_planner_config().pts_offset_y();
    const double vx = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_VX_IDX];
    const double vy = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_VY_IDX];
    const double ax = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_AX_IDX];
    const double ay = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_AY_IDX];
    // const double ux = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_UX_IDX];
    // const double uy = traj[trajidx * TRAJECTORY_SIZE + TRAJECTORY_UY_IDX];

    // at the first invalid vx vy point cut off the current trajectory
    if (low_speed_check && !IsVxVyValid(vx, vy)) {
      AINFO << "Trajectory at idx = " << trajidx << " has invalid (vx,vy) = ("
            << vx << ", " << vy << "); skipping further points.";
      break;
    }

    const double theta = atan2(vy, vx);
    const double v = vx / cos(theta);
    // const double a = ax / cos(theta); // probably wrong
    const double a = cos(theta) * ax + sin(M_PI_4 - theta) * ay;  // TODO: check
    s += sqrt(pow(x - lastx, 2) + pow(y - lasty, 2));
    double kappa;
    if ((vx * vx + vy * vy) < 1e-3) {
      kappa = 0;
    } else {
      kappa = (vx * ay - ax * vy) / (pow((vx * vx + vy * vy), 3 / 2));
    }
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
  FillTimeDerivativesInApolloTrajectory(apollo_trajectory);

  for (int trajidx = 0; trajidx < apollo_trajectory.size(); ++trajidx) {
    AINFO << "Planned trajectory at i=" << trajidx << ": "
          << apollo_trajectory[trajidx].DebugString();
  }

  return apollo_trajectory;
}

bool MiqpPlanner::IsVxVyValid(const double& vx, const double& vy) {
  return (fabs(vx) > minimum_valid_speed_vx_vy_ ||
          fabs(vy) > minimum_valid_speed_vx_vy_);
}

void MiqpPlanner::ConvertToInitialStateSecondOrder(
    const TrajectoryPoint& planning_init_point, double initial_state[]) {
  // Intial position to raw c format
  AINFO << std::setprecision(15) << "planning_init_point = "
        << " rel. time:" << planning_init_point.relative_time()
        << " x:" << planning_init_point.path_point().x()
        << ", y:" << planning_init_point.path_point().y()
        << ", v:" << planning_init_point.v()
        << ", a:" << planning_init_point.a()
        << ", theta:" << planning_init_point.path_point().theta();

  double vel = std::max(planning_init_point.v(), 0.1);
  double theta = planning_init_point.path_point().theta();
  // cplex throws an exception if vel=0
  initial_state[0] = planning_init_point.path_point().x() -
                     config_.miqp_planner_config().pts_offset_x();
  initial_state[1] = vel * cos(theta);
  initial_state[2] = planning_init_point.a() * cos(theta);
  initial_state[3] = planning_init_point.path_point().y() -
                     config_.miqp_planner_config().pts_offset_y();
  initial_state[4] = vel * sin(theta);
  initial_state[5] = planning_init_point.a() * sin(theta);
  AINFO << std::setprecision(15)
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
    poly_pts[2 * i] = it->x() - config_.miqp_planner_config().pts_offset_x();
    poly_pts[2 * i + 1] =
        it->y() - config_.miqp_planner_config().pts_offset_y();
    i++;
  }
  for (auto it = right_pts.rbegin(); it != right_pts.rend(); ++it) {
    poly_pts[2 * i] = it->x() - config_.miqp_planner_config().pts_offset_x();
    poly_pts[2 * i + 1] =
        it->y() - config_.miqp_planner_config().pts_offset_y();
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
  if (conf.has_max_velocity_fitting()) {
    s.max_velocity_fitting = conf.max_velocity_fitting();
  } else {
    s.max_velocity_fitting = 10;
  }
  if (conf.has_nr_steps()) {
    s.nr_steps = conf.nr_steps();
  } else {
    s.nr_steps = 20;
  }
  if (conf.has_nr_neighbouring_possible_regions()) {
    s.nr_neighbouring_possible_regions =
        conf.nr_neighbouring_possible_regions();
  } else {
    s.nr_neighbouring_possible_regions = 1;
  }
  if (conf.has_ts()) {
    s.ts = conf.ts();
  } else {
    s.ts = 0.25;
  }
  if (conf.has_max_solution_time()) {
    s.max_solution_time = conf.max_solution_time();
  } else {
    s.max_solution_time = 5.0;
  }
  if (conf.has_relative_mip_gap_tolerance()) {
    s.relative_mip_gap_tolerance = conf.relative_mip_gap_tolerance();
  } else {
    s.relative_mip_gap_tolerance = 0.1;
  }
  if (conf.has_mipemphasis()) {
    s.mipemphasis = conf.mipemphasis();
  } else {
    s.mipemphasis = 1;
  }
  if (conf.has_relobjdif()) {
    s.relobjdif = conf.relobjdif();
  } else {
    s.relobjdif = 0.9;
  }
  if (conf.has_minimum_region_change_speed()) {
    s.minimum_region_change_speed = conf.minimum_region_change_speed();
  } else {
    s.minimum_region_change_speed = 2;
  }
  if (conf.has_additional_steps_for_reference_longer_horizon()) {
    s.additionalStepsForReferenceLongerHorizon =
        conf.additional_steps_for_reference_longer_horizon();
  } else {
    s.additionalStepsForReferenceLongerHorizon = 2;
  }
  if (conf.has_use_sos()) {
    s.useSos = conf.use_sos();
  } else {
    s.useSos = false;
  }
  if (conf.has_use_branching_priorities()) {
    s.useBranchingPriorities = conf.use_branching_priorities();
  } else {
    s.useBranchingPriorities = true;
  }
  if (conf.has_warmstart_type()) {
    s.warmstartType =
        static_cast<MiqpPlannerWarmstartType>(conf.warmstart_type());
  } else {
    s.warmstartType = MiqpPlannerWarmstartType::NO_WARMSTART;
  }
  float collision_radius_add;
  if (conf.has_collision_radius_add()) {
    collision_radius_add = conf.collision_radius_add();
  } else {
    collision_radius_add = 0.0;
  }
  float wheelbase_add;
  if (conf.has_wheelbase_add()) {
    wheelbase_add = conf.wheelbase_add();
  } else {
    wheelbase_add = 0.0;
  }
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
  if (conf.has_obstacle_roi_filter()) {
    s.obstacle_roi_filter = conf.obstacle_roi_filter();
  } else {
    s.obstacle_roi_filter = false;
  }
  if (conf.has_obstacle_roi_behind_distance()) {
    s.obstacle_roi_behind_distance = conf.obstacle_roi_behind_distance();
  } else {
    s.obstacle_roi_behind_distance = 10.0;
  }
  s.wheelBase = common::VehicleConfigHelper::Instance()
                    ->GetConfig()
                    .vehicle_param()
                    .wheel_base() +
                wheelbase_add;
  s.collisionRadius = common::VehicleConfigHelper::Instance()
                              ->GetConfig()
                              .vehicle_param()
                              .width() /
                          2 +
                      collision_radius_add;

  s.slackWeight = 30;
  s.slackWeightObstacle = 2000;
  s.acclerationWeight = 0;
  if (conf.has_acc_lon_max_limit()) {
    s.accLonMaxLimit = conf.acc_lon_max_limit();
  } else {
    s.accLonMaxLimit = 2;
  }
  if (conf.has_acc_lon_min_limit()) {
    s.accLonMinLimit = conf.acc_lon_min_limit();
  } else {
    s.accLonMinLimit = -4;
  }
  if (conf.has_jerk_lon_max_limit()) {
    s.jerkLonMaxLimit = conf.jerk_lon_max_limit();
  } else {
    s.jerkLonMaxLimit = 3;
  }
  if (conf.has_acc_lat_min_max_limit()) {
    s.accLatMinMaxLimit = conf.acc_lat_min_max_limit();
  } else {
    s.accLatMinMaxLimit = 1.6;
  }
  if (conf.has_jerk_lat_min_max_limit()) {
    s.jerkLatMinMaxLimit = conf.jerk_lat_min_max_limit();
  } else {
    s.jerkLatMinMaxLimit = 1.4;
  }
  s.simplificationDistanceMap = 0.2;
  s.simplificationDistanceReferenceLine = 0.05;
  s.bufferReference = 1.0;
  s.buffer_for_merging_tolerance = 1.0;  // probably too high
  s.refLineInterpInc = 0.2;
  s.cplexModelpath =
      "../bazel-bin/modules/planning/libplanning_component.so.runfiles/"
      "miqp_planner/cplex_modfiles/";
  s.mipdisplay = 3;
  s.cutpass = 0;
  s.probe = 0;
  s.repairtries = 5;
  s.rinsheur = 5;
  s.varsel = 0;
  s.mircuts = 0;
  s.precision = 12;
  s.constant_agent_safety_distance_slack = 3;
  s.lambda = 0.5;
  s.buffer_cplex_outputs = true;
  return s;
}

//! @note copied from apollo's CollisionChecker::InCollision() function
bool MiqpPlanner::EnvironmentCollision(
    std::vector<Vec2d> left_pts, std::vector<Vec2d> right_pts,
    const DiscretizedTrajectory& ego_trajectory) {
  // append reversed right points to the left points
  left_pts.insert(left_pts.end(), right_pts.rbegin(), right_pts.rend());
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
      // Debug outputs
      // {
      //   std::stringstream ss;
      //   ss << std::setprecision(15) << "envpoly = [";
      //   const char* sep = "";
      //   for (auto& pt : envpoly.points()) {
      //     ss << sep << pt.x() << ", " << pt.y();
      //     sep = "; ";
      //   }
      //   ss << "]";
      //   AINFO << std::setprecision(15) << ss.str().c_str();
      // }
      // {
      //   std::stringstream ss;
      //   ss << std::setprecision(15) << "carpoly = [";
      //   const char* sep = "";
      //   for (auto& pt : carpoly.points()) {
      //     ss << sep << pt.x() << ", " << pt.y();
      //   }
      //   ss << "]";
      //   AINFO << std::setprecision(15) << ss.str().c_str();
      // }
      return true;
    }
  }
  return false;
}

std::vector<const Obstacle*> MiqpPlanner::FilterNonVirtualObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  std::vector<const Obstacle*> obstacles_out;
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      AINFO << "Skipping virtual obstacle for post-collision check: "
            << obstacle->DebugString();
      continue;
    } else {
      obstacles_out.push_back(obstacle);
    }
  }
  return obstacles_out;
}

bool MiqpPlanner::ProcessObstacles(
    const std::vector<const Obstacle*>& obstacles, double timestep) {
  RemoveAllObstaclesCMiqpPlanner(planner_);

  // Add obstacles
  const int N = GetNCMiqpPlanner(planner_);
  for (const Obstacle* obstacle : obstacles) {
    double p1_x[N], p1_y[N], p2_x[N], p2_y[N], p3_x[N], p3_y[N], p4_x[N],
        p4_y[N];
    bool is_static;
    bool is_soft = false;
    if (obstacle->IsVirtual()) {
      continue;
      // } else if (!obstacle->IsLaneBlocking()) {
      //   AINFO << "Skipping obstacle " << obstacle->Id() << " (not blocking
      //   lane)"; continue;
    } else if (!obstacle->HasTrajectory()) {
      // AINFO << "Static obstacle " << obstacle->Id();
      const common::math::Polygon2d& polygon = obstacle->PerceptionPolygon();
      for (int i = 0; i < N; ++i) {
        FillInflatedPtsFromPolygon(polygon, p1_x[i], p1_y[i], p2_x[i], p2_y[i],
                                   p3_x[i], p3_y[i], p4_x[i], p4_y[i]);
      }
      is_static = true;
    } else {
      const float ts = GetTsCMiqpPlanner(planner_);
      AINFO << "Dynamic obstacle " << obstacle->Id();
      for (int i = 0; i < N; ++i) {
        double pred_time = timestep + i * ts;
        TrajectoryPoint point = obstacle->GetPointAtTime(pred_time);

        common::math::Box2d box_i = obstacle->GetBoundingBox(point);
        AINFO << "idx: " << i << ", box: " << box_i.DebugString();
        box_i.LongitudinalExtend(
            config_.miqp_planner_config().extension_length_dynamic());
        AINFO << "idx: " << i << ", extended box: " << box_i.DebugString();
        common::math::Polygon2d poly2d_i = Polygon2d(box_i);
        FillInflatedPtsFromPolygon(poly2d_i, p1_x[i], p1_y[i], p2_x[i], p2_y[i],
                                   p3_x[i], p3_y[i], p4_x[i], p4_y[i]);
      }
      is_static = false;
      is_soft = true;
    }

    int idx_obs =
        AddObstacleCMiqpPlanner(planner_, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y,
                                p4_x, p4_y, N, is_static, is_soft);
    if (idx_obs != -1) {
      AINFO << "Added obstacle " << obstacle->Id()
            << " with miqp idx = " << idx_obs << " is_static = " << is_static
            << " is_soft = " << is_soft;
    }
  }
  return true;
}

bool MiqpPlanner::FillInflatedPtsFromPolygon(const common::math::Polygon2d poly,
                                             double& p1_x, double& p1_y,
                                             double& p2_x, double& p2_y,
                                             double& p3_x, double& p3_y,
                                             double& p4_x, double& p4_y) {
  const double radius = GetCollisionRadius(planner_);
  common::math::Polygon2d poly2d_buff = poly.ExpandByDistance(radius);
  common::math::Box2d box_buff = poly2d_buff.MinAreaBoundingBox();
  std::vector<Vec2d> pts = box_buff.GetAllCorners();
  if (pts.size() != 4) {
    return false;
  }
  // AINFO << "pts(0): " << pts.at(0).x() -
  // config_.miqp_planner_config().pts_offset_x() << ", "
  //       << pts.at(0).y() - config_.miqp_planner_config().pts_offset_y();
  // AINFO << "pts(1): " << pts.at(1).x() -
  // config_.miqp_planner_config().pts_offset_x() << ", "
  //       << pts.at(1).y() - config_.miqp_planner_config().pts_offset_y();
  // AINFO << "pts(2): " << pts.at(2).x() -
  // config_.miqp_planner_config().pts_offset_x() << ", "
  //       << pts.at(2).y() - config_.miqp_planner_config().pts_offset_y();
  // AINFO << "pts(3): " << pts.at(3).x() -
  // config_.miqp_planner_config().pts_offset_x() << ", "
  //       << pts.at(3).y() - config_.miqp_planner_config().pts_offset_y();

  p1_x = pts.at(0).x() - config_.miqp_planner_config().pts_offset_x();
  p1_y = pts.at(0).y() - config_.miqp_planner_config().pts_offset_y();
  p2_x = pts.at(1).x() - config_.miqp_planner_config().pts_offset_x();
  p2_y = pts.at(1).y() - config_.miqp_planner_config().pts_offset_y();
  p3_x = pts.at(2).x() - config_.miqp_planner_config().pts_offset_x();
  p3_y = pts.at(2).y() - config_.miqp_planner_config().pts_offset_y();
  p4_x = pts.at(3).x() - config_.miqp_planner_config().pts_offset_x();
  p4_y = pts.at(3).y() - config_.miqp_planner_config().pts_offset_y();

  return true;
}

double MiqpPlanner::CalculateSDistanceToStop(
    ReferenceLineInfo* reference_line_info, bool brake_for_inlane) {
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

apollo::planning::PlannerState MiqpPlanner::DeterminePlannerState(
    const double planning_init_v, ReferenceLineInfo* reference_line_info,
    double& stop_dist) {
  PlannerState status;
  // issue hard stop trajectory without optimization if velocity is
  // low enough and goal is nearer than this
  const double destination_dist_threshold =
      config_.miqp_planner_config().destination_distance_stop_threshold();

  // calculate the stop distance under the assumption to not brake for every
  // obstacle in the lane
  bool brake_for_inlane = false;
  stop_dist = MiqpPlanner::CalculateSDistanceToStop(reference_line_info,
                                                    brake_for_inlane);

  if (stop_dist < destination_dist_threshold) {  // Approaching end or at end
    if (planning_init_v <=
        standstill_velocity_threshold_) {  // Standstill at end
      status = PlannerState::STANDSTILL_TRAJECTORY;
    } else if (planning_init_v <=
               minimum_valid_speed_planning_) {  // Create stopping traj
      status = PlannerState::STOP_TRAJECTORY;
      // overwriting start stop_distance!!
      bool brake_for_inlane = true;
      stop_dist = MiqpPlanner::CalculateSDistanceToStop(reference_line_info,
                                                        brake_for_inlane);
    } else {  // Let miqp optimizier plan the stopping traj
      status = PlannerState::DRIVING_TRAJECTORY;
    }
  } else {  // Driving or want to start driving
    if (planning_init_v <=
        minimum_valid_speed_planning_) {  // Low speed -> modify start
      status = PlannerState::START_TRAJECTORY;
      // overwriting start stop_distance!!
      bool brake_for_inlane = true;
      stop_dist = MiqpPlanner::CalculateSDistanceToStop(reference_line_info,
                                                        brake_for_inlane);
    } else {  // Driving: default case
      status = PlannerState::DRIVING_TRAJECTORY;
    }
  }
  AINFO << "Planner status is: " << static_cast<int>(status)
        << " v_init = " << planning_init_v << " stop dist = " << stop_dist;

  return status;
}

int MiqpPlanner::CutoffTrajectoryAtV(DiscretizedTrajectory& traj, double vmin) {
  // reverse, to not cutoff accelerating trajectories
  for (size_t i = traj.size() - 1; i > 1; --i) {
    if (traj.at(i).v() < vmin && traj.at(i - 1).v() >= vmin) {
      traj.at(i).set_v(vmin);
      traj.resize(i + 1);  // delete everything after this point
      AINFO << "Cutting trajectory off at pt idx = " << i;
      return i;
    }
  }
  return traj.size();
}

void MiqpPlanner::CreateStandstillTrajectory(
    const TrajectoryPoint& planning_init_point,
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

// void MiqpPlanner::CreateStopTrajectory(
//     const TrajectoryPoint& planning_init_point,
//     ReferenceLineInfo* reference_line_info) {
//   double lastrefpt[5];
//   GetLastReferencePointCMiqpPlanner(planner_, egoCarIdx_, lastrefpt);
//   const double time = std::move(lastrefpt[0]);
//   if (time > planning_init_point.relative_time()) {
//     AINFO << "Stop trajectory already should start at endpoint; Creating "
//              "Standstill trajectory instead!";
//     CreateStandstillTrajectory(planning_init_point, reference_line_info);

//   } else {  // Default case
//     TrajectoryPoint trajectory_point_1;
//     trajectory_point_1.mutable_path_point()->set_x(
//         planning_init_point.path_point().x());
//     trajectory_point_1.mutable_path_point()->set_y(
//         planning_init_point.path_point().y());
//     trajectory_point_1.mutable_path_point()->set_s(0.0);
//     trajectory_point_1.mutable_path_point()->set_theta(
//         planning_init_point.path_point().theta());
//     trajectory_point_1.set_v(planning_init_point.v());
//     trajectory_point_1.set_a(planning_init_point.a());
//     trajectory_point_1.set_relative_time(planning_init_point.relative_time());

//     TrajectoryPoint trajectory_point_2;

//     const double x = std::move(lastrefpt[1]);
//     const double y = std::move(lastrefpt[2]);
//     const double theta = std::move(lastrefpt[3]);
//     const double v = std::move(lastrefpt[4]);
//     if (v > minimum_valid_speed_planning_) {
//       AERROR << "Invalid stopping Trajectory! Reference trajectory last point
//       "
//                 "speed = "
//              << v;
//     }
//     trajectory_point_2.mutable_path_point()->set_x(x);
//     trajectory_point_2.mutable_path_point()->set_y(y);
//     double s = std::sqrt(std::pow(planning_init_point.path_point().x() - x,
//     2) +
//                          std::pow(planning_init_point.path_point().y() - y,
//                          2));
//     trajectory_point_2.mutable_path_point()->set_s(s);
//     trajectory_point_2.mutable_path_point()->set_theta(theta);
//     trajectory_point_2.set_v(0.0);
//     trajectory_point_2.set_a(0.0);  // TODO better set to something negative?
//     double time = s / planning_init_point.v();  // =ds/dt
//     trajectory_point_2.set_relative_time(planning_init_point.relative_time());

//     DiscretizedTrajectory stopping_trajectory;
//     stopping_trajectory.AppendTrajectoryPoint(trajectory_point_1);
//     stopping_trajectory.AppendTrajectoryPoint(trajectory_point_2);

//     reference_line_info->SetTrajectory(stopping_trajectory);
//     reference_line_info->SetCost(0);  // TODO necessary?
//     reference_line_info->SetDrivable(true);

//     AINFO << "Setting Stop trajectory at point t = "
//           << planning_init_point.relative_time()
//           << " x = " << planning_init_point.path_point().x()
//           << " y = " << planning_init_point.path_point().y()
//           << "with end point at t = " << time << " x = " << x << " y = " <<
//           y;
//   }
// }

std::pair<bool, apollo::planning::DiscretizedTrajectory>
MiqpPlanner::SmoothTrajectory(
    const apollo::planning::DiscretizedTrajectory& traj_in,
    const common::TrajectoryPoint& planning_init_point) {
  int subsampling = 3;
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt(
      logdir_.c_str(), config_.miqp_planner_config().pts_offset_x(),
      config_.miqp_planner_config().pts_offset_y());
  tsm.InitializeProblem(subsampling, traj_in, planning_init_point);
  AINFO << "Planning init point is " << planning_init_point.DebugString();
  tsm.Optimize();
  auto traj = tsm.GetOptimizedTrajectory();
  if (tsm.ValidateSmoothingSolution()) {
    for (int idx = 0; idx < traj.size(); ++idx) {
      AINFO << "Smoothed trajectory at idx = " << idx << " : "
            << traj.at(idx).DebugString();
    }
    return {true, traj};
  } else {
    AERROR << "Trajectory smoothing not valid or failed!";
    return {false, traj};
  }
}

bool MiqpPlanner::ThetaChangeLargerThan(
    const apollo::planning::DiscretizedTrajectory& traj,
    const double delta_theta_max) {
  for (size_t idx = 0; idx < traj.size() - 1; ++idx) {
    const double delta_theta =
        common::math::NormalizeAngle(traj.at(idx).path_point().theta() -
                                     traj.at(idx + 1).path_point().theta());
    if (delta_theta > delta_theta_max) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
