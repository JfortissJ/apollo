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

#include "modules/planning/planner/miqp/trajectory_smoother_nlopt.h"

#include <cmath>

#include "cyber/common/file.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

void OptimizeFromFileHelper(std::string path_to_file, std::string input_file, int subsampling) {
  const std::string path_of_standard_trajectory =
      path_to_file + "/" + input_file;
  ADCTrajectory trajectory;
  EXPECT_TRUE(cyber::common::GetProtoFromFile(path_of_standard_trajectory,
                                              &trajectory));
  DiscretizedTrajectory traj_in(trajectory);
  const double dt_in = 1.0;
  const double T_in = (traj_in.NumOfPoints() - 1) * dt_in;

  // OPTIMIZER
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  tsm.InitializeProblem(subsampling, traj_in, 0);
  int status = tsm.Optimize();
  EXPECT_GT(status, 0);
  auto traj_opt = tsm.GetOptimizedTrajectory();
  for (int trajidx = 0; trajidx < traj_opt.size(); ++trajidx) {
    AINFO << "Smoothed trajectory at i=" << trajidx << ": "
          << traj_opt[trajidx].DebugString();
  }
  tsm.DebugDumpU();
  tsm.DebugDumpX();
  tsm.DebugDumpXref();

  apollo::planning::PublishableTrajectory publishable_trajectory(0.0, traj_opt);
  ADCTrajectory traj_pb;
  publishable_trajectory.PopulateTrajectoryProtobuf(&traj_pb);

  // This will dump the optimized trajectory. Analysis is possible using the
  // matlab script analyze_smoother.m
  const std::string txt_file = path_to_file + "/" + "sqp_out_"  + std::to_string(subsampling) + "_" + input_file;
  apollo::cyber::common::SetProtoToASCIIFile(traj_pb, txt_file);

  EXPECT_DOUBLE_EQ(traj_in.GetTemporalLength(), T_in);
  EXPECT_DOUBLE_EQ(traj_opt.GetTemporalLength(), T_in);
}

TEST(TrajectorySmootherNLOpt, Constructor) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  EXPECT_NE(&tsm, nullptr);
}

TEST(TrajectorySmootherNLOpt, Optimize_Empty) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  DiscretizedTrajectory tmp;
  tsm.InitializeProblem(1, tmp, 0);
  int status = tsm.Optimize();
  EXPECT_EQ(-100, status);
}

TEST(TrajectorySmootherNLOpt, Optimize1) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  DiscretizedTrajectory traj;
  common::TrajectoryPoint tp1;
  tp1.mutable_path_point()->set_x(0);
  tp1.mutable_path_point()->set_y(0);
  tp1.mutable_path_point()->set_s(0);
  tp1.mutable_path_point()->set_theta(0);
  tp1.mutable_path_point()->set_kappa(0);
  tp1.mutable_path_point()->set_dkappa(0);
  tp1.set_v(0);
  tp1.set_a(0);
  tp1.set_da(0);
  tp1.set_relative_time(0);
  traj.AppendTrajectoryPoint(tp1);
  common::TrajectoryPoint tp2;
  tp2.mutable_path_point()->set_x(5);
  tp2.mutable_path_point()->set_y(0);
  tp2.mutable_path_point()->set_s(5);
  tp2.mutable_path_point()->set_theta(0);
  tp2.mutable_path_point()->set_kappa(0);
  tp2.mutable_path_point()->set_dkappa(0);
  tp2.set_v(1);
  tp2.set_a(1);
  tp2.set_da(0);
  tp2.set_relative_time(5);
  traj.AppendTrajectoryPoint(tp2);
  tsm.InitializeProblem(0, traj, 0);
  int status = tsm.Optimize();
  auto traj_opt = tsm.GetOptimizedTrajectory();
  for (int trajidx = 0; trajidx < traj_opt.size(); ++trajidx) {
    AINFO << "Smoothed trajectory at i=" << trajidx << ": "
          << traj_opt[trajidx].DebugString();
  }
  EXPECT_GT(status, 0);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile) {
  const std::string path_to_file = "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp.pb.txt";
  int subsampling = 0; // no subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFileSubsampling) {
  const std::string path_to_file = "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp.pb.txt";
  int subsampling = 1; // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
  EXPECT_FALSE(true);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFileStartDriving) {
  const std::string path_to_file = "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_from_standstill.pb.txt";
  int subsampling = 0; // no subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, model_f) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  TrajectorySmootherNLOpt::Vector6d x;
  x << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  const Eigen::Vector2d u = Eigen::Vector2d(0.2, 0.5);
  const double h = 0.1;
  TrajectorySmootherNLOpt::Vector6d x_out;
  tsm.model_f(x, u, h, x_out);

  // assertion values come from verified model in test_model.m
  EXPECT_NEAR(x_out(0), 0.1, 1e-9);
  EXPECT_NEAR(x_out(1), 0.0, 1e-9);
  EXPECT_NEAR(x_out(2), 0.0025, 1e-9);
  EXPECT_NEAR(x_out(3), 1.001, 1e-9);
  EXPECT_NEAR(x_out(4), 0.02, 1e-9);
  EXPECT_NEAR(x_out(5), 0.05, 1e-9);
}

TEST(TrajectorySmootherNLOpt, IntegrateModelConstInput) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  TrajectorySmootherNLOpt::Vector6d x0;
  x0 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  size_t dimU = 2;
  size_t dimX = 6;
  size_t num_integration_steps = 3;
  Eigen::VectorXd u;
  u.resize(dimU * num_integration_steps);
  u << 0.2, 0.5, 0.2, 0.5, 0.2, 0.5;
  const double h = 0.1;
  Eigen::VectorXd X;
  Eigen::MatrixXd dXdU;
  tsm.IntegrateModel(x0, u, num_integration_steps, h, X, dXdU);

  EXPECT_NEAR(X(0), 0, 1e-9);
  EXPECT_NEAR(X(1), 0, 1e-9);
  EXPECT_NEAR(X(2), 0, 1e-9);
  EXPECT_NEAR(X(3), 1, 1e-9);
  EXPECT_NEAR(X(4), 0, 1e-9);
  EXPECT_NEAR(X(5), 0, 1e-9);

  EXPECT_NEAR(X(1 * dimX + 0), 0.1000, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 1), 0, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 2), 0.0025, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 3), 1.0010, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 4), 0.0200, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 5), 0.0500, 1e-9);

  EXPECT_NEAR(X(2 * dimX + 0), 0.200198431250459, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 1), 5.014970864425282e-4, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 2), 0.0100175, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 3), 1.004, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 4), 0.04, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 5), 0.1, 1e-9);
}

TEST(TrajectorySmootherNLOpt, IntegrateModelNonconstInput) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  TrajectorySmootherNLOpt::Vector6d x0;
  x0 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  size_t dimU = 2;
  size_t dimX = 6;
  size_t num_integration_steps = 3;
  Eigen::VectorXd u;
  u.resize(dimU * num_integration_steps);
  u << 0.2, 0.5, 0.3, 0.6, 0.1, 0.4;
  const double h = 0.1;
  Eigen::VectorXd X;
  Eigen::MatrixXd dXdU;
  tsm.IntegrateModel(x0, u, num_integration_steps, h, X, dXdU);
  std::cout << "X:" << std::endl << X << std::endl;
  std::cout << "dXdU:" << std::endl << dXdU << std::endl;

  EXPECT_NEAR(X(0), 0, 1e-9);
  EXPECT_NEAR(X(1), 0, 1e-9);
  EXPECT_NEAR(X(2), 0, 1e-9);
  EXPECT_NEAR(X(3), 1, 1e-9);
  EXPECT_NEAR(X(4), 0, 1e-9);
  EXPECT_NEAR(X(5), 0, 1e-9);

  EXPECT_NEAR(X(1 * dimX + 0), 0.1000, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 1), 0, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 2), 0.0025, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 3), 1.001, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 4), 0.02, 1e-9);
  EXPECT_NEAR(X(1 * dimX + 5), 0.05, 1e-9);

  EXPECT_NEAR(X(2 * dimX + 0), 0.200198431250459, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 1), 5.014970864425282e-04, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 2), 0.010519, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 3), 1.0045, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 4), 0.05, 1e-9);
  EXPECT_NEAR(X(2 * dimX + 5), 0.11, 1e-9);
}

// TEST 1: Integration Model

// Test 2: Constraint Checking

// Test 3: Optimization

}  // namespace planning
}  // namespace apollo
