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

#include "gtest/gtest.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

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
  tp2.mutable_path_point()->set_dkappa(0);
  tp2.set_v(1);
  tp2.set_a(1);
  tp2.set_da(0);
  tp2.set_relative_time(5);
  traj.AppendTrajectoryPoint(tp2);
  tsm.InitializeProblem(0, traj, 0);
  int status = tsm.Optimize();
  EXPECT_GT(status, 0);
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

TEST(TrajectorySmootherNLOpt, IntegrateModel) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  TrajectorySmootherNLOpt::Vector6d x0;
  x0 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  size_t num_integration_steps = 1;
  Eigen::VectorXd u;
  u.resize(2,3);
  u << 0.2, 0.5, 0.2, 0.5, 0.2, 0.5;
  const double h = 0.1;
  Eigen::VectorXd X;
  Eigen::MatrixXd dXdU;
  tsm.IntegrateModel(x0, u, num_integration_steps, h, X, dXdU);
  std::cout << X << std::endl;
  std::cout << dXdU << std::endl;
  EXPECT_TRUE(false);
}

// TEST 1: Integration Model

// Test 2: Constraint Checking

// Test 3: Optimization

}  // namespace planning
}  // namespace apollo
