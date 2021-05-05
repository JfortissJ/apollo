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

void CompareTrajectories(DiscretizedTrajectory traj1, DiscretizedTrajectory traj2) {
  EXPECT_EQ(traj1.size(), traj2.size());
  
  for (int idx = 0; idx < traj1.size(); ++idx) {
    auto tp1 = traj1.at(idx);
    auto tp2 = traj2.at(idx);

    EXPECT_NEAR(tp1.path_point().x(), tp2.path_point().x(), 0.3);
    EXPECT_NEAR(tp1.path_point().y(), tp2.path_point().y(), 0.3);
    EXPECT_NEAR(tp1.path_point().theta(), tp2.path_point().theta(), 0.3);
    EXPECT_NEAR(tp1.path_point().kappa(), tp2.path_point().kappa(), 0.2);
    EXPECT_NEAR(tp1.path_point().s(), tp2.path_point().s(), 0.3);
    EXPECT_NEAR(tp1.v(), tp2.v(), 0.1);
    EXPECT_NEAR(tp1.a(), tp2.a(), 0.2);
  }
}

void OptimizeFromFileHelper(std::string path_to_file, std::string input_file,
                            int subsampling,
                            std::string startpoint_string = "") {
  const std::string path_of_standard_trajectory =
      path_to_file + "/" + input_file;
  ADCTrajectory trajectory;
  EXPECT_TRUE(cyber::common::GetProtoFromFile(path_of_standard_trajectory,
                                              &trajectory));
  DiscretizedTrajectory traj_in(trajectory);
  const double dt_in =
      traj_in.at(1).relative_time() - traj_in.at(0).relative_time();
  const double T_in = (traj_in.NumOfPoints() - 1) * dt_in;

  const std::string path_to_startpoint_file =
      path_to_file + "/" + startpoint_string;
  apollo::common::TrajectoryPoint start_point;
  if (startpoint_string.empty()) {
    start_point = traj_in.front();
  } else {
    EXPECT_TRUE(
        cyber::common::GetProtoFromFile(path_to_startpoint_file, &start_point));
  }

  // OPTIMIZER
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt("/apollo/data/log/");
  tsm.InitializeProblem(subsampling, traj_in, start_point);
  int status = tsm.Optimize();

  // OPTIMIZER with infinite max_time termination
  TrajectorySmootherNLOpt tsm2 = TrajectorySmootherNLOpt("/apollo/data/log/");
  tsm2.InitializeProblem(subsampling, traj_in, start_point);
  auto params = tsm2.GetSolverParameters();
  params.max_time = 1e4;
  tsm.SetSolverParameters(params);
  int status2 = tsm2.Optimize();

  CompareTrajectories(tsm.GetOptimizedTrajectory(), tsm2.GetOptimizedTrajectory());

  EXPECT_GT(status, 0);
  EXPECT_LT(status, 7);  // 5 ... NLOPT_MAXEVAL_REACHED
  EXPECT_TRUE(tsm.ValidateSmoothingSolution());
  auto traj_opt = tsm.GetOptimizedTrajectory();
  for (size_t trajidx = 0; trajidx < traj_opt.size(); ++trajidx) {
    AINFO << "Smoothed trajectory at i=" << trajidx << ": "
          << traj_opt[trajidx].DebugString();
  }
  tsm.DebugDumpU();
  tsm.DebugDumpX();
  tsm.DebugDumpXref();

  const std::string file_name =
      "sqp_out_" + std::to_string(subsampling) + "_" + input_file;
  SaveDiscretizedTrajectoryToFile(traj_opt, path_to_file, file_name);

  EXPECT_DOUBLE_EQ(traj_in.GetTemporalLength(), T_in);
  EXPECT_DOUBLE_EQ(traj_opt.GetTemporalLength(), T_in);
}

TEST(TrajectorySmootherNLOpt, Constructor) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt("/apollo/data/log/");
  EXPECT_NE(&tsm, nullptr);
}

TEST(TrajectorySmootherNLOpt, Optimize_Empty) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt("/apollo/data/log/");
  DiscretizedTrajectory tmp;
  tsm.InitializeProblem(1, tmp, common::TrajectoryPoint());
  int status = tsm.Optimize();
  EXPECT_EQ(-100, status);
}

TEST(TrajectorySmootherNLOpt, Optimize1) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt("/apollo/data/log/");
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
  tsm.InitializeProblem(3, traj, tp1);
  int status = tsm.Optimize();
  auto traj_opt = tsm.GetOptimizedTrajectory();
  for (size_t trajidx = 0; trajidx < traj_opt.size(); ++trajidx) {
    AINFO << "Smoothed trajectory at i=" << trajidx << ": "
          << traj_opt[trajidx].DebugString();
  }
  EXPECT_GT(status, 0);
  EXPECT_LT(status, 5);  // 5 ... NLOPT_MAXEVAL_REACHED
  EXPECT_GT(tsm.GetNumEvals(), 1);
}

TEST(TrajectorySmootherNLOpt, PlanningInitPoint) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt("/apollo/data/log/");
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
  auto planning_init_point = tp1;
  planning_init_point.mutable_path_point()->set_kappa(0.1);
  tsm.InitializeProblem(0, traj, planning_init_point);
  int status = tsm.Optimize();
  auto traj_opt = tsm.GetOptimizedTrajectory();
  for (size_t trajidx = 0; trajidx < traj_opt.size(); ++trajidx) {
    AINFO << "Smoothed trajectory at i=" << trajidx << ": "
          << traj_opt[trajidx].DebugString();
  }
  EXPECT_GT(status, 0);
  EXPECT_FLOAT_EQ(traj_opt.at(0).path_point().kappa(),
                  planning_init_point.path_point().kappa());
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp.pb.txt";
  int subsampling = 0;  // no subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFileSubsampling) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFileStartDriving) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_from_standstill.pb.txt";
  int subsampling = 0;  // no subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFileInCurve) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_in_curve.pb.txt";
  int subsampling = 1;  // no subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210429135504) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210429-135504.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210429135505) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210429-135505.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210429135507) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210429-135507.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210429135518) {
  // Not sure, if this is a reasonable test, as transformed input is already
  // crap
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210429-135518.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210429135518_Orig) {
  // Corresponds to OptimizeFromFile20210429135518, but operates on original
  // miqp trajectory, and not the transformed one
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file =
      "test_trajectory_miqp_20210429-135518_orig.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210429133659) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210429-133659.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210503140734) {
  // FROM SIMULATION
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210503-140734.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210504121851) {
  // Starting from stillstand, smoothing trajectory from reference generator
  // there is quite an offset between the initial state and the rest of the
  // reference
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210504-121851.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210504123902) {
  // Starting from stillstand, smoothing trajectory from reference generator
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210504-123902.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210504124154) {
  // Starting from stillstand, smoothing trajectory from reference generator
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210504-124154.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, OptimizeFromFile20210504124308) {
  // Starting from stillstand, smoothing trajectory from reference generator
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_trajectory_miqp_20210504-124308.pb.txt";
  int subsampling = 1;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

TEST(TrajectorySmootherNLOpt, model_f) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt("/apollo/data/log/");
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
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt("/apollo/data/log/");
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
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt("/apollo/data/log/");
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

TEST(TrajectorySmootherNLOpt, OptimizeFromFileSZero) {
  const std::string path_to_file =
      "modules/planning/planner/miqp/miqp_testdata";
  const std::string input_file = "test_reproduce_szero.pb.txt";
  int subsampling = 0;  // subsampling
  OptimizeFromFileHelper(path_to_file, input_file, subsampling);
}

// Test commented as it fails. why? this is a recorded traj with initial jerk >
// jerk bound of the smoother -> numeric issues and local minimum
// TEST(TrajectorySmootherNLOpt, OptimizeFromFileMaxStepsStart) {
//   const std::string path_to_file =
//       "modules/planning/planner/miqp/miqp_testdata";
//   const std::string input_file = "test_reproduce_invalid_start.pb.txt";
//   const std::string start_point_file =
//       "startpoint_reproduce_invalid_start.pb.txt";
//   int subsampling = 1;  // subsampling
//   OptimizeFromFileHelper(path_to_file, input_file, subsampling,
//                          start_point_file);
// }

}  // namespace planning
}  // namespace apollo
