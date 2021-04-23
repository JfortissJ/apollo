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

#pragma once

#include <nlopt.hpp>
#include "modules/planning/common/trajectory/discretized_trajectory.h"

#include "Eigen/Dense"

namespace apollo {
namespace planning {

class TrajectorySmootherNLOpt {
 public:
  struct ProblemParameters {
    ProblemParameters() {}
    // costs for deviation from the initial reference
    double cost_offset_x;
    double cost_offset_y;
    double cost_offset_theta;
    double cost_offset_v;
    // costs on absolute values
    double cost_steering;
    double cost_acceleration;
    // costs on input
    double cost_steering_change;  // xi
    double cost_acceleration_change;
    // tolerances for the initial and final steering
    double steering_tolerance;
  };

  struct SolverParameters {
   public:
    SolverParameters()
        : algorithm(nlopt::LD_SLSQP),
          x_tol_rel(1e-6),
          x_tol_abs(1e-6),
          ineq_const_tol(1e-4),
          eq_const_tol(1e-4),
          max_num_evals(1000) {}

    // algorithm to use for optimization. check NLOPT Documentation
    // http://ab-initio.mit.edu/wiki/index.php/NLopt_Algorithms
    nlopt::algorithm algorithm;
    // tolerance in relative (scaled by parameter value) change of the
    // parameters. relative tolerance has problems when optimal parameters are
    // close to zero
    double x_tol_rel;
    // tolerance in absolute change of the parameters
    double x_tol_abs;
    // tolerance for each inequality constraint
    double ineq_const_tol;
    // tolerance for each equality constraint
    double eq_const_tol;
    // maximum number of function evaluations
    size_t max_num_evals;
  };

  explicit TrajectorySmootherNLOpt();
  virtual ~TrajectorySmootherNLOpt() = default;

  int Optimize();

  // initialize the optimization problem
  // initialize _X_ref, lb und ub of inputs, _x0
  // TODO(@Klemens)
  // TODO what is the best interface for the input traj? custom matrix? apollo
  // adctraj format?
  void InitializeProblem(const int subsampling,
                         const DiscretizedTrajectory& input_trajectory,
                         double initial_steering = 0.0f);

  // TODO(@Tobias)
  // take care of different length of reference -> only use costs on reference
  // at pts where there is one
  // has to be public due to the function pointer wrapper
  double ObjectiveFunction(unsigned n, const double* x, double* grad);

  // bounds on acc and steering
  void InequalityConstraintFunction(unsigned m, double* result, unsigned n,
                                    const double* x, double* grad);
  // no equality constraints for now
  void EqualityConstraintFunction(unsigned m, double* result, unsigned n,
                                  const double* x, double* grad);

  std::vector<double>& GetInputVector() { return u_; }

 private:
  // TODO(@Klemens): Heuns Method, fill block matrices A_
  // x0: state at t=0 -> from vehicle state
  // u: initial input vector -> chosen by optimizer
  // X: stacked state vector over time
  // dXdU
  void IntegrateModel(const Eigen::VectorXd& x0, const Eigen::VectorXd& u,
                      const size_t num_integration_steps, Eigen::VectorXd& X,
                      Eigen::MatrixXd& dXdU);

 private:
  // stores the positions of the reference
  Eigen::VectorXd X_ref_;
  // stores the initial state
  Eigen::VectorXd x0_;

  // stores the currently integrated trajectory
  Eigen::VectorXd X_;
  // stores the gradient of the trajectory w.r.t. to the inputs of the
  // optimization
  Eigen::MatrixXd dXdU_;

  // TODO (@Klemens): agree with Tobias on integration method!
  Eigen::MatrixXd A_;
  Eigen::Vector3d currx_;
  Eigen::Matrix3d currA_;
  Eigen::Vector3d currH_;
  Eigen::Vector3d currB_;
  Eigen::VectorXd last_u_;

  // why is this all using vector, not eigen? -> tk: because of the nlopt api.
  std::vector<double> u_;

  double j_opt_;
  int status_;
  std::vector<double> lower_bound_;
  std::vector<double> upper_bound_;
  std::vector<double> ineq_constraint_tol_;
  std::vector<double> eq_constraint_tol_;

  size_t problem_size_;
  size_t num_ineq_constr_;  // TODO(@Klemens) which ones?
  size_t num_eq_constr_;    // TODO(@Klemens) which ones?
  SolverParameters solver_params_;
  ProblemParameters params_;

  //Indices and sizes of our model
  enum STATES {
    X = 0,
    Y = 1,
    THETA = 2,
    V = 3,
    A = 4,
    KAPPA = 5,
    STATES_SIZE = 6
  };

  enum INPUTS {
    J = 0,
    XI = 1,
    INPUTS_SIZE = 2
  };
};

}  // namespace planning
}  // namespace apollo
