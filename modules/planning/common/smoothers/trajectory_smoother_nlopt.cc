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

#include "modules/planning/common/smoothers/trajectory_smoother_nlopt.h"

#include <nlopt.h>

#include <iomanip>
#include <iostream>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/logger/logger_util.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"

// Create function pointers for nlopt outside the namespace
double nlopt_objective_wrapper(unsigned n, const double* x, double* grad,
                               void* data) {
  apollo::planning::TrajectorySmootherNLOpt* obj =
      static_cast<apollo::planning::TrajectorySmootherNLOpt*>(data);
  return obj->ObjectiveFunction(n, x, grad);
}

void nlopt_inequality_constraint_wrapper(unsigned m, double* result, unsigned n,
                                         const double* x, double* grad,
                                         void* data) {
  apollo::planning::TrajectorySmootherNLOpt* obj =
      static_cast<apollo::planning::TrajectorySmootherNLOpt*>(data);
  obj->InequalityConstraintFunction(m, result, n, x, grad);
}

void nlopt_equality_constraint_wrapper(unsigned m, double* result, unsigned n,
                                       const double* x, double* grad,
                                       void* data) {
  apollo::planning::TrajectorySmootherNLOpt* obj =
      static_cast<apollo::planning::TrajectorySmootherNLOpt*>(data);
  obj->EqualityConstraintFunction(m, result, n, x, grad);
}

std::string GetTimeString() {
  // from cyber/logger/log_file_object.cc
  struct ::tm tm_time;
  const time_t timestamp =
      static_cast<time_t>(apollo::common::time::Clock::NowInSeconds());
  localtime_r(&timestamp, &tm_time);
  std::ostringstream time_pid_stream;
  time_pid_stream.fill('0');
  time_pid_stream << 1900 + tm_time.tm_year << std::setw(2)
                  << 1 + tm_time.tm_mon << std::setw(2) << tm_time.tm_mday
                  << '-' << std::setw(2) << tm_time.tm_hour << std::setw(2)
                  << tm_time.tm_min << std::setw(2) << tm_time.tm_sec << '.'
                  << apollo::cyber::logger::GetMainThreadPid();
  return time_pid_stream.str();
}

namespace apollo {
namespace planning {

using namespace Eigen;
using apollo::common::time::Clock;

TrajectorySmootherNLOpt::TrajectorySmootherNLOpt(const char logdir[],
                                                 const double pts_offset_x,
                                                 const double pts_offset_y)
    : logdir_(logdir),
      pts_offset_x_(pts_offset_x),
      pts_offset_y_(pts_offset_y),
      modified_input_trajectory_() {
  x0_.resize(STATES::STATES_SIZE);

  num_ineq_constr_ = 0;
  num_eq_constr_ = 0;
  numevals_ = 0;
}

TrajectorySmootherNLOpt::TrajectorySmootherNLOpt(const char logdir[])
    : TrajectorySmootherNLOpt::TrajectorySmootherNLOpt(logdir, 0.0, 0.0) {}

void TrajectorySmootherNLOpt::InitializeProblem(
    const int subsampling, const DiscretizedTrajectory& input_trajectory,
    const common::TrajectoryPoint& planning_init_point) {
  ready_to_optimize_ = false;
  input_traj_size_ = input_trajectory.size();
  subsampling_ = subsampling;
  if (input_traj_size_ < 1) {
    AERROR << "Empty input trajectory!";
    return;
  }
  if (input_traj_size_ == 1) {
    AINFO << "Input trajectory has only one point, no need for smoothing!";
    return;
  }

  // replace first point by planning_init_point (which is the current or
  // predicted state of the vehicle) to use a reasonable kappa during standstill
  // and to mitigate transformation issues
  modified_input_trajectory_ = input_trajectory;
  modified_input_trajectory_.at(0) = planning_init_point;

  // set problem size
  int nr_intermediate_pts = (input_traj_size_ - 1) * subsampling_;
  nr_integration_steps_ = input_traj_size_ + nr_intermediate_pts;
  problem_size_ = nr_integration_steps_ * INPUTS::INPUTS_SIZE;
  // x_ is resized in IntegrateModel()

  stepsize_ = Round(modified_input_trajectory_.at(1).relative_time() -
                        modified_input_trajectory_.at(0).relative_time(),
                    2);
  stepsize_ = stepsize_ / (subsampling_ + 1);
  initial_time_ = modified_input_trajectory_.at(0).relative_time();

  // set x0 using the reference traj
  const common::TrajectoryPoint& first_pt = modified_input_trajectory_.front();
  x0_[STATES::X] = Round(first_pt.path_point().x() - pts_offset_x_, precision_);
  x0_[STATES::Y] = Round(first_pt.path_point().y() - pts_offset_y_, precision_);
  x0_[STATES::THETA] = Round(first_pt.path_point().theta(), precision_);
  x0_[STATES::V] = BoundedVelocity(Round(first_pt.v(), precision_));
  x0_[STATES::A] = BoundedAcceleration(Round(first_pt.a(), precision_));
  x0_[STATES::KAPPA] =
      BoundedCurvature(Round(first_pt.path_point().kappa(), precision_));

  if (!CheckBoundsAfterIntegration(0.0, 0.0, subsampling_ + 1)) {
    AINFO << "Overwriting initial acceleration";
    x0_[STATES::A] = 0.0;
  }

  // set reference from input
  X_ref_.resize(input_traj_size_ * STATES::STATES_SIZE);
  int offset = 0;
  for (auto& pt : modified_input_trajectory_) {
    X_ref_[offset + STATES::X] =
        Round(pt.path_point().x() - pts_offset_x_, precision_);
    X_ref_[offset + STATES::Y] =
        Round(pt.path_point().y() - pts_offset_y_, precision_);
    X_ref_[offset + STATES::THETA] = Round(pt.path_point().theta(), precision_);
    X_ref_[offset + STATES::V] = BoundedVelocity(Round(pt.v(), precision_));
    X_ref_[offset + STATES::A] = BoundedAcceleration(Round(pt.a(), precision_));
    X_ref_[offset + STATES::KAPPA] =
        BoundedCurvature(Round(pt.path_point().kappa(), precision_));
    offset += STATES::STATES_SIZE;
  }

  u_.resize(problem_size_, 0.0);
  last_u_.setZero(problem_size_);

  // set lower and upper bound vector
  lower_bound_.resize(problem_size_);
  upper_bound_.resize(problem_size_);
  for (size_t idx = 0; idx < problem_size_; idx += 2) {
    lower_bound_.at(idx + INPUTS::J) = params_.lower_bound_jerk - 1e-4;
    lower_bound_.at(idx + INPUTS::XI) =
        params_.lower_bound_curvature_change - 1e-4;
    upper_bound_.at(idx + INPUTS::J) = params_.upper_bound_jerk + 1e-4;
    upper_bound_.at(idx + INPUTS::XI) =
        params_.upper_bound_curvature_change + 1e-4;
  }

  X_lb_.setZero(STATES::STATES_SIZE * nr_integration_steps_);
  X_ub_.setZero(STATES::STATES_SIZE * nr_integration_steps_);
  C_kappa_.resize(STATES::STATES_SIZE * nr_integration_steps_,
                  nr_integration_steps_);
  C_vel_.resize(STATES::STATES_SIZE * nr_integration_steps_,
                nr_integration_steps_);
  offset = 0;
  for (size_t idx = 0; idx < nr_integration_steps_; ++idx) {
    X_lb_[offset + STATES::X] = -1e3;
    X_lb_[offset + STATES::Y] = -1e3;
    X_lb_[offset + STATES::THETA] = -1e3;
    X_lb_[offset + STATES::V] = params_.lower_bound_velocity - 1e-4;
    X_lb_[offset + STATES::A] = -1e3;
    X_lb_[offset + STATES::KAPPA] = params_.lower_bound_curvature - 1e-4;
    X_ub_[offset + STATES::X] = 1e3;
    X_ub_[offset + STATES::Y] = 1e3;
    X_ub_[offset + STATES::THETA] = 1e3;
    X_ub_[offset + STATES::V] = params_.upper_bound_velocity + 1e-4;
    X_ub_[offset + STATES::A] = 1e3;
    X_ub_[offset + STATES::KAPPA] = params_.upper_bound_curvature + 1e-4;
    if (offset > 0) {  // no constraints for the initial point
      C_kappa_.insert(offset + STATES::KAPPA, idx) = 1;
      C_vel_.insert(offset + STATES::V, idx) = 1;
    }

    offset += STATES::STATES_SIZE;
  }
  num_ineq_constr_ =
      2 * 2 *
      nr_integration_steps_;  // upper and lower bound for kappa and velocity

  CalculateJthreshold();

  // Constraints
  if (num_ineq_constr_ > 0) {
    ineq_constraint_tol_.clear();
    ineq_constraint_tol_.resize(num_ineq_constr_,
                                solver_params_.ineq_const_tol);
  }

  if (num_eq_constr_ > 0) {
    eq_constraint_tol_.clear();
    eq_constraint_tol_.resize(num_eq_constr_, solver_params_.eq_const_tol);
  }

  status_ = 0;
  ready_to_optimize_ = true;
}

int TrajectorySmootherNLOpt::Optimize() {
  if (!ready_to_optimize_) {
    AERROR << "Optimization Problem was not initialized!";
    return -100;
  }

  // Initialize the optimization problem
  nlopt::opt opt(solver_params_.algorithm, problem_size_);

  // Options
  opt.set_xtol_rel(solver_params_.x_tol_rel);
  opt.set_xtol_abs(solver_params_.x_tol_abs);
  opt.set_maxeval(solver_params_.max_num_evals);
  opt.set_maxtime(solver_params_.max_time);

  // Upper and lower bound on u
  if (!lower_bound_.empty()) {
    opt.set_lower_bounds(lower_bound_);
  }
  if (!upper_bound_.empty()) {
    opt.set_upper_bounds(upper_bound_);
  }

  // Objective Function
  opt.set_min_objective(nlopt_objective_wrapper, this);

  // Constraints
  if (num_ineq_constr_ > 0) {
    opt.add_inequality_mconstraint(nlopt_inequality_constraint_wrapper, this,
                                   ineq_constraint_tol_);
  }

  if (num_eq_constr_ > 0) {
    opt.add_equality_mconstraint(nlopt_equality_constraint_wrapper, this,
                                 eq_constraint_tol_);
  }

  AINFO << "Starting smoothing optimization";
  double current_time = Clock::NowInSeconds();

  // Optimization
  try {
    status_ = static_cast<int>(opt.optimize(u_, j_opt_));
  } catch (nlopt::roundoff_limited ex) {
    AWARN << "Roundoff limited exception:" << ex.what();
  } catch (std::bad_alloc ex) {
    AWARN << "Out of memory exception:" << ex.what();
  } catch (std::invalid_argument ex) {
    AWARN << "Invalid argument exception:" << ex.what();
  } catch (std::runtime_error ex) {
    AWARN << "Generic failure exception:" << ex.what();
  } catch (std::exception ex) {
    AERROR << "Unhandled Exception while optimization: " << ex.what();
    status_ = -11;
    return status_;
  }

  AINFO << "Smoothing optimization finished with final cost of " << j_opt_
        << " in " << (Clock::NowInSeconds() - current_time) << "s and with "
        << numevals_ << " iterations";

  switch (status_) {
    case nlopt::SUCCESS:
      AINFO << "Generic success return value.";
      break;
    case nlopt::STOPVAL_REACHED:
      AINFO << "Optimization stopped because stopval was reached.";
      break;
    case nlopt::FTOL_REACHED:
      AINFO << "Optimization stopped because ftol_rel or ftol_abs was reached.";
      break;
    case nlopt::XTOL_REACHED:
      AINFO << "Optimization stopped because xtol_rel or xtol_abs was reached.";
      break;
    case nlopt::MAXEVAL_REACHED:
      AINFO << "Optimization stopped because maxeval was reached.";
      break;
    case nlopt::MAXTIME_REACHED:
      AINFO << "Optimization stopped because maxtime was reached.";
      break;
    case nlopt::INVALID_ARGS:
      AWARN << "Invalid arguments (e.g. lower bounds are bigger than upper "
               "bounds, "
               "an unknown algorithm was specified, etcetera).";
      break;
    case nlopt::OUT_OF_MEMORY:
      AWARN << "Ran out of memory.";
      break;
    case nlopt::ROUNDOFF_LIMITED:
      AWARN << "Halted because roundoff errors limited progress. (In this "
               "case, the "
               "optimization still typically returns a useful result.)";
      status_ = 10;
      break;
    default:
      ///@see http://ab-initio.mit.edu/wiki/index.php/NLopt_Reference
      AINFO << "Generic return value: " << status_;
  }

  //  TODO do we need this???
  //   if (!checkConstraints()) {
  //     AERROR << "Constraints are not satisfied within tolerance.";
  //     status_ = -10;  // extra status value...
  //   }

  if (status_ > 0) {
    AINFO << "Smoothing optimization successful."
          << " NlOpt Status: " << status_;
  } else {
    AERROR << "Smoothing optimization failed."
           << " NlOpt Status: " << status_;
  }
  return status_;
}

DiscretizedTrajectory TrajectorySmootherNLOpt::GetOptimizedTrajectory() {
  DiscretizedTrajectory traj;
  const int size_state_vector = X_.rows();
  double s = 0.0f;
  double lastx = X_[STATES::X];
  double lasty = X_[STATES::Y];
  for (int idx = 0; idx < size_state_vector / STATES::STATES_SIZE; ++idx) {
    common::TrajectoryPoint tp;
    const double x = X_[idx * STATES::STATES_SIZE + STATES::X];
    const double y = X_[idx * STATES::STATES_SIZE + STATES::Y];
    tp.mutable_path_point()->set_x(x + pts_offset_x_);
    tp.mutable_path_point()->set_y(y + pts_offset_y_);
    s += sqrt(pow(x - lastx, 2) + pow(y - lasty, 2));
    tp.mutable_path_point()->set_s(s);
    tp.mutable_path_point()->set_theta(
        X_[idx * STATES::STATES_SIZE + STATES::THETA]);
    tp.mutable_path_point()->set_kappa(
        X_[idx * STATES::STATES_SIZE + STATES::KAPPA]);
    tp.set_v(X_[idx * STATES::STATES_SIZE + STATES::V]);
    tp.set_a(X_[idx * STATES::STATES_SIZE + STATES::A]);
    // TODO do we have to use idx-1 for the inputs?
    tp.set_da(u_[idx * INPUTS::INPUTS_SIZE + INPUTS::J]);
    tp.mutable_path_point()->set_dkappa(
        u_[idx * INPUTS::INPUTS_SIZE + INPUTS::XI]);
    tp.set_relative_time(initial_time_ + idx * stepsize_);
    traj.AppendTrajectoryPoint(tp);
    lastx = x;
    lasty = y;
  }
  return traj;
}

double TrajectorySmootherNLOpt::ObjectiveFunction(unsigned n, const double* x,
                                                  double* grad) {
  // TODO optimize computation time: only compute values if the respective cost
  // terms are nonzero!

  double J = 0;
  Map<const VectorXd> u_eigen(x, n);
  Map<VectorXd> grad_eigen(grad, n);
  if (grad != NULL) {
    grad_eigen.fill(0);
  }

  CalculateCommonDataIfNecessary(u_eigen);

  // Costs on reference deviation and states
  // differences vector: for x,y,theta,v compute state-ref for each
  // non-subsampled point absolute vector: for a,kappa copy the value from X_
  const int size_state_vector = X_.rows();
  VectorXd difference;
  difference.setZero(size_state_vector);
  VectorXd absolute;
  absolute.setZero(size_state_vector);
  VectorXd difference_costs;
  difference_costs.setZero(size_state_vector);
  VectorXd absolute_costs;
  absolute_costs.setZero(size_state_vector);
  Vector6d costs_state;
  costs_state << params_.cost_offset_x, params_.cost_offset_y,
      params_.cost_offset_theta, params_.cost_offset_v,
      params_.cost_acceleration, params_.cost_curvature;
  bool any_difference_term =
      (params_.cost_offset_x != 0) || (params_.cost_offset_y != 0) ||
      (params_.cost_offset_theta != 0) || (params_.cost_offset_v != 0);
  bool any_absolute_term =
      (params_.cost_acceleration != 0) || (params_.cost_curvature != 0);

  // int idx0 = 3;
  // int idx1 = 15;

  for (int idx = 0; idx < size_state_vector / STATES::STATES_SIZE; ++idx) {
    size_t idx_vec = idx * STATES::STATES_SIZE;
    size_t idx_vec_sub = idx / (subsampling_ + 1) * STATES::STATES_SIZE;
    if (any_difference_term) {
      if (idx % (subsampling_ + 1) == 0) {  // not a subsampled step
        for (int element = 0; element < 4; ++element) {  // only for x,y,theta,v
          difference[idx_vec + element] =
              X_[idx_vec + element] - X_ref_[idx_vec_sub + element];
          difference_costs[idx_vec + element] = costs_state[element];
          // difference_costs[idx_vec + element] = InterpolateWithinBounds(idx0,
          // 0.1*costs_state[element], idx1, 1.0*costs_state[element], idx);
        }
      }
    }
    if (any_absolute_term) {
      absolute[idx_vec + STATES::A] = X_[idx_vec + STATES::A];
      absolute_costs[idx_vec + STATES::A] = costs_state[STATES::A];
      absolute[idx_vec + STATES::KAPPA] = X_[idx_vec + STATES::KAPPA];
      absolute_costs[idx_vec + STATES::KAPPA] = costs_state[STATES::KAPPA];
      // absolute_costs[idx_vec + STATES::KAPPA] =
      // InterpolateWithinBounds(idx0, 2.0*costs_state[STATES::KAPPA],
      // idx1, 1.0*costs_state[STATES::KAPPA], idx);
    }
  }

  // Costs on inputs
  VectorXd absolute_inputs;
  VectorXd costs_inputs;
  bool any_absolute_input_term =
      (params_.cost_acceleration_change) || (params_.cost_curvature_change);
  if (any_absolute_input_term) {
    absolute_inputs.setZero(n);
    costs_inputs.setZero(n);
    for (int idx = 0; idx < static_cast<int>(n) / INPUTS::INPUTS_SIZE; ++idx) {
      absolute_inputs[idx * INPUTS::INPUTS_SIZE + INPUTS::J] =
          u_eigen[idx * INPUTS::INPUTS_SIZE + INPUTS::J];
      absolute_inputs[idx * INPUTS::INPUTS_SIZE + INPUTS::XI] =
          u_eigen[idx * INPUTS::INPUTS_SIZE + INPUTS::XI];
      costs_inputs[idx * INPUTS::INPUTS_SIZE + INPUTS::J] =
          params_.cost_acceleration_change;
      costs_inputs[idx * INPUTS::INPUTS_SIZE + INPUTS::XI] =
          params_.cost_curvature_change;
      // costs_inputs[idx * INPUTS::INPUTS_SIZE + INPUTS::XI] =
      //     InterpolateWithinBounds(idx0, 2.0*params_.cost_curvature_change,
      //     idx1, 1.0*params_.cost_curvature_change, idx);
    }
  }

  // Compute cost term
  if (any_difference_term) {
    J += difference.transpose() * difference.cwiseProduct(difference_costs);
  }
  if (any_absolute_term) {
    J += absolute.transpose() * absolute.cwiseProduct(absolute_costs);
  }
  if (any_absolute_input_term) {
    J += absolute_inputs.transpose() *
         absolute_inputs.cwiseProduct(costs_inputs);
  }

  // Gradients: Derivate of J
  if (grad != NULL) {
    if (any_difference_term) {
      grad_eigen +=
          2 * dXdU_.transpose() * difference.cwiseProduct(difference_costs);
    }
    if (any_absolute_term) {
      grad_eigen +=
          2 * dXdU_.transpose() * absolute.cwiseProduct(absolute_costs);
    }
    if (any_absolute_input_term) {
      grad_eigen += 2 * absolute_inputs.cwiseProduct(costs_inputs);
    }
    // std::cout << "cost function grad_eigen: \n" << grad_eigen << std::endl;
  }

  numevals_ += 1;
  return J;
}

void TrajectorySmootherNLOpt::InequalityConstraintFunction(
    unsigned m, double* result, unsigned n, const double* x, double* grad) {
  // m ... number of constraints
  // n ... size of input vector aka x (nlopt)
  // map input values to eigen vectors
  Map<const VectorXd> u_eigen(x, n);
  Map<MatrixXd> grad_eigen(grad, n, m);
  // Map<MatrixXd> grad_eigen(grad, m, n);
  if (grad != NULL) grad_eigen.fill(0);
  const size_t nr_is = nr_integration_steps_;
  constexpr size_t dimU = INPUTS::INPUTS_SIZE;
  Map<VectorXd> cineq_eigen(result, m);

  CalculateCommonDataIfNecessary(u_eigen);

  // upper bounds
  // cineq_eigen.topRows(nr_is*6) = (X_ - X_ub_).transpose();
  cineq_eigen.block(nr_is * 0, 0, nr_is, 1) =
      ((X_ - X_ub_).transpose() * C_kappa_).transpose();
  cineq_eigen.block(nr_is * 1, 0, nr_is, 1) =
      ((X_ - X_ub_).transpose() * C_vel_).transpose();

  // lower bounds
  // cineq_eigen.bottomRows(nr_is*6) = (-X_ + X_lb_).transpose();
  cineq_eigen.block(nr_is * 2, 0, nr_is, 1) =
      ((-X_ + X_lb_).transpose() * C_kappa_).transpose();
  cineq_eigen.block(nr_is * 3, 0, nr_is, 1) =
      ((-X_ + X_lb_).transpose() * C_vel_).transpose();

  if (grad != NULL) {
    // upper bounds
    // std::cout << "dXdU_.size() " << dXdU_.rows() << ", " << dXdU_.cols() <<
    // "grad_eigen.size() " << grad_eigen.rows() << ", " << grad_eigen.cols() <<
    // "C_kappa_.size() " << C_kappa_.rows() << ", " << C_kappa_.cols()
    // <<std::endl ; grad_eigen.topRows(m/2) = dXdU_;
    // grad_eigen.leftCols(nr_is*6) = dXdU_.transpose();
    grad_eigen.block(0, nr_is * 0, dimU * nr_is, nr_is) =
        dXdU_.transpose() * C_kappa_;
    grad_eigen.block(0, nr_is * 1, dimU * nr_is, nr_is) =
        dXdU_.transpose() * C_vel_;
    // std::cout << "dXdU_.transpose()  * C_kappa_  \n" << dXdU_.transpose()  *
    // C_kappa_ << std::endl; std::cout << "grad_eigen.block(nr_is * 1, 0,
    // nr_is, dimU * nr_is)  \n" << grad_eigen.block(nr_is * 1, 0, nr_is, dimU *
    // nr_is) << std::endl; grad_eigen.block(nr_is * 0, 0, nr_is, dimU * nr_is)
    // = (dXdU_.transpose()  * C_kappa_).transpose(); grad_eigen.block(nr_is *
    // 1, 0, nr_is, dimU * nr_is) = (dXdU_.transpose()  * C_vel_).transpose();

    // lower bounds
    // grad_eigen.bottomRows(m/2) = -dXdU_;
    // grad_eigen.rightCols(nr_is*6) = -dXdU_.transpose();
    grad_eigen.block(0, nr_is * 2, dimU * nr_is, nr_is) =
        -dXdU_.transpose() * C_kappa_;
    grad_eigen.block(0, nr_is * 3, dimU * nr_is, nr_is) =
        -dXdU_.transpose() * C_vel_;
    // grad_eigen.block(nr_is * 2, 0, nr_is, dimU * nr_is) = (-dXdU_.transpose()
    // * C_kappa_).transpose(); grad_eigen.block(nr_is * 3, 0, nr_is, dimU *
    // nr_is) = (-dXdU_.transpose()  * C_vel_).transpose(); std::cout <<
    // "u_eigen: \n" << u_eigen << std::endl; std::cout << "constraint
    // grad_eigen: \n" << grad_eigen << std::endl;
    // std::cout << "grad_eigen * // u_eigen: \n" << grad_eigen.transpose() *
    // u_eigen << std::endl;
  }
}

void TrajectorySmootherNLOpt::EqualityConstraintFunction(
    unsigned m, double* result, unsigned n, const double* x, double* grad) {}

void TrajectorySmootherNLOpt::IntegrateModel(const Vector6d& x0,
                                             const Eigen::VectorXd& u,
                                             const size_t num_integration_steps,
                                             const double h, Eigen::VectorXd& X,
                                             Eigen::MatrixXd& dXdU) {
  constexpr size_t dimX = STATES::STATES_SIZE;
  constexpr size_t dimU = INPUTS::INPUTS_SIZE;
  const size_t N = num_integration_steps;

  // X.resize(dimX * N);
  X.setZero(dimX * N);
  X.block<dimX, 1>(0, 0) = x0;

  // dXdU.resize(dimX * N, dimU * N);
  // dXdU.fill(0.0f);
  dXdU.setZero(dimX * N, dimU * N);

  // currB_.resize(dimX, dimU);
  currB_.setZero(dimX, dimU);

  size_t row_idx, row_idx_before;

  for (size_t i = 1; i < N; ++i) {
    row_idx = i * dimX;
    row_idx_before = (i - 1) * dimX;

    Eigen::Vector2d u_curr = u.block<dimU, 1>((i - 1) * dimU, 0);

    const Vector6d& x_before = X.block<dimX, 1>(row_idx_before, 0);
    model_f(x_before, u_curr, h, currx_);
    model_dfdx(x_before, u_curr, h, currA_);
    model_dfdu(x_before, u_curr, h, currB_);

    X.block<dimX, 1>(row_idx, 0) = currx_;

    // diagonal
    dXdU.block<dimX, dimU>(row_idx, (i - 1) * dimU) = currB_;

    for (size_t idx_n = 0; idx_n < i - 1; ++idx_n) {
      // from left to diagonal
      dXdU.block<dimX, dimU>(row_idx, idx_n * dimU) =
          currA_ * dXdU.block<dimX, dimU>(row_idx_before, idx_n * dimU);
    }
  }
  // std::cout << "IntegrateModel()\nx0 \n" << x0 << std::endl;
  // std::cout << "u \n" << u << std::endl;
  // std::cout << "X \n" << X << std::endl;
  // std::cout << "dXdU \n" << dXdU << std::endl;
}

void TrajectorySmootherNLOpt::model_f(const Vector6d& x,
                                      const Eigen::Vector2d& u, const double h,
                                      Vector6d& x_out) const {
  const double sinth = sin(x(STATES::THETA));
  const double costh = cos(x(STATES::THETA));
  const double c1 = x(STATES::V) + h * x(STATES::A);
  const double c2 = x(STATES::THETA) + h * x(STATES::V) * x(STATES::KAPPA);
  const double c3 = x(STATES::KAPPA) + h * u(INPUTS::XI);
  const double c4 = x(STATES::A) + h * u(INPUTS::J);

  const double x1 =
      x(STATES::X) + 0.5 * h * x(STATES::V) * costh + 0.5 * h * c1 * cos(c2);
  const double y1 =
      x(STATES::Y) + 0.5 * h * x(STATES::V) * sinth + 0.5 * h * c1 * sin(c2);
  const double theta1 = x(STATES::THETA) +
                        0.5 * h * x(STATES::V) * x(STATES::KAPPA) +
                        0.5 * h * c1 * c3;
  // const double theta1 = common::math::NormalizeAngle(x(STATES::THETA) + 0.5 *
  // h * x(STATES::V) * x(STATES::KAPPA) +
  //     0.5 * h * c1 * c3);
  const double v1 = x(STATES::V) + 0.5 * h * x(STATES::A) + 0.5 * h * c4;
  const double a1 = c4;
  const double kappa1 = c3;
  x_out << x1, y1, theta1, v1, a1, kappa1;
}

void TrajectorySmootherNLOpt::model_dfdx(const Vector6d& x,
                                         const Eigen::Vector2d& u,
                                         const double h,
                                         Matrix6d& dfdx_out) const {
  const double sinth = sin(x(STATES::THETA));
  const double costh = cos(x(STATES::THETA));
  const double c1 = x(STATES::V) + h * x(STATES::A);
  const double c2 = x(STATES::THETA) + h * x(STATES::V) * x(STATES::KAPPA);

  const double dx1_dth0 =
      -0.5 * h * x(STATES::V) * sinth - 0.5 * h * c1 * sin(c2);
  const double dy1_dth0 =
      0.5 * h * x(STATES::V) * costh + 0.5 * h * c1 * cos(c2);

  const double dx1_dv0 = 0.5 * h * costh + 0.5 * h * cos(c2) -
                         0.5 * pow(h, 2) * x(STATES::KAPPA) * c1 * sin(c2);
  const double dy1_dv0 = 0.5 * h * sinth + 0.5 * h * sin(c2) +
                         0.5 * pow(h, 2) * x(STATES::KAPPA) * c1 * cos(c2);
  const double dth1_dv0 =
      h * x(STATES::KAPPA) + 0.5 * pow(h, 2) * u(INPUTS::XI);

  const double dx1_da0 = 0.5 * pow(h, 2) * cos(c2);
  const double dy1_da0 = 0.5 * pow(h, 2) * sin(c2);
  const double dth1_da0 =
      0.5 * pow(h, 2) * (x(STATES::KAPPA) + h * u(INPUTS::XI));

  const double dx1_dkappa0 = -0.5 * pow(h, 2) * x(STATES::V) * c1 * sin(c2);
  const double dy1_dkappa0 = 0.5 * pow(h, 2) * x(STATES::V) * c1 * cos(c2);
  const double dth1_dkappa0 = h * x(STATES::V) + 0.5 * pow(h, 2) * x(STATES::A);

  // dfdx_out << 1, 0, dx1_dth0, dx1_dv0, dx1_da0, dx1_dkappa0, 0, 1, dy1_dth0,
  //     dy1_dv0, dy1_da0, dy1_dkappa0, 0, 0, 1, dth1_dv0, dth1_da0,
  //     dth1_dkappa0, 0, 0, 0, 1, h, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  // std::cout << "dfdx_out1\n" << dfdx_out << std::endl;

  dfdx_out.setConstant(STATES::STATES_SIZE, STATES::STATES_SIZE, 0.0);
  dfdx_out(STATES::X, STATES::X) = 1.0;
  dfdx_out(STATES::X, STATES::THETA) = dx1_dth0;
  dfdx_out(STATES::X, STATES::V) = dx1_dv0;
  dfdx_out(STATES::X, STATES::A) = dx1_da0;
  dfdx_out(STATES::X, STATES::KAPPA) = dx1_dkappa0;

  dfdx_out(STATES::Y, STATES::Y) = 1.0;
  dfdx_out(STATES::Y, STATES::THETA) = dy1_dth0;
  dfdx_out(STATES::Y, STATES::V) = dy1_dv0;
  dfdx_out(STATES::Y, STATES::A) = dy1_da0;
  dfdx_out(STATES::Y, STATES::KAPPA) = dy1_dkappa0;

  dfdx_out(STATES::THETA, STATES::THETA) = 1.0;
  dfdx_out(STATES::THETA, STATES::V) = dth1_dv0;
  dfdx_out(STATES::THETA, STATES::A) = dth1_da0;
  dfdx_out(STATES::THETA, STATES::KAPPA) = dth1_dkappa0;

  dfdx_out(STATES::V, STATES::V) = 1.0;
  dfdx_out(STATES::V, STATES::A) = h;

  dfdx_out(STATES::A, STATES::A) = 1.0;

  dfdx_out(STATES::KAPPA, STATES::KAPPA) = 1.0;
  // std::cout << "dfdx_out2\n" << dfdx_out << std::endl;
}

void TrajectorySmootherNLOpt::model_dfdu(const Vector6d& x,
                                         const Eigen::Vector2d& u,
                                         const double h,
                                         Eigen::MatrixXd& dfdxi_out) const {
  dfdxi_out.setConstant(6, 2, 0.0);
  dfdxi_out(STATES::V, INPUTS::J) = 0.5 * pow(h, 2);
  dfdxi_out(STATES::A, INPUTS::J) = h;
  dfdxi_out(STATES::THETA, INPUTS::XI) =
      0.5 * pow(h, 2) * x(STATES::V) + 0.5 * pow(h, 3) * x(STATES::A);
  dfdxi_out(STATES::KAPPA, INPUTS::XI) = h;
}

void TrajectorySmootherNLOpt::CalculateCommonDataIfNecessary(
    const Eigen::VectorXd& u) {
  if (last_u_.size() != u.size() || u != last_u_ || last_u_.isZero()) {
    last_u_ = u;
    IntegrateModel(x0_, u, nr_integration_steps_, stepsize_, X_, dXdU_);
  }
}

bool TrajectorySmootherNLOpt::CheckConstraints(const std::vector<double>& u,
                                               const Eigen::VectorXd& X) const {
  const int size_state_vector = X.rows();
  for (int idx = 0; idx < size_state_vector / STATES::STATES_SIZE; ++idx) {
    double kappa = X[idx * STATES::STATES_SIZE + STATES::KAPPA];
    double v = X[idx * STATES::STATES_SIZE + STATES::V];
    double a = X[idx * STATES::STATES_SIZE + STATES::A];
    double j = u[idx * INPUTS::INPUTS_SIZE + INPUTS::J];
    double xi = u[idx * INPUTS::INPUTS_SIZE + INPUTS::XI];
    // evaluate acc and kappa only at idx>0, as that's what the optimizer can
    // influence
    if ((idx > 0) && !IsCurvatureWithinBounds(kappa)) {
      AERROR << "solution exceeds bounds at idx " << idx;
      return false;
      // } else if ((idx > 0) && !IsAccelerationWithinBounds(a)) {
      //   AERROR << "solution exceeds bounds at idx " << idx;
      //   return false;
    } else if (!IsJerkWithinBounds(j)) {
      AERROR << "solution exceeds bounds at idx " << idx;
      return false;
    } else if (!IsCurvatureChangeWithinBounds(xi)) {
      AERROR << "solution exceeds bounds at idx " << idx;
      return false;
    } else if (!IsVelocityWithinBounds(v)) {
      AERROR << "solution exceeds bounds at idx " << idx;
      return false;
    }
  }
  return true;
}

bool TrajectorySmootherNLOpt::ValidateSmoothingSolution() const {
  bool valid = true;
  if (status_ < 0 || status_ > 6) {
    AERROR << "Smoothing solution invalid due to solver's return value = "
           << status_;
    valid = false;
  } else if (j_opt_ > j_threshold_) {
    AERROR << "Smoothing solution invalid due to j_opt_ = " << j_opt_
           << " > j_max = " << j_threshold_;
    valid = false;
  } else if (numevals_ < 2) {
    AERROR << "Smoothing solution invalid due to num_eval < num_eval_min";
    valid = false;
  } else if (CheckConstraints(u_, X_) == false) {
    valid = false;
  }
  if (!valid) {
    const std::string& time_pid_string = GetTimeString();
    const std::string& file_name = "trajectory_smoother_" + time_pid_string +
                                   "_" + std::to_string(subsampling_) +
                                   ".pb.txt";
    SaveDiscretizedTrajectoryToFile(modified_input_trajectory_, logdir_,
                                    file_name);
  }
  return valid;
}

double TrajectorySmootherNLOpt::BoundedJerk(const double val) const {
  return BoundValue(val, params_.upper_bound_jerk, params_.lower_bound_jerk,
                    params_.tol_jerk);
}

bool TrajectorySmootherNLOpt::IsJerkWithinBounds(const double j) const {
  if (std::isgreater(j, params_.upper_bound_jerk + params_.tol_jerk) ||
      std::isless(j, params_.lower_bound_jerk - params_.tol_jerk)) {
    AERROR << "solution.jerk = " << j << " exceeds bounds";
    return false;
  } else {
    return true;
  }
}

double TrajectorySmootherNLOpt::BoundedCurvatureChange(const double val) const {
  return BoundValue(val, params_.upper_bound_curvature_change,
                    params_.lower_bound_curvature_change,
                    params_.tol_curvature_change);
}

bool TrajectorySmootherNLOpt::IsCurvatureChangeWithinBounds(
    const double xi) const {
  if (std::isgreater(xi, params_.upper_bound_curvature_change +
                             params_.tol_curvature_change) ||
      std::isless(xi, params_.lower_bound_curvature_change -
                          params_.tol_curvature_change)) {
    AERROR << "solution.curvature_change = " << xi << " exceeds bounds";
    return false;
  } else {
    return true;
  }
}

double TrajectorySmootherNLOpt::BoundedAcceleration(const double val) const {
  return BoundValue(val, params_.upper_bound_acceleration,
                    params_.lower_bound_acceleration, params_.tol_acceleration);
}

bool TrajectorySmootherNLOpt::IsAccelerationWithinBounds(const double a) const {
  if (std::isgreater(
          a, params_.upper_bound_acceleration + params_.tol_acceleration) ||
      std::isless(
          a, params_.lower_bound_acceleration - params_.tol_acceleration)) {
    AERROR << "solution.acceleration = " << a << " exceeds bounds";
    return false;
  } else {
    return true;
  }
}

double TrajectorySmootherNLOpt::BoundedCurvature(const double val) const {
  return BoundValue(val, params_.upper_bound_curvature,
                    params_.lower_bound_curvature, params_.tol_curvature);
}

bool TrajectorySmootherNLOpt::IsCurvatureWithinBounds(
    const double kappa) const {
  if (std::isgreater(kappa,
                     params_.upper_bound_curvature + params_.tol_curvature) ||
      std::isless(kappa,
                  params_.lower_bound_curvature - params_.tol_curvature)) {
    AERROR << "solution.kappa = " << kappa << " exceeds bounds";
    return false;
  } else {
    return true;
  }
}

double TrajectorySmootherNLOpt::BoundedVelocity(const double val) const {
  return BoundValue(val, params_.upper_bound_velocity,
                    params_.lower_bound_velocity, params_.tol_velocity);
}

bool TrajectorySmootherNLOpt::IsVelocityWithinBounds(const double vel) const {
  if (std::isgreater(vel,
                     params_.upper_bound_velocity + params_.tol_velocity) ||
      std::isless(vel, params_.lower_bound_velocity - params_.tol_velocity)) {
    AERROR << "solution.velocity = " << vel << " exceeds bounds";
    return false;
  } else {
    return true;
  }
}

void TrajectorySmootherNLOpt::CalculateJthreshold() {
  j_threshold_ = 0;
  // costs for deviation from the initial reference
  j_threshold_ +=
      input_traj_size_ * params_.cost_offset_x * 3.0;  // 3.0m deviation
  j_threshold_ +=
      input_traj_size_ * params_.cost_offset_y * 3.0;  // 3.0m deviation
  j_threshold_ += input_traj_size_ * params_.cost_offset_theta * 0.2;
  j_threshold_ +=
      input_traj_size_ * params_.cost_offset_v * 3.0;  // 3.0m/s deviation

  // costs on absolute values
  j_threshold_ += nr_integration_steps_ * params_.cost_curvature *
                  params_.upper_bound_curvature;
  j_threshold_ += nr_integration_steps_ * params_.cost_acceleration *
                  params_.upper_bound_acceleration;
  // costs on input
  j_threshold_ += nr_integration_steps_ * params_.cost_curvature_change *
                  params_.upper_bound_curvature_change;
  j_threshold_ += nr_integration_steps_ * params_.cost_acceleration_change *
                  params_.upper_bound_jerk;
  AINFO << "j_threshold is set to " << j_threshold_;
}

bool TrajectorySmootherNLOpt::CheckBoundsAfterIntegration(double jerk,
                                                          double dkappa,
                                                          size_t steps) const {
  const Eigen::Vector2d u_eigen = Eigen::Vector2d(jerk, dkappa);
  const std::vector<double> u_vector = {jerk, dkappa};
  Vector6d x_in = x0_;
  for (int idx = 0; idx < steps; ++idx) {
    Vector6d x_out;
    model_f(x_in, u_eigen, stepsize_, x_out);
    x_in = x_out;
    bool in_bounds = CheckConstraints(u_vector, x_out);
    if (!in_bounds) {
      return false;
    }
  }
  return true;
}

void SaveDiscretizedTrajectoryToFile(
    const apollo::planning::DiscretizedTrajectory& traj,
    const std::string& path_to_file, const std::string& file_name) {
  apollo::planning::PublishableTrajectory publishable_trajectory(0.0, traj);
  apollo::planning::ADCTrajectory traj_pb;
  publishable_trajectory.PopulateTrajectoryProtobuf(&traj_pb);

  // This will dump the optimized trajectory. Analysis is possible using the
  // matlab script analyze_smoother.m
  const std::string txt_file = path_to_file + "/" + file_name;
  apollo::cyber::common::SetProtoToASCIIFile(traj_pb, txt_file);
}

double BoundValue(const double v, const double vmax, const double vmin,
                  const double tol) {
  return std::max(std::min(v, vmax - tol), vmin + tol);
}

double InterpolateWithinBounds(int idx0, double v0, int idx1, double v1,
                               int idx) {
  if (idx < idx0) {
    return v0;
  } else if (idx > idx1) {
    return v1;
  } else {
    double m = (v0 - v1) / (idx0 - idx1);
    double n = v1 - m * idx1;
    return m * idx + n;
  }
}

double Round(double a, size_t p) { return round(a * pow(10, p)) / pow(10, p); }

}  // namespace planning
}  // namespace apollo
