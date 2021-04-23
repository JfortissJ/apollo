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

#include <nlopt.h>

#include "cyber/common/log.h"

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

namespace apollo {
namespace planning {

TrajectorySmootherNLOpt::TrajectorySmootherNLOpt() {
  // TODO Set costs

  x0_.resize(STATES::STATES_SIZE);

  num_ineq_constr_ = 0;
  num_eq_constr_ = 0;
}

void TrajectorySmootherNLOpt::InitializeProblem(
    const int subsampling, const DiscretizedTrajectory& input_trajectory,
    double initial_steering) {
  // set sizes
  const int input_traj_size = input_trajectory.size();

  problem_size_ = 2;  // input_traj_size*subsampling*INPUTS::SIZE_INPUTS;
  u_.resize(problem_size_);

  // set x0 using the reference traj
  

  // set reference from input
  // set u0: start values for the optimizer

  // set lower bound vector
  // set upper bound vector

  status_ = 0;
}

int TrajectorySmootherNLOpt::Optimize() {
  // Initialize the optimization problem
  nlopt::opt opt(solver_params_.algorithm, problem_size_);

  // Options
  opt.set_xtol_rel(solver_params_.x_tol_rel);
  opt.set_xtol_abs(solver_params_.x_tol_abs);
  opt.set_maxeval(solver_params_.max_num_evals);

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
    ineq_constraint_tol_.clear();
    ineq_constraint_tol_.resize(num_ineq_constr_,
                                solver_params_.ineq_const_tol);
    opt.add_inequality_mconstraint(nlopt_inequality_constraint_wrapper, this,
                                   ineq_constraint_tol_);
  }

  if (num_eq_constr_ > 0) {
    eq_constraint_tol_.clear();
    eq_constraint_tol_.resize(num_eq_constr_, solver_params_.eq_const_tol);
    opt.add_equality_mconstraint(nlopt_equality_constraint_wrapper, this,
                                 eq_constraint_tol_);
  }

  // Optimization
  try {
    status_ = static_cast<int>(opt.optimize(u_, j_opt_));
  } catch (nlopt::roundoff_limited ex) {
    AWARN << "Roundoff limited exception:" << ex.what();
  } catch (std::exception ex) {
    AERROR << "Unhandled Exception while optimization: " << ex.what();
    status_ = -11;
    return status_;
  }

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

double TrajectorySmootherNLOpt::ObjectiveFunction(unsigned n, const double* x,
                                                  double* grad) {
  if (grad) {
    grad[0] = 0.0;
    grad[1] = 0.5 / sqrt(x[1]);
  }
  return sqrt(x[1]);
}

void TrajectorySmootherNLOpt::InequalityConstraintFunction(
    unsigned m, double* result, unsigned n, const double* x, double* grad) {}

void TrajectorySmootherNLOpt::EqualityConstraintFunction(
    unsigned m, double* result, unsigned n, const double* x, double* grad) {}

void TrajectorySmootherNLOpt::IntegrateModel(const Eigen::VectorXd& x0,
                                             const Eigen::VectorXd& u,
                                             const size_t num_integration_steps,
                                             Eigen::VectorXd& X,
                                             Eigen::MatrixXd& dXdU) {

	const size_t dimX = 6;
	const size_t dimU = 1; // change to 2?
	const size_t N = num_integration_steps;

	X.resize(dimX*N);
	X.block<6,1>(0,0) = x0;

	A_.resize(dimX*N, dimX);
	A_.fill(0.0f);
	A_.block<6,6>(0,0) = Matrix3d::Identity(); // TODO: change Matrix3d

	dXdU.resize(dimX*N, dimU*N + 1);
	dXdU.fill(0.0f);

	
	size_t row_idx, row_idx_before, u_idx;

	for(size_t i = 1; i < N ; ++i)
	{
		row_idx = i*dimX;
		row_idx_before = (i-1)*dimX;

		model_f(X.block<6,1>(row_idx_before,0),u(i-1),h,currx_);
		model_dfdx(X.block<6,1>(row_idx_before,0),u(i-1),h,currA_);
		model_dfdkappa(X.block<6,1>(row_idx_before,0),u(i-1),h,currB_);
		model_dfdh(X.block<6,1>(row_idx_before,0),u(i-1),h,currH_);

		X.block<6,1>(row_idx,0) = currx_;
		A_.block<6,6>(row_idx, 0) = currA_ * A_.block<6,6>(row_idx_before,0);
		dXdU.block<6,1>(row_idx,(i-1)*dimU) = currB_;
		dXdU.block<6,1>(row_idx, dimU*N) = currA_ * dXdU.block<6,1>(row_idx_before, dimU*N) + currH_;

		for (size_t n = 1; n < i; ++n)
		{
			u_idx = (n-1)*dimU;
			dXdU.block<6,1>(row_idx,u_idx) = currA_ * dXdU.block<6,1>(row_idx_before,u_idx);
		}
	}
}


void PathOptimizerNLOpt::model_f(const Vector6d & x, const double u, const double h, Vector6d & x_out)
{
  const double sinth = sin(x(STATES::THETA));
  const double costh = cos(x(STATES::THETA));
  const double c1 = x(STATES::V)+h*x(STATES::A);
  const double c2 = x(STATES::THETA) + h*x(STATES::V)*x(STATES::KAPPA);
  const double c3 = x(STATES::KAPPA) + h*u(INPUTS::XI);
  const double c4 = x(STATES::A) + h*u(INPUTS::J);
	
  const double x1 = x(STATES::X) + 0.5*h*x(STATES::V)*costh + 0.5*h*c1*cos(c2);
  const double y1 = x(STATES::Y) + 0.5*h*x(STATES::V)*sinth + 0.5*h*c1*sin(c2);
  const double theta1 = x(STATES::THETA) + 0.5*h*x(STATES::V)*x(STATES::KAPPA) + 0.5*h*c1*c3;
  const double v1 = x(STATES::V) + 0.5*h*x(STATES::A) + 0.5*c4;
  const double a1 = c4;
  const double kappa1 = c3;
	x_out << x1, y1, theta1, v1, a1, kappa1;
}

void PathOptimizerNLOpt::model_dfdx(const Vector6d & x, const double u, const double h, Matrix6d& dfdx_out)
{
  const double sinth = sin(x(STATES::THETA));
  const double costh = cos(x(STATES::THETA));
  const double c1 = x(STATES::V)+h*x(STATES::A);
  const double c2 = x(STATES::THETA) + h*x(STATES::V)*x(STATES::KAPPA);

  const double dx1_dth0 = -0.5*h*x(STATES::V)*sinth - 0.5*c1*sin(c2);
  const double dy1_dth0 = 0.5*h*x(STATES::V)*costh + 0.5*h*c1*cos(c2);

  const double dx1_dv0 = 0.5*h*costh + 0.5*h*cos(c2) - 0.5*h*c1*sin(c2);
  const double dy1_dv0 = 0.5*h*sinth + 0.5*h*sin(c2) + 0.5*h*c1*cos(c2);
  const double dth1_dv0 = h*x(STATES::KAPPA) + 0.5*pow(h,2)*u(INPUTS::XI);
  
  const double dx1_da0 = 0.5*pow(h,2)*cos(c2);
  const double dy1_da0 = 0.5*pow(h,2)*sin(c2);
  const double dth1_da0 = 0.5*pow(h,2)*(x(STATES::KAPPA)+h*u(INPUTS::XI));
  
  const double dx1_dkappa0 = -0.5*pow(h,2)*x(STATES::V)*c1*sin(c2);
  const double dy1_dkappa0 = 0.5*pow(h,2)*x(STATES::V)*c1*cos(c2);
  const double dth1_kappa0 = h*x(STATES::V)+0.5*pow(h,2)*x(INPUTS::A);

  dfdx_out << 1,    0,    dx1_dth0,   dx1_dv0,    dx1_da0,    dx1_dkappa0,
              0,    1,    dy1_dth0,   dy1_dv0,    dy1_da0,    dy1_dkappa0,
              0,    0,    1,          dth1_dv0,   dth1_da0,   dth1_dkappa0,
              0,    0,    0,          1           h,          0,
              0,    0,    0,          0,          1,          0,
              0,    0,    0,          0,          0,          1;
}


void PathOptimizerNLOpt::model_dfdjerk( const Vector6d & x, const double u, const double h, Vector6d& dfdjerk_out )
{
  dfdjerk_out << 0, 0, 0, 0.5*h*h, h, 0;
}

void PathOptimizerNLOpt::model_dfdkappa( const Vector6d & x, const double u, const double h, Vector6d& dfdkappa_out )
{
  dfdh_kappa << 0, 0, 0.5*pow(h,2)*x(STATES::V) + 0.5*pow(h,3)*x(STATES::A), 0, 0, h;
}


}  // namespace planning
}  // namespace apollo
