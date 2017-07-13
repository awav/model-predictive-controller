/*
 * Created on Jul 11, 2017
 * Authour: Artem Artemev, @artemav
 */

#include "mpc.h"
#include "Eigen-3.3/Eigen/Core"
#include "aux.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

namespace mpc {

using CppAD::AD;

static constexpr double vel_ref = 75.0;
static constexpr double cte_ref = 0.0;
static constexpr double epsi_ref = 0.0;

static constexpr double var_bound = 1.0e6;
static constexpr double delta_bound = rad25;
static constexpr double acc_bound = 1.0;

static constexpr double delta_weight = 1000.0;
static constexpr double cte_weight = 9.0;
static constexpr double acc_weight = 20;
static constexpr double vel_weight = 0.5;
static constexpr double epsi_weight = 0.5;
static constexpr double diff_delta_weight = 0.1;
static constexpr double diff_acc_weight = 0.001;

class MpcEval {
public:
  using ADvector = CPPAD_TESTVECTOR(AD<double>);

  MpcEval(const Eigen::VectorXd &coeffs) { coeffs_ = coeffs; }

  void operator()(ADvector &fg, const ADvector &vars) {
    fg[0] = 0;

    // Reference state minimization
    for (int i = 0; i < num_pred; ++i) {
      fg[0] += cte_weight * CppAD::pow(vars[kVarCte + i] - cte_ref, 2);
      fg[0] += vel_weight * CppAD::pow(vars[kVarVel + i] - vel_ref, 2);
      fg[0] += epsi_weight * CppAD::pow(vars[kVarEpsi + i] - epsi_ref, 2);
    }

    // Actuators minimization
    for (int i = 0; i < num_pred - 1; ++i) {
      fg[0] += delta_weight * CppAD::pow(vars[kVarDelta + i], 2);
      fg[0] += acc_weight * CppAD::pow(vars[kVarAcc + i], 2);
    }

    // Gap minimization between sequential actuations
    for (int i = 0; i < num_pred - 2; ++i) {
      auto deltasq =
          CppAD::pow(vars[kVarDelta + i + 1] - vars[kVarDelta + i], 2);
      auto accsq = CppAD::pow(vars[kVarDelta + i + 1] - vars[kVarDelta + i], 2);
      fg[0] += diff_delta_weight * deltasq;
      fg[0] += diff_acc_weight * accsq;
    }

    fg[kVarX + 1] = vars[kVarX];
    fg[kVarY + 1] = vars[kVarY];
    fg[kVarPsi + 1] = vars[kVarPsi];
    fg[kVarVel + 1] = vars[kVarVel];
    fg[kVarCte + 1] = vars[kVarCte];
    fg[kVarEpsi + 1] = vars[kVarEpsi];

    for (int i = 0; i < num_pred - 1; ++i) {
      // The state at time t+1 .
      auto x1 = vars[kVarX + i + 1];
      auto y1 = vars[kVarY + i + 1];
      auto psi1 = vars[kVarPsi + i + 1];
      auto v1 = vars[kVarVel + i + 1];
      auto cte1 = vars[kVarCte + i + 1];
      auto epsi1 = vars[kVarPsi + i + 1];

      // The state at time t.
      auto x0 = vars[kVarX + i];
      auto y0 = vars[kVarY + i];
      auto psi0 = vars[kVarPsi + i];
      auto v0 = vars[kVarVel + i];
      auto cte0 = vars[kVarCte + i];
      auto epsi0 = vars[kVarEpsi + i];

      // Only consider the actuation at time t.
      auto delta0 = vars[kVarDelta + i];
      auto a0 = vars[kVarAcc + i];

      AD<double> f0 = 0.0;
      for (int i = 0; i < coeffs_.size(); i++) {
        f0 += coeffs_[i] * CppAD::pow(x0, i);
      }

      AD<double> psides0 = 0.0;
      for (int i = 1; i < coeffs_.size(); i++) {
        psides0 += i * coeffs_[i] * CppAD::pow(x0, i - 1);
      }

      psides0 = CppAD::atan(psides0);

      fg[2 + kVarX + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + kVarY + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + kVarPsi + i] = psi1 - (psi0 + v0 * delta0 / lf * dt);
      fg[2 + kVarVel + i] = v1 - (v0 + a0 * dt);
      fg[2 + kVarCte + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + kVarEpsi + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / lf * dt);
    }
  }

private:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs_;
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  state_ = std::vector<double>(kStateInternalSize);
  x_coord_ = std::vector<double>(num_);
}

MPC::~MPC() {}

void MPC::Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs) {
  using Dvector = CPPAD_TESTVECTOR(double);

  const auto x = state[kStateX];
  const auto y = state[kStateY];
  const auto psi = state[kStatePsi];
  const auto vel = state[kStateVel];
  const auto cte = state[kStateCte];
  const auto epsi = state[kStateEpsi];

  const auto n_constraints = kStateSize * num_pred;

  Dvector vars(kVarSize);

  // Lower and Upper Boundaries
  Dvector vars_low(kVarSize);
  Dvector vars_up(kVarSize);
  Dvector constraints_low(n_constraints);
  Dvector constraints_up(n_constraints);

  for (auto i = 0; i < kVarSize; i++) {
    vars[i] = 0;
  }

  for (int i = kVarX; i < kVarDelta; ++i) {
    vars_low[i] = -var_bound;
    vars_up[i] = var_bound;
  }

  for (int i = kVarDelta; i < kVarAcc; ++i) {
    vars_low[i] = -delta_bound;
    vars_up[i] = delta_bound;
  }

  for (int i = kVarAcc; i < kVarSize; ++i) {
    vars_low[i] = -acc_bound;
    vars_up[i] = acc_bound;
  }

  for (int i = 0; i < n_constraints; i++) {
    constraints_low[i] = 0;
    constraints_up[i] = 0;
  }

  constraints_up[kVarX] = constraints_low[kVarX] = x;
  constraints_up[kVarY] = constraints_low[kVarY] = y;
  constraints_up[kVarPsi] = constraints_low[kVarPsi] = psi;
  constraints_up[kVarVel] = constraints_low[kVarVel] = vel;
  constraints_up[kVarCte] = constraints_low[kVarCte] = cte;
  constraints_up[kVarEpsi] = constraints_low[kVarEpsi] = epsi;

  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // object that computes objective and constraints
  MpcEval mcp_eval(coeffs);

  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, MpcEval>(options, vars, vars_low, vars_up,
                                        constraints_low, constraints_up,
                                        mcp_eval, solution);


  AUX_DEBUG("Cost: ", solution.obj_value);
  AUX_DEBUG("Status: ", solution.status);
  AUX_DEBUG("Success index: ", CppAD::ipopt::solve_result<Dvector>::success);
  AUX_DEBUG("Solution size: ", solution.x.size());

  // Check some of the solution values
  if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
    AUX_DEBUG("Solution status is not successful\n");
    return;
  }

  state_[kStateX] = solution.x[kStateX];
  AUX_DEBUG("1");
  state_[kStateY] = solution.x[kStateY];
  AUX_DEBUG("2");
  state_[kStatePsi] = solution.x[kStatePsi];
  AUX_DEBUG("3");
  state_[kStateVel] = solution.x[kStateVel];
  AUX_DEBUG("4");
  state_[kStateCte] = solution.x[kStateCte];
  AUX_DEBUG("5");
  state_[kStateEpsi] = solution.x[kStateEpsi];
  AUX_DEBUG("6");

  for (int i = 0; i < num_pred; ++i) {
    x_coord_[i] = solution.x[kStateX + i];
    y_coord_[i] = solution.x[kStateY + i];
  }
  AUX_DEBUG("7");
}

} /* namespace mpc */
