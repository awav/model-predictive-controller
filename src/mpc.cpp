/*
 * Created on Jul 11, 2017
 * Author: Artem Artemev, @artemav
 */

#include "mpc.h"
#include "Eigen-3.3/Eigen/Core"
#include "aux.h"

namespace mpc {

using CppAD::AD;

static constexpr double vel_ref = 80.0;
static constexpr double cte_ref = 0.0;
static constexpr double epsi_ref = 0.0;

static constexpr double var_bound = 1e3;
static constexpr double delta_bound = 0.8; // forces to shrink wide steering angles
static constexpr double acc_bound = 1.0;

static constexpr double cte_weight = 1700.0; // minimizes distance from path lane
static constexpr double epsi_weight = 1700.0;
static constexpr double vel_weight = 1.0;
static constexpr double delta_weight = 30; // causes turn slowly and more stable
static constexpr double acc_weight = 35; // causes slow accelaration if value is big
static constexpr double diff_delta_weight = 145; // affects reaction on turns
static constexpr double diff_acc_weight = 14; // affects turns drastically

class CostEval {
public:
  using ADvector = CPPAD_TESTVECTOR(AD<double>);

  CostEval(const Eigen::VectorXd &coeffs) : coeffs_(coeffs) {}

  void operator()(ADvector &fg, const ADvector &vars) {
    fg[0] = 0;

    // Reference state minimization

    for (std::size_t i = 0; i < num_pred; ++i) {
      const auto vel = vars[kVarVel + i] - vel_ref;
      const auto cte = vars[kVarCte + i] - cte_ref;
      const auto epsi = vars[kVarEpsi + i] - epsi_ref;

      fg[0] += cte_weight * CppAD::pow(cte, 2);
      fg[0] += vel_weight * CppAD::pow(vel, 2);
      fg[0] += epsi_weight * CppAD::pow(epsi, 2);
    }

    // Actuators minimization
    for (std::size_t i = 0; i < num_pred - 1; ++i) {
      fg[0] += delta_weight * CppAD::pow(vars[kVarDelta + i], 2);
      fg[0] += acc_weight * CppAD::pow(vars[kVarAcc + i], 2);
    }

    // Gap minimization between sequential actuations
    for (std::size_t i = 0; i < num_pred - 2; ++i) {
      auto delta = vars[kVarDelta + i + 1] - vars[kVarDelta + i];
      auto acc = vars[kVarAcc + i + 1] - vars[kVarAcc + i];
      fg[0] += diff_delta_weight * CppAD::pow(delta, 2);
      fg[0] += diff_acc_weight * CppAD::pow(acc, 2);
    }

    fg[kVarX + 1] = vars[kVarX];
    fg[kVarY + 1] = vars[kVarY];
    fg[kVarPsi + 1] = vars[kVarPsi];
    fg[kVarVel + 1] = vars[kVarVel];
    fg[kVarCte + 1] = vars[kVarCte];
    fg[kVarEpsi + 1] = vars[kVarEpsi];

    for (std::size_t i = 0; i < num_pred - 1; ++i) {
      // The state at time t.
      const auto x0 = vars[kVarX + i];
      const auto y0 = vars[kVarY + i];
      const auto psi0 = vars[kVarPsi + i];
      const auto vel0 = vars[kVarVel + i];
      const auto cte0 = vars[kVarCte + i];
      const auto epsi0 = vars[kVarEpsi + i];
      // Only consider the actuation at time t.

      const auto k = i > 0 ? i - 1 : i;
      const auto delta0 = vars[kVarDelta + k];
      const auto acc0 = vars[kVarAcc + k];

      // The state at time t+1 .
      const auto j = i + 1;
      const auto x1 = vars[kVarX + j];
      const auto y1 = vars[kVarY + j];
      const auto psi1 = vars[kVarPsi + j];
      const auto vel1 = vars[kVarVel + j];
      const auto cte1 = vars[kVarCte + j];
      const auto epsi1 = vars[kVarEpsi + j];

      const auto f_aim = coeffs_[3] * CppAD::pow(x0, 3) +
                         coeffs_[2] * CppAD::pow(x0, 2) +
                         coeffs_[1] * x0 +
                         coeffs_[0];

      const auto psi_aim = CppAD::atan(coeffs_[3] * CppAD::pow(x0, 2) * 3.0 +
                                       coeffs_[2] * x0 * 2.0 +
                                       coeffs_[1]);

      const auto x_dest = x0 + vel0 * CppAD::cos(psi0) * dt;
      const auto y_dest = y0 + vel0 * CppAD::sin(psi0) * dt;
      const auto psi_dest = psi0 + vel0 * (-delta0) / lf * dt;
      const auto vel_dest = vel0 + acc0 * dt;
      const auto cte_dest = f_aim - y0 + vel0 * CppAD::sin(epsi0) * dt;
      const auto epsi_dest = psi0 - psi_aim + vel0 * (-delta0) / lf * dt;

      const auto l = i + 2;
      fg[kVarX + l] = x1 - x_dest;
      fg[kVarY + l] = y1 - y_dest;
      fg[kVarPsi + l] = psi1 - psi_dest;
      fg[kVarVel + l] = vel1 - vel_dest;
      fg[kVarCte + l] = cte1 - cte_dest;
      fg[kVarEpsi + l] = epsi1 - epsi_dest;
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
  x_coord_ = std::vector<double>(num_pred);
  y_coord_ = std::vector<double>(num_pred);

  vars_ = Dvector(kVarSize);
  vars_low_ = Dvector(kVarSize);
  vars_up_ = Dvector(kVarSize);
  constraints_low_ = Dvector(kVarConstraintsSize);
  constraints_up_ = Dvector(kVarConstraintsSize);

  for (std::size_t i = 0; i < kVarSize; i++) {
    vars_[i] = 0.0;
  }

  for (std::size_t i = kVarX; i < kVarDelta; ++i) {
    vars_low_[i] = -var_bound;
    vars_up_[i] = var_bound;
  }

  for (std::size_t i = kVarDelta; i < kVarAcc; ++i) {
    vars_low_[i] = -delta_bound;
    vars_up_[i] = delta_bound;
  }

  for (std::size_t i = kVarAcc; i < kVarSize; ++i) {
    vars_low_[i] = -acc_bound;
    vars_up_[i] = acc_bound;
  }

  for (std::size_t i = 0; i < kVarConstraintsSize; i++) {
    constraints_low_[i] = 0;
    constraints_up_[i] = 0;
  }
}

MPC::~MPC() {}

void MPC::Solve(double x, double y, double psi, double vel, double cte,
                double epsi, const Eigen::VectorXd &coeffs) {
  vars_[kVarX] = x;
  vars_[kVarY] = y;
  vars_[kVarPsi] = psi;
  vars_[kVarVel] = vel;
  vars_[kVarCte] = cte;
  vars_[kVarEpsi] = epsi;

  constraints_up_[kVarX] = x;
  constraints_up_[kVarY] = y;
  constraints_up_[kVarPsi] = psi;
  constraints_up_[kVarVel] = vel;
  constraints_up_[kVarCte] = cte;
  constraints_up_[kVarEpsi] = epsi;

  constraints_low_[kVarX] = x;
  constraints_low_[kVarY] = y;
  constraints_low_[kVarPsi] = psi;
  constraints_low_[kVarVel] = vel;
  constraints_low_[kVarCte] = cte;
  constraints_low_[kVarEpsi] = epsi;

  std::string options ;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // object that computes objective and constraints
  CostEval mcp_eval(coeffs);

  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, CostEval>(options, vars_, vars_low_, vars_up_,
                                        constraints_low_, constraints_up_,
                                        mcp_eval, solution);

  AUX_DEBUG("Cost: ", solution.obj_value);

  // Check some of the solution values
  if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
    AUX_DEBUG("Solution status is not successful, status [", solution.status,
              "]");
  }

  steer_ = solution.x[kVarDelta];
  throttle_ = solution.x[kVarAcc];

  for (std::size_t i = 0; i < num_pred; ++i) {
    x_coord_[i] = solution.x[kVarX + i];
    y_coord_[i] = solution.x[kVarY + i];
  }
}

const std::vector<double> &MPC::XCoord() const { return x_coord_; }

const std::vector<double> &MPC::YCoord() const { return y_coord_; }

double MPC::Throttle() const { return throttle_; }

double MPC::Steer() const { return steer_; }

} /* namespace mpc */
