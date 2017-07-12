#include "MPC.h"
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

enum McpStateIndex : size_t {
  kStateX = 0,
  kStateY = 1,
  kStatePsi = 2,
  kStateVel = 3,
  kStateCte = 4,
  kStateEpsi = 5,
  kStateSize = 6
};

enum VarIndex : size_t {
  kVarX = kStateX * MPC::n,
  kVarY = kStateY * MPC::n,
  kVarPsi = kStatePsi * MPC::n,
  kVarVel = kStateVel * MPC::n,
  kVarCte = kStateCte * MPC::n,
  kVarEpsi = kStateEpsi * MPC::n,
  kVarDelta = kStateSize * MPC::n,
  kVarAcc = (kStateSize + 1) * MPC::n - 1,
  kVarSize = kStateSize * MPC::n + (MPC::n - 1) * 2
};

class MpcEval {
public:
  using ADvector = CPPAD_TESTVECTOR(AD<double>);

  MpcEval(const Eigen::VectorXd &coeffs) { coeffs_ = coeffs; }

  void operator()(ADvector &fg, const ADvector &vars) {
    // TODO: !!!
  }

private:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs_;
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  using Dvector = CPPAD_TESTVECTOR(double);

  bool ok = true;

  const auto x = state[kStateX];
  const auto y = state[kStateY];
  const auto psi = state[kStatePsi];
  const auto vel = state[kStateVel];
  const auto cte = state[kStateCte];
  const auto epsi = state[kStateEpsi];

  const auto n_constraints = kStateSize * n;

  Dvector vars(kVarSize);

  // Lower and Upper Boundaries
  Dvector vars_low(kVarSize);
  Dvector vars_up(kVarSize);
  Dvector constraints_low(n_constraints);
  Dvector constraints_up(n_constraints);

  for (auto i = 0; i < kVarSize; i++) {
    vars[i] = 0;
  }

  for (size_t i = kVarX; i < kVarDelta; ++i) {
    vars_low[i] = -var_bound;
    vars_up[i] = var_bound;
  }

  for (size_t i = kVarDelta; i < kVarAcc; ++i) {
    vars_low[i] = -delta_bound;
    vars_up[i] = delta_bound;
  }

  for (size_t i = kVarAcc; i < kVarSize; ++i) {
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

  // object that computes objective and constraints
  MpcEval mcp_eval(coeffs);

  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, MpcEval>(options, vars, vars_low, vars_up,
                                        constraints_low, constraints_up,
                                        mcp_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}
