/*
 * Created on Jul 11, 2017
 * Authour: Artem Artemev, @artemav
 */

#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include <vector>

namespace mpc {

static constexpr size_t num_pred = 20;
static constexpr double rad25 = 0.436332313;

static constexpr double dt = 0.1;
static constexpr double lf = 2.67;

class MPC {
public:
  MPC();
  ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs);

  const std::vector<double> &State() const { return state_; };
  const std::vector<double> &XCoord() const { return x_coord_; };
  const std::vector<double> &YCoord() const { return y_coord_; };

private:
  std::vector<double> state_;
  std::vector<double> x_coord_;
  std::vector<double> y_coord_;
};

enum McpStateIndex : size_t {
  kStateX = 0,
  kStateY = 1,
  kStatePsi = 2,
  kStateVel = 3,
  kStateCte = 4,
  kStateEpsi = 5,
  kStateSize = 6,

  // Extra fields for internal state
  kStateDelta = kStateSize,
  kStateAcc = kStateSize + 1,
  kStateInternalSize = kStateAcc + 1
};

enum VarIndex : size_t {
  kVarX = kStateX * num_pred,
  kVarY = kStateY * num_pred,
  kVarPsi = kStatePsi * num_pred,
  kVarVel = kStateVel * num_pred,
  kVarCte = kStateCte * num_pred,
  kVarEpsi = kStateEpsi * num_pred,
  kVarDelta = kStateSize * num_pred,
  kVarAcc = (kStateSize + 1) * num_pred - 1,
  kVarSize = kStateSize * num_pred + (num_pred - 1) * 2
};

} /* namespace mpc */

#endif /* MPC_H */
