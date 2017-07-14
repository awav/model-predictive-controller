/*
 * Created on Jul 11, 2017
 * Authour: Artem Artemev, @artemav
 */

#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>

namespace mpc {

static constexpr std::size_t num_pred = 10;
static constexpr double rad25 = 0.436332313;

static constexpr double dt = 0.1;
static constexpr double lf = 2.67;

class MPC {
public:
  MPC();
  ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(double x, double y, double psi, double vel, double cte,
             double epsi, Eigen::VectorXd coeffs);

  const std::vector<double> &XCoord() const;
  const std::vector<double> &YCoord() const;
  double Steer() const;
  double Throttle() const;

private:
  using Dvector = CPPAD_TESTVECTOR(double);

  double steer_ = 0;
  double throttle_ = 0;
  std::vector<double> x_coord_;
  std::vector<double> y_coord_;

  Dvector vars_;
  Dvector vars_low_;
  Dvector vars_up_;
  Dvector constraints_low_;
  Dvector constraints_up_;
};

enum McpStateIndex : std::size_t {
  kStateX = 0,
  kStateY = 1,
  kStatePsi = 2,
  kStateVel = 3,
  kStateCte = 4,
  kStateEpsi = 5,
  // Extra fields for internal state with actuators
  kStateDelta = 6,
  kStateAcc = 7,

  kStateSize = kStateEpsi + 1,
  kStateActuatorsSize = 2
};

enum VarIndex : std::size_t {
  kVarX = kStateX,
  kVarY = kVarX + num_pred,
  kVarPsi = kVarY + num_pred,
  kVarVel = kVarPsi + num_pred,
  kVarCte = kVarVel + num_pred,
  kVarEpsi = kVarCte + num_pred,
  kVarDelta = kVarEpsi + num_pred,
  kVarAcc = kVarDelta + num_pred - 1,
  kVarSize = num_pred * kStateSize + (num_pred - 1) * kStateActuatorsSize,
  kVarConstraintsSize = kStateSize * num_pred
};

} /* namespace mpc */

#endif /* MPC_H */
