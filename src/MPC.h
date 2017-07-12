#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include <vector>

using namespace std;

class MPC {
public:
  static constexpr size_t n = 20;
  static constexpr double dt = 0.8;
  static constexpr double lf = 2.67;
  static constexpr double vel_ref = 50.0;
  static constexpr double cte_ref = 0.0;
  static constexpr double epsi_ref = 0.0;
  static constexpr double var_bound = 1.0e19;
  static constexpr double delta_bound = 0.436332313;
  static constexpr double acc_bound = 1.0;

public:
  MPC();
  ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
