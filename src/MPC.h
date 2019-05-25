#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC() {};

  virtual ~MPC() {};

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};


// For converting back and forth between radians and degrees.
constexpr double pi(); //pai
double deg2rad(double x); //角度转弧度
double rad2deg(double x); //弧度转角读

#endif /* MPC_H */
