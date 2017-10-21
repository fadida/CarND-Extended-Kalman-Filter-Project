#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  
  
  if (!estimations.size()) {
    cout << "CalculateRMSE Error - estimation size is zero." << endl;
    return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
    cout << "CalculateRMSE Error - estimation size does not match ground_truth size." << endl;
    return rmse;
  }
  
  //accumulate squared residuals
  for (int i = 0; i < estimations.size(); i++) {
    // Calculate residuals
    VectorXd residual = estimations[i] - ground_truth[i];
    // Square residuals coeffs
    VectorXd squared = residual.array().square();
    rmse = rmse + squared;
  }
  
  //calculate the mean
  rmse = rmse / estimations.size();
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3, 4);

  Hj << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;

  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  
  // denominator_1 = px^2 + px^2
  double denominator_1 = px * px + py * py;
  // denominator_2 = (px^2 + px^2)^0.5
  double denominator_2 = sqrt(denominator_1);
  // denominator_3 = (px^2 + px^2)^3/2
  double denominator_3 = denominator_1 * denominator_2;
  
  if (fabs(denominator_1) < 0.0001) {
    cout << "CalculateJacobian() ERROR - Divide by zero error." << endl;
    return Hj;
  }
  
  Hj(0, 0) = px / denominator_2;
  Hj(0, 1) = py / denominator_2;
  
  Hj(1, 0) = -py / denominator_1;
  Hj(1, 1) = px / denominator_1;
  
  Hj(2, 0) = py * (vx * py - vy * px) / denominator_3;
  Hj(2, 1) = px * (vy * px - vx * py) / denominator_3;
  Hj(2, 2) = px / denominator_2;
  Hj(2, 3) = py / denominator_2;
  
  return Hj;
}
