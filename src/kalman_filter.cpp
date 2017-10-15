#include "kalman_filter.h"
#include <iostream>

#define PI 3.14159265
#define DEBUG_PRINT
//#define DEBUG_PRINT std::cout << __FILE__ << ":" <<  __LINE__ << std::endl;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /*
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred = VectorXd(3);
  CalculateExtPredVec(z_pred);
  VectorXd y = z - z_pred;
  // fix angle
  float y_theta = y(1);
  if (y_theta < -1 * PI) {
    y(1) = y_theta + 2* PI;
  } else if (y_theta >  PI) {
    y(1) = y_theta - 2* PI;
  }
  std::cout << "y=" << y << std::endl;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  VectorXd d_x = K * y;
  if (fabs(d_x(0)) == NAN)
  {
    std::cout << "d_x is too big. d_x=" << d_x << std::endl;
    return;
  }
  x_ = x_ + d_x;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::CalculateExtPredVec(VectorXd &z_pred) {
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float ro = sqrt(px*px + py*py);
  if (ro < 1e-3) {
    std::cout << "zero ro value for z_pred. skipping calc" << std::endl;
    return;
  }
  float phi = atan2(py, px);
  float ro_dot = (px*vx + py*vy) / ro;

  z_pred << ro,
            phi,
            ro_dot;
}