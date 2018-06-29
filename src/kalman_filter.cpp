#include "kalman_filter.h"
#include <math.h>
#include <iostream>
#define  PI 3.1415926 

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  std::cout<<"Finish Prediction!"<<std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd K = PHt * S.inverse();

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_ ) * P_;

  std::cout<<"Finish Laser!"<<std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float b = sqrt(px * px + py * py);
  float rho_dot;
  if(fabs(b) < 0.0001){
    rho_dot = (px * vx + py * vy) / 0.0001;
  }
  else {
    rho_dot = (px * vx + py * vy)/b;
  }

  VectorXd hx;
  hx = VectorXd(3);
  hx << b, atan2(py, px), rho_dot;
  VectorXd z_trans = z;
  for(;z_trans(1)<(-PI);)
  {
    z_trans(1) += 2 * PI;
  }
  for(;z_trans(1)>=PI;)
  {
    z_trans(1) -= 2 * PI;
  }

  VectorXd y = z_trans - hx;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd K = PHt * S.inverse();

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_ ) * P_;

  std::cout<<"Finish Radar!"<<std::endl;
}
