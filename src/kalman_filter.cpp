#include "kalman_filter.h"
#include "tools.h"
#include<iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in, MatrixXd &Hj_in, MatrixXd &R_ekf_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  Hj_ = Hj_in;
  R_ekf = R_ekf_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_*x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y;
  MatrixXd S, K;
  MatrixXd I = MatrixXd::Identity(4,4);
  y = z - H_ * x_;
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  VectorXd y;
  MatrixXd S, K;
  MatrixXd I = MatrixXd::Identity(4,4);
  Hj_ = tools.CalculateJacobian(x_);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float rho = sqrt(px * px + py * py);
  float phi = atan2(py,px);
  float phi_dot = (px*vx + py*vy)/rho;
  VectorXd hx(3);
  hx << rho,phi,phi_dot;
  y = z - hx;
  
  S = Hj_ * P_ * Hj_.transpose() + R_ekf;
  K = P_ * Hj_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K*Hj_)*P_;
  
  
}
