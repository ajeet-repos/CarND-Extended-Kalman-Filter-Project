#include "kalman_filter.h"

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_; 

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // New state
  x_ = x_ + (K * y);

  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  double rho = sqrt(x_[0]*x_[0]+x_[1]*x_[1]);
  double theta = atan2(x_[1],x_[0]);
  double rho_dot = (x_[0]*x_[2]+x_[1]*x_[3])/sqrt(x_[0]*x_[0]+x_[1]*x_[1]);

  VectorXd h = VectorXd(3); // h(x_)
  h << rho, theta, rho_dot;

  VectorXd y = z - h;

  // adjusting the angle so that its always within desired range.
  if(y[1]>=M_PI){
       while(y[1]>M_PI){
         y[1]-=2*M_PI;
       }
  }else if(y[1]<=-M_PI){
      while(y[1]<=-M_PI){
          y[1]+=2*M_PI;
      }
  }
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // New state
  x_ = x_ + (K * y);
  int x_size = x_.size();

  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
