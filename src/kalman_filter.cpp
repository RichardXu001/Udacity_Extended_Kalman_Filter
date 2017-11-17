#include "kalman_filter.h"
#include "tools.h"
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;
Tools tools;

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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  //std::cout<<"Laser after KF update: x_:\n"<<x_<<endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  //std::cout<<"Laser after KF update: P_:\n"<<P_<<endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //cout<<"Now enter EKF!!"<<endl;
  double px=x_[0],py=x_[1],vx=x_[2],vy=x_[3];
    //cout<<"px is "<<px<<" py is "<<py<<" vx is "<<vx<<" vy is "<<vy<<endl;

    double rho_=sqrt(px * px + py * py);
    float phi_;
    float rho_dot_;
    float PI = 3.1415926;
    //cout<<"rho is "<<rho_<<endl;
    if(abs(px)<0.0001){
        phi_=PI/2.0;
    }
    else{
        phi_=atan(py/px);
    }
    //cout<<"original phi is "<<phi_;
    //cout<<"measurement phi is "<<z(1)<<endl;
    if((z(1)>0.5*PI)&&(abs(z(1)-phi_)>PI/2.0) ){
        phi_ += PI;
        //cout<<"after modify phi is "<<phi_<<endl;
    }
    if((z(1)<=-0.5*PI)&&(abs(z(1)-phi_)>PI/2.0)){
        phi_ -=PI;
        //cout<<"after modify phi is "<<phi_<<endl;
    }

    /*
    while ( phi_ < -PI) {
        phi_  +=  2*PI;
    }
    while (phi_ > PI) {
        phi_  -= 2*PI;
    }

    if (phi_  > PI || phi_ < -PI) {
        cout << "Error" << endl;
    }
    cout<<"phi is "<<phi_<<endl;
     */

    if (rho_ < 0.0001) {
        cout << "Error2" << endl;
    }else{

        rho_dot_ = (px*vx+py*vy)/rho_;
    }
    //float rho_dot_=(px*vx+py*vy)/sqrt(px*px+py*py);
    //cout<<"rho dot is "<<rho_dot_<<endl;
  VectorXd z_pred(3);
    //VectorXd h(3);
  z_pred<< rho_,phi_,rho_dot_;
    /*
  std::cout<<"z_pred is:\n"<<z_pred<<endl;
  std::cout<<"before change: measurement z is :\n"<<z<<endl;
    while ( z(1) < -PI) {
        z(1)  +=  2*PI;
    }
    while (z(1) > PI) {
        z(1)  -= 2*PI;
    }
  std::cout<<"after change: measurement z is \n:"<<z<<endl;
    //cout<<"measurement pred is :"<<z_pred<<endl;

     */
  VectorXd y = z -z_pred;
  //H_=tools.CalculateJacobian(x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  //std::cout<<"RADAR after EKF update: x_:"<<x_<<endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  //std::cout<<"RADAR after EKF update: P_:"<<P_<<endl;
}



  
