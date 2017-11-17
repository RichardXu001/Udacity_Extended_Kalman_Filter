#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
  Hj_ = MatrixXd(3, 4);  
  
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
        

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000; 

  

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
   noise_ax = 9;
   noise_ay = 9;


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement


    //cout << "EKF Initialization!! " << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    //cout<<"previous_timestamp_ is :"<<previous_timestamp_<<endl;

    //cout << "EKF: " << endl;
    //x_in = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;
    //Eigen::VectorXd x_in;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 0, 0, 0, 0;
    
    //Eigen::MatrixXd H_in;
    //cout<<"measurement_pack.sensor_type_ is:"<<measurement_pack.sensor_type_<<endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //cout<<"RADAR Initialization!!"<<endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]), 
      -measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]), 0.0, 0.0;
      //previous_timestamp_ = measurement_pack.timestamp_;
      //is_initialized_ = true;
      //std::cout<<"efk_.x_:"<<ekf_.x_<<endl;
       

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //cout<<"LIDAR Initialization!!"<<endl;
      //cout<<"measurement_pack.raw_measurements_ is:"<<measurement_pack.raw_measurements_<<endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
      //ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      //cout<<"efk_.x_:"<<ekf_.x_<<endl;
       
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  //cout<<"dt is:"<<dt<<endl;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  ekf_.Q_ = MatrixXd(4, 4);
   
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  //std::cout<<"ekf_.Q_:"<<ekf_.Q_<<endl;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_=R_radar_;
    Hj_=tools.CalculateJacobian(ekf_.x_);
    ekf_.H_=Hj_;
    //std::cout<<"after Jacobian CALC ekf_.H_:"<<ekf_.H_<<endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    //std::cout<<"after RADAR ekf Update"<<endl;

  } else {
    // Laser updates
    ekf_.H_=H_laser_;
    ekf_.R_=R_laser_;    
    ekf_.Update(measurement_pack.raw_measurements_);
    //std::cout<<"After Laser ekf Update"<<endl;
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
