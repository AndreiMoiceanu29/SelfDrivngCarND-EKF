#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  
   //Set the process and measurement noises
  
  noise_ax = 9;
  noise_ay = 9;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    cout<<"FusionEKF Initialising..."<<endl;
    

    // first measurement
    
    VectorXd x(4);
	
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      //Converting from polar coords to carthesian coords.
      float Px, Py;
      Px = rho * cos(phi);
      Py = rho * sin(phi);
      /***Adding the states to the state vector
      	px -> rho * cos(phi)
        py -> rho * sin(phi)
        vx -> 0
        vy -> 0
      ***/
      x << Px, Py, 0, 0; 
      

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }
    
    //Initialising the matrices
    previous_timestamp_ = measurement_pack.timestamp_;
    MatrixXd P(4,4);
    //Initial position coordinates are given so we give them a small covariance, while the velocities are unknown so we give them a higher covariance.
    P<<1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;
    MatrixXd F = MatrixXd::Identity(4,4); //We initialy make F the Identity matrix due to similarity.
    H_laser_ << 1, 0, 0, 0,
    		   0, 1, 0, 0;
    
    MatrixXd Q(4,4);
    
    ekf_.Init(x, P, F, H_laser_,R_laser_, Q, Hj_, R_radar_);
      
	
    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
  if(dt > 0){
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;
    
    MatrixXd G(4,2);
    G << (dt * dt)/2, 0,
    	 0, (dt * dt)/2,
         dt, 0,
    	 0, dt;
    MatrixXd Qv(2,2);
    Qv << noise_ax, 0,
          0, noise_ay;
    
    ekf_.Q_ = G * Qv * G.transpose();
    
    ekf_.Predict();
  }

  

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
    
  } else {
    // TODO: Laser updates
    
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
