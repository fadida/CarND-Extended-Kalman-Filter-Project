#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  
  tools = Tools();

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_      = MatrixXd(3, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  ekf_.F_ = Eigen::MatrixXd::Identity(4, 4);
  
  //measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
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
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
    */
    // first measurement
    cout << "Initalizing EKF. " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float ro     = measurement_pack.raw_measurements_(0);
      float theta  = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);

      /**
      Convert radar from polar to Cartesian coordinates and initialize state.
      */
      ekf_.x_(0) = ro * cos(theta);
      ekf_.x_(1) = ro * sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }
    
    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }
  
 /* if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)*/
 /*   return;*/
  
  // Update the time delta between measurements.
  double dt = measurement_pack.timestamp_ - previous_timestamp_;
  // Convert dt to seconds from miliseconds.
  dt *= 1e-6;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  if (dt < 1e-3) {
    cout << __FUNCTION__ << ": Found zero time between measurements, skipping prediction step." << endl ;
  } else {

	//Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    /**
  	 * Update the state transition matrix F according to the new elapsed time.
  	  - Time is measured in seconds.
  	 * Update the process noise covariance matrix.
  	 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    double noise_ax = 9;
    double noise_ay = 9;

    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
  			  0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
  			  dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
  			  0, dt_3/2*noise_ay, 0, dt_2*noise_ay;


    ekf_.Predict();
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Calculate Jacobian for h(x) when x is the predicted vector.
    Hj_ = tools.CalculateJacobian(ekf_.x_);

    ekf_.H_= Hj_;
    ekf_.R_ = R_radar_;
  
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
  
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  
}
