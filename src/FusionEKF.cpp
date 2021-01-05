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

  /**
   * 
   * Set the process and measurement noises
   */
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1, 1, 1, 1;
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  Hj_<< 1, 0, 0, 0,
        1, 0, 0, 0,
        0, 1, 0, 0;
  noise_ax = 9;
  noise_ay= 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  cout << "EKF! " << endl;

  if (!is_initialized_) {
   

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    previous_timestamp_ = measurement_pack.timestamp_;
    Eigen::MatrixXd P_in= MatrixXd(4,4);
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    Eigen::MatrixXd Q_in= MatrixXd(4,4);
    Q_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Eigen::MatrixXd F_in= MatrixXd(4,4);
    F_in << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
   Eigen::VectorXd x_in= VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float ro     = measurement_pack.raw_measurements_[0]; //range
      float phi    = measurement_pack.raw_measurements_[1]; //bearing
      float ro_dot = measurement_pack.raw_measurements_[2]; //vel rho
      x_in << ro*cos(phi), 
              ro*sin(phi), 
              ro_dot*cos(phi), 
              ro_dot*sin(phi);
      ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //Initialize state.
      x_in << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    std::cout<< "is initialized : " << is_initialized_ << std::endl;
    return;
  }

  /**
   * Prediction
   */


  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  std::cout << "dt : " << dt<< std::endl;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;



  ekf_.Predict();


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
    ekf_.R_=R_radar_;
    ekf_.H_=tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.R_=R_laser_;
    ekf_.H_=H_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
