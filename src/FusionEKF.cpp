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

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ <<	0.0225, 0,
				0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ <<	0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;

  H_laser_ <<	1, 0, 0, 0,
				0, 1, 0, 0;

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

    // Initialization of state vector ekf_.x_ with first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

		//Initialization for radar measurement
		Hj_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.R_ = MatrixXd(3, 3);
		ekf_.R_ << R_radar_;
		ekf_.H_ = MatrixXd(3, 4);
		ekf_.H_ << Hj_;

		//intermediate variables (to avoid unnecessary calls to trigonometric functions)
		double Cos_theta = cos(measurement_pack.raw_measurements_[1]);
		double Sin_theta = sin(measurement_pack.raw_measurements_[1]);

		//mapping polar radar measurements to Cartesian state variables
		ekf_.x_ <<	measurement_pack.raw_measurements_[0] * Cos_theta,
					measurement_pack.raw_measurements_[0] * Sin_theta,
					measurement_pack.raw_measurements_[2] * Cos_theta,
					measurement_pack.raw_measurements_[2] * Sin_theta;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      //Initialization for lidar measurement
	  //set the state with the initial location and zero velocity
		ekf_.H_ = MatrixXd(2, 4);
		ekf_.H_ << H_laser_;
		ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ <<	1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 10, 0,
				0, 0, 0, 10;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ <<	1, 0, 1, 0,
				0, 1, 0, 1,
				0, 0, 1, 0,
				0, 0, 0, 1;

    // done initializing, no need to predict or update
	// but first timestamp is needed for the next cycle
	previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

   //compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;


  //1. Modify the F matrix so that the time is integrated
  ekf_.F_ <<	1, 0, dt, 0,
				0, 1, 0, dt,
				0, 0, 1, 0,
				0, 0, 0, 1;
  //2. Set the process covariance matrix Q
  double t4 = (dt*dt*dt*dt) / 4;
  double t3 = (dt*dt*dt) / 2;
  double t2 = dt*dt;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<	t4*noise_ax, 0, t3*noise_ax, 0,
				0, t4*noise_ay, 0, t3*noise_ay,
				t3*noise_ax, 0, t2*noise_ax, 0,
				0, t3*noise_ay, 0, t2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  Hj_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.R_ =  MatrixXd(3, 3);
	  ekf_.R_ << R_radar_;
	  ekf_.H_ = MatrixXd(3, 4);
	  ekf_.H_ << Hj_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
	  ekf_.R_ = MatrixXd(2, 2);
	  ekf_.R_ << R_laser_;
	  ekf_.H_ = MatrixXd(2, 4);
	  ekf_.H_ << H_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
