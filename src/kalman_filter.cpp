#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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

	// intermediate computation. for rho
	double mag = sqrt((x_[0] * x_[0]) + (x_[1] * x_[1]));

	// rho, theta and rho_dot in a vector
	VectorXd z_pred = VectorXd(3);
	z_pred <<	mag,
				atan2(x_[1], x_[0]),
				((x_[0] * x_[2]) + (x_[1] * x_[3])) / mag;
	
	// Equation changes in EKF. The prediction does not use the H matrix but the h functions to map cartesian to polar coordinates
	VectorXd y = z - z_pred;		//

	//Normalize theta between -pi and pi
	while (y(1)<-M_PI || y(1)>M_PI) {
		if (y(1) < -M_PI) {
			y(1) += M_PI;
		}
		else {
			y(1) -= M_PI;
		}
	}

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;  // Jacobian matrix "Hj" is used instead
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
