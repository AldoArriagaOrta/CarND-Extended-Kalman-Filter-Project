#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	
	//this method was taken directly from the lesson
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse *(1.0/ estimations.size());

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3, 4);

	//fetch state variables
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//intermediate computations (to avoid duplications)
	double psq = (px*px) + (py*py);
	double pmag = sqrt(psq);
	double p32 = pmag*psq;

	//check division by zero. x^2 + y^2 > 0
	if (psq < 0) {
		cout << "Division by zero error" << endl;
		return Hj;
	}

	//matrix elements to avoid duplicating computations
	double h11 = px / pmag;
	double h12 = py / pmag;
	double h21 = -py / psq;
	double h22 = px / psq;
	double h31 = (py*(vx*py - vy*px)) / p32;
	double h32 = (px*(vy*px - vx*py)) / p32;

	//Jacobian Matrix. Some elements are repeated to avoid unnecessary variables
	Hj <<	h11, h12, 0, 0,
			h21, h22, 0, 0,
			h31, h32, h11, h12;

	return Hj;
}
