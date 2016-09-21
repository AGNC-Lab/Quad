#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

float dt;
float vicon_c;

MatrixXd x_est;
MatrixXd p_est;
MatrixXd Q;           
MatrixXd R;
MatrixXd A(9,9);
MatrixXd H(3,9);
MatrixXd x_prd;
MatrixXd p_prd;
MatrixXd S;
MatrixXd B;
MatrixXd klm_gain;
MatrixXd y;

void kalman_init()
{

	dt = 0.01;   // 100 Hz
	vicon_c = 1; // vicon error

	x_est = MatrixXd::Zero(9,1);
  p_est = MatrixXd::Zero(9,9);

	Q = MatrixXd::Identity(9,9);           // process noise
	R = vicon_c * MatrixXd::Identity(3,3); // Measurement error

  A << 1,  0,  0, dt,  0,  0, dt*dt/2.0,         0,         0,
       0,  1,  0,  0, dt,  0,         0, dt*dt/2.0,         0,
       0,  0,  1,  0,  0, dt,         0,         0, dt*dt/2.0,
       0,  0,  0,  1,  0,  0,        dt,         0,         0,
       0,  0,  0,  0,  1,  0,         0,        dt,         0,
       0,  0,  0,  0,  0,  1,         0,         0,        dt,
       0,  0,  0,  0,  0,  0,         1,         0,         0,
       0,  0,  0,  0,  0,  0,         0,         1,         0,
       0,  0,  0,  0,  0,  0,         0,         0,         1;

  H << 1,  0,  0,  0,  0,  0,  0,  0,  0,
       0,  1,  0,  0,  0,  0,  0,  0,  0,
       0,  0,  1,  0,  0,  0,  0,  0,  0;

}

MatrixXd kalman(MatrixXd z)
{

// Predicting state and covariance

  x_prd = A * x_est;
  p_prd = A * p_est * A.transpose() + Q;

// Estimating state and covariance

  S = H * p_prd.transpose() * H.transpose() + R;
  B = H * p_prd.transpose();

  klm_gain = (S.inverse() * B).transpose();

  x_est = x_prd + klm_gain * (z - H * x_prd);

  p_est = p_prd - klm_gain * H * p_prd;

// Computing Measurements

  y = H * x_est; 

  return y;
}

int main()
{

	kalman_init();

	MatrixXd z(3,1);   // measurement vector

	z << 1,
	     1,
	     1;

	std::cout << kalman(z) << std::endl;

}