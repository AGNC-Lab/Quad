#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using Eigen::Matrix;

float dt;
float vicon_R;
float sigma_Q;

Matrix<float, 9, 1> x_est;
Matrix<float, 9, 9> p_est;
Matrix<float, 9, 9> p_pos;
Matrix<float, 9, 9> Q;           
Matrix<float, 3, 3> R;
Matrix<float, 9, 9> A;
Matrix<float, 3, 9> H;
Matrix<float, 3, 9> H_v;
Matrix<float, 9, 1> x_prd;
Matrix<float, 9, 9> p_prd;
Matrix<float, 3, 3> S;
Matrix<float, 3, 9> B;
Matrix<float, 9, 3> K;
Matrix<float, 3, 1> y;
Matrix<float, 3, 1> v;
Matrix<float, 6, 1> result;
Matrix<float, 9, 3> G;
Matrix<float, 9, 9> I_9x9;
Matrix<float, 3, 3> I_3x3;


void kalman_init()
{

	dt = 0.005;       // 100 Hz
  sigma_Q = 2.2;   // constant for Q
	vicon_R = 0.0001; // estimated vicon error
  I_9x9 = Matrix<float, 9, 9>::Identity();
  I_3x3 = Matrix<float, 3, 3>::Identity();

	x_est = Matrix<float, 9, 1>::Zero();
  
  G << I_3x3 * (pow(dt, 3)/6),
       I_3x3 * (pow(dt, 2)/2),
       I_3x3 * dt;

	Q = pow(sigma_Q, 2) * (G * G.transpose());     // process noise
	R = pow(vicon_R, 2) * I_3x3; // Measurement error

  A << 1,  0,  0, dt,  0,  0, dt*dt/2.0,         0,         0,
       0,  1,  0,  0, dt,  0,         0, dt*dt/2.0,         0,
       0,  0,  1,  0,  0, dt,         0,         0, dt*dt/2.0,
       0,  0,  0,  1,  0,  0,        dt,         0,         0,
       0,  0,  0,  0,  1,  0,         0,        dt,         0,
       0,  0,  0,  0,  0,  1,         0,         0,        dt,
       0,  0,  0,  0,  0,  0,         1,         0,         0,
       0,  0,  0,  0,  0,  0,         0,         1,         0,
       0,  0,  0,  0,  0,  0,         0,         0,         1;

  p_est <<   1,    0,    0,       0,       0,       0,      0,       0,       0,
             0,    1,    0,       0,       0,       0,      0,       0,       0,
             0,    0,    1,       0,       0,       0,      0,       0,       0,
             0,    0,    0,  0.0001,       0,       0,      0,       0,       0,
             0,    0,    0,       0,  0.0001,       0,      0,       0,       0,
             0,    0,    0,       0,       0,  0.0001,      0,       0,       0,
             0,    0,    0,       0,       0,       0, 0.0001,       0,       0,
             0,    0,    0,       0,       0,       0,      0,  0.0001,       0,
             0,    0,    0,       0,       0,       0,      0,       0,  0.0001;

  H << 1,  0,  0,  0,  0,  0,  0,  0,  0,
       0,  1,  0,  0,  0,  0,  0,  0,  0,
       0,  0,  1,  0,  0,  0,  0,  0,  0;

  H_v << 0,  0,  0,  1,  0,  0,  0,  0,  0,
         0,  0,  0,  0,  1,  0,  0,  0,  0,
         0,  0,  0,  0,  0,  1,  0,  0,  0;

}

Matrix<float, 6, 1> kalman_propagate()
{
  // Predicting state and covariance

  x_prd = A * x_est;
  p_prd = A * p_est * A.transpose() + Q;

  y = H * x_prd;   // Position estimate

  v = H_v * x_prd; // Velocity estimate

  result << y,
            v;

  return result;

}

Matrix<float, 6, 1> kalman_estimate(Matrix<float, 3, 1> z)
{

// Estimating state and covariance

  S = H * p_prd.transpose() * H.transpose() + R;
  B = H * p_prd.transpose();

  K = (S.inverse() * B).transpose();

  x_est = x_prd + K * (z - H * x_prd);

  //p_est = p_prd - klm_gain * H * p_prd; // unstable formula

  p_est = (I_9x9 - K * H) * p_prd * (I_9x9 - K * H).transpose() + K * R * K.transpose();   // Joseph form 

// Computing measurements

  y = H * x_est;   // Position estimate

  v = H_v * x_est; // Velocity estimate

  result << y,
            v;

  return result;
}
// int main()
// {

// 	kalman_init();

// 	MatrixXd z(3,1);   // measurement vector

// 	z << 1,
// 	     1,
// 	     1;

// 	std::cout << kalman(z) << std::endl;

// }