#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using Eigen::Matrix;

float dt;
float vicon_R;
float accelerometer_R;
float sigma_J;

Matrix<float, 9, 1> x_est;
Matrix<float, 9, 9> p_est;
Matrix<float, 9, 9> p_pos;
Matrix<float, 9, 9> Q;           
Matrix<float, 3, 3> R_vicon;
Matrix<float, 3, 3> R_acc;
Matrix<float, 9, 9> A;
Matrix<float, 3, 9> H_pos;
Matrix<float, 3, 9> H_acc;
Matrix<float, 3, 9> H_v;
Matrix<float, 9, 1> x_prd;
Matrix<float, 9, 9> p_prd;
Matrix<float, 3, 3> S;
Matrix<float, 3, 9> B;
Matrix<float, 9, 3> K;
Matrix<float, 3, 1> y;
Matrix<float, 3, 1> v;
Matrix<float, 3, 1> a;
Matrix<float, 9, 1> result;
Matrix<float, 9, 3> G;
Matrix<float, 9, 9> I_9x9;
Matrix<float, 3, 3> I_3x3;
Matrix<float, 3, 3> Zero_3x3;


void kalman_init()
{

	dt = 0.010;             // 100 Hz
  sigma_J = 10;           // Jerk process noise
  // sigma_bAcc = 10;        // Accelerometer bias process noise
	vicon_R = 0.0001;       // estimated vicon standard deviation
  accelerometer_R = 15.0; // estimated accelerometer standard deviation
  I_9x9 = Matrix<float, 9, 9>::Identity();
  I_3x3 = Matrix<float, 3, 3>::Identity();
  Zero_3x3 = Matrix<float, 3, 3>::Zero(3, 3);

	x_est = Matrix<float, 9, 1>::Zero();
  
  G << I_3x3 * (pow(dt, 3)/6),
       I_3x3 * (pow(dt, 2)/2),
       I_3x3 * dt;

	Q = pow(sigma_J, 2) * (G * G.transpose());     // process noise
	R_vicon = pow(vicon_R, 2) * I_3x3; // Measurement error

  R_acc = pow(accelerometer_R, 2) * I_3x3; // Measurement error

  A << I_3x3,    dt*I_3x3, (pow(dt,2)/2)*I_3x3,
       Zero_3x3,    I_3x3,            dt*I_3x3,
       Zero_3x3, Zero_3x3,               I_3x3;

  p_est << 1.0*I_3x3,     Zero_3x3,     Zero_3x3,
            Zero_3x3, 0.0001*I_3x3,     Zero_3x3,
            Zero_3x3,     Zero_3x3, 0.0001*I_3x3;

  H_pos << I_3x3,    Zero_3x3, Zero_3x3;
  H_v   << Zero_3x3, I_3x3,    Zero_3x3;
  H_acc << Zero_3x3, Zero_3x3,    I_3x3;

  // p_est <<   1,    0,    0,       0,       0,       0,      0,       0,       0,
  //            0,    1,    0,       0,       0,       0,      0,       0,       0,
  //            0,    0,    1,       0,       0,       0,      0,       0,       0,
  //            0,    0,    0,  0.0001,       0,       0,      0,       0,       0,
  //            0,    0,    0,       0,  0.0001,       0,      0,       0,       0,
  //            0,    0,    0,       0,       0,  0.0001,      0,       0,       0,
  //            0,    0,    0,       0,       0,       0, 0.0001,       0,       0,
  //            0,    0,    0,       0,       0,       0,      0,  0.0001,       0,
  //            0,    0,    0,       0,       0,       0,      0,       0,  0.0001;

  // H_pos << 1,  0,  0,  0,  0,  0,  0,  0,  0,
  //          0,  1,  0,  0,  0,  0,  0,  0,  0,
  //          0,  0,  1,  0,  0,  0,  0,  0,  0;

  // H_acc << 0,  0,  0,  0,  0,  0,  1,  0,  0,
  //          0,  0,  0,  0,  0,  0,  0,  1,  0,
  //          0,  0,  0,  0,  0,  0,  0,  0,  1;

  // H_v << 0,  0,  0,  1,  0,  0,  0,  0,  0,
  //        0,  0,  0,  0,  1,  0,  0,  0,  0,
  //        0,  0,  0,  0,  0,  1,  0,  0,  0;

}

Matrix<float, 9, 1> kalman_propagate()
{
  // Predicting state and covariance

  x_prd = A * x_est;
  p_prd = A * p_est * A.transpose() + Q;

  y = H_pos * x_prd;   // Position estimate

  v = H_v * x_prd; // Velocity estimate

  a = H_acc * x_prd; // Velocity estimate

  result << y,
            v,
            a;

  x_est = x_prd;
  p_est = p_prd;

  return result;

}

Matrix<float, 9, 1> kalman_estimate_pos(Matrix<float, 3, 1> z)
{

// Estimating state and covariance

  S = H_pos * p_prd.transpose() * H_pos.transpose() + R_vicon;
  B = H_pos * p_prd.transpose();
  K = (S.inverse() * B).transpose();

  x_est = x_prd + K * (z - H_pos * x_prd);
  p_est = (I_9x9 - K * H_pos) * p_prd * (I_9x9 - K * H_pos).transpose() + K * R_vicon * K.transpose();   // Joseph form 

// Computing measurements
  y = H_pos * x_est;   // Position estimate
  v = H_v * x_est; // Velocity estimate
  a = H_acc * x_est; // Acc estimate

  result << y,
            v,
            a;

  //These are necessary if the acceleration is updated
  x_prd = x_est;
  p_prd = p_est;

  return result;
}


Matrix<float, 9, 1> kalman_estimate_acc(Matrix<float, 3, 1> z)
{

// Estimating state and covariance

  S = H_acc * p_prd.transpose() * H_acc.transpose() + R_acc;
  B = H_acc * p_prd.transpose();
  K = (S.inverse() * B).transpose();

  x_est = x_prd + K * (z - H_acc * x_prd);
  p_est = (I_9x9 - K * H_acc) * p_prd * (I_9x9 - K * H_acc).transpose() + K * R_acc * K.transpose();   // Joseph form 

// Computing measurements
  y = H_pos * x_est;   // Position estimate
  v = H_v * x_est; // Velocity estimate
  a = H_acc * x_est;

  result << y,
            v,
            a;

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