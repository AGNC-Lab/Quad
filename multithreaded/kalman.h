#ifndef _KALMAN_H_
#define _KALMAN_H_
#include <Eigen/Dense>

void kalman_init();

void kalman_propagate();

Eigen::Matrix<float, 6, 1> kalman_estimate(Eigen::Matrix<float, 3, 1> z);

#endif