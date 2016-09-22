#ifndef _KALMAN_H_
#define _KALMAN_H_
#include <Eigen/Dense>

void kalman_init();

Eigen::MatrixXd kalman(Eigen::MatrixXd z);

Eigen::MatrixXd pest();

#endif