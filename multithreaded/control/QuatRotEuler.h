#ifndef FSW_CONTROL_QUATROTEULER_H
#define FSW_CONTROL_QUATROTEULER_H

#include "MatricesAndVectors.h"

//Rotation matrix around x
Mat3x3 Rotx(double theta);

//Rotation matrix around y
Mat3x3 Roty(double theta);

//Rotation matrix around z
Mat3x3 Rotz(double theta);

//Convert Euler Angles (Roll-pitch-yaw) to Rotation Matrix
Mat3x3 RPY2Rot(double roll, double pitch, double yaw);

//Convert quaternion to rotation matrix 
Mat3x3 Quat2rot(Vec4 q);

//Convert rotation matrix to quaternion
Vec4 Rot2quat(Mat3x3 M);

//Convert quaternion to Roll-Pitch-Yaw
Vec3 Quat2RPY(Vec4 q);

// Quaternion Multiplication
Vec4 QuaternionProduct(Vec4 q1, Vec4 q2);

//Triad Algorithm for Finding attitude from two vectors
Mat3x3 Triad(Vec3 V1_body,Vec3 V2_body,Vec3 V1_inertial,Vec3 V2_inertial);

//Propagation quaternion for propagating quaternion with gyroscope data
//Equation 34 in https://users.aalto.fi/~ssarkka/pub/quat.pdf
Vec4 propagationQuat(Vec3 w, double dt);

//Propagate rotation matrix from initial point through angular velocity
//Equation 33 in https://users.aalto.fi/~ssarkka/pub/quat.pdf, but instead of
// using quaternion multiplication, we use rotation matrix multiplication
Mat3x3 propagateRotMat(Mat3x3 M0, Vec3 w, double dt);

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
