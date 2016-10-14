
#include "MatricesAndVectors.h"
#include "QuatRotEuler.h"
#include <math.h>

//Rotation matrix around x
Mat3x3 Rotx(double theta){
	Mat3x3 R;
	R.M[0][0] = 1;
	R.M[0][1] = 0;                
	R.M[0][2] = 0;

	R.M[1][0] = 0;
	R.M[1][1] = cos(theta);
	R.M[1][2] = -sin(theta);

	R.M[2][0] = 0;
	R.M[2][1] = sin(theta);
	R.M[2][2] = cos(theta);

	return R;
}

//Rotation matrix around y
Mat3x3 Roty(double theta){
	Mat3x3 R;
	R.M[0][0] = cos(theta);
	R.M[0][1] = 0;                
	R.M[0][2] = sin(theta);

	R.M[1][0] = 0;
	R.M[1][1] = 1;
	R.M[1][2] = 0;

	R.M[2][0] = -sin(theta);
	R.M[2][1] = 0;
	R.M[2][2] = cos(theta);

	return R;
}

//Rotation matrix around z
Mat3x3 Rotz(double theta){
	Mat3x3 R;
	R.M[0][0] = cos(theta);
	R.M[0][1] = -sin(theta);              
	R.M[0][2] = 0;

	R.M[1][0] = sin(theta);
	R.M[1][1] = cos(theta);
	R.M[1][2] = 0;

	R.M[2][0] = 0;
	R.M[2][1] = 0;
	R.M[2][2] = 1;

	return R;
}

//Convert Euler Angles (Roll-pitch-yaw) to Rotation Matrix
Mat3x3 RPY2Rot(double roll, double pitch, double yaw){
	Mat3x3 Rx = Rotx(roll);
	Mat3x3 Ry = Roty(pitch);
	Mat3x3 Rz = Rotz(yaw);

	return MultiplyMat3x3(Rz, MultiplyMat3x3(Ry, Rx)); //Rout = Rz*Ry*Rx
	
}

//Convert quaternion to rotation matrix 
Mat3x3 Quat2rot(Vec4 q){

	Mat3x3 R;
	double qw = q.v[0];
	double qx = q.v[1];
	double qy = q.v[2];
	double qz = q.v[3];

	R.M[0][0] = 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2);
	R.M[0][1] = 2 * qx*qy - 2 * qz*qw;
	R.M[0][2] = 2 * qx*qz + 2 * qy*qw;
	R.M[1][0] = 2 * qx*qy + 2 * qz*qw;
	R.M[1][1] = 1 - 2 * pow(qx, 2) - 2 * pow(qz, 2);
	R.M[1][2] = 2 * qy*qz - 2 * qx*qw;
	R.M[2][0] = 2 * qx*qz - 2 * qy*qw;
	R.M[2][1] = 2 * qy*qz + 2 * qx*qw;
	R.M[2][2] = 1 - 2 * pow(qx, 2) - 2 * pow(qy, 2);

	return R;
}

//Convert rotation matrix to quaternion
//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
Vec4 Rot2quat(Mat3x3 M){
	double trace = M.M[0][0] + M.M[1][1] + M.M[2][2];
	Vec4 q;
	if (trace > 0) {// M_EPSILON = 0
		double s = 0.5 / sqrt(trace + 1.0);
		q.v[0] = 0.25 / s;
		q.v[1] = (M.M[2][1] - M.M[1][2]) * s;
		q.v[2] = (M.M[0][2] - M.M[2][0]) * s;
		q.v[3] = (M.M[1][0] - M.M[0][1]) * s;
	}
	else {
		if (M.M[0][0] > M.M[1][1] && M.M[0][0] > M.M[2][2]) {
			double s = 2.0 * sqrt(1.0 + M.M[0][0] - M.M[1][1] - M.M[2][2]);
			q.v[0] = (M.M[2][1] - M.M[1][2]) / s;
			q.v[1] = 0.25 * s;
			q.v[2] = (M.M[0][1] + M.M[1][0]) / s;
			q.v[3] = (M.M[0][2] + M.M[2][0]) / s;
		}
		else if (M.M[1][1] > M.M[2][2]) {
			double s = 2.0 * sqrt(1.0 + M.M[1][1] - M.M[0][0] - M.M[2][2]);
			q.v[0] = (M.M[0][2] - M.M[2][0]) / s;
			q.v[1] = (M.M[0][1] + M.M[1][0]) / s;
			q.v[2] = 0.25 * s;
			q.v[3] = (M.M[1][2] + M.M[2][1]) / s;
		}
		else {
			double s = 2.0 * sqrt(1.0 + M.M[2][2] - M.M[0][0] - M.M[1][1]);
			q.v[0] = (M.M[1][0] - M.M[0][1]) / s;
			q.v[1] = (M.M[0][2] + M.M[2][0]) / s;
			q.v[2] = (M.M[1][2] + M.M[2][1]) / s;
			q.v[3] = 0.25 * s;
		}
	}
        return q;
}

//Convert quaternion to Roll-Pitch-Yaw
Vec3 Quat2RPY(Vec4 q){
	float q0 = q.v[0];
	float q1 = q.v[1];
	float q2 = q.v[2];
	float q3 = q.v[3];

	Vec3 RPY;

	RPY.v[0] = atan2(2*(q0*q1 + q2*q3) , 1 - 2*(q1*q1 + q2*q2) );	//Roll
	RPY.v[1] = asin(2*(q0*q2 - q3*q1));				//Pitch
	RPY.v[2] = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2*q2 + q3*q3) );	//Yaw

	return RPY;
}

// Quaternion Multiplication
Vec4 QuaternionProduct(Vec4 q1, Vec4 q2){
	Mat4x4 H;
	H.M[0][0] = q1.v[0]; H.M[0][1] = -q1.v[1]; H.M[0][2] = -q1.v[2]; H.M[0][3] = -q1.v[3];
	H.M[1][0] = q1.v[1]; H.M[1][1] =  q1.v[0]; H.M[1][2] = -q1.v[3]; H.M[1][3] =  q1.v[2];
	H.M[2][0] = q1.v[2]; H.M[2][1] =  q1.v[3]; H.M[2][2] =  q1.v[0]; H.M[2][3] = -q1.v[1];
	H.M[3][0] = q1.v[3]; H.M[3][1] = -q1.v[2]; H.M[3][2] =  q1.v[1]; H.M[3][3] =  q1.v[0];

	return MultiplyMat4x4Vec4(H, q2);
}

//Triad Algorithm for Finding attitude from two vectors
// https://en.wikipedia.org/wiki/Triad_method
Mat3x3 Triad(Vec3 V1_body, Vec3 V2_body, Vec3 V1_inertial, Vec3 V2_inertial){
	Vec3 V1_bodyNorm, V2_bodyNorm, V1_inertialNorm, V2_inertialNorm, r1, r2, r3, R1, R2, R3;

	//Normalize vectors and make them orthogonal
	V1_bodyNorm = ScaleVec3(V1_body, 1.0 / p_normVec3(V1_body, 2));
	V2_bodyNorm = ScaleVec3(V2_body, 1.0 / p_normVec3(V2_body, 2));
	V1_inertialNorm = ScaleVec3(V1_inertial, 1.0 / p_normVec3(V1_inertial, 2));
	V2_inertialNorm = ScaleVec3(V2_inertial, 1.0 / p_normVec3(V2_inertial, 2));

	//Set orthogonal vectors from body frame
	r1 = V1_bodyNorm; 
	r2 = ScaleVec3(cross(V1_bodyNorm, V2_bodyNorm), 1.0/p_normVec3(cross(V1_bodyNorm, V2_bodyNorm), 2)); //(V1 x V2)/norm(V1 x V2)
	r3 = cross(r1, r2);

	//Set orthogonal vectors from inertial frame
	R1 = V1_inertialNorm;
	R2 = ScaleVec3(cross(V1_inertialNorm, V2_inertialNorm), 1.0/p_normVec3(cross(V1_inertialNorm, V2_inertialNorm), 2)); //(V1 x V2)/norm(V1 x V2)
	R3=cross(R1,R2);

	//We need to find the rotation matrix that solves [R1:R2:R3]=Rot.[r1:r2:r3]
	// ==> Rot = [R1:R2:R3].[r1:r2:r3]'
	Mat3x3 R = Concatenate3Vec3_2_Mat3x3(R1, R2, R3); //Creates matrix R = [R1:R2:R3]
	Mat3x3 r = Concatenate3Vec3_2_Mat3x3(r1, r2, r3); //Creates matrix r = [r1:r2:r3]
	return MultiplyMat3x3(R, TransposeMat3x3(r));
}

//Propagation quaternion for propagating quaternion with gyroscope data
//Equation 34 in https://users.aalto.fi/~ssarkka/pub/quat.pdf
Vec4 propagationQuat(Vec3 w, double dt){
	Vec4 dq; //delta quaternion
	double norm_w = p_normVec3(w, 2);
	double angle = dt*norm_w / 2;
	double sin_angle = sin(angle);

	dq.v[0] = cos(angle);
	if (norm_w != 0){
		dq.v[1] = w.v[0] * sin_angle / norm_w;
		dq.v[2] = w.v[1] * sin_angle / norm_w;
		dq.v[3] = w.v[2] * sin_angle / norm_w;
	}
	else{
		dq.v[1] = 0;
		dq.v[2] = 0;
		dq.v[3] = 0;
	}
	return dq;
}

//Propagate rotation matrix from initial point through angular velocity
//Equation 33 in https://users.aalto.fi/~ssarkka/pub/quat.pdf, but instead of
// using quaternion multiplication, we use rotation matrix multiplication
Mat3x3 propagateRotMat(Mat3x3 M0, Vec3 w, double dt){
	Vec4 dq = propagationQuat(w, dt);
	Mat3x3 dRot = Quat2rot(dq);

	return MultiplyMat3x3(M0, dRot);
}
