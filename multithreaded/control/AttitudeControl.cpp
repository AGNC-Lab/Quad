//Libraries
#include <Eigen/Dense>
#include "MathFuncs.h"
#include "AttitudeControl.h"
#include "QuatRotEuler.h"
#include <stdio.h>		/* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <math.h>		/* pow, sqrt */
using Eigen::Matrix;
// Constant definitions
#define PI 3.14159265
#define K_ROLL  5			//Control gain for roll motion
#define K_PITCH 5			//Control gain for pitch motion
#define K_YAW   0			//Control gain for yaw motion
#define K_WX    2			//Control gain for angular velocity about x
#define K_WY    2			//Control gain for angular velocity about y
#define K_WZ    1			//Control gain for angular velocity about z
#define KT	1.09862e-6		//Thrust coefficient for the quadrotor's propellers
#define KM	9.58815e-9		//Moment coefficient for the quadrotor's propellers
#define RAD     0.12		//Radius of quadcopter (distance between center and farther tip of propeller)


//Functions

//Attitude error: e_r = 0.5*invSkew(Rref'*Rbw - Rbw'*Rref)
Matrix<float, 3, 1> AttitudeErrorVector(Matrix<float, 3, 3> Rbw, Matrix<float, 3, 3> Rdes){
	Matrix<float, 3, 3> M_aux1 = Rdes.transpose()*Rbw; //M_aux1 = Rdes'*Rbw
	Matrix<float, 3, 3> M_aux2 = Rbw.transpose()*Rdes; //M_aux2 = Rbw'*Rdes
	Matrix<float, 3, 1> V_aux = invSkew(M_aux1-M_aux2);		//V_aux = invSkew(M_aux1 - M_aux2)

	return V_aux*-0.5; //0.5*V_aux
}

//Attitude control inputs: u_att = -Kr*e_r - Kw*e_w
Matrix<float, 3, 1> AttitudeControlInputs(Matrix<float, 3, 3> Kr, Matrix<float, 3, 3> Kw, Matrix<float, 3, 1> e_r, Matrix<float, 3, 1> e_w){
	Matrix<float, 3, 1> V_aux1 = Kr*e_r*(-1); //V_aux1 = -Kr*e_r
	Matrix<float, 3, 1> V_aux2 = Kw*e_w*(-1); //V_aux2 = -Kw*e_w
	//PrintVec3(V_aux1, "Proportional");
	//PrintVec3(V_aux2, "Derivative");

	//return V_aux2;

	return V_aux1+V_aux2; //V_aux1 + V_aux2
}


/*Return inputs to quadrotor for attitude control
q[4];		//Attitude quaternion
Rdes;		//Desired rotation matrix
w_bw[3];	//Angular velocity of the system
wDes[3];	//Desired angular velocity
u1;			//Thrust 
This algorithm was extracted from Mellinger, 2011: Minimum Snap Trajectory Generation and Control for Quadrotors*/
Matrix<float, 4, 1> attitudeControl(Matrix<float, 4, 1> q, Matrix<float, 3, 3> Rdes, Matrix<float, 3, 1> w_bw, Matrix<float, 3, 1> wDes, float u1){
	
	//Controller gain matrices
	double K_eVec[] = { K_ROLL, K_PITCH, K_YAW };	//Attitude error gains
	double K_wVec[] = { K_WX, K_WY, K_WZ };			//Angular velocity gains
	Matrix<float, 3, 3> K_r = Diag3(K_eVec);
	Matrix<float, 3, 3> K_w = Diag3(K_wVec);

	//Get rotation matrix from quaternion
	Matrix<float, 3, 3> Rbw = Quat2rot(q); //Matrix obtained in the NED parameterization
        //PrintMat3x3(Rbw);
	//PrintMat3x3(Rdes);
	
	//Attitude error: e_r = 0.5*invSkew(Rref'*Rbw - Rbw'*Rref)
	Matrix<float, 3, 1> e_r = AttitudeErrorVector(Rbw, Rdes);
	//PrintVec3(e_r, "ErrorAtt");

	//Angular velocity error: e_w = w_bw - wDes
	Matrix<float, 3, 1> e_w = Subtract3x1Vec(w_bw,wDes);

	//Attitude control input: u_att = -K_r*e_r - K_w*e_w
	Matrix<float, 3, 1> u_att = AttitudeControlInputs(K_r, K_w, e_r, e_w);

	//Saturate the attitude torques to the system so that they are not too high
	u_att.v[0] = saturate(u_att.v[0], -20, 20);
	u_att.v[1] = saturate(u_att.v[1], -20, 20);
	u_att.v[2] = saturate(u_att.v[2], -20, 20);

	//Output variable (the negatives below are due to the fact of using NED coordinates instead of NWU)
	Matrix<float, 4, 1> u;
	u.v[0] = u1;
	u.v[1] = u_att.v[0];
	u.v[2] = u_att.v[1];
	u.v[3] = u_att.v[2];

	return u;
}

/* Convert inputs u = [Thrust, torque_roll, torque_pitch, torque_yaw] into pwm values (T coordinates)
T coordinates assumes: u = [KT    KT    KT   KT   ] [pwm1^2]
                            0     -KT*L 0    KT*L ] [pwm2^2]
                            -KT*L 0     KT*L 0    ] [pwm3^2]
                            KM    -KM   KM   -KM  ] [pwm4^2]    
KT = thrust coefficient for the propellers
KM = moment coefficient for the propellers */
Matrix<float, 4, 1> u2pwmTshape(Matrix<float, 4, 1> u){
	//u = M.pwm^2 ==> pwm^2 = inv(M)*u

	Matrix<float, 4, 4> inv_M;
	inv_M.M[0][0] = 1 / (4 * KT); 
	inv_M.M[0][1] = 0;                  
	inv_M.M[0][2] = -1 / (2 * KT * RAD);
	inv_M.M[0][3] = 1 / (4 * KM);

	inv_M.M[1][0] = 1 / (4 * KT); 
	inv_M.M[1][1] = -1 / (2 * KT * RAD); 
	inv_M.M[1][2] = 0;
	inv_M.M[1][3] = -1 / (4 * KM);

	inv_M.M[2][0] = 1 / (4 * KT);
	inv_M.M[2][1] = 0;
	inv_M.M[2][2] = 1 / (2 * KT * RAD);
	inv_M.M[2][3] = 1 / (4 * KM);

	inv_M.M[3][0] = 1 / (4 * KT);
	inv_M.M[3][1] = 1 / (2 * KT * RAD);
	inv_M.M[3][2] = 0;
	inv_M.M[3][3] = -1 / (4 * KM);

	Matrix<float, 4, 1> pwm_squared = MultiplyMat4x4Vec4(inv_M, u);

	//Saturate pwm values before taking square root to avoid square root of negative numbers
	float mean_pwmSq = u.v[0] / (4 * KT);
	pwm_squared.v[0] = saturate(pwm_squared.v[0], 0, 2 * mean_pwmSq);
	pwm_squared.v[1] = saturate(pwm_squared.v[1], 0, 2 * mean_pwmSq);
	pwm_squared.v[2] = saturate(pwm_squared.v[2], 0, 2 * mean_pwmSq);
	pwm_squared.v[3] = saturate(pwm_squared.v[3], 0, 2 * mean_pwmSq);

	//Assign outputs (note that Mikicopter assign minimum value at 1000
	Matrix<float, 4, 1> pwm;
	pwm.v[0] = sqrt(pwm_squared.v[0])/1000;
	pwm.v[1] = sqrt(pwm_squared.v[1])/1000;
	pwm.v[2] = sqrt(pwm_squared.v[2])/1000;
	pwm.v[3] = sqrt(pwm_squared.v[3])/1000;
//PrintVec4(pwm,"pwm");
	return pwm;
}

/* Convert inputs u = [Thrust, torque_roll, torque_pitch, torque_yaw] into pwm values (X coordinates)
X coordinates assumes: u =[1    0      0       0 ] [KT    KT    KT   KT   ] [pwm1^2]
			  [0    cos(o) -sin(o) 0 ] [0     -KT*L 0    KT*L ] [pwm2^2]
			  [0    sin(o) cos(o)  0 ] [-KT*L 0     KT*L 0    ] [pwm3^2]
			  [0    0      0       1 ] [KM    -KM   KM   -KM  ] [pwm4^2]    
KT = thrust coefficient for the propellers
KM = moment coefficient for the propellers */
Matrix<float, 4, 1> u2pwmXshape(Matrix<float, 4, 1> u){
	double angle = PI / 4;	//Angle between T coordinates and X coordinates
	Matrix<float, 4, 4> Rot;				//Rotation matrix (note that the inverse of a rotation matrix is its transpose)
	//u.v[3] = 0;
	// Rot(.M[0][0]) = 1; Rot.M[0][1] = 0;           Rot.M[0][2] = 0;          Rot.M[0][3] = 0;
	// Rot.M[1][0] = 0; Rot.M[1][1] = cos(angle);  Rot.M[1][2] = sin(angle); Rot.M[1][3] = 0;
	// Rot.M[2][0] = 0; Rot.M[2][1] = -sin(angle); Rot.M[2][2] = cos(angle); Rot.M[2][3] = 0;
	// Rot.M[3][0] = 0; Rot.M[3][1] = 0;           Rot.M[3][2] = 0;          Rot.M[3][3] = 1;

	Rot << 1,           0,          0, 0,
		   0,  cos(angle), sin(angle), 0,
		   0, -sin(angle), cos(angle), 0,
		   0,           0,          0, 1
	//PrintVec4(u,"u");

	//Multiply inverse of rotation matrix by input u
	Matrix<float, 4, 1> uX = Rot*u;
	//PrintMat4x4(Rot);
	//PrintVec4(uX,"uX");
	return u2pwmTshape(uX);
}
