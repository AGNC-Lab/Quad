
#ifndef _PID_3DOF_H_
#define _PID_3DOF_H_

struct PID_3DOF{
	Vec3 e_prop;
	Vec3 e_deriv;
	Vec3 e_integ;
	Vec3 K_p;
	Vec3 K_i;
	Vec3 K_d;
	Vec3 maxInteg;
};

//Sets initial errors to zero
void initializePID(PID_3DOF* PID);

//Sets integral error to zero
void resetIntegralErrorPID(PID_3DOF* PID);

//Update Kp, Ki and Kd in the PID
void updateControlParamPID(PID_3DOF* PID, Vec3 K_p, Vec3 K_i, Vec3 K_d, Vec3 maxInteg);

//Update all errors
void updateErrorPID(PID_3DOF* PID, Vec3 e_prop, Vec3 e_deriv, float dt);

//Calculate output of PID
Vec3 outputPID(PID_3DOF PID);

#endif