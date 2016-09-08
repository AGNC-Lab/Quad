

#include "MatricesAndVectors.h"
#include "PID_3DOF.h"

//Sets initial errors to zero
void initializePID(PID_3DOF* PID){
	Vec3 zeros;
	zeros.v[0] = 0; 
	zeros.v[1] = 0; 
	zeros.v[2] = 0;

	PID->e_prop = zeros;
	PID->e_deriv = zeros;
	PID->e_integ = zeros;
}

//Sets integral error to zero
void resetIntegralErrorPID(PID_3DOF* PID){
	PID->e_integ.v[0] = 0;
	PID->e_integ.v[1] = 0;
	PID->e_integ.v[2] = 0;
}

//Update Kp, Ki and Kd in the PID
void updateControlParamPID(PID_3DOF* PID, Vec3 K_p, Vec3 K_i, Vec3 K_d, Vec3 maxInteg){
	PID->K_p = K_p;
	PID->K_d = K_d;
	PID->K_i = K_i;
	PID->maxInteg = maxInteg;
}

//Update all errors
void updateErrorPID(PID_3DOF* PID, Vec3 e_prop, Vec3 e_deriv, float dt){


	PID->e_prop = e_prop;
	PID->e_deriv = e_deriv;
	PID->e_integ = Add3x1Vec(PID->e_integ, ScaleVec3(e_prop, dt)); //e_integ = e_integ + e_prop*dt

	//Saturate integral error
	for (int i = 0; i < 3; i++){
		if (PID->e_integ.v[i] > PID->maxInteg.v[i]){
			PID->e_integ.v[i] = PID->maxInteg.v[i];
		}
		else if (PID->e_integ.v[i] < -PID->maxInteg.v[i]){
			PID->e_integ.v[i] = -PID->maxInteg.v[i];
		}
	}
}

//Calculate output of PID
Vec3 outputPID(PID_3DOF PID){
	Vec3 PID_out;
	for (int i = 0; i < 3; i++)
	{
		PID_out.v[i] = PID.e_prop.v[i] * PID.K_p.v[i] + PID.e_deriv.v[i] * PID.K_d.v[i] + PID.e_integ.v[i] * PID.K_i.v[i];
	}

	return PID_out;
}