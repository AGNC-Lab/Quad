
#ifndef _PID_3DOF_H_
#define _PID_3DOF_H_

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "MatricesAndVectors.h"
#include <sstream>
#include <string>

using std::string;
using std::ostringstream;
using namespace std;

struct PID_3DOF{
	Vec3 e_prop;
	Vec3 e_deriv;
	Vec3 e_integ;
	Vec3 feedForward;
	Vec3 K_p;
	Vec3 K_i;
	Vec3 K_d;
	Vec3 maxInteg;
};

extern pthread_mutex_t PID_Mutex;
// extern PID_3DOF PID_angVel, PID_att, PID_pos; 	//Control PIDs

//Sets initial errors to zero
void initializePID(PID_3DOF* PID);

//Sets integral error to zero
void resetIntegralErrorPID(PID_3DOF* PID);

//Update Kp, Ki and Kd in the PID
void updateControlParamPID(PID_3DOF* PID, Vec3 K_p, Vec3 K_i, Vec3 K_d, Vec3 maxInteg);

//Update all errors
void updateErrorPID(PID_3DOF* PID, Vec3 feedForward, Vec3 e_prop, Vec3 e_deriv, float dt);

//Calculate output of PID
Vec3 outputPID(PID_3DOF PID);

//Update PID parameters from file
void updatePar(PID_3DOF *PID_att, PID_3DOF *PID_angVel, PID_3DOF *PID_pos);

#endif