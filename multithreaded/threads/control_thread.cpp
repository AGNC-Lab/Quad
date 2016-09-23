

#include "control_thread.h"
#include "control/MatricesAndVectors.h"
#include "control/QuatRotEuler.h"
#include "control/MathFuncs.h"
#include "control/AttitudeControl.h"
#include "control/PID_3DOF.h"
#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include <pthread.h>
#include <cstdio>
//#include "rosserial/ros.h"




#define yaw_Inc PI/720
#define TERMINATE 6
#define PI 3.1415


using namespace neosmart;


//extern ros::NodeHandle _nh;

extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
extern neosmart_event_t e_Control_trigger;

extern pthread_mutex_t attRef_Mutex;
extern pthread_mutex_t Motor_Speed_Mutex;
extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t IMU_Mutex;	//protect IMU data
extern pthread_mutex_t PCA_Mutex;
extern pthread_mutex_t PID_Mutex;


extern Vec3 attRef;
extern Vec3 posRef;
extern Vec4 PCA_Data;
extern Vec4 Contr_Input;

extern Vec3 IMU_Data_RPY;
extern Vec4 IMU_Data_Quat;
extern Vec3 IMU_Data_Accel;
extern Vec3 IMU_Data_AngVel;
extern PID_3DOF PID_angVel; 	//Angular velocity PID
extern PID_3DOF PID_att;		//Attitude PID


extern pthread_mutex_t Contr_Input_Mutex;


//extern float yaw_ctr_pos, yaw_ctr_neg;
extern float Motor_Speed;

extern int threadCount;	
extern int currentState;

extern void updatePar();


extern Vec4 PCA_Data;

void *Control_Timer(void *threadID){

	printf("Control_Timer has started!\n");
	int SamplingTime = 3;	//Sampling time in milliseconds
	int localCurrentState;

	while(1){
		WaitForEvent(e_Timeout,SamplingTime);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}

		SetEvent(e_Control_trigger);
	}
	
	printf("Control_Timer stopping...\n");
	//Shutdown here
	threadCount -= 1;
	pthread_exit(NULL);
}


void *Control_Task(void *threadID){
	printf("Control_Task has started!\n");

	//Initialize PIDs (sets initial errors to zero)
	pthread_mutex_lock(&PID_Mutex);
	initializePID(&PID_att);
	initializePID(&PID_angVel);
	pthread_mutex_unlock(&PID_Mutex);
	float dt = 0.003; 			//Sampling time
	float takeOffThrust = 6; 	//Minimum thrust to take off
	Vec3 localAttRef;

	//Vectors of zeros
	Vec3 zeros;
	zeros.v[0] = 0; zeros.v[1] = 0; zeros.v[2] = 0; 

	Vec4 IMU_localData_Quat;
	Vec3 IMU_localData_Vel;
	Vec3 IMU_localData_RPY;
	Vec3 inputTorque;
	Vec4 PCA_localData;
	float localMotor_Speed;
	int localCurrentState;


	Vec3 error_att;
	Vec3 error_att_vel;

	Vec3 wDes;
	Mat3x3 Rdes;
	Mat3x3 Rbw;
	

	//Mat3x3 Rdes = RPY2Rot(0,0,0);

	updatePar();

	//PrintMat3x3(Concatenate3Vec3_2_Mat3x3(PID_att.K_p, PID_att.K_d, PID_att.K_i));
	//PrintMat3x3(Concatenate3Vec3_2_Mat3x3(PID_angVel.K_p, PID_angVel.K_d, PID_angVel.K_i));

	while(1){

		WaitForEvent(e_Control_trigger,500);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}

		//Throttle
	    pthread_mutex_lock(&Motor_Speed_Mutex);
			localMotor_Speed = Motor_Speed;
		pthread_mutex_unlock(&Motor_Speed_Mutex);

		//Grab attitude estimation
		pthread_mutex_lock(&IMU_Mutex);
			IMU_localData_Quat = IMU_Data_Quat;
			IMU_localData_Vel = IMU_Data_AngVel;
			IMU_localData_RPY = IMU_Data_RPY;
		pthread_mutex_unlock(&IMU_Mutex);

		//Grab attitude reference (set yaw to measured yaw if quad isnt flying)
		pthread_mutex_lock(&attRef_Mutex);	
			if (localMotor_Speed <= 0) {
		    	attRef.v[2] = IMU_localData_RPY.v[2];
		    }
		    localAttRef = attRef;
	    pthread_mutex_unlock(&attRef_Mutex);


	    Rdes = RPY2Rot(localAttRef.v[0], localAttRef.v[1], localAttRef.v[2]);
	    Rbw = Quat2rot(IMU_localData_Quat);

	    //Calculate attitude error
	    error_att = AttitudeErrorVector(Rbw, Rdes);
	    //PrintVec3(error_att, "error_att"); 
		//error_att = Subtract3x1Vec(localAttRef, IMU_localData_RPY);
		//PrintVec3(error_att,"error_att");

		//Update PID
		pthread_mutex_lock(&PID_Mutex);
			if(!isNanVec3(error_att)){
				updateErrorPID(&PID_att, error_att, zeros, dt);
			}

			//Dont integrate integrator if not in minimum thrust
			if (localMotor_Speed < takeOffThrust){
				resetIntegralErrorPID(&PID_att);
			}
			
			//Reference for inner loop (angular velocity control)
			wDes = outputPID(PID_att);

			//Calculate angular velocity error and update PID
			error_att_vel = Subtract3x1Vec(wDes, IMU_localData_Vel);
			updateErrorPID(&PID_angVel, error_att_vel, zeros, dt);

			if (localMotor_Speed < takeOffThrust){
				resetIntegralErrorPID(&PID_angVel);
			}

			inputTorque = outputPID(PID_angVel);
		pthread_mutex_unlock(&PID_Mutex);

		//Distribute power to motors
		pthread_mutex_lock(&Contr_Input_Mutex);
			Contr_Input.v[0] = localMotor_Speed;
			Contr_Input.v[1] = inputTorque.v[0];
			Contr_Input.v[2] = inputTorque.v[1];
			Contr_Input.v[3] = inputTorque.v[2];
			PCA_localData = u2pwmXshape(Contr_Input);
		pthread_mutex_unlock(&Contr_Input_Mutex);

		//Send motor commands
		pthread_mutex_lock(&PCA_Mutex);
			PCA_Data = PCA_localData;
		pthread_mutex_unlock(&PCA_Mutex);


	}
	
	printf("Control_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}