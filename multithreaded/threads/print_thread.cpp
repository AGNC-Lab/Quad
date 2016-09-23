
#include "threads/print_thread.h"

#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include <pthread.h>
#include "control/QuatRotEuler.h"
#include <cstdio>
#include "rosserial/qcontrol_defs/PVA.h"

#include "control/MatricesAndVectors.h"

#define TERMINATE 6
#define PI 3.1415

using namespace neosmart;


extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t IMU_Mutex;	//protect IMU data
extern pthread_mutex_t PCA_Mutex;
extern pthread_mutex_t Contr_Input_Mutex;
extern pthread_mutex_t attRef_Mutex;
extern pthread_mutex_t PVA_Vicon_Mutex;
extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent


extern Vec3 attRef;
extern Vec4 PCA_Data;
extern Vec3 IMU_Data_RPY;
extern Vec4 IMU_Data_Quat;
extern Vec3 IMU_Data_Accel;
extern Vec3 IMU_Data_AngVel;
extern Vec4 Contr_Input;
extern qcontrol_defs::PVA PVA_quadVicon;


extern int threadCount;	
extern int currentState;
extern neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_KeyESC;
extern neosmart_event_t e_Key6, e_Key7, e_Key8, e_Key9;



void *PrintTask(void *threadID){
	printf("PrintTask has started!\n");
	Vec3 localIMU_Data_RPY;
	Vec4 localIMU_Data_Quat;
	Vec3 localIMU_Data_Accel;
	Vec3 localIMU_Data_Vel;
	Vec3 localattRef;
	Vec3 localViconPos;
	Vec3 localViconVel;
	Vec4 localContr_Input;
	Vec4 localPCA_Data;
	float localMotor_Speed;
	int localCurrentState;

	while(1){
	    WaitForEvent(e_Timeout,100);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}

	    if(WaitForEvent(e_Key1, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
			localIMU_Data_RPY = IMU_Data_RPY;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_RPY = %-7f %-7f %-7f\n", localIMU_Data_RPY.v[0]*180/PI, localIMU_Data_RPY.v[1]*180/PI, localIMU_Data_RPY.v[2]*180/PI);
	    }
	    if(WaitForEvent(e_Key2, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
			localIMU_Data_Quat = IMU_Data_Quat;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Quat = %-7f %-7f %-7f %-7f\n", localIMU_Data_Quat.v[1], localIMU_Data_Quat.v[2], localIMU_Data_Quat.v[3], localIMU_Data_Quat.v[0]);
	    }
	    if(WaitForEvent(e_Key3, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
			localIMU_Data_Accel = IMU_Data_Accel;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Accel = %-7f %-7f %-7f\n", localIMU_Data_Accel.v[0], localIMU_Data_Accel.v[1], localIMU_Data_Accel.v[2]);
	    }
	    if(WaitForEvent(e_Key4, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&Contr_Input_Mutex);
			localContr_Input = Contr_Input;
		pthread_mutex_unlock(&Contr_Input_Mutex);
		pthread_mutex_lock(&PCA_Mutex);
			localPCA_Data = PCA_Data;
		pthread_mutex_unlock(&PCA_Mutex);
		PrintVec4(localContr_Input, "Contr_Input");
		PrintVec4(localPCA_Data, "PCA_Data");
	    }
	    if(WaitForEvent(e_Key5, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
			localIMU_Data_Vel = IMU_Data_AngVel;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Vel = %-7f %-7f %-7f\n", localIMU_Data_Vel.v[0], localIMU_Data_Vel.v[1], localIMU_Data_Vel.v[2]);
	    }
	    if(WaitForEvent(e_Key6, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&attRef_Mutex);
			localattRef = attRef;
		pthread_mutex_unlock(&attRef_Mutex);
		printf("Attitide Reference (RPY) = %-7f %-7f %-7f\n", localattRef.v[0], localattRef.v[1], localattRef.v[2]);
	    }
	    if(WaitForEvent(e_Key7, 0) == 0)
	    {	
		//WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&PVA_Vicon_Mutex);	
			localViconPos.v[0] = PVA_quadVicon.pos.position.x;
			localViconPos.v[1] = PVA_quadVicon.pos.position.y;
			localViconPos.v[2] = PVA_quadVicon.pos.position.z;
			localViconVel.v[0] = PVA_quadVicon.vel.linear.x;
			localViconVel.v[1] = PVA_quadVicon.vel.linear.y;
			localViconVel.v[2] = PVA_quadVicon.vel.linear.z;
	  	pthread_mutex_unlock(&PVA_Vicon_Mutex);	
		printf("Vicon Position = %-7f %-7f %-7f\n", localViconPos.v[0], localViconPos.v[1], localViconPos.v[2]);
		printf("Vicon Velocity = %-7f %-7f %-7f\n", localViconVel.v[0], localViconVel.v[1], localViconVel.v[2]);
	    }



		

	}
	

	printf("PrintTask stopping...\n");	
	threadCount -= 1;
	pthread_exit(NULL);
}
