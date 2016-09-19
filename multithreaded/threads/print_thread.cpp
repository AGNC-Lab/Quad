
#include "threads/print_thread.h"

#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include <pthread.h>
#include "control/QuatRotEuler.h"
#include <cstdio>

#include "control/MatricesAndVectors.h"

#define TERMINATE 6
#define PI 3.1415

using namespace neosmart;


extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t IMU_Mutex;	//protect IMU data
extern pthread_mutex_t PCA_Mutex;
extern pthread_mutex_t Contr_Input_Mutex;
extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent



extern Vec4 PCA_Data;
extern Vec3 IMU_Data_RPY;
extern Vec4 IMU_Data_Quat;
extern Vec3 IMU_Data_Accel;
extern Vec3 IMU_Data_Vel;
extern Vec4 Contr_Input;


extern int threadCount;	
extern int currentState;
extern neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_KeyESC;



void *PrintTask(void *threadID){
	printf("PrintTask has started!\n");
	Vec3 localIMU_Data_RPY;
	Vec4 localIMU_Data_Quat;
	Vec3 localIMU_Data_Accel;
	Vec3 localIMU_Data_Vel;
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
	    {	//If Key1 was pressed, wait 100ms for every new print
		WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
		localIMU_Data_RPY = IMU_Data_RPY;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_RPY = %-7f %-7f %-7f\n", localIMU_Data_RPY.v[0]*180/PI, localIMU_Data_RPY.v[1]*180/PI, localIMU_Data_RPY.v[2]*180/PI);
	    }
	    if(WaitForEvent(e_Key2, 0) == 0)
	    {	//If Key2 was pressed, wait 100ms for every new print
		WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
		localIMU_Data_Quat = IMU_Data_Quat;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Quat = %-7f %-7f %-7f %-7f\n", localIMU_Data_Quat.v[1], localIMU_Data_Quat.v[2], localIMU_Data_Quat.v[3], localIMU_Data_Quat.v[0]);
	    }
	    if(WaitForEvent(e_Key3, 0) == 0)
	    {	//If Key1 was pressed, wait 100ms for every new print
		WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
		localIMU_Data_Accel = IMU_Data_Accel;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Accel = %-7f %-7f %-7f\n", localIMU_Data_Accel.v[0], localIMU_Data_Accel.v[1], localIMU_Data_Accel.v[2]);
	    }
	    if(WaitForEvent(e_Key4, 0) == 0)
	    {	//If Key1 was pressed, wait 100ms for every new print
		WaitForEvent(e_Timeout,5);
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
	    {	//If Key1 was pressed, wait 100ms for every new print
		WaitForEvent(e_Timeout,5);
		pthread_mutex_lock(&IMU_Mutex);
		localIMU_Data_Vel = IMU_Data_Vel;
		pthread_mutex_unlock(&IMU_Mutex);
		printf("IMU_Data_Vel = %-7f %-7f %-7f\n", localIMU_Data_Vel.v[0], localIMU_Data_Vel.v[1], localIMU_Data_Vel.v[2]);
	    }
		

	}
	

	printf("PrintTask stopping...\n");	
	threadCount -= 1;
	pthread_exit(NULL);
}
