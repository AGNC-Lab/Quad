
#include "mpu_thread.h"

#include "MPU6050/dmp.h"	
#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include <pthread.h>
#include "control/QuatRotEuler.h"

#include "control/MatricesAndVectors.h"


#define TERMINATE 6
#define PI 3.1415


using namespace neosmart;

extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t IMU_Mutex;	//protect IMU data

extern Vec3 IMU_Data_RPY;
extern Vec4 IMU_Data_Quat;
extern Vec3 IMU_Data_Accel;
extern Vec3 IMU_Data_Vel;

extern neosmart_event_t e_IMU_trigger;
extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent

extern int threadCount;	
extern int currentState;



void *IMU_Timer(void *threadID){

	printf("IMU_Timer has started!\n");
	int SamplingTime = 3;	//Sampling time in milliseconds
	int localCurrentState;

	//setup();

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

		SetEvent(e_IMU_trigger);

	}
	
	printf("IMU_Timer stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}


void *IMU_Task(void *threadID){
	printf("IMU_Task has started!\n");

	Vec3 IMU_localData_RPY;
	Vec4 IMU_localData_Quat;
	Vec4 IMU_Quat_Conversion; //When sitting on the ground, measured attitude indicates 180Deg Roll (need to unroll)
	IMU_Quat_Conversion.v[0] = cos(PI/2);
	IMU_Quat_Conversion.v[1] = sin(PI/2);
	IMU_Quat_Conversion.v[2] = 0;
	IMU_Quat_Conversion.v[3] = 0;

	int calibrate = 0;
	int cal_amount = 100;
	float Vel_Cal_X = 0;
	float Vel_Cal_Y = 0;
	float Vel_Cal_Z = 0;
	int localCurrentState;

	setup();

	while (calibrate < cal_amount) {
	    getDMP();
	    Vel_Cal_X +=  (float)gx/131;
	    Vel_Cal_Y +=  (float)gy/131;
	    Vel_Cal_Z +=  (float)gz/131;
	    calibrate++;
	}
	Vel_Cal_X /= cal_amount;
	Vel_Cal_Z /= cal_amount;
	Vel_Cal_Y /= cal_amount;


	while(1){
		
		WaitForEvent(e_IMU_trigger,500);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
		localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}
		
		getDMP();
		IMU_localData_Quat.v[0] = q.w;
		IMU_localData_Quat.v[1] = q.y;
		IMU_localData_Quat.v[2] = q.x;
		IMU_localData_Quat.v[3] = -q.z; //Negative seemed necessary

		
		IMU_localData_Quat = QuaternionProduct(IMU_Quat_Conversion, IMU_localData_Quat); //Unroll vehicle		
		IMU_localData_RPY = Quat2RPY(IMU_localData_Quat);
		//Do whatever we need here below
		pthread_mutex_lock(&IMU_Mutex);
		IMU_Data_RPY.v[0] = IMU_localData_RPY.v[0];//*180/PI;
		IMU_Data_RPY.v[1] = IMU_localData_RPY.v[1];//*180/PI;
		IMU_Data_RPY.v[2] = IMU_localData_RPY.v[2];//*180/PI;
		IMU_Data_Quat = IMU_localData_Quat;
		IMU_Data_Accel.v[0] = (float)aa.x;
		IMU_Data_Accel.v[1] = (float)aa.y;
		IMU_Data_Accel.v[2] = (float)aa.z;
		IMU_Data_Vel.v[1] = ((float)gx/131 - Vel_Cal_X)*PI/180;
		IMU_Data_Vel.v[0] = ((float)gy/131 - Vel_Cal_Y)*PI/180;
		IMU_Data_Vel.v[2] = -((float)gz/131 - Vel_Cal_Z)*PI/180;
		pthread_mutex_unlock(&IMU_Mutex);
	}
	
	printf("IMU_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}
