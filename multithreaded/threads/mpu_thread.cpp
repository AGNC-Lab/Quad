
#include "mpu_thread.h"
#include "MPU6050/dmp.h"

void *IMU_Timer(void *threadID){

	printf("IMU_Timer has started!\n");
	int SamplingTime = 5;	//Sampling time in milliseconds
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
	Vec4 IMU_localData_QuatNoYaw;
	Vec4 IMU_Quat_Yaw; //Quaternion with only yaw
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

		//Take off yaw from quaternion
		IMU_Quat_Yaw.v[0] = cos(IMU_localData_RPY.v[2]/2);
		IMU_Quat_Yaw.v[1] = 0;
		IMU_Quat_Yaw.v[2] = 0;
		IMU_Quat_Yaw.v[3] = -sin(IMU_localData_RPY.v[2]/2);
		IMU_localData_QuatNoYaw = QuaternionProduct(IMU_Quat_Yaw, IMU_localData_Quat);
		
		pthread_mutex_lock(&IMU_Mutex);
		IMU_Data_RPY = IMU_localData_RPY;
		IMU_Data_Quat = IMU_localData_Quat;
		IMU_Data_QuatNoYaw = IMU_localData_QuatNoYaw;
		IMU_Data_Accel.v[0] = (double)aa.x;
		IMU_Data_Accel.v[1] = (double)aa.y;
		IMU_Data_Accel.v[2] = (double)aa.z;
		IMU_Data_AngVel.v[1] = ((double)gx/131 - Vel_Cal_X)*PI/180;
		IMU_Data_AngVel.v[0] = ((double)gy/131 - Vel_Cal_Y)*PI/180;
		IMU_Data_AngVel.v[2] = -((double)gz/131 - Vel_Cal_Z)*PI/180;
		pthread_mutex_unlock(&IMU_Mutex);
	}
	
	printf("IMU_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}
