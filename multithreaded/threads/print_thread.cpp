
#include "threads/print_thread.h"


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
		pthread_mutex_lock(&attRefJoy_Mutex);
			localattRef = attRefJoy;
		pthread_mutex_unlock(&attRefJoy_Mutex);
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
	    if(WaitForEvent(e_Key8, 0) == 0)
	    {
			// pthread_mutex_lock(&PID_Mutex);
			// 	PrintVec3(PID_att.e_integ, "e_integ Att");
			// pthread_mutex_unlock(&PID_Mutex);
	     }


		

	}
	

	printf("PrintTask stopping...\n");	
	threadCount -= 1;
	pthread_exit(NULL);
}
