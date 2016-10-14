#include <pthread.h>

#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include "I2C/i2c.h"
#include "control/MatricesAndVectors.h"
#include "control/QuatRotEuler.h"
#include "control/AttitudeControl.h"
#include "control/PID_3DOF.h"

#include "threads/keyboard_thread.h"
#include "threads/mpu_thread.h"
// #include "overo/overo.h"

#include "kalman.h"
#include <string>


using namespace neosmart;
using namespace std;


#define PI 3.1415

//Events and mutexes
neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_Key6, e_Key7, e_Key8, e_Key9, e_KeyESC, e_KeyEnter;
neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
neosmart_event_t e_IMU_trigger;
pthread_mutex_t IMU_Mutex;	//protect IMU data

//Global variables
float ThrustJoy = 0;
float ThrustPosControl = 0;
Vec3 IMU_Data_RPY, IMU_Data_RPY_ViconYaw;
Vec3 IMU_Data_Accel, IMU_Data_AngVel;
Vec4 IMU_Data_Quat, IMU_Data_QuatNoYaw, IMU_Data_Quat_ViconYaw;
Vec3 attRefJoy;
Mat3x3 Rdes_PosControl;
Vec4 Contr_Input;
Vec4 PCA_Data;
PID_3DOF PID_angVel, PID_att, PID_pos; 	//Control PIDs
int threadCount = 0;		//Counts active threads

// ofstream kalman_v;
// ofstream vicon_p;

I2C i2c('3'); 


int main(int argc, char *argv[])
{
	//handles for threads
	pthread_t keyboardThread;		//Handle for keyboard thread
	pthread_t IMU_Thread;			//handle for IMU thread
	pthread_t IMU_TimerThread;		//handle for IMU Timer thread
	pthread_t CalibThread;
	//long IDthreadKeyboard, IDthreadIMU, IDthreadMAG; //Stores ID for threads
	int ReturnCode;

	//Keyboard Event for starting calibration procedure
	e_KeyEnter = CreateEvent(false,false);		//auto-reset event

	//Keyboard event for execution termination
	e_KeyESC = CreateEvent(true,false); 		//abort manual-reset event

	//Events that trigget threads
	e_Timeout = CreateEvent(false,false);		//timeout event (always false)
	e_IMU_trigger = CreateEvent(false,false); 	//auto-reset event

	//Create mutexes
	pthread_mutex_init(&IMU_Mutex, NULL);

	// //Load PID parameters from file
	// pthread_mutex_lock(&PID_Mutex);
	// 	updatePar(&PID_att, &PID_angVel, &PID_pos);
	// pthread_mutex_unlock(&PID_Mutex);
	//Start keyboard task
	if (ReturnCode = pthread_create(&keyboardThread, NULL, KeyboardTask, NULL)){
		printf("Start KeyboardTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start IMU task
	if (ReturnCode = pthread_create(&IMU_Thread, NULL, IMU_Task, NULL)){
		printf("Start IMU_Task failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start IMU timer task
	if (ReturnCode = pthread_create(&IMU_TimerThread, NULL, IMU_Timer, NULL)){
		printf("Start IMU_TimerTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start Calibration task
	if (ReturnCode = pthread_create(&CalibThread, NULL, Calib_Sensors, NULL)){
		printf("Start Calib_Sensors failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Check every 500ms if all threads have returned
	while(1){
		WaitForEvent(e_Timeout,500);
		if(threadCount == 0)
			break;
	}

	printf("Closing program... \n");

	//Destroy events
	DestroyEvent(e_KeyESC);
	DestroyEvent(e_KeyEnter);
	DestroyEvent(e_Timeout);
	DestroyEvent(e_IMU_trigger);

	//Destroy mutexes
	pthread_mutex_destroy(&IMU_Mutex);

   /* Last thing that main() should do */
	// vicon_p.close();
	// kalman_v.close(); 
    pthread_exit(NULL);
}
