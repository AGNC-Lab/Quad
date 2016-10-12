

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)

using namespace neosmart;

//Define system states
#define INITIALIZING 0
#define INITIALIZED 1
#define MOTOR_MODE 2
#define ATTITUDE_MODE 3
#define POSITION_JOY_MODE 4
#define POSITION_ROS_MODE 5
#define TERMINATE 6

//Define states for Yaw sources
#define _IMU 0
#define _VICON 1

extern neosmart_event_t e_KeyESC, e_Timeout, e_endInit;
extern neosmart_event_t e_buttonX, e_buttonY, e_buttonA, e_buttonB;
extern neosmart_event_t e_SwitchYawSource;
extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t YawSource_Mutex;
extern int currentState;
extern int YawSource;
extern int threadCount;

void *StateMachineTask(void *threadID);