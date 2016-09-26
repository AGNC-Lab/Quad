

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

extern neosmart_event_t e_KeyESC, e_Timeout, e_endInit;
extern neosmart_event_t e_buttonX, e_buttonY, e_buttonA, e_buttonB;
extern pthread_mutex_t stateMachine_Mutex;
extern int currentState;
extern int threadCount;

void *StateMachineTask(void *threadID);