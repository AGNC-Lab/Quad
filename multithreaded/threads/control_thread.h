#ifndef _H_CONTROL_THREAD_
#define _H_CONTROL_THREAD_


#include "control/MatricesAndVectors.h"
#include "control/QuatRotEuler.h"
#include "control/MathFuncs.h"
#include "control/AttitudeControl.h"
#include "control/PID_3DOF.h"
#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include <pthread.h>
#include <cstdio>
#include "threads/stateMachine.h"
//#include "rosserial/ros.h"
#include "rosserial/qcontrol_defs/PVA.h"

#define PI 3.1415


using namespace neosmart;

extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
extern neosmart_event_t e_AttControl_trigger, e_PosControl_trigger;

extern pthread_mutex_t attRefJoy_Mutex;
extern pthread_mutex_t ThrustJoy_Mutex;
extern pthread_mutex_t attRefPosControl_Mutex;
extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t IMU_Mutex;	//protect IMU data
extern pthread_mutex_t PCA_Mutex;
extern pthread_mutex_t PID_Mutex;
extern pthread_mutex_t PVA_Vicon_Mutex;
extern pthread_mutex_t posRefJoy_Mutex;
extern pthread_mutex_t ThrustPosControl_Mutex;


extern Vec3 attRefJoy;
extern Vec4 Contr_Input;
extern Vec4 PCA_Data;
extern Vec3 IMU_Data_RPY;
extern Vec4 IMU_Data_Quat;
extern Vec3 IMU_Data_AngVel;
extern PID_3DOF PID_angVel, PID_att, PID_pos; 	//APIDs structures
extern qcontrol_defs::PVA PVA_quadVicon, PVA_RefJoy;
extern float ThrustPosControl;
extern Mat3x3 Rdes_PosControl;


extern pthread_mutex_t Contr_Input_Mutex;
extern float ThrustJoy;

extern int threadCount;	
extern int currentState;


//Timer for attitude controller
void *AttControl_Timer(void *threadID);

//Attitude controller
void *AttControl_Task(void *threadID);

//Timer for position controller
void *PosControl_Timer(void *threadID);

//Position controller
void *PosControl_Task(void *threadID);

#endif