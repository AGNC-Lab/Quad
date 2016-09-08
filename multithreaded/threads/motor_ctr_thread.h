

#ifndef _MOTOR_CTR_H
#define _MOTOR_CTR_H



#include "pevents/pevents.h"
#include <pthread.h>

using namespace neosmart;


#define Motor_Speed_Max 20.0
#define Motor_Speed_Min 0.0
#define Motor_Speed_Inc 2.0

#define PI 3.1415

#define attRef_Inc 10.0*PI/180 //Increments for attitude reference in degrees

extern float Motor_Speed;
extern neosmart_event_t e_Control_trigger;
extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
extern pthread_mutex_t attRef_Mutex;
extern pthread_mutex_t Motor_Speed_Mutex;
extern Vec3 attRef;


void *Motor_Control(void *threadID){

	printf("Motor_Control has started!\n");
	int SamplingTime = 10;	//Sampling time in milliseconds
	float localMotor_Speed;
	int localCurrentState;

        //Initialize here
	
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
		
		if (WaitForEvent(e_Motor_Up, 0) == 0) {
		    //motor up
		    pthread_mutex_lock(&Motor_Speed_Mutex);
		    if (Motor_Speed < Motor_Speed_Max)
				Motor_Speed += Motor_Speed_Inc;
			localMotor_Speed = Motor_Speed;
		    pthread_mutex_unlock(&Motor_Speed_Mutex);
		    printf("Motor Speed: %f\n", localMotor_Speed);
		}
		else if (WaitForEvent(e_Motor_Down, 0) == 0) {
		    //motor down
		    pthread_mutex_lock(&Motor_Speed_Mutex);
		    if (Motor_Speed > Motor_Speed_Min)
				Motor_Speed -= Motor_Speed_Inc;
			localMotor_Speed = Motor_Speed;
		    pthread_mutex_unlock(&Motor_Speed_Mutex);
		    printf("Motor Speed: %f\n", localMotor_Speed);
		}
		else if (WaitForEvent(e_Motor_Kill, 0) == 0) {
		    //motor kill
		    pthread_mutex_lock(&Motor_Speed_Mutex);
		    Motor_Speed = 0;
		    localMotor_Speed = Motor_Speed;
		    pthread_mutex_unlock(&Motor_Speed_Mutex);
		    printf("Motor Speed: %f\n", localMotor_Speed);
		}
		if (WaitForEvent(e_Roll_Pos, 0) == 0) {
		    //motor up
		    pthread_mutex_lock(&attRef_Mutex);
			attRef.v[0] += attRef_Inc;
		    pthread_mutex_unlock(&attRef_Mutex);
		    printf("Roll Reference: %f\n", attRef.v[0]);
		}
		if (WaitForEvent(e_Roll_Neg, 0) == 0) {
		    //motor up
		    pthread_mutex_lock(&attRef_Mutex);
			attRef.v[0] -= attRef_Inc;
		    pthread_mutex_unlock(&attRef_Mutex);
		    printf("Roll Reference: %f\n", attRef.v[0]);
		}
		if (WaitForEvent(e_Pitch_pos, 0) == 0) {
		    //motor up
		    pthread_mutex_lock(&attRef_Mutex);
			attRef.v[1] += attRef_Inc;
		    pthread_mutex_unlock(&attRef_Mutex);
		    printf("Roll Reference: %f\n", attRef.v[0]);
		}
		if (WaitForEvent(e_Pitch_Neg, 0) == 0) {
		    //motor up
		    pthread_mutex_lock(&attRef_Mutex);
			attRef.v[1] -= attRef_Inc;
		    pthread_mutex_unlock(&attRef_Mutex);
		    printf("Roll Reference: %f\n", attRef.v[0]);
		}
	}
	
	printf("Motor_Control stopping...\n");
	//Shutdown here
	threadCount -= 1;
	pthread_exit(NULL);
}





#endif