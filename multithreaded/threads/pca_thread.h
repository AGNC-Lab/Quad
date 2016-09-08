#include "PCA9685/pca9685.h"
#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include <pthread.h>

#include "control/MatricesAndVectors.h"



extern pthread_mutex_t PCA_Mutex;
extern pthread_mutex_t stateMachine_Mutex;
extern Vec4 PCA_Data;


extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
extern neosmart_event_t e_PCA_trigger;
extern I2C i2c;

pca9685 pca(&i2c);

void *PCA_Task(void *threadID);


void *PCA_Timer(void *threadID){

	printf("PCA_Timer has started!\n");
	int SamplingTime = 3;	//Sampling time in milliseconds
	int localCurrentState;

	pca.initialize();
	float freq = 500;

	if(pca.pwmFreq(freq)!=1)
	    printf("ERROR: setting pwm frequency\n");
	else
	    printf("Frequency set to %f Hz.\n", freq);

	
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

		SetEvent(e_PCA_trigger);
	}
	
	printf("PCA_Timer stopping...\n");
	pca.resetDev();
	threadCount -= 1;
	pthread_exit(NULL);
}

void *PCA_Task(void *threadID){
	printf("PCA_Task has started!\n");
	Vec4 localPCA_Data;
	int localCurrentState;

	while(1){
		
		WaitForEvent(e_PCA_trigger,500);

		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
		localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}

		//Grab motor commands
		pthread_mutex_lock(&PCA_Mutex);
		localPCA_Data = PCA_Data;
		pthread_mutex_unlock(&PCA_Mutex);

		//localPCA_Data.v[0] = 4090;
		//localPCA_Data.v[1] = 4090;
		//localPCA_Data.v[2] = 4090;
		//localPCA_Data.v[3] = 4090;
		//PrintVec4(localPCA_Data, "PCA");

		//Send motor pulses
		pca.pwmPulse(0, 0, localPCA_Data.v[1]*2048 + 2048);
        pca.pwmPulse(1, 0, localPCA_Data.v[2]*2048 + 2048);
        pca.pwmPulse(2, 0, localPCA_Data.v[3]*2048 + 2048);
		pca.pwmPulse(3, 0, localPCA_Data.v[0]*2048 + 2048);


	}
	
	printf("PCA_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}
