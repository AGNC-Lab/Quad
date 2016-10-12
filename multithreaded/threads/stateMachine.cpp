#include "threads/stateMachine.h"

void *StateMachineTask(void *threadID){
	int localCurrentState;

	printf("Initializing system... \n");

	while(1){
		
		WaitForEvent(e_Timeout,50); //Run every 50ms

		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		if(localCurrentState == INITIALIZING){
			if(WaitForEvent(e_endInit,0) == 0){
				pthread_mutex_lock(&stateMachine_Mutex);
				currentState = INITIALIZED;
				pthread_mutex_unlock(&stateMachine_Mutex);
				printf("System Initialized! \n");
			}
		}

		//Check if ESC was pressed
		if(WaitForEvent(e_KeyESC,0) == 0){
			pthread_mutex_lock(&stateMachine_Mutex);
			currentState = TERMINATE;
			pthread_mutex_unlock(&stateMachine_Mutex);
			printf("System Terminating... \n\n");
			break;
		}

		//If the current state is not INITIALIZING, then any transition is acceptable
		if(currentState != INITIALIZING){
			if(WaitForEvent(e_buttonA,0) == 0){
				ResetEvent(e_buttonA);
				pthread_mutex_lock(&stateMachine_Mutex);
				currentState = MOTOR_MODE;
				pthread_mutex_unlock(&stateMachine_Mutex);
				printf("Motor Mode!\n");
			}
			if(WaitForEvent(e_buttonX,0) == 0){
				ResetEvent(e_buttonX);
				pthread_mutex_lock(&stateMachine_Mutex);
				currentState = POSITION_JOY_MODE;
				pthread_mutex_unlock(&stateMachine_Mutex);
				printf("Position Joy Mode!\n");
			}
			if(WaitForEvent(e_buttonY,0) == 0){
				ResetEvent(e_buttonY);
				pthread_mutex_lock(&stateMachine_Mutex);
				currentState = ATTITUDE_MODE;
				pthread_mutex_unlock(&stateMachine_Mutex);
				printf("Attitude Mode!\n");
			}
		}

		//check if source for yaw should be changed
		if(WaitForEvent(e_SwitchYawSource,0) == 0){
			ResetEvent(e_SwitchYawSource);
			pthread_mutex_lock(&YawSource_Mutex);
			if(YawSource == _IMU){
				YawSource = _VICON;
				pthread_mutex_unlock(&YawSource_Mutex);
				printf("Yaw data being read from Vicon!\n");
			}
			else{
				YawSource = _IMU;	
				pthread_mutex_unlock(&YawSource_Mutex);
				printf("Yaw data being read from IMU!\n");
			}
		}	
		
			
		switch(currentState)
		{
		    case INITIALIZING      : 
		    	//printf("INITIALIZING\n");      
		    	break;
		    case INITIALIZED       : 
		    	//printf("INITIALIZED\n");       
		    	break;
		    case MOTOR_MODE        : 
			    //printf("MOTOR_MODE\n");        
			    break;
		    case ATTITUDE_MODE     : 
		    	//printf("ATTITUDE_MODE\n");     
		    		break;
		    case POSITION_JOY_MODE : 
		    	///printf("POSITION_JOY_MODE\n"); 
		    		break;
		    case POSITION_ROS_MODE : 
		    	//printf("POSITION_ROS_MODE\n"); 
		    	break;
		    case TERMINATE		   : 
		    	//printf("POSITION_ROS_MODE\n"); 
		    	break;
		}	
		
	}

	printf("StateMachineTask stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}