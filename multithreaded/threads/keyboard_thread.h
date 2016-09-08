

#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_


#include "pevents/pevents.h"
#include <pthread.h>


using namespace neosmart;

extern pthread_mutex_t stateMachine_Mutex;
extern neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_KeyESC;
extern neosmart_event_t e_Motor_Up, e_Motor_Down, e_Motor_Kill;
extern neosmart_event_t e_Roll_Pos, e_Roll_Neg, e_Pitch_pos, e_Pitch_Neg;
extern int threadCount;
extern int currentState;

extern void updatePar();
extern char getch();

#define TERMINATE 6

void *KeyboardTask(void *threadID)
{
	char ch;	//Char for storing keystroke
	printf("KeyboardTask has started!\n");
	int Key1 = 0, Key2 = 0, Key3 = 0, Key4 = 0, Key5 = 0;		//Values for key states (0 or 1)
	int localCurrentState;

	while(1){
		//Check system state
		pthread_mutex_lock(&stateMachine_Mutex);
		localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}


		ch = getch();
		if (ch == '1'){
			Key1 = (Key1 + 1) % 2;
			if (Key1 == 1)
			    SetEvent(e_Key1);
			else
			    ResetEvent(e_Key1);
			printf("Key1 State: %d\n",Key1);
		}
		else if (ch == '2'){
			Key2 = (Key2 + 1) % 2;
			if (Key2 == 1)
			    SetEvent(e_Key2);
			else
			    ResetEvent(e_Key2);
			printf("Key2 State: %d\n",Key2);
		}
		else if (ch == '3'){
			Key3 = (Key3 + 1) % 2;
			if (Key3 == 1)
			    SetEvent(e_Key3);
			else
			    ResetEvent(e_Key3);
			printf("Key3 State: %d\n",Key3);
		}
		else if (ch == '4'){
			Key4 = (Key4 + 1) % 2;
			if (Key4 == 1)
			    SetEvent(e_Key4);
			else
			    ResetEvent(e_Key4);
			printf("Key4 State: %d\n",Key4);
		}
		else if (ch == '5'){
			Key5 = (Key5 + 1) % 2;
			if (Key5 == 1)
			    SetEvent(e_Key5);
			else
			    ResetEvent(e_Key5);
			printf("Key5 State: %d\n",Key5);
		}
		else if (ch == '0'){
		    updatePar();
		    printf("Updated parameters! \n");
		}
		else if (ch == 'w') { //the 'w' key
		    //motor up
		    SetEvent(e_Motor_Up);
		}
		else if (ch == 's') { //the 's' key
		    //motor down
		    SetEvent(e_Motor_Down);
		}
		else if (ch == 'k') { //the 'k' key
		    //motor kill
		    SetEvent(e_Motor_Kill);
		}
		else if (ch == 't') { //the 't' key
		    //motor kill
		    SetEvent(e_Pitch_pos);
		}
		else if (ch == 'g') { //the 'g' key
		    //motor kill
		    SetEvent(e_Pitch_Neg);
		}
		else if (ch == 'h') { //the 'h' key
		    //motor kill
		    SetEvent(e_Roll_Pos);
		}
		else if (ch == 'f') { //the 'f' key
		    //motor kill
		    SetEvent(e_Roll_Neg);
		}
		else if (ch == 27){	//ESC character
			SetEvent(e_KeyESC);
			printf("ESC has been activated!\n\n");
			//break;
		}

	}
	
	printf("KeyboardTask stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}


#endif