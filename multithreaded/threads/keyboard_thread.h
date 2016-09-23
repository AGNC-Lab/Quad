

#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "pevents/pevents.h"
#include <pthread.h>

#include <cstdio>


using namespace neosmart;

extern pthread_mutex_t stateMachine_Mutex;
extern neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_Key6, e_Key7, e_Key8, e_Key9, e_KeyESC;
extern neosmart_event_t e_Motor_Up, e_Motor_Down, e_Motor_Kill;
extern int threadCount;
extern int currentState;

extern void updatePar();
extern char getch();

#define TERMINATE 6


//Keyboard inputs
void *KeyboardTask(void *threadID);


#endif