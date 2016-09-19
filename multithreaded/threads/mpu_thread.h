#ifndef _H_MPU_THREAD_
#define _H_MPU_THREAD_


//Gather MPU data
void *IMU_Task(void *threadID);

//Timer
void *IMU_Timer(void *threadID);

#endif