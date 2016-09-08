
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <termios.h>
#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include "MPU6050/dmp.h"				
#include "PCA9685/pca9685.h"
#include "I2C/i2c.h"
#include "control/MatricesAndVectors.h"
#include "control/QuatRotEuler.h"
#include "control/MathFuncs.h"
#include "control/AttitudeControl.h"
#include "control/PID_3DOF.h"
#include "MPU6050/helper_3dmath.h"
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <sstream>
#include "rosserial/ros.h"
#include "rosserial/std_msgs/Float32MultiArray.h"
#include "rosserial/sensor_msgs/Joy.h"
#include "rosserial/geometry_msgs/TransformStamped.h"
#include "rosserial/geometry_msgs/PoseStamped.h"
#include "threads/keyboard_thread.h"
#include "threads/motor_ctr_thread.h"
#include "threads/control_thread.h"
//#include "threads/overo.h"
#include "threads/pca_thread.h"
#include "threads/mpu_thread.h"
#include "threads/print_thread.h"
//#include "rosserial/ros.h"

//using std::cin;
using namespace neosmart;
using namespace std;
using std::string;
using std::ostringstream;
//using namespace thread;

#define PI 3.1415
#define Motor_Speed_Max 20.0
#define Motor_Speed_Min 0.0
#define Motor_Speed_Inc 2.0
#define attRef_Inc 10.0*PI/180 //Increments for attitude reference in degrees
#define yaw_Inc PI/720

//Define system states
#define INITIALIZING 0
#define INITIALIZED 1
#define MOTOR_MODE 2
#define ATTITUDE_MODE 3
#define POSITION_JOY_MODE 4
#define POSITION_ROS_MODE 5
#define TERMINATE 6

int currentState = INITIALIZING;

ros::NodeHandle  _nh;  

neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_KeyESC;
neosmart_event_t e_Motor_Up, e_Motor_Down, e_Motor_Kill;
neosmart_event_t e_Roll_Pos, e_Roll_Neg, e_Pitch_pos, e_Pitch_Neg;
neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
neosmart_event_t e_IMU_trigger;
neosmart_event_t e_PCA_trigger;
neosmart_event_t e_Control_trigger;
neosmart_event_t e_endInit; //Event that indicates that initialization is done
pthread_mutex_t IMU_Mutex;	//protect IMU data
pthread_mutex_t PCA_Mutex;
pthread_mutex_t Motor_Speed_Mutex;
pthread_mutex_t attRef_Mutex;
pthread_mutex_t posRef_Mutex;
pthread_mutex_t Contr_Input_Mutex;
pthread_mutex_t PID_Mutex;
pthread_mutex_t stateMachine_Mutex;
float Motor_Speed = 0;
Vec3 IMU_Data_RPY;
Vec4 IMU_Data_Quat;
Vec3 IMU_Data_Accel;
Vec3 IMU_Data_Vel;
Vec3 attRef;
Vec3 posRef;
Vec4 Contr_Input;
Vec4 PCA_Data;
PID_3DOF PID_angVel; 	//Angular velocity PID
PID_3DOF PID_att;		//Attitude PID
//float* dmp_ypr;
int threadCount = 0;		//Counts active threads
float yaw_ctr_pos, yaw_ctr_neg;

I2C i2c('3'); 

void *StateMachineTask(void *threadID){

	printf("Initializing system... \n");

	while(1){
		
		WaitForEvent(e_Timeout,50); //Run every 50ms

		if(currentState == INITIALIZING){
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

void handle_mp_joy_msg(const sensor_msgs::Joy& msg){
	pthread_mutex_lock(&attRef_Mutex);	
	attRef.v[0] = -msg.axes[3]*PI/6; //roll
	attRef.v[1] = msg.axes[4]*PI/6; //pitch
	yaw_ctr_pos = msg.axes[2];
	yaw_ctr_neg = msg.axes[5];
	//PrintVec3(attRef, "attRef");
	pthread_mutex_unlock(&attRef_Mutex);

	pthread_mutex_lock(&Motor_Speed_Mutex);
	Motor_Speed = msg.axes[1] * 20;
	//printf("Motor_Speed = %-7f \n", Motor_Speed);
	pthread_mutex_unlock(&Motor_Speed_Mutex);


}

void handle_pose_tform_msg(const geometry_msgs::TransformStamped& msg){

	pthread_mutex_lock(&posRef_Mutex);	

 	posRef.v[0] = msg.transform.translation.x;
  	posRef.v[1] = -msg.transform.translation.y;
  	posRef.v[2] = -msg.transform.translation.z;

  // _v_est = (_p_est - _p_est_prev)/(t - ts_last_pose);
  // std::cout << _v_est(0) << "\t" << _v_est(1) << "\t" << _v_est(2) << "\n";

  //_q_est.w() = msg.transform.rotation.w;
  //_q_est.x() = msg.transform.rotation.x;
  //_q_est.y() = -msg.transform.rotation.y;
  //_q_est.z() = -msg.transform.rotation.z;

  //_psi_est = atan2(2.0*(_q_est.x()*_q_est.y()+_q_est.w()*_q_est.z()), 
                   //1.0-2.0*(_q_est.y()*_q_est.y()+_q_est.z()*_q_est.z()));

  // _p_est_prev = _p_est;
  	PrintVec3(posRef, "posRef");

  	pthread_mutex_unlock(&posRef_Mutex);	

  }

//change red light to green after restart
/*void set_pwm_mux_to_ap() {
  // This function is intended to toggle the PWM mux on the MikiPilot AP Board (R2) to autopilot
  // mode (green LED). Once done, the channel is turned back into an input to avoid contention 
  // if something is driving this GPIO. It is assumed that JP2 is populated (see page 5 of 
  // MikiPilot AP Board (R2) Schematics).
  mc_overo.gpio_set_direction(overo::GPIO_147,overo::OUT);
  mc_overo.gpio_set_value(overo::GPIO_147,overo::HIGH);
  mc_overo.gpio_set_value(overo::GPIO_147,overo::LOW);
  mc_overo.gpio_set_direction(overo::GPIO_147,overo::IN);
} */

//Helper functions to parse the config file
void split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
}


vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}


void updatePar() {
	Vec3 KP_RPY, KD_RPY, KI_RPY, maxInteg_RPY;
	Vec3 KP_w, KD_w, KI_w, maxInteg_w;

    string line;
    vector<string> line_vec;
    ifstream myfile ("config.txt");
    if (myfile.is_open()) {
		while (getline (myfile ,line)) {
		    line_vec = split(line, ' ');
		    if (line_vec[0] == "KP_R") {
	            KP_RPY.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_P") {
				KP_RPY.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_Y") {
				KP_RPY.v[2] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_R") {
	            	KD_RPY.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_P") {
				KD_RPY.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_Y") {
				KD_RPY.v[2] = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "KI_R") {
	            KI_RPY.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_P") {
				KI_RPY.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_Y") {
				KI_RPY.v[2] = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "maxInteg_R") {
	            maxInteg_RPY.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_P") {
				maxInteg_RPY.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_Y") {
				maxInteg_RPY.v[2] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wx") {
				KP_w.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wy") {
				KP_w.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wz") {
		        KP_w.v[2]  = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wx") {
				KD_w.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wy") {
				KD_w.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wz") {
		        KD_w.v[2]   = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wx") {
				KI_w.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wy") {
				KI_w.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wz") {
		        KI_w.v[2]  = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "maxInteg_wx") {
	            maxInteg_w.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_wy") {
				maxInteg_w.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_wz") {
				maxInteg_w.v[2] = atof(line_vec[2].c_str());
		    }
		}
		myfile.close();
		pthread_mutex_lock(&PID_Mutex);
		updateControlParamPID(&PID_att, KP_RPY, KI_RPY, KD_RPY, maxInteg_RPY);
		updateControlParamPID(&PID_angVel, KP_w, KI_w, KD_w, maxInteg_w);
		pthread_mutex_unlock(&PID_Mutex);
    }
    
    else {
		printf("Unable to open file"); 
    }

	printf("Done updating control parameters\n");
}


char getch() { //This function allows to capture char without needing to press 'ENTER'
	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 0;
	old.c_cc[VTIME] = 5;
	if (tcsetattr(0, TCSANOW, &old) < 0)
		    perror("tcsetattr ICANON");
	if (read(0, &buf, 1) < 0)
		    perror ("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0)
		    perror ("tcsetattr ~ICANON");
	return (buf);
}



int main(int argc, char *argv[])
{
	//handles for threads
	pthread_t keyboardThread;		//Handle for keyboard thread
	pthread_t Motor_ControlThread;
	pthread_t StateMachine_Thread;
	pthread_t Control_Thread;               //handle for Control thread
	pthread_t Control_TimerThread;          //handle for Control Timer thread
	pthread_t IMU_Thread;			//handle for IMU thread
	pthread_t IMU_TimerThread;		//handle for IMU Timer thread
	pthread_t PCA_Thread;			//handle for PCA thread
	pthread_t PCA_TimerThread;		//handle for PCA Timer thread
	pthread_t MAG_Thread;			//handle for MAG thread
	pthread_t PrintThread;			//handle for Printing thread
	long IDthreadKeyboard, IDthreadIMU, IDthreadMAG; //Stores ID for threads
	int ReturnCode;

	//ros::init(argc, argv, "talker");

	char *rosSrvrIp; // = "192.168.1.54";

	printf("%d\n",argc);
	if(argc != 2) {
	  _nh.logfatal("Incorrect number of aruguments\n");
	  return -1;
	}
	else {
	  rosSrvrIp = argv[1];
	}
	  
	_nh.initNode(rosSrvrIp);

	//ros joy subscriber
	ros::Subscriber<sensor_msgs::Joy> sub_mp_joy("/joy", handle_mp_joy_msg);
	_nh.subscribe(sub_mp_joy);	

  	ros::Subscriber<geometry_msgs::TransformStamped> sub_tform("/vicon/mk1/mk1", handle_pose_tform_msg);
  	_nh.subscribe(sub_tform);

  	//ros publisher
  	std_msgs::Float32MultiArray rpyimu;
  	ros::Publisher imurpy_pub("IMU RPY", &rpyimu);

  	float Roll_pitch_yaw[] = {IMU_Data_RPY.v[0],IMU_Data_RPY.v[1],IMU_Data_RPY.v[2]};

  	_nh.advertise(imurpy_pub);

  	rpyimu.data = Roll_pitch_yaw;
    imurpy_pub.publish( &rpyimu);
    _nh.spinOnce();
    //delay(1000);


	//std_msgs::Float32 imu_rpy;
	
	///imu_rpy.data = {};
	
	//Publish array
	//imu_publish.publish(imu_rpy);



	//Set initial reference
	attRef.v[0] = 0; attRef.v[1] = 0; attRef.v[2] = 0;

	//Start events
	e_endInit = CreateEvent(false,false);		//auto-reset event
	e_Key1 = CreateEvent(true,false); 			//manual-reset event
	e_Key2 = CreateEvent(true,false); 			//manual-reset event
	e_Key3 = CreateEvent(true,false);           //manual-reset event
	e_Key4 = CreateEvent(true,false);           //manual-reset event
	e_Key5 = CreateEvent(true,false);           //manual-reset event
	e_Motor_Up = CreateEvent(false,false);      //auto-reset event
	e_Motor_Down = CreateEvent(false,false);    //auto-reset event
	e_Motor_Kill = CreateEvent(false,false);    //auto-reset event
	e_Roll_Pos = CreateEvent(false,false);    	//auto-reset event
	e_Roll_Neg = CreateEvent(false,false);    	//auto-reset event
	e_Pitch_pos = CreateEvent(false,false);    	//auto-reset event
	e_Pitch_Neg = CreateEvent(false,false);    	//auto-reset event

	e_KeyESC = CreateEvent(true,false); 		//abort manual-reset event
	e_Timeout = CreateEvent(false,false);		//timeout event (always false)
	e_IMU_trigger = CreateEvent(false,false); 	//auto-reset event
	e_PCA_trigger = CreateEvent(false,false); 	//auto-reset event
	e_Control_trigger = CreateEvent(false, false);  //auto-reset event

	//Create mutexes
	pthread_mutex_init(&IMU_Mutex, NULL);
	pthread_mutex_init(&PCA_Mutex, NULL);
	pthread_mutex_init(&Motor_Speed_Mutex, NULL);
	pthread_mutex_init(&Contr_Input_Mutex, NULL);
	pthread_mutex_init(&PID_Mutex, NULL);
	pthread_mutex_init(&attRef_Mutex, NULL);
	pthread_mutex_init(&posRef_Mutex, NULL);
	pthread_mutex_init(&stateMachine_Mutex, NULL);

	//Start keyboard task
	if (ReturnCode = pthread_create(&keyboardThread, NULL, KeyboardTask, NULL)){
		printf("Start KeyboardTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start state StateMachineTask
	if (ReturnCode = pthread_create(&StateMachine_Thread, NULL, StateMachineTask, NULL)){
		printf("Start KeyboardTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;


	//Start Control task
	if (ReturnCode = pthread_create(&Control_Thread, NULL, Control_Task, NULL)){
		printf("Start Control_Task failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
			threadCount += 1;

	//Start Control timer task
	if (ReturnCode = pthread_create(&Motor_ControlThread, NULL, Motor_Control, NULL)){
		printf("Start Motor_ContorlTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;


	//Start Control timer task
	if (ReturnCode = pthread_create(&Control_TimerThread, NULL, Control_Timer, NULL)){
		printf("Start Control_TimerTask failed; return code from pthread_create() is %d\n", ReturnCode);
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
	//Start PCA task
	if (ReturnCode = pthread_create(&PCA_Thread, NULL, PCA_Task, NULL)){
		printf("Start PCA_Task failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
			threadCount += 1;

	//Start PCA timer task
	if (ReturnCode = pthread_create(&PCA_TimerThread, NULL, PCA_Timer, NULL)){
		printf("Start PCA_TimerTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;


	//Start printing task
	if (ReturnCode = pthread_create(&PrintThread, NULL, PrintTask, NULL)){
		printf("Start PrintThread failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//This has to be removed from here and put somewhere that indicates configurating the vechicle has been done
	SetEvent(e_endInit);

	//Check every 500ms if all threads have returned
	while(1){
		WaitForEvent(e_Timeout,500);
		if(threadCount == 0)
			break;
	}

	printf("Closing program... \n");

	//Destroy events
	DestroyEvent(e_Key1);
	DestroyEvent(e_Key2);
	DestroyEvent(e_Key3);
	DestroyEvent(e_Key4);
	DestroyEvent(e_Key5);
	DestroyEvent(e_Motor_Up);
	DestroyEvent(e_Motor_Down);
	DestroyEvent(e_Motor_Kill);
	DestroyEvent(e_Roll_Pos);
	DestroyEvent(e_Roll_Neg);
	DestroyEvent(e_Pitch_pos);
	DestroyEvent(e_Pitch_Neg);
	DestroyEvent(e_KeyESC);
	DestroyEvent(e_Timeout);
	DestroyEvent(e_IMU_trigger);
	DestroyEvent(e_PCA_trigger);
	DestroyEvent(e_Control_trigger);
	DestroyEvent(e_endInit);

	//Destroy mutexes
	pthread_mutex_destroy(&IMU_Mutex);
	pthread_mutex_destroy(&PCA_Mutex);
	pthread_mutex_destroy(&Motor_Speed_Mutex);
	pthread_mutex_destroy(&Contr_Input_Mutex);
	pthread_mutex_destroy(&PID_Mutex);
	pthread_mutex_destroy(&attRef_Mutex);
	pthread_mutex_destroy(&posRef_Mutex);
	pthread_mutex_destroy(&stateMachine_Mutex);

   /* Last thing that main() should do */
   pthread_exit(NULL);
}
