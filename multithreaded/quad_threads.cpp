
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <termios.h>
#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
//#include "MPU6050/dmp.h"				
//#include "PCA9685/pca9685.h"
#include "I2C/i2c.h"
#include "control/MatricesAndVectors.h"
#include "control/QuatRotEuler.h"
//#include "control/MathFuncs.h"
#include "control/AttitudeControl.h"
#include "control/PID_3DOF.h"
//#include "MPU6050/helper_3dmath.h"
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <sstream>
#include "rosserial/ros.h"
//#include "rosserial/std_msgs/Float32MultiArray.h"
#include "rosserial/sensor_msgs/Joy.h"
#include "rosserial/geometry_msgs/TransformStamped.h"
#include "rosserial/qcontrol_defs/PVA.h"
#include "rosserial/qcontrol_defs/PVA.h"
#include "threads/keyboard_thread.h"
#include "threads/control_thread.h"
#include "threads/pca_thread.h"
#include "threads/print_thread.h"
#include "threads/mpu_thread.h"
//#include "threads/stateMachine.h"
//#include "threads/overo.h"
#include "kalman.h"
#include <cmath> 
using namespace neosmart;
using namespace std;
using std::string;
using std::ostringstream;
//using namespace thread;

#define PI 3.1415
#define Motor_Speed_Max 20.0
#define Motor_Speed_Min 0.0
#define Motor_Speed_Inc 2.0
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

//Ros handle and IP variable
ros::NodeHandle  _nh;  
char *rosSrvrIp;

qcontrol_defs::PVA PVA_quadVicon;
qcontrol_defs::PVA PVA_quadKalman;

neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_Key6, e_Key7, e_Key8, e_Key9, e_KeyESC;
neosmart_event_t e_Motor_Up, e_Motor_Down, e_Motor_Kill;
neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
neosmart_event_t e_IMU_trigger;
neosmart_event_t e_PCA_trigger;
neosmart_event_t e_Control_trigger;
neosmart_event_t e_endInit; //Event that indicates that initialization is done

pthread_mutex_t IMU_Mutex;	//protect IMU data
pthread_mutex_t PCA_Mutex;
pthread_mutex_t Motor_Speed_Mutex;
pthread_mutex_t attRef_Mutex;
pthread_mutex_t Contr_Input_Mutex;
pthread_mutex_t PID_Mutex;
pthread_mutex_t stateMachine_Mutex;
pthread_mutex_t PVA_Vicon_Mutex;
pthread_mutex_t PVA_Kalman_Mutex;

float Motor_Speed = 0;

Vec3 IMU_Data_RPY;
Vec4 IMU_Data_Quat;
Vec3 IMU_Data_Accel;
Vec3 IMU_Data_AngVel;
Vec3 attRef;
Vec4 Contr_Input;
Vec4 PCA_Data;

PID_3DOF PID_angVel; 	//Angular velocity PID
PID_3DOF PID_att;		//Attitude PID
//float* dmp_ypr;

int threadCount = 0;		//Counts active threads
float yaw_ctr_pos, yaw_ctr_neg;

ofstream kalman_v;
ofstream vicon_p;

I2C i2c('3'); 

void handle_mp_joy_msg(const sensor_msgs::Joy& msg){
	pthread_mutex_lock(&attRef_Mutex);	
	attRef.v[0] = -msg.axes[3]*PI/6; //roll
	attRef.v[1] = msg.axes[4]*PI/6; //pitch
	yaw_ctr_pos = msg.axes[2];
	yaw_ctr_neg = msg.axes[5];
	if (yaw_ctr_neg < 0) {
		attRef.v[2] -= yaw_Inc;
	}								//yaw
	if (yaw_ctr_pos < 0) {
		attRef.v[2] += yaw_Inc;
	}
	pthread_mutex_unlock(&attRef_Mutex);
	//PrintVec3(attRef, "attRef");

	pthread_mutex_lock(&Motor_Speed_Mutex);
	Motor_Speed = msg.axes[1] * 20;
	pthread_mutex_unlock(&Motor_Speed_Mutex);

}

void handle_Vicon(const geometry_msgs::TransformStamped& msg){

 	pthread_mutex_lock(&PVA_Vicon_Mutex);	

		PVA_quadVicon.pos.position.x = msg.transform.translation.x;
		PVA_quadVicon.pos.position.y = msg.transform.translation.y;
		PVA_quadVicon.pos.position.z = msg.transform.translation.z;

		PVA_quadVicon.t = msg.header.stamp;

		PVA_quadVicon.pos.orientation.w = msg.transform.rotation.w;
		PVA_quadVicon.pos.orientation.x = msg.transform.rotation.x;
		PVA_quadVicon.pos.orientation.y = msg.transform.rotation.y;
		PVA_quadVicon.pos.orientation.z = msg.transform.rotation.z;

  	pthread_mutex_unlock(&PVA_Vicon_Mutex);

  	// kalman_v << kalman_state(3,0) << "," << kalman_state(4,0) << "," << kalman_state(5,0) << "\n";
  	// vicon_p << z(0,0) << "," << z(1,0) << "," << z(2,0) << "\n";

  }

void *rosSpinTask(void *threadID){
	int SamplingTime = 5;	//Sampling time in milliseconds
	int localCurrentState;

	_nh.initNode(rosSrvrIp);

	//ros joy subscriber
	ros::Subscriber<sensor_msgs::Joy> sub_mp_joy("/joy", handle_mp_joy_msg);
	_nh.subscribe(sub_mp_joy);	

  	ros::Subscriber<geometry_msgs::TransformStamped> sub_tform("/vicon/Niki/Niki", handle_Vicon);
  	_nh.subscribe(sub_tform);

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


		_nh.spinOnce();
  	}

	printf("rosSpinTask stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}


void *Kalman_Task(void *threadID){
	int SamplingTime = 5;	//Sampling time in milliseconds
	int localCurrentState;

	ros::Time current_time = ros::Time(0,0);
	ros::Time prev_time = ros::Time(0,0);

	Eigen::Matrix<float, 6, 1> kalman_state = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 3, 1> z;   // measurement vector

    kalman_init(); //Initialize kalman filter

  	while(1){

		WaitForEvent(e_Timeout,SamplingTime);

		pthread_mutex_lock(&stateMachine_Mutex);
		
		localCurrentState = currentState;
		
		pthread_mutex_unlock(&stateMachine_Mutex);

		kalman_propagate();

		//Check system state
		pthread_mutex_lock(&PVA_Vicon_Mutex);

		z <<  PVA_quadVicon.pos.position.x,
		      PVA_quadVicon.pos.position.y,
		      PVA_quadVicon.pos.position.z;

		//current_time = PVA_quadVicon.t;

		pthread_mutex_unlock(&PVA_Vicon_Mutex);

		if (prev_time.toSec() != current_time.toSec())
		{
			kalman_state = kalman_estimate(z);	
		}

		prev_time = current_time;

		pthread_mutex_lock(&PVA_Kalman_Mutex);

		PVA_quadKalman.pos.position.x = kalman_state(0,0);
		PVA_quadKalman.pos.position.y = kalman_state(1,0);
		PVA_quadKalman.pos.position.z = kalman_state(2,0);
		PVA_quadKalman.vel.linear.x = kalman_state(3,0);
		PVA_quadKalman.vel.linear.y = kalman_state(4,0);
		PVA_quadKalman.vel.linear.z = kalman_state(5,0);

		pthread_mutex_unlock(&PVA_Kalman_Mutex);
	  	kalman_v << kalman_state(3,0) << "," << kalman_state(4,0) << "," << kalman_state(5,0) << "\n";
	  	vicon_p << z(0,0) << "," << z(1,0) << "," << z(2,0) << "\n";
		//check if system should be terminated
		if(localCurrentState == TERMINATE){
			break;
		}
  	}
  		
	printf("Kalman_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}


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
    ifstream myfile ("/home/root/config.txt");
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
	}
	
	printf("Motor_Control stopping...\n");
	//Shutdown here
	threadCount -= 1;
	pthread_exit(NULL);
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
	pthread_t Kalman_Thread;          //handle for Control Timer thread
	pthread_t rosSpinThread;
	//long IDthreadKeyboard, IDthreadIMU, IDthreadMAG; //Stores ID for threads
	int ReturnCode;
	

  	kalman_v.open ("kalman_velocity.txt");
  	vicon_p.open ("vicon_position.txt");

	printf("%d\n",argc);
	if(argc != 2) {
	  _nh.logfatal("Incorrect number of arguments\n");
	  return -1;
	}
	else {
	  rosSrvrIp = argv[1];
	}
	 
  	// //ros publisher
  	// std_msgs::Float32MultiArray rpyimu;
  	// ros::Publisher imurpy_pub("IMU RPY", &rpyimu);

  	// float Roll_pitch_yaw[] = {IMU_Data_RPY.v[0],IMU_Data_RPY.v[1],IMU_Data_RPY.v[2]};

  	// _nh.advertise(imurpy_pub);

  	// rpyimu.data = Roll_pitch_yaw;
    //  imurpy_pub.publish( &rpyimu);

	//std_msgs::Float32 imu_rpy;
	
	///imu_rpy.data = {};
	
	//Publish array
	//imu_publish.publish(imu_rpy);

	//Set initial reference
	attRef.v[0] = 0; attRef.v[1] = 0; attRef.v[2] = 0;

	//Start events
	e_endInit = CreateEvent(false,false);		//event that signalizes end of initialization

	//Events for printing information
	e_Key1 = CreateEvent(true,false); 			//manual-reset event
	e_Key2 = CreateEvent(true,false); 			//manual-reset event
	e_Key3 = CreateEvent(true,false);           //manual-reset event
	e_Key4 = CreateEvent(true,false);           //manual-reset event
	e_Key5 = CreateEvent(true,false);           //manual-reset event
	e_Key6 = CreateEvent(true,false);           //manual-reset event
	e_Key7 = CreateEvent(true,false);           //manual-reset event
	e_Key8 = CreateEvent(true,false);           //manual-reset event
	e_Key9 = CreateEvent(true,false);           //manual-reset event

	//Events for changing thrust values
	e_Motor_Up = CreateEvent(false,false);      //auto-reset event
	e_Motor_Down = CreateEvent(false,false);    //auto-reset event
	e_Motor_Kill = CreateEvent(false,false);    //auto-reset event

	//Termination 
	e_KeyESC = CreateEvent(true,false); 		//abort manual-reset event

	//Events that trigget threads
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
	pthread_mutex_init(&stateMachine_Mutex, NULL);
	pthread_mutex_init(&PVA_Vicon_Mutex, NULL);
	pthread_mutex_init(&PVA_Kalman_Mutex, NULL);

	//Start keyboard task
	if (ReturnCode = pthread_create(&keyboardThread, NULL, KeyboardTask, NULL)){
		printf("Start KeyboardTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	// Start rosSpin task
	if (ReturnCode = pthread_create(&rosSpinThread, NULL, rosSpinTask, NULL)){
		printf("Start rosSpin failed; return code from pthread_create() is %d\n", ReturnCode);
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

	if (ReturnCode = pthread_create(&Kalman_Thread, NULL, Kalman_Task, NULL)){
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
	DestroyEvent(e_Key6);
	DestroyEvent(e_Key7);
	DestroyEvent(e_Key8);
	DestroyEvent(e_Key9);
	DestroyEvent(e_Motor_Up);
	DestroyEvent(e_Motor_Down);
	DestroyEvent(e_Motor_Kill);
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
	pthread_mutex_destroy(&stateMachine_Mutex);
	pthread_mutex_destroy(&PVA_Vicon_Mutex);
	pthread_mutex_destroy(&PVA_Kalman_Mutex);

   /* Last thing that main() should do */
   pthread_exit(NULL);
   vicon_p.close();
   kalman_v.close();	
}
