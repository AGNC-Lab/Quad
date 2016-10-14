#include <pthread.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <iostream>
//#include <fstream>
//#include <cmath> 
//#include <math.h>

#include "pevents/pevents.h"	//Includes event handling (https://github.com/NeoSmart/PEvents)
#include "I2C/i2c.h"
#include "control/MatricesAndVectors.h"
#include "control/QuatRotEuler.h"
#include "control/AttitudeControl.h"
#include "control/PID_3DOF.h"

#include "rosserial/ros.h"
#include "rosserial/geometry_msgs/Wrench.h"
#include "rosserial/qcontrol_defs/PVA.h"

#include "threads/keyboard_thread.h"
#include "threads/control_thread.h"
#include "threads/pca_thread.h"
#include "threads/print_thread.h"
#include "threads/mpu_thread.h"
#include "threads/stateMachine.h"
#include "threads/Ros_threads.h"
// #include "overo/overo.h"

#include "kalman.h"


using namespace neosmart;
using namespace std;


#define PI 3.1415

int currentState = INITIALIZING;
int YawSource = _IMU;

//Ros handle and IP variable
ros::NodeHandle  _nh;  
char *rosSrvrIp;
qcontrol_defs::PVA PVA_quadVicon;
qcontrol_defs::PVA PVA_quadKalman;
qcontrol_defs::PVA PVA_RefJoy;

//Events and mutexes
neosmart_event_t e_Key1, e_Key2, e_Key3, e_Key4, e_Key5, e_Key6, e_Key7, e_Key8, e_Key9, e_KeyESC;
neosmart_event_t e_buttonX, e_buttonY, e_buttonA, e_buttonB;
neosmart_event_t e_Motor_Up, e_Motor_Down, e_Motor_Kill;
neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
neosmart_event_t e_SwitchYawSource;
neosmart_event_t e_IMU_trigger;
neosmart_event_t e_PCA_trigger;
neosmart_event_t e_AttControl_trigger, e_PosControl_trigger;
neosmart_event_t e_endInit; //Event that indicates that initialization is done
pthread_mutex_t IMU_Mutex;	//protect IMU data
pthread_mutex_t PCA_Mutex;
pthread_mutex_t ThrustJoy_Mutex;
pthread_mutex_t ThrustPosControl_Mutex;
pthread_mutex_t attRefJoy_Mutex, posRefJoy_Mutex;
pthread_mutex_t attRefPosControl_Mutex;
pthread_mutex_t Contr_Input_Mutex;
pthread_mutex_t PID_Mutex;
pthread_mutex_t stateMachine_Mutex;
pthread_mutex_t PVA_Vicon_Mutex;
pthread_mutex_t PVA_Kalman_Mutex;
pthread_mutex_t ROS_Mutex;
pthread_mutex_t YawSource_Mutex;

//Global variables
float ThrustJoy = 0;
float ThrustPosControl = 0;
Vec3 IMU_Data_RPY, IMU_Data_RPY_ViconYaw;
Vec3 IMU_Data_Accel, IMU_Data_AngVel;
Vec4 IMU_Data_Quat, IMU_Data_QuatNoYaw, IMU_Data_Quat_ViconYaw;
Vec3 attRefJoy;
Mat3x3 Rdes_PosControl;
Vec4 Contr_Input;
Vec4 PCA_Data;
PID_3DOF PID_angVel, PID_att, PID_pos; 	//Control PIDs
int threadCount = 0;		//Counts active threads

// ofstream kalman_v;
// ofstream vicon_p;

I2C i2c('3'); 


void *rosSpinTask(void *threadID){
	int SamplingTime = 10;	//Sampling time in milliseconds
	int localCurrentState;
	printf("rosSpinTask has started!\n");

	//ros joy subscriber
	ros::Subscriber<sensor_msgs::Joy> sub_mp_joy("/joy", handle_mp_joy_msg);
	_nh.subscribe(sub_mp_joy);	

  	ros::Subscriber<geometry_msgs::TransformStamped> sub_tform("/vicon/Quad7/Quad7", handle_Vicon);
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

		//Trigger subscribers
	  	pthread_mutex_lock(&ROS_Mutex);
		    _nh.spinOnce();
	    pthread_mutex_unlock(&ROS_Mutex);
		
  	}

  	printf("rosSpinTask stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}

void *rosPublisherTask(void *threadID){
	printf("rosPublisherTask has started!\n");
	int SamplingTime = 50;	//Sampling time in milliseconds
	int localCurrentState, localYawSource;
	Vec3 RPY;

	//ros publisher
  	geometry_msgs::Point RPY_IMU;
  	geometry_msgs::Point RPY_Ref;
  	geometry_msgs::Wrench controlInputs;
  	qcontrol_defs::PVA PVA_EstimatedPVA;
  	qcontrol_defs::PVA PVA_Vicon;

  	ros::Publisher imurpy_pub("Quad_RPY", &RPY_IMU);
  	ros::Publisher Refrpy_pub("Quad_Ref", &RPY_Ref);
  	ros::Publisher imuWrench_pub("Quad_Inputs", &controlInputs);
  	ros::Publisher PvaEst_pub("Quad_EstimatedPVA", &PVA_EstimatedPVA);
  	ros::Publisher PvaVicon_pub("Quad_ViconPVA", &PVA_Vicon);
  	_nh.advertise(imurpy_pub);
  	_nh.advertise(Refrpy_pub);
  	_nh.advertise(imuWrench_pub);
	_nh.advertise(PvaEst_pub);
	_nh.advertise(PvaVicon_pub);

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

		//Check if yaw source should come from IMU of Vicon
		pthread_mutex_lock(&YawSource_Mutex);
			localYawSource = YawSource;
		pthread_mutex_unlock(&YawSource_Mutex);

		//Get IMU roll pitch yaw data
		if (localYawSource == _IMU){
		  	pthread_mutex_lock(&IMU_Mutex);
			  	RPY_IMU.x = IMU_Data_RPY.v[0];
			  	RPY_IMU.y = IMU_Data_RPY.v[1];
			  	RPY_IMU.z = IMU_Data_RPY.v[2];
		  	pthread_mutex_unlock(&IMU_Mutex);
		}
		else{ //Get IMU roll and pitch; yaw comes from vicon
			pthread_mutex_lock(&PVA_Vicon_Mutex);
				RPY_IMU.x = IMU_Data_RPY_ViconYaw.v[0];
			  	RPY_IMU.y = IMU_Data_RPY_ViconYaw.v[1];
			  	RPY_IMU.z = IMU_Data_RPY_ViconYaw.v[2];
			pthread_mutex_unlock(&PVA_Vicon_Mutex);
		}

		//Get position/velocity estimations
		pthread_mutex_lock(&PVA_Kalman_Mutex);
			PVA_EstimatedPVA = PVA_quadKalman;
		pthread_mutex_unlock(&PVA_Kalman_Mutex);

		//Get position and attitude from vicon
	 	pthread_mutex_lock(&PVA_Vicon_Mutex);
	 		PVA_Vicon = PVA_quadVicon;
	  	pthread_mutex_unlock(&PVA_Vicon_Mutex);

		//Get attitude references
		if(localCurrentState == ATTITUDE_MODE){
			pthread_mutex_lock(&attRefJoy_Mutex);
				RPY_Ref.x = attRefJoy.v[0];
				RPY_Ref.y = attRefJoy.v[1];
				RPY_Ref.z = attRefJoy.v[2];
			pthread_mutex_unlock(&attRefJoy_Mutex);
		}
		else if(localCurrentState == POSITION_JOY_MODE){
			pthread_mutex_lock(&attRefPosControl_Mutex);
				RPY = Quat2RPY(Rot2quat(Rdes_PosControl));
			pthread_mutex_unlock(&attRefPosControl_Mutex);
			RPY_Ref.x = RPY.v[0];
			RPY_Ref.y = RPY.v[1];
			RPY_Ref.z = RPY.v[2];
		}
		else{
			RPY_Ref.x = 0;
			RPY_Ref.y = 0;
			RPY_Ref.z = 0;
		}

	  	//Get inputs for quadcopter
		pthread_mutex_lock(&Contr_Input_Mutex);
			controlInputs.force.x = 0;
			controlInputs.force.y = 0;
			controlInputs.force.z =  Contr_Input.v[0];
			controlInputs.torque.x = Contr_Input.v[1];
			controlInputs.torque.y = Contr_Input.v[2];
			controlInputs.torque.z = Contr_Input.v[3];
		pthread_mutex_unlock(&Contr_Input_Mutex);

	  	//Publish everything
	  	pthread_mutex_lock(&ROS_Mutex);
		    imurpy_pub.publish( &RPY_IMU);
		    Refrpy_pub.publish( &RPY_Ref);
		    imuWrench_pub.publish( &controlInputs);
		    PvaEst_pub.publish( &PVA_EstimatedPVA);
		    PvaVicon_pub.publish(&PVA_Vicon);
	    pthread_mutex_unlock(&ROS_Mutex);

  	}

  	printf("rosPublisherTask stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}

// //change red light to green after restart
// void set_pwm_mux_to_ap() {
//   // This function is intended to toggle the PWM mux on the MikiPilot AP Board (R2) to autopilot
//   // mode (green LED). Once done, the channel is turned back into an input to avoid contention 
//   // if something is driving this GPIO. It is assumed that JP2 is populated (see page 5 of 
//   // MikiPilot AP Board (R2) Schematics).
//   mc_overo.gpio_set_direction(overo::GPIO_147,overo::OUT);
//   mc_overo.gpio_set_value(overo::GPIO_147,overo::HIGH);
//   mc_overo.gpio_set_value(overo::GPIO_147,overo::LOW);
//   mc_overo.gpio_set_direction(overo::GPIO_147,overo::IN);
// } 

void *Kalman_Task(void *threadID){
	int SamplingTime = 10;	//Sampling time in milliseconds
	int localCurrentState;

	ros::Time current_time = ros::Time(0,0);
	ros::Time prev_time = ros::Time(0,0);

	Eigen::Matrix<float, 6, 1> kalman_state = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 3, 1> z;   // measurement vector
    qcontrol_defs::PVA localPVA_quadVicon;

    kalman_init(); //Initialize kalman filter

  	while(1){

		WaitForEvent(e_Timeout,SamplingTime);

		pthread_mutex_lock(&stateMachine_Mutex);
			localCurrentState = currentState;
		pthread_mutex_unlock(&stateMachine_Mutex);

		if(localCurrentState == TERMINATE){
			break;
		}

		kalman_state = kalman_propagate();

		//Check system state
		pthread_mutex_lock(&PVA_Vicon_Mutex);
			localPVA_quadVicon = PVA_quadVicon;
		pthread_mutex_unlock(&PVA_Vicon_Mutex);

		//Measurement vector z
		z <<  localPVA_quadVicon.pos.position.x,
		      localPVA_quadVicon.pos.position.y,
		      localPVA_quadVicon.pos.position.z;

		//Check if there is a new measurement. If so, update estimation
		current_time = localPVA_quadVicon.t;
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
			PVA_quadKalman.pos.orientation = localPVA_quadVicon.pos.orientation;
		pthread_mutex_unlock(&PVA_Kalman_Mutex);
	  	// kalman_v << kalman_state(3,0) << "," << kalman_state(4,0) << "," << kalman_state(5,0) << "\n";
	  	// vicon_p << z(0,0) << "," << z(1,0) << "," << z(2,0) << "\n";
		//check if system should be terminated

  	}
  		
	printf("Kalman_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
	//handles for threads
	pthread_t keyboardThread;		//Handle for keyboard thread
	pthread_t Motor_ControlThread;
	pthread_t StateMachine_Thread;
	pthread_t AttControl_Thread;     
	pthread_t AttControl_TimerThread;
	pthread_t PosControl_Thread;     
	pthread_t PosControl_TimerThread;
	pthread_t IMU_Thread;			//handle for IMU thread
	pthread_t IMU_TimerThread;		//handle for IMU Timer thread
	pthread_t PCA_Thread;			//handle for PCA thread
	pthread_t PCA_TimerThread;		//handle for PCA Timer thread
	pthread_t MAG_Thread;			//handle for MAG thread
	pthread_t PrintThread;			//handle for Printing thread
	pthread_t rosSpinThread;
	pthread_t rosPublisherThread;
	pthread_t Kalman_Thread;
	//long IDthreadKeyboard, IDthreadIMU, IDthreadMAG; //Stores ID for threads
	int ReturnCode;

	// kalman_v.open ("kalman_velocity.txt");
	// vicon_p.open ("vicon_position.txt");

	//Check if number of arguments are correct
	printf("Number of arguments: %d\n",argc);
	if(argc != 2) {
	  _nh.logfatal("Incorrect number of arguments\n");
	  return -1;
	}
	else {
	  rosSrvrIp = argv[1];
	}

	//Start ROS handle
	_nh.initNode(rosSrvrIp);

	//Set initial reference
	attRefJoy.v[0] = 0; attRefJoy.v[1] = 0; attRefJoy.v[2] = 0;
	PVA_RefJoy.pos.position.x = 0;
	PVA_RefJoy.pos.position.y = 0;
	PVA_RefJoy.pos.position.z = 1;
	PVA_RefJoy.vel.linear.x = 0;
	PVA_RefJoy.vel.linear.y = 0;
	PVA_RefJoy.vel.linear.z = 0;

	//event that signalizes end of initialization
	e_endInit = CreateEvent(false,false);

	//switch source of yaw (vicon x imu)
	e_SwitchYawSource = CreateEvent(true,false);

	//Keyboard Events for printing information
	e_Key1 = CreateEvent(true,false); 			//manual-reset event
	e_Key2 = CreateEvent(true,false); 			//manual-reset event
	e_Key3 = CreateEvent(true,false);           //manual-reset event
	e_Key4 = CreateEvent(true,false);           //manual-reset event
	e_Key5 = CreateEvent(true,false);           //manual-reset event
	e_Key6 = CreateEvent(true,false);           //manual-reset event
	e_Key7 = CreateEvent(true,false);           //manual-reset event
	e_Key8 = CreateEvent(true,false);           //manual-reset event
	e_Key9 = CreateEvent(true,false);           //manual-reset event

	//Keyboard event for execution termination
	e_KeyESC = CreateEvent(true,false); 		//abort manual-reset event

	//Joystick events for changing flight modes
	e_buttonX = CreateEvent(true,false);
	e_buttonY = CreateEvent(true,false);
	e_buttonA = CreateEvent(true,false);
	e_buttonB = CreateEvent(true,false);

	//Events for changing thrust values
	e_Motor_Up = CreateEvent(false,false);      //auto-reset event
	e_Motor_Down = CreateEvent(false,false);    //auto-reset event
	e_Motor_Kill = CreateEvent(false,false);    //auto-reset event

	//Events that trigget threads
	e_Timeout = CreateEvent(false,false);		//timeout event (always false)
	e_IMU_trigger = CreateEvent(false,false); 	//auto-reset event
	e_PCA_trigger = CreateEvent(false,false); 	//auto-reset event
	e_AttControl_trigger = CreateEvent(false, false);  //auto-reset event
	e_PosControl_trigger = CreateEvent(false, false);  //auto-reset event

	//Create mutexes
	pthread_mutex_init(&IMU_Mutex, NULL);
	pthread_mutex_init(&PCA_Mutex, NULL);
	pthread_mutex_init(&ThrustJoy_Mutex, NULL);
	pthread_mutex_init(&Contr_Input_Mutex, NULL);
	pthread_mutex_init(&PID_Mutex, NULL);
	pthread_mutex_init(&attRefJoy_Mutex, NULL);
	pthread_mutex_init(&stateMachine_Mutex, NULL);
	pthread_mutex_init(&PVA_Vicon_Mutex, NULL);
	pthread_mutex_init(&ROS_Mutex, NULL);
	pthread_mutex_init(&posRefJoy_Mutex, NULL);
	pthread_mutex_init(&ThrustPosControl_Mutex, NULL);
	pthread_mutex_init(&YawSource_Mutex, NULL);

	//Load PID parameters from file
	pthread_mutex_lock(&PID_Mutex);
		updatePar(&PID_att, &PID_angVel, &PID_pos);
	pthread_mutex_unlock(&PID_Mutex);

	//Start keyboard task
	if (ReturnCode = pthread_create(&keyboardThread, NULL, KeyboardTask, NULL)){
		printf("Start KeyboardTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start rosSpin task
	if (ReturnCode = pthread_create(&rosSpinThread, NULL, rosSpinTask, NULL)){
		printf("Start rosSpin failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start ros publisher task
	if (ReturnCode = pthread_create(&rosPublisherThread, NULL, rosPublisherTask, NULL)){
		printf("Start rosPublisher failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;


	//Start state StateMachineTask
	if (ReturnCode = pthread_create(&StateMachine_Thread, NULL, StateMachineTask, NULL)){
		printf("Start StateMachineTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start Attitude Control timer task
	if (ReturnCode = pthread_create(&AttControl_TimerThread, NULL, AttControl_Timer, NULL)){
		printf("Start Control_TimerTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start Attitude Control task
	if (ReturnCode = pthread_create(&AttControl_Thread, NULL, AttControl_Task, NULL)){
		printf("Start Control_Task failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
			threadCount += 1;

	//Start Position Control timer task
	if (ReturnCode = pthread_create(&PosControl_TimerThread, NULL, PosControl_Timer, NULL)){
		printf("Start PosControl_Timer failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	//Start position Control task
	if (ReturnCode = pthread_create(&PosControl_Thread, NULL, PosControl_Task, NULL)){
		printf("Start PosControl_Task failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
			threadCount += 1;

	//Start keyboard control task
	if (ReturnCode = pthread_create(&Motor_ControlThread, NULL, Motor_KeyboardControl, NULL)){
		printf("Start Motor_ContorlTask failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else
		threadCount += 1;

	// //Start IMU task
	// if (ReturnCode = pthread_create(&IMU_Thread, NULL, IMU_Task, NULL)){
	// 	printf("Start IMU_Task failed; return code from pthread_create() is %d\n", ReturnCode);
	// 	exit(-1);
	// }
	// else
	// 	threadCount += 1;

	// //Start IMU timer task
	// if (ReturnCode = pthread_create(&IMU_TimerThread, NULL, IMU_Timer, NULL)){
	// 	printf("Start IMU_TimerTask failed; return code from pthread_create() is %d\n", ReturnCode);
	// 	exit(-1);
	// }
	// else
	// 	threadCount += 1;

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

	//Start kalman filter task
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
	DestroyEvent(e_KeyESC);
	DestroyEvent(e_buttonX);
	DestroyEvent(e_buttonY);
	DestroyEvent(e_buttonA);
	DestroyEvent(e_buttonB);
	DestroyEvent(e_Motor_Up);
	DestroyEvent(e_Motor_Down);
	DestroyEvent(e_Motor_Kill);
	DestroyEvent(e_Timeout);
	DestroyEvent(e_IMU_trigger);
	DestroyEvent(e_PCA_trigger);
	DestroyEvent(e_AttControl_trigger);
	DestroyEvent(e_PosControl_trigger);
	DestroyEvent(e_endInit);
	DestroyEvent(e_SwitchYawSource);

	//Destroy mutexes
	pthread_mutex_destroy(&IMU_Mutex);
	pthread_mutex_destroy(&PCA_Mutex);
	pthread_mutex_destroy(&ThrustJoy_Mutex);
	pthread_mutex_destroy(&Contr_Input_Mutex);
	pthread_mutex_destroy(&PID_Mutex);
	pthread_mutex_destroy(&attRefJoy_Mutex);
	pthread_mutex_destroy(&stateMachine_Mutex);
	pthread_mutex_destroy(&PVA_Vicon_Mutex);
	pthread_mutex_destroy(&ROS_Mutex);
	pthread_mutex_destroy(&posRefJoy_Mutex);
	pthread_mutex_destroy(&ThrustPosControl_Mutex);
	pthread_mutex_destroy(&YawSource_Mutex);

   /* Last thing that main() should do */
	// vicon_p.close();
	// kalman_v.close(); 
    pthread_exit(NULL);
}
