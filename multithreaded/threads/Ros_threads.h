

#include "control/MatricesAndVectors.h"
#include "threads/stateMachine.h"
// #include "rosserial/ros.h"
//#include "rosserial/std_msgs/Float32MultiArray.h"
#include "rosserial/sensor_msgs/Joy.h"
#include "rosserial/geometry_msgs/TransformStamped.h"
#include "rosserial/qcontrol_defs/PVA.h"
#include "kalman.h"

extern qcontrol_defs::PVA PVA_quadVicon, PVA_RefJoy;

//Events and mutexes
extern neosmart_event_t e_buttonX, e_buttonY, e_buttonA, e_buttonB;
extern neosmart_event_t e_Timeout; //Always false event for forcing timeout of WaitForEvent
extern pthread_mutex_t IMU_Mutex;	//protect IMU data
extern pthread_mutex_t ThrustJoy_Mutex;
extern pthread_mutex_t attRefJoy_Mutex;
extern pthread_mutex_t attRefPosControl_Mutex;
extern pthread_mutex_t Contr_Input_Mutex;
extern pthread_mutex_t stateMachine_Mutex;
extern pthread_mutex_t PVA_Vicon_Mutex;
extern pthread_mutex_t ROS_Mutex;

//Global variables
extern float ThrustJoy;
extern Vec3 IMU_Data_RPY;
extern Vec3 attRefJoy;
extern Vec4 Contr_Input;
extern int threadCount;		//Counts active threads

#define PI 3.1415
#define maxThrust_AttMode 3
#define maxThrust_MotorMode 0.3
#define yaw_Inc PI/90


void handle_mp_joy_msg(const sensor_msgs::Joy& msg);

void handle_Vicon(const geometry_msgs::TransformStamped& msg);