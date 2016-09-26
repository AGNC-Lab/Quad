

#include "threads/Ros_threads.h"


int ButtonX = 0;
int ButtonY = 0;
int ButtonA = 0;
int ButtonB = 0;

void handle_mp_joy_msg(const sensor_msgs::Joy& msg){
	float yaw_ctr_pos, yaw_ctr_neg;
	int localCurrentState;
	Vec3 IMU_localData_RPY;

	//Grab attitude estimation
	pthread_mutex_lock(&IMU_Mutex);
		IMU_localData_RPY = IMU_Data_RPY;
	pthread_mutex_unlock(&IMU_Mutex);

	pthread_mutex_lock(&stateMachine_Mutex);
		localCurrentState = currentState;
	pthread_mutex_unlock(&stateMachine_Mutex);

	//If in attitude mode
	if(localCurrentState == ATTITUDE_MODE){
		//Set thrust
		pthread_mutex_lock(&ThrustJoy_Mutex);
			ThrustJoy = msg.axes[1] * maxThrust_AttMode;
		pthread_mutex_unlock(&ThrustJoy_Mutex);

		//Set references
		pthread_mutex_lock(&attRefJoy_Mutex);	
			attRefJoy.v[0] = -msg.axes[3]*PI/6; //roll
			attRefJoy.v[1] = msg.axes[4]*PI/6; //pitch
			yaw_ctr_pos = msg.axes[2];
			yaw_ctr_neg = msg.axes[5];
			//Set yaw to measured yaw if quad isnt flying
			if (msg.axes[1] <= 0) {
		    	attRefJoy.v[2] = IMU_localData_RPY.v[2];
		    }
		    else{ //If quad is flying, increment yaw
		    	if (yaw_ctr_neg < 0) {
					attRefJoy.v[2] -= yaw_Inc;
				}								//yaw
				if (yaw_ctr_pos < 0) {
					attRefJoy.v[2] += yaw_Inc;
				}
		    }
		pthread_mutex_unlock(&attRefJoy_Mutex);
	}
	else if(localCurrentState == MOTOR_MODE){ 	//If in motor mode
		//Set thrust
		pthread_mutex_lock(&ThrustJoy_Mutex);
			ThrustJoy = msg.axes[1] * maxThrust_MotorMode;
		pthread_mutex_unlock(&ThrustJoy_Mutex);

		//Set attitude with zero error
		pthread_mutex_lock(&attRefJoy_Mutex);	
			attRefJoy.v[0] = IMU_localData_RPY.v[0]; //Set ref to actual IMU value
			attRefJoy.v[1] = IMU_localData_RPY.v[1]; //Set ref to actual IMU value
			attRefJoy.v[2] = IMU_localData_RPY.v[2]; //Set ref to actual IMU value
		pthread_mutex_unlock(&attRefJoy_Mutex);
	}
	else{ //If not in a fly mode, set everything to zero
		pthread_mutex_lock(&ThrustJoy_Mutex);
			ThrustJoy = 0;
		pthread_mutex_unlock(&ThrustJoy_Mutex);

		//Set attitude with zero error
		pthread_mutex_lock(&attRefJoy_Mutex);	
			attRefJoy.v[0] = IMU_localData_RPY.v[0]; //Set ref to actual IMU value
			attRefJoy.v[1] = IMU_localData_RPY.v[1]; //Set ref to actual IMU value
			attRefJoy.v[2] = IMU_localData_RPY.v[2]; //Set ref to actual IMU value
		pthread_mutex_unlock(&attRefJoy_Mutex);
	}
	
	//Compare joystick buttons with previously read (check if state changed)
	if (msg.buttons[0] && !ButtonA){
		SetEvent(e_buttonA);
		//printf("ButtonA Pushed!\n");
	}
	if (msg.buttons[1] && !ButtonB){
		SetEvent(e_buttonB);
		printf("ButtonB Pushed!\n");
	}
	if (msg.buttons[2] && !ButtonX){
		SetEvent(e_buttonX);
		//printf("ButtonX Pushed!\n");
	}
	if (msg.buttons[3] && !ButtonY){
		SetEvent(e_buttonY);
		//printf("ButtonY Pushed!\n");
	}
	ButtonA = msg.buttons[0];
	ButtonB = msg.buttons[1];
	ButtonX = msg.buttons[2];
	ButtonY = msg.buttons[3];

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